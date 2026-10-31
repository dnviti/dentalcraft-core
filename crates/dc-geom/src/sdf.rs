//! Narrow-band signed distance fields: offset, closing, CSG and meshing (WP18).
//!
//! A [`SdfGrid`] is a sparse narrow-band level set: 8³ voxel blocks in a hash
//! map keyed by block coordinate, `f32` distances (negative inside), a voxel
//! size of typically 0.1–0.4 mm and a band half width. Voxels outside the
//! stored blocks read `±band` (inside: blocks recorded as interior tiles).
//! Voxel `ijk` sits at `origin + ijk * voxel_size`; grids built by
//! [`SdfGrid::from_mesh`] use the world origin, so grids of the same voxel
//! size line up for CSG.
//!
//! The construction follows the OpenVDB operations that the Blender add-on
//! uses (`aligner._sdf_group`, replicated on the GPU by `gpu_sdf.py`):
//!
//! * [`SdfGrid::from_mesh`] (`meshToLevelSet`): exact distances through the
//!   BVH, sign by a flood fill from outside (holes in scans narrower than
//!   about a voxel stay closed; overlapping closed meshes give their union);
//! * [`SdfGrid::offset`] moves the level set and rebuilds the band with exact
//!   distances to the new surface, so [`SdfGrid::close`] (dilate then erode)
//!   is the exact morphological closing that the blockout needs;
//! * [`SdfGrid::csg`] combines two grids with sharp or smooth min / max;
//! * [`SdfGrid::to_mesh`] (`volumeToMesh`) gives a closed, manifold,
//!   outward-oriented triangle mesh, deterministic for any thread count.
//!
//! Everything runs in parallel with rayon.

mod build;
mod flood;
mod mesher;
mod ops;
mod store;

use glam::{IVec3, Vec3};

use dc_mesh::TriMesh;

use crate::Result;

use store::Store;

/// Smallest band half width, in voxels (the surface and its first voxels on
/// both sides must be exact for meshing and offsets).
const MIN_BAND_VOXELS: f32 = 1.5;

/// SDF sampling parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct SdfParams {
    /// Voxel edge length (mm).
    pub voxel_size: f32,
    /// Half width of the narrow band (mm); raised to at least 1.5 voxels.
    /// Offsets beyond the band widen it first (one extra pass), so set it to
    /// the largest offset planned plus a few voxels.
    pub band: f32,
}

impl Default for SdfParams {
    fn default() -> Self {
        Self {
            voxel_size: 0.1,
            band: 1.0,
        }
    }
}

/// Boolean combination of two grids (`self` op `other`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CsgOp {
    /// `min(a, b)`.
    Union,
    /// `max(a, b)`.
    Intersection,
    /// `max(a, -b)`: `self` minus `other`.
    Difference,
    /// Polynomial smooth minimum with blend radius `k` (mm).
    SmoothUnion(f32),
    /// Smooth maximum with blend radius `k` (mm).
    SmoothIntersection(f32),
    /// Smooth difference with blend radius `k` (mm).
    SmoothDifference(f32),
}

/// Sparse narrow-band signed distance grid (negative inside).
#[derive(Clone)]
pub struct SdfGrid {
    /// Voxel edge length (mm).
    pub voxel_size: f32,
    /// World position of voxel `(0, 0, 0)`.
    pub origin: Vec3,
    band: f32,
    /// The band holds exact distances (false after CSG).
    exact: bool,
    store: Store,
}

impl std::fmt::Debug for SdfGrid {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SdfGrid")
            .field("voxel_size", &self.voxel_size)
            .field("origin", &self.origin)
            .field("band", &self.band)
            .field("blocks", &self.store.len())
            .field("tiles", &self.store.tile_count())
            .finish()
    }
}

impl SdfGrid {
    fn from_store(voxel_size: f32, origin: Vec3, band: f32, store: Store) -> Self {
        Self {
            voxel_size,
            origin,
            band,
            exact: true,
            store,
        }
    }

    /// Samples the signed distance of `mesh`: the solid seen from outside
    /// (overlapping closed parts are merged, cavities that cannot be reached
    /// from outside are filled, holes narrower than about a voxel are
    /// bridged).
    ///
    /// # Errors
    /// [`crate::GeomError::InvalidInput`] for an empty mesh, out-of-range
    /// indices, a voxel size that is not positive or a mesh too large for it.
    pub fn from_mesh(mesh: &TriMesh, params: &SdfParams) -> Result<Self> {
        build::from_mesh(mesh, params.voxel_size, params.band)
    }

    /// Empty grid (no solid) with the given parameters.
    ///
    /// # Errors
    /// [`crate::GeomError::InvalidInput`] for invalid parameters.
    pub fn empty(params: &SdfParams) -> Result<Self> {
        let band = build::check_params(params.voxel_size, params.band)?;
        Ok(Self::from_store(
            params.voxel_size,
            Vec3::ZERO,
            band,
            Store::default(),
        ))
    }

    /// Half width of the narrow band (mm); values outside read `±band`.
    pub fn band(&self) -> f32 {
        self.band
    }

    /// Number of stored 8³ blocks.
    pub fn block_count(&self) -> usize {
        self.store.len()
    }

    /// Number of blocks recorded as entirely inside.
    pub fn interior_tile_count(&self) -> usize {
        self.store.tile_count()
    }

    /// Number of stored voxels closer to the surface than the band.
    pub fn band_voxel_count(&self) -> usize {
        ops::band_voxels(self)
    }

    /// Approximate heap size in bytes.
    pub fn approx_bytes(&self) -> usize {
        self.store.approx_bytes()
    }

    /// World position of voxel `ijk`.
    pub fn voxel_position(&self, ijk: IVec3) -> Vec3 {
        self.origin + ijk.as_vec3() * self.voxel_size
    }

    /// Value stored at voxel `ijk` (`±band` outside the stored blocks).
    pub fn voxel(&self, ijk: IVec3) -> f32 {
        self.store.value(ijk, self.band)
    }

    /// Calls `f(ijk, value)` for every stored voxel, block after block in
    /// storage order.
    pub fn for_each_voxel(&self, f: impl FnMut(IVec3, f32)) {
        ops::for_each_voxel(self, f);
    }

    /// Signed distance at `p` by trilinear interpolation (`NaN` for a point
    /// that is not finite).
    pub fn sample(&self, p: Vec3) -> f32 {
        ops::sample(self, p)
    }

    /// Gradient of the field at `p` (central differences of
    /// [`SdfGrid::sample`] one voxel apart); about unit length in the band.
    pub fn gradient(&self, p: Vec3) -> Vec3 {
        let h = self.voxel_size;
        let d = |o: Vec3| self.sample(p + o) - self.sample(p - o);
        Vec3::new(d(Vec3::X * h), d(Vec3::Y * h), d(Vec3::Z * h)) / (2.0 * h)
    }

    /// `true` when `p` is inside the solid.
    pub fn contains(&self, p: Vec3) -> bool {
        self.sample(p) < 0.0
    }

    /// Offsets the surface by `distance` (mm, positive = outwards) and
    /// rebuilds the band with exact distances to the new surface.
    ///
    /// # Errors
    /// [`crate::GeomError::InvalidInput`] for a distance that is not finite.
    pub fn offset(&mut self, distance: f32) -> Result<()> {
        ops::offset(self, distance)
    }

    /// Morphological closing (dilate then erode) by `radius` (mm): crevices
    /// narrower than `2 * radius` are filled, concave corners rounded.
    ///
    /// # Errors
    /// [`crate::GeomError::InvalidInput`] for a negative or non-finite radius.
    pub fn close(&mut self, radius: f32) -> Result<()> {
        check_radius(radius)?;
        self.offset(radius)?;
        self.offset(-radius)
    }

    /// Morphological opening (erode then dilate) by `radius` (mm): parts
    /// thinner than `2 * radius` are removed, convex edges rounded.
    ///
    /// # Errors
    /// [`crate::GeomError::InvalidInput`] for a negative or non-finite radius.
    pub fn open(&mut self, radius: f32) -> Result<()> {
        check_radius(radius)?;
        self.offset(-radius)?;
        self.offset(radius)
    }

    /// Combines `other` into `self` voxel by voxel. The band becomes the
    /// narrower of the two; distances away from the surface are no longer
    /// exact, which the next [`SdfGrid::offset`] corrects first (or call
    /// [`SdfGrid::redistance`]).
    ///
    /// # Errors
    /// [`crate::GeomError::InvalidInput`] when the grids do not share voxel
    /// size and origin, or for an invalid blend radius.
    pub fn csg(&mut self, other: &SdfGrid, op: CsgOp) -> Result<()> {
        ops::csg(self, other, op)
    }

    /// `self ∪ other`.
    ///
    /// # Errors
    /// See [`SdfGrid::csg`].
    pub fn union(&mut self, other: &SdfGrid) -> Result<()> {
        self.csg(other, CsgOp::Union)
    }

    /// `self ∩ other`.
    ///
    /// # Errors
    /// See [`SdfGrid::csg`].
    pub fn intersect(&mut self, other: &SdfGrid) -> Result<()> {
        self.csg(other, CsgOp::Intersection)
    }

    /// `self − other`.
    ///
    /// # Errors
    /// See [`SdfGrid::csg`].
    pub fn subtract(&mut self, other: &SdfGrid) -> Result<()> {
        self.csg(other, CsgOp::Difference)
    }

    /// Fills the cavities: outside regions that cannot be reached from far
    /// away become inside (flood fill from outside), then the band is
    /// rebuilt.
    ///
    /// # Errors
    /// [`crate::GeomError::InvalidInput`] when the grid spans too many blocks.
    pub fn fill_interior(&mut self) -> Result<()> {
        ops::fill_interior(self)
    }

    /// Rebuilds the band with exact distances to the current zero level.
    ///
    /// # Errors
    /// Errors of the BVH build (not expected for a valid grid).
    pub fn redistance(&mut self) -> Result<()> {
        ops::redistance(self, self.band)
    }

    fn redistance_to(&mut self, band: f32) -> Result<()> {
        ops::redistance(self, band)
    }

    /// Extracts the zero level set as a closed, manifold triangle mesh with
    /// counter-clockwise (outward) faces. Empty for an empty solid.
    pub fn to_mesh(&self) -> Result<TriMesh> {
        self.to_mesh_at(0.0)
    }

    /// Extracts the level set `value = iso` (mm), like [`SdfGrid::to_mesh`];
    /// `iso` must stay inside the band to be accurate.
    ///
    /// # Errors
    /// [`crate::GeomError::InvalidInput`] for an `iso` that is not finite.
    pub fn to_mesh_at(&self, iso: f32) -> Result<TriMesh> {
        if !iso.is_finite() {
            return Err(crate::GeomError::InvalidInput(format!(
                "iso level must be finite, got {iso}"
            )));
        }
        let (positions, tris) =
            mesher::extract(&self.store, self.band, self.origin, self.voxel_size, iso);
        Ok(TriMesh::new(positions, tris))
    }
}

fn check_radius(radius: f32) -> Result<()> {
    if radius.is_finite() && radius >= 0.0 {
        Ok(())
    } else {
        Err(crate::GeomError::InvalidInput(format!(
            "radius must be finite and not negative, got {radius}"
        )))
    }
}
