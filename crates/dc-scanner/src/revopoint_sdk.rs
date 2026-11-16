//! Revopoint `3DCamera` SDK backend `revopoint-sdk` (alpha).
//!
//! Revopoint scanners (MINI 2 and the other models of the Revopoint SDK
//! authorization service) stream depth through the vendor's closed
//! `3DCamera` library. DentalCraft never links nor ships it: the library is
//! loaded at run time with [`libloading`] from the path in the environment
//! variable [`SDK_ENV`] (the library file, or the directory that holds it),
//! else from [`default_sdk_dir`] (`<data dir>/dentalcraft/revopoint-sdk/`)
//! under the platform name [`LIBRARY_NAME`].
//!
//! The C API is declared by hand from the vendor headers of SDK v3.2.229
//! (`h/system.h`, `h/camera.h`, `h/frame.h`, `hpp/Types.hpp`); the layout
//! tests pin every struct to the sizes and offsets of those headers.
//!
//! * Listing never fails because of the SDK: without the library
//!   [`ScannerBackend::list`] returns no device and
//!   [`ScannerBackend::availability`] reports `sdk-missing`.
//! * The depth stream uses the `Z16` profile with the highest frame rate
//!   (largest image on ties); depth in millimetres is `raw * depth scale`
//!   (`PROPERTY_EXT_DEPTH_SCALE`). Frames carry the depth intrinsics scaled
//!   from the calibration resolution to the stream resolution. Colour is not
//!   read (the RGB stream is not registered to the depth image).
//! * The vendor enables SDK access per device, by serial number, after the
//!   purchase of an authorization. An unauthorised device is detected when
//!   the SDK refuses to start the depth stream, and reported as
//!   `sdk-unauthorized` ([`ScanError::SdkUnauthorized`]).
//! * Other SDK failures are [`ScanError::Device`] errors whose detail names
//!   the call and the vendor `ERROR_CODE` (number and name).
//!
//! Thread safety: the library and its `CSystem` context are shared; the
//! system calls (listing, connect, disconnect) are serialised by a mutex. An
//! opened [`RevopointDevice`] is `Send` but not `Sync`: every camera, stream
//! and frame call goes through `&mut self`, so one thread at a time uses the
//! handles, and during a scan only the acquisition thread does. Frame
//! buffers are copied before the frame is released to the SDK.

#![allow(unsafe_code)]
#![warn(clippy::undocumented_unsafe_blocks)]

use std::ffi::{OsStr, c_char, c_int, c_void};
use std::marker::{PhantomData, PhantomPinned};
use std::path::{Path, PathBuf};
use std::ptr::NonNull;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use libloading::Library;

use crate::{
    CameraIntrinsics, DepthFrame, DeviceInfo, Result, ScanError, ScannerBackend, ScannerDevice,
};

/// Id of the Revopoint SDK backend.
pub const BACKEND_ID: &str = "revopoint-sdk";
/// Environment variable naming the SDK library file, or its directory.
pub const SDK_ENV: &str = "DC_REVOPOINT_SDK";
/// File name of the SDK library on this platform.
#[cfg(windows)]
pub const LIBRARY_NAME: &str = "3DCamera.dll";
/// File name of the SDK library on this platform.
#[cfg(target_os = "macos")]
pub const LIBRARY_NAME: &str = "lib3DCamera.dylib";
/// File name of the SDK library on this platform.
#[cfg(all(unix, not(target_os = "macos")))]
pub const LIBRARY_NAME: &str = "lib3DCamera.so";

// ----- C API (vendor headers, SDK v3.2.229) --------------------------------

/// Declares an opaque C handle type (`typedef struct X X;`).
macro_rules! opaque {
    ($($name:ident),*) => {$(
        #[repr(C)]
        #[allow(dead_code, reason = "only handled through pointers from the SDK")]
        struct $name {
            _data: [u8; 0],
            _marker: PhantomData<(*mut u8, PhantomPinned)>,
        }
    )*};
}
opaque!(CSystem, CCamera, CStream, CFrame);

/// `ERROR_CODE::SUCCESS`.
const SUCCESS: c_int = 0;
/// `ERROR_CODE::ERROR_FRAME_TIMEOUT`.
const ERROR_FRAME_TIMEOUT: c_int = 7;
/// `ERROR_CODE::ERROR_NOT_SUPPORT`.
const ERROR_NOT_SUPPORT: c_int = 8;
/// `STREAM_TYPE::STREAM_TYPE_DEPTH`.
const STREAM_TYPE_DEPTH: c_int = 0;
/// `STREAM_FORMAT::STREAM_FORMAT_Z16`: one `u16` per pixel.
const STREAM_FORMAT_Z16: c_int = 0x02;
/// `STREAM_FORMAT::STREAM_FORMAT_Z16Y8Y8`: depth plus both infrared images.
const STREAM_FORMAT_Z16Y8Y8: c_int = 0x03;
/// `STREAM_FORMAT::STREAM_FORMAT_Z16Y8Y8P`: depth plus four infrared images.
const STREAM_FORMAT_Z16Y8Y8P: c_int = 0x109;
/// `FRAME_DATA_FORMAT::FRAME_DATA_FORMAT_Z16`: the depth part of a frame.
const FRAME_DATA_FORMAT_Z16: c_int = 0x00;
/// `PROPERTY_TYPE::PROPERTY_EXPOSURE`.
const PROPERTY_EXPOSURE: c_int = 0x01;
/// `PROPERTY_TYPE_EXTENSION::PROPERTY_EXT_DEPTH_SCALE`.
const PROPERTY_EXT_DEPTH_SCALE: c_int = 0x00;

/// Names of the vendor `ERROR_CODE` values, by value.
const ERROR_NAMES: [&str; 19] = [
    "SUCCESS",
    "ERROR_PARAM",
    "ERROR_DEVICE_NOT_FOUND",
    "ERROR_DEVICE_NOT_CONNECT",
    "ERROR_DEVICE_BUSY",
    "ERROR_STREAM_NOT_START",
    "ERROR_STREAM_BUSY",
    "ERROR_FRAME_TIMEOUT",
    "ERROR_NOT_SUPPORT",
    "ERROR_PROPERTY_GET_FAILED",
    "ERROR_PROPERTY_SET_FAILED",
    "ERROR_HID_CHANNEL_ERROR",
    "ERROR_HID_WRITE_ERROR",
    "ERROR_HID_READ_ERROR",
    "ERROR_DYNAMIC_RES_ABSENCE",
    "ERROR_DYNAMIC_FUC_ABSENCE",
    "ERROR_DEVICE_PARAM_ERROR",
    "ERROR_GET_INTERNAL_RES_FAILED",
    "ERROR_UNKNOW",
];

/// `StreamInfo` (`hpp/Types.hpp`): one stream profile.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
struct StreamInfo {
    /// `STREAM_FORMAT`.
    format: c_int,
    width: c_int,
    height: c_int,
    fps: f32,
}

/// `CameraInfo` (`hpp/Types.hpp`): identity of a connected camera.
#[repr(C)]
#[derive(Clone, Copy)]
#[allow(dead_code, reason = "mirrors the C layout; the versions are not read")]
struct CameraInfo {
    name: [c_char; 32],
    serial: [c_char; 32],
    unique_id: [c_char; 32],
    firmware_version: [c_char; 32],
    algorithm_version: [c_char; 32],
}

/// `Intrinsics` (`hpp/Types.hpp`): 3x3 camera matrix, row-major, at the
/// calibration resolution `width x height`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
#[allow(
    dead_code,
    reason = "mirrors the C layout; the zero / one cells are not read"
)]
struct Intrinsics {
    width: i16,
    height: i16,
    fx: f32,
    zero01: f32,
    cx: f32,
    zero10: f32,
    fy: f32,
    cy: f32,
    zero20: f32,
    zero21: f32,
    one22: f32,
}

/// `PropertyExtension` (`hpp/Types.hpp`): a union of 1032 bytes, aligned to
/// 8 (its largest member is `CameraParamMng`, 1028 bytes; it holds 64-bit
/// fields). Only the members DentalCraft reads are named.
#[repr(C, align(8))]
#[allow(dead_code, reason = "`raw` only gives the union the header's size")]
union PropertyExtension {
    depth_scale: f32,
    raw: [u8; 1032],
}

/// Signature of `cameraStartStream`'s frame callback (`CFrameCallback`).
type CFrameCallback = unsafe extern "C" fn(frame: *mut CFrame, user: *mut c_void);

/// Declares the function table of the SDK and its loader.
macro_rules! api {
    ($($field:ident = $symbol:literal: fn($($arg:ty),*) $(-> $ret:ty)?;)*) => {
        /// The SDK functions used by the backend, resolved from the library.
        struct Api {
            $($field: unsafe extern "C" fn($($arg),*) $(-> $ret)?,)*
        }

        impl Api {
            /// Resolves every function; the error names the missing symbol.
            ///
            /// # Safety
            /// The signatures above must match the library's exports, and the
            /// pointers must not be called after `lib` is unloaded.
            unsafe fn load(lib: &Library) -> std::result::Result<Self, String> {
                Ok(Self {$(
                    // SAFETY: the caller guarantees the declared signature;
                    // the symbol is copied out as a plain function pointer.
                    $field: *unsafe {
                        lib.get::<unsafe extern "C" fn($($arg),*) $(-> $ret)?>($symbol.as_bytes())
                    }
                    .map_err(|e| format!("symbol `{}`: {e}", $symbol))?,
                )*})
            }
        }
    };
}

api! {
    create_system = "createSystem": fn() -> *mut CSystem;
    delete_system = "deleteSystem": fn(*mut CSystem);
    create_camera_info_list = "systemCreateCameraInfoList": fn(*const CSystem, *mut c_int) -> *mut CameraInfo;
    delete_camera_info_list = "systemDeleteCameraInfoList": fn(*mut CameraInfo);
    connect_camera = "systemConnectCamera": fn(*mut CSystem, *mut CameraInfo) -> *mut CCamera;
    disconnect_camera = "systemDisconnectCamera": fn(*mut CSystem, *mut CCamera);
    create_stream_info_list = "cameraCreateStreamInfoList": fn(*const CCamera, c_int, *mut c_int) -> *mut StreamInfo;
    delete_stream_info_list = "cameraDeleteStreamInfoList": fn(*mut StreamInfo);
    start_stream = "cameraStartStream": fn(*const CCamera, c_int, StreamInfo, Option<CFrameCallback>, *mut c_void) -> *mut CStream;
    stop_stream = "cameraStopStream": fn(*mut CStream);
    // `ERROR_CODE cameraGetFrame(CStream*, CFrame*& frame, int)`: a C++
    // reference is passed as a pointer.
    get_frame = "cameraGetFrame": fn(*mut CStream, *mut *mut CFrame, c_int) -> c_int;
    release_frame = "cameraReleaseFrame": fn(*mut CStream, *mut CFrame) -> c_int;
    get_property_range = "cameraGetPropertyRange": fn(*const CCamera, c_int, c_int, *mut f32, *mut f32, *mut f32) -> c_int;
    set_property = "cameraSetProperty": fn(*const CCamera, c_int, c_int, f32) -> c_int;
    get_property_extension = "cameraGetPropertyExtension": fn(*const CCamera, c_int, *mut PropertyExtension) -> c_int;
    get_stream_intrinsics = "cameraGetStreamIntrinsics": fn(*const CCamera, c_int, *mut Intrinsics) -> c_int;
    frame_timestamp = "frameGetTimestamp": fn(*const CFrame) -> f64;
    frame_data = "frameGetData": fn(*const CFrame) -> *const c_void;
    frame_data_by_format = "frameGetDataByFormat": fn(*const CFrame, c_int) -> *const c_void;
    frame_data_size = "frameGetDataSize": fn(*const CFrame) -> c_int;
    frame_width = "frameGetWidth": fn(*const CFrame) -> c_int;
    frame_height = "frameGetHeight": fn(*const CFrame) -> c_int;
    frame_format = "frameGetFormat": fn(*const CFrame) -> c_int;
}

// ----- library location ----------------------------------------------------

/// The per-user directory searched for the SDK library when [`SDK_ENV`] is
/// not set: `<data dir>/dentalcraft/revopoint-sdk`, where the data directory
/// is `%APPDATA%` on Windows, `~/Library/Application Support` on macOS and
/// `$XDG_DATA_HOME` (default `~/.local/share`) elsewhere. `None` when the
/// environment does not define it.
pub fn default_sdk_dir() -> Option<PathBuf> {
    data_dir().map(|d| d.join("dentalcraft").join("revopoint-sdk"))
}

#[cfg(windows)]
pub(crate) fn data_dir() -> Option<PathBuf> {
    absolute_env("APPDATA")
}

#[cfg(target_os = "macos")]
pub(crate) fn data_dir() -> Option<PathBuf> {
    absolute_env("HOME").map(|h| h.join("Library").join("Application Support"))
}

#[cfg(all(unix, not(target_os = "macos")))]
pub(crate) fn data_dir() -> Option<PathBuf> {
    absolute_env("XDG_DATA_HOME")
        .or_else(|| absolute_env("HOME").map(|h| h.join(".local").join("share")))
}

/// The value of `var` as a path, when it is set to an absolute path.
fn absolute_env(var: &str) -> Option<PathBuf> {
    std::env::var_os(var)
        .map(PathBuf::from)
        .filter(|p| p.is_absolute())
}

/// The SDK library path: `env_value` (the value of [`SDK_ENV`]) names the
/// library file or its directory; without it the library is searched in
/// `sdk_dir`. Err `sdk-missing` when the library file does not exist.
fn resolve_library(env_value: Option<&OsStr>, sdk_dir: Option<&Path>) -> Result<PathBuf> {
    let path = match env_value.filter(|v| !v.is_empty()) {
        Some(v) => {
            let p = PathBuf::from(v);
            if p.is_dir() { p.join(LIBRARY_NAME) } else { p }
        }
        None => sdk_dir
            .ok_or_else(|| {
                ScanError::SdkMissing(format!(
                    "no data directory to search for {LIBRARY_NAME}; set {SDK_ENV}"
                ))
            })?
            .join(LIBRARY_NAME),
    };
    if path.is_file() {
        Ok(path)
    } else {
        Err(ScanError::SdkMissing(format!(
            "{} not found (install the Revopoint 3DCamera SDK there or set {SDK_ENV})",
            path.display()
        )))
    }
}

/// Where this backend takes its library from.
#[derive(Debug, Clone, PartialEq, Eq)]
enum LibrarySource {
    /// [`SDK_ENV`], else [`default_sdk_dir`], resolved at every use (an SDK
    /// installed while the app runs is picked up).
    Default,
    /// A fixed library file.
    Path(PathBuf),
}

impl LibrarySource {
    fn resolve(&self) -> Result<PathBuf> {
        match self {
            Self::Default => resolve_library(
                std::env::var_os(SDK_ENV).as_deref(),
                default_sdk_dir().as_deref(),
            ),
            Self::Path(p) => resolve_library(Some(p.as_os_str()), None),
        }
    }
}

// ----- loaded SDK ----------------------------------------------------------

/// The loaded library with its system context.
struct Sdk {
    api: Api,
    system: NonNull<CSystem>,
    /// Serialises the calls on `system` (listing, connect, disconnect).
    lock: Mutex<()>,
    path: PathBuf,
    /// Keeps the code behind `api` mapped; dropped after `Drop::drop`.
    _lib: Library,
}

// SAFETY: `api` holds plain function pointers and `_lib` is a loaded library
// handle, both shareable across threads. The only state is the `CSystem`
// context, and every call on it holds `lock`.
unsafe impl Send for Sdk {}
// SAFETY: as for `Send`: shared access to `system` is serialised by `lock`.
unsafe impl Sync for Sdk {}

impl Drop for Sdk {
    fn drop(&mut self) {
        // SAFETY: `system` came from `createSystem` of this library, which is
        // still loaded (`_lib` drops after this), and no device holds it: a
        // device keeps an `Arc<Sdk>`, so the last one is gone.
        unsafe { (self.api.delete_system)(self.system.as_ptr()) };
    }
}

/// The SDK loaded by [`Sdk::shared`], kept for the life of the process
/// (vendor libraries start threads; unloading them is not safe).
static LOADED: Mutex<Option<Arc<Sdk>>> = Mutex::new(None);

impl Sdk {
    /// The SDK of `source`, loaded on first use and then shared. A failed
    /// load is not cached.
    fn shared(source: &LibrarySource) -> Result<Arc<Self>> {
        let path = source.resolve()?;
        let mut loaded = LOADED.lock().unwrap_or_else(PoisonError::into_inner);
        if let Some(sdk) = loaded.as_ref().filter(|s| s.path == path) {
            return Ok(Arc::clone(sdk));
        }
        let sdk = Arc::new(Self::load(&path)?);
        *loaded = Some(Arc::clone(&sdk));
        Ok(sdk)
    }

    /// Loads the library at `path`, resolves the API and creates the system
    /// context. Every failure is `sdk-missing`: the file is not a usable SDK.
    fn load(path: &Path) -> Result<Self> {
        let missing = |e: &dyn std::fmt::Display| {
            ScanError::SdkMissing(format!("cannot load {}: {e}", path.display()))
        };
        let lib = open_library(path).map_err(|e| missing(&e))?;
        // SAFETY: the signatures of `api!` are transcribed from the vendor
        // headers (layouts checked by the tests); the pointers live in `Sdk`
        // next to `_lib`, which is dropped last.
        let api = unsafe { Api::load(&lib) }.map_err(|e| missing(&e))?;
        // SAFETY: `createSystem` takes no argument and returns a new context
        // or null.
        let system = NonNull::new(unsafe { (api.create_system)() })
            .ok_or_else(|| missing(&"createSystem returned no context"))?;
        Ok(Self {
            api,
            system,
            lock: Mutex::new(()),
            path: path.to_path_buf(),
            _lib: lib,
        })
    }

    fn lock(&self) -> MutexGuard<'_, ()> {
        self.lock.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Snapshot of the connected cameras.
    fn cameras(&self) -> Vec<DeviceInfo> {
        let _guard = self.lock();
        self.with_camera_list(|list| list.iter().map(device_info).collect())
    }

    /// Runs `f` on the SDK camera list, then frees the list. Holds no lock:
    /// the caller holds `self.lock`.
    fn with_camera_list<T>(&self, f: impl FnOnce(&mut [CameraInfo]) -> T) -> T {
        let mut count: c_int = 0;
        // SAFETY: `system` is valid; `count` is a valid out pointer.
        let list = unsafe { (self.api.create_camera_info_list)(self.system.as_ptr(), &mut count) };
        let len = usize::try_from(count).unwrap_or(0);
        if list.is_null() || len == 0 {
            if !list.is_null() {
                // SAFETY: `list` came from `systemCreateCameraInfoList`.
                unsafe { (self.api.delete_camera_info_list)(list) };
            }
            return f(&mut []);
        }
        // SAFETY: the SDK returned `count` contiguous `CameraInfo` at `list`
        // (layout pinned by the tests), alive until the delete below.
        let slice = unsafe { std::slice::from_raw_parts_mut(list, len) };
        let out = f(slice);
        // SAFETY: `list` came from `systemCreateCameraInfoList` and the slice
        // borrowed from it is gone.
        unsafe { (self.api.delete_camera_info_list)(list) };
        out
    }

    /// Connects the camera listed with id `device_id`.
    fn connect(&self, device_id: &str) -> Result<(NonNull<CCamera>, DeviceInfo)> {
        let _guard = self.lock();
        self.with_camera_list(|list| {
            let entry = list
                .iter_mut()
                .find(|c| device_info(c).id == device_id)
                .ok_or_else(|| ScanError::DeviceNotFound(device_id.to_owned()))?;
            let info = device_info(entry);
            // SAFETY: `system` is valid and `entry` points into the live list
            // returned by the SDK, as `systemConnectCamera` expects.
            let camera = unsafe { (self.api.connect_camera)(self.system.as_ptr(), entry) };
            let camera = NonNull::new(camera).ok_or_else(|| {
                ScanError::Device(format!(
                    "systemConnectCamera failed for {} ({device_id}): the device may be in use \
                     by another program (e.g. Revo Scan)",
                    info.name
                ))
            })?;
            Ok((camera, info))
        })
    }

    fn disconnect(&self, camera: NonNull<CCamera>) {
        let _guard = self.lock();
        // SAFETY: `camera` came from `systemConnectCamera` on this system and
        // is disconnected once (the device drops it right after).
        unsafe { (self.api.disconnect_camera)(self.system.as_ptr(), camera.as_ptr()) };
    }
}

#[cfg(windows)]
fn open_library(path: &Path) -> std::result::Result<Library, libloading::Error> {
    use libloading::os::windows::{LOAD_WITH_ALTERED_SEARCH_PATH, Library as WinLibrary};
    let path = std::path::absolute(path).unwrap_or_else(|_| path.to_path_buf());
    // SAFETY: loading runs the library's initialisers; the file is the
    // vendor SDK chosen by the user. The altered search path lets Windows
    // find the SDK's own DLLs next to it.
    unsafe { WinLibrary::load_with_flags(&path, LOAD_WITH_ALTERED_SEARCH_PATH) }.map(Into::into)
}

#[cfg(not(windows))]
fn open_library(path: &Path) -> std::result::Result<Library, libloading::Error> {
    // SAFETY: loading runs the library's initialisers; the file is the
    // vendor SDK chosen by the user.
    unsafe { Library::new(path) }
}

// ----- conversions ---------------------------------------------------------

/// A NUL-terminated (or full) C char array as text.
fn c_text(chars: &[c_char]) -> String {
    let bytes: Vec<u8> = chars
        .iter()
        .map(|&c| u8::from_ne_bytes(c.to_ne_bytes()))
        .take_while(|&b| b != 0)
        .collect();
    String::from_utf8_lossy(&bytes).trim().to_owned()
}

/// The listed identity of an SDK camera: id is the serial number (the
/// unique id when the serial is empty).
fn device_info(c: &CameraInfo) -> DeviceInfo {
    let serial = c_text(&c.serial);
    let name = c_text(&c.name);
    DeviceInfo {
        backend: BACKEND_ID.to_owned(),
        id: if serial.is_empty() {
            c_text(&c.unique_id)
        } else {
            serial.clone()
        },
        name: if name.is_empty() {
            "Revopoint".to_owned()
        } else {
            format!("Revopoint {name}")
        },
        serial: (!serial.is_empty()).then_some(serial),
    }
}

/// Name of a vendor `ERROR_CODE`.
fn error_name(code: c_int) -> &'static str {
    usize::try_from(code)
        .ok()
        .and_then(|i| ERROR_NAMES.get(i))
        .copied()
        .unwrap_or("unknown")
}

/// A failed SDK call as a device error naming the vendor code.
fn vendor_error(call: &str, code: c_int) -> ScanError {
    ScanError::Device(format!(
        "{call} failed: vendor error {code} ({})",
        error_name(code)
    ))
}

/// Whether the depth of a frame of this format can be read.
fn has_depth(format: c_int) -> bool {
    matches!(
        format,
        STREAM_FORMAT_Z16 | STREAM_FORMAT_Z16Y8Y8 | STREAM_FORMAT_Z16Y8Y8P
    )
}

/// The depth profile to stream: plain `Z16` before the combined formats,
/// then the highest frame rate, then the largest image.
fn pick_profile(profiles: &[StreamInfo]) -> Option<StreamInfo> {
    profiles
        .iter()
        .filter(|p| has_depth(p.format) && p.width > 0 && p.height > 0)
        .max_by(|a, b| {
            let area = |p: &StreamInfo| i64::from(p.width) * i64::from(p.height);
            (a.format == STREAM_FORMAT_Z16)
                .cmp(&(b.format == STREAM_FORMAT_Z16))
                .then(a.fps.total_cmp(&b.fps))
                .then(area(a).cmp(&area(b)))
        })
        .copied()
}

/// The SDK intrinsics (at the calibration resolution) scaled to an image of
/// `width x height`.
fn scale_intrinsics(intr: &Intrinsics, width: u32, height: u32) -> CameraIntrinsics {
    let sx = if intr.width > 0 {
        width as f32 / f32::from(intr.width)
    } else {
        1.0
    };
    let sy = if intr.height > 0 {
        height as f32 / f32::from(intr.height)
    } else {
        1.0
    };
    CameraIntrinsics {
        width,
        height,
        fx: intr.fx * sx,
        fy: intr.fy * sy,
        cx: intr.cx * sx,
        cy: intr.cy * sy,
    }
}

/// Raw `Z16` pixels (native-endian `u16`, row-major) to millimetres; `0`
/// stays `0` (invalid).
fn depth_to_mm(raw: &[u8], scale_mm: f32) -> Vec<f32> {
    raw.as_chunks::<2>()
        .0
        .iter()
        .map(|&b| f32::from(u16::from_ne_bytes(b)) * scale_mm)
        .collect()
}

/// The exposure value to set for `us` microseconds given the property
/// range. A negative minimum means the UVC log2-seconds scale the SDK uses
/// on Windows (e.g. `-12..2`).
fn exposure_value(us: u32, range: Option<(f32, f32)>) -> f32 {
    match range {
        Some((min, max)) if min < 0.0 => ((us.max(1) as f32) * 1e-6).log2().round().clamp(min, max),
        Some((min, max)) if min <= max => (us as f32).clamp(min, max),
        _ => us as f32,
    }
}

// ----- backend -------------------------------------------------------------

/// The Revopoint SDK backend (devices listed by the loaded `3DCamera` SDK).
#[derive(Debug, Clone)]
pub struct RevopointSdkBackend {
    source: LibrarySource,
}

impl Default for RevopointSdkBackend {
    fn default() -> Self {
        Self {
            source: LibrarySource::Default,
        }
    }
}

impl RevopointSdkBackend {
    /// A backend that loads the SDK library from `path` (the library file or
    /// its directory) instead of [`SDK_ENV`] / [`default_sdk_dir`].
    pub fn with_library(path: impl Into<PathBuf>) -> Self {
        Self {
            source: LibrarySource::Path(path.into()),
        }
    }

    /// The library file this backend would load. Err `sdk-missing` when it
    /// does not exist.
    pub fn library_path(&self) -> Result<PathBuf> {
        self.source.resolve()
    }
}

impl ScannerBackend for RevopointSdkBackend {
    fn id(&self) -> &'static str {
        BACKEND_ID
    }

    fn availability(&self) -> Result<()> {
        Sdk::shared(&self.source).map(drop)
    }

    fn list(&self) -> Result<Vec<DeviceInfo>> {
        Ok(Sdk::shared(&self.source).map_or_else(|_| Vec::new(), |sdk| sdk.cameras()))
    }

    fn open(&self, device_id: &str) -> Result<Box<dyn ScannerDevice>> {
        let sdk = Sdk::shared(&self.source)?;
        let (camera, info) = sdk.connect(device_id)?;
        Ok(Box::new(RevopointDevice {
            sdk,
            camera,
            info,
            stream: None,
            exposure_us: None,
        }))
    }
}

// ----- device --------------------------------------------------------------

/// A running depth stream.
struct Stream {
    handle: NonNull<CStream>,
    /// Depth intrinsics at the calibration resolution.
    intrinsics: Intrinsics,
    /// Millimetres per raw depth unit.
    depth_scale: f32,
}

/// A Revopoint camera connected through the SDK.
pub struct RevopointDevice {
    sdk: Arc<Sdk>,
    camera: NonNull<CCamera>,
    info: DeviceInfo,
    stream: Option<Stream>,
    exposure_us: Option<u32>,
}

// SAFETY: the camera and stream handles are used only through `&mut self`
// (the type is not `Sync`), so one thread at a time calls the SDK on them;
// the SDK does not bind them to the thread that created them. The shared
// `Sdk` is `Send + Sync`.
unsafe impl Send for RevopointDevice {}

impl std::fmt::Debug for RevopointDevice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RevopointDevice")
            .field("info", &self.info)
            .field("streaming", &self.stream.is_some())
            .finish_non_exhaustive()
    }
}

impl RevopointDevice {
    fn api(&self) -> &Api {
        &self.sdk.api
    }

    fn camera(&self) -> *const CCamera {
        self.camera.as_ptr()
    }

    /// The depth profiles offered by the camera.
    fn depth_profiles(&self) -> Vec<StreamInfo> {
        let mut count: c_int = 0;
        // SAFETY: `camera` is connected; `count` is a valid out pointer.
        let list = unsafe {
            (self.api().create_stream_info_list)(self.camera(), STREAM_TYPE_DEPTH, &mut count)
        };
        if list.is_null() {
            return Vec::new();
        }
        let len = usize::try_from(count).unwrap_or(0);
        // SAFETY: the SDK returned `count` contiguous `StreamInfo` at `list`
        // (layout pinned by the tests); they are copied before the delete.
        let profiles = unsafe { std::slice::from_raw_parts(list, len) }.to_vec();
        // SAFETY: `list` came from `cameraCreateStreamInfoList`.
        unsafe { (self.api().delete_stream_info_list)(list) };
        profiles
    }

    fn depth_scale(&self) -> Result<f32> {
        let mut value = PropertyExtension { raw: [0; 1032] };
        // SAFETY: `camera` is connected; `value` is a writable union of the
        // header's size and alignment.
        let code = unsafe {
            (self.api().get_property_extension)(self.camera(), PROPERTY_EXT_DEPTH_SCALE, &mut value)
        };
        if code != SUCCESS {
            return Err(vendor_error(
                "cameraGetPropertyExtension(DEPTH_SCALE)",
                code,
            ));
        }
        // SAFETY: every bit pattern is a valid `f32`; the SDK filled the
        // `depthScale` member for `PROPERTY_EXT_DEPTH_SCALE`.
        let scale = unsafe { value.depth_scale };
        if scale.is_finite() && scale > 0.0 {
            Ok(scale)
        } else {
            Err(ScanError::Device(format!(
                "the SDK reports depth scale {scale}"
            )))
        }
    }

    fn depth_intrinsics(&self) -> Result<Intrinsics> {
        let mut intr = Intrinsics::default();
        // SAFETY: `camera` is connected; `intr` is a writable `Intrinsics`.
        let code = unsafe {
            (self.api().get_stream_intrinsics)(self.camera(), STREAM_TYPE_DEPTH, &mut intr)
        };
        if code != SUCCESS {
            return Err(vendor_error("cameraGetStreamIntrinsics(DEPTH)", code));
        }
        if intr.fx > 0.0 && intr.fy > 0.0 {
            Ok(intr)
        } else {
            Err(ScanError::Device(format!(
                "the SDK reports invalid depth intrinsics fx {} fy {}",
                intr.fx, intr.fy
            )))
        }
    }

    fn apply_exposure(&self, us: u32) -> Result<()> {
        let (mut min, mut max, mut step) = (0.0_f32, 0.0_f32, 0.0_f32);
        // SAFETY: `camera` is connected; the three out pointers are valid.
        let range_code = unsafe {
            (self.api().get_property_range)(
                self.camera(),
                STREAM_TYPE_DEPTH,
                PROPERTY_EXPOSURE,
                &mut min,
                &mut max,
                &mut step,
            )
        };
        let range = (range_code == SUCCESS).then_some((min, max));
        let value = exposure_value(us, range);
        // SAFETY: `camera` is connected; the value is passed by value.
        let code = unsafe {
            (self.api().set_property)(self.camera(), STREAM_TYPE_DEPTH, PROPERTY_EXPOSURE, value)
        };
        if code == SUCCESS {
            Ok(())
        } else {
            Err(vendor_error("cameraSetProperty(EXPOSURE)", code))
        }
    }

    /// Starts the depth stream and reads its scale and intrinsics.
    fn start_stream(&mut self) -> Result<()> {
        let profiles = self.depth_profiles();
        let profile = pick_profile(&profiles).ok_or_else(|| {
            ScanError::Device(format!(
                "the camera offers no Z16 depth profile ({} profiles)",
                profiles.len()
            ))
        })?;
        // SAFETY: `camera` is connected; `profile` is one of its profiles,
        // passed by value; no callback, so frames are pulled with
        // `cameraGetFrame`.
        let handle = unsafe {
            (self.api().start_stream)(
                self.camera(),
                STREAM_TYPE_DEPTH,
                profile,
                None,
                std::ptr::null_mut(),
            )
        };
        let handle = NonNull::new(handle).ok_or_else(|| {
            ScanError::SdkUnauthorized(format!(
                "the SDK refused to start the depth stream of {} (serial {}): the device is \
                 probably not authorised for SDK use",
                self.info.name, self.info.id
            ))
        })?;
        // Owned from here: `stop` releases it on every error below.
        self.stream = Some(Stream {
            handle,
            intrinsics: Intrinsics::default(),
            depth_scale: 1.0,
        });
        let setup = (|| {
            let intrinsics = self.depth_intrinsics()?;
            let depth_scale = self.depth_scale()?;
            if let Some(us) = self.exposure_us {
                self.apply_exposure(us)?;
            }
            Ok((intrinsics, depth_scale))
        })();
        match setup {
            Ok((intrinsics, depth_scale)) => {
                if let Some(s) = self.stream.as_mut() {
                    s.intrinsics = intrinsics;
                    s.depth_scale = depth_scale;
                }
                Ok(())
            }
            Err(e) => {
                self.stop();
                Err(e)
            }
        }
    }
}

/// A frame borrowed from the SDK, released on drop.
struct FrameGuard<'a> {
    api: &'a Api,
    stream: NonNull<CStream>,
    frame: NonNull<CFrame>,
}

impl Drop for FrameGuard<'_> {
    fn drop(&mut self) {
        // SAFETY: `frame` came from `cameraGetFrame` on `stream`, which is
        // still running, and is released once; its data was copied.
        unsafe { (self.api.release_frame)(self.stream.as_ptr(), self.frame.as_ptr()) };
    }
}

impl FrameGuard<'_> {
    /// Copies the frame into a [`DepthFrame`].
    fn to_depth_frame(&self, stream: &Stream) -> Result<DepthFrame> {
        let f = self.frame.as_ptr().cast_const();
        // SAFETY: `f` is a live frame of this SDK (released only by `drop`).
        let (format, width, height, size, timestamp_ms) = unsafe {
            (
                (self.api.frame_format)(f),
                (self.api.frame_width)(f),
                (self.api.frame_height)(f),
                (self.api.frame_data_size)(f),
                (self.api.frame_timestamp)(f),
            )
        };
        let (Ok(w), Ok(h)) = (u32::try_from(width), u32::try_from(height)) else {
            return Err(ScanError::Device(format!(
                "frame of {width}x{height} pixels"
            )));
        };
        if !has_depth(format) {
            return Err(ScanError::Device(format!(
                "frame format {format:#x} has no depth"
            )));
        }
        let pixels = w as usize * h as usize;
        let bytes = pixels * 2;
        if usize::try_from(size).unwrap_or(0) < bytes {
            return Err(ScanError::Device(format!(
                "depth frame {w}x{h} with {size} bytes"
            )));
        }
        // SAFETY: `f` is live; the format selects the accessor of the depth
        // part (whole buffer for Z16, the Z16 plane of combined formats).
        let data = unsafe {
            if format == STREAM_FORMAT_Z16 {
                (self.api.frame_data)(f)
            } else {
                (self.api.frame_data_by_format)(f, FRAME_DATA_FORMAT_Z16)
            }
        };
        if data.is_null() {
            return Err(ScanError::Device("frame without depth data".into()));
        }
        // SAFETY: the depth plane holds `w * h` `u16` values (checked against
        // the frame size) and stays valid until the frame is released, after
        // this copy. Read as bytes: no alignment requirement.
        let raw = unsafe { std::slice::from_raw_parts(data.cast::<u8>(), bytes) };
        Ok(DepthFrame {
            depth_mm: depth_to_mm(raw, stream.depth_scale),
            intrinsics: scale_intrinsics(&stream.intrinsics, w, h),
            timestamp_us: (timestamp_ms * 1000.0).max(0.0) as u64,
            color: None,
            imu_orientation: None,
        })
    }
}

impl ScannerDevice for RevopointDevice {
    fn info(&self) -> DeviceInfo {
        self.info.clone()
    }

    fn start(&mut self) -> Result<()> {
        if self.stream.is_some() {
            return Ok(());
        }
        self.start_stream()
    }

    fn stop(&mut self) {
        if let Some(stream) = self.stream.take() {
            // SAFETY: `handle` came from `cameraStartStream` and is stopped
            // once; no frame of it is held (guards end inside `next_frame`).
            unsafe { (self.api().stop_stream)(stream.handle.as_ptr()) };
        }
    }

    fn next_frame(&mut self, timeout: Duration) -> Result<Option<DepthFrame>> {
        let stream = self
            .stream
            .as_ref()
            .ok_or_else(|| ScanError::Device("the depth stream is not started".into()))?;
        let timeout_ms = c_int::try_from(timeout.as_millis()).unwrap_or(c_int::MAX);
        let mut frame: *mut CFrame = std::ptr::null_mut();
        // SAFETY: `handle` is a running stream; `frame` is a valid out
        // pointer (the C++ reference parameter).
        let code =
            unsafe { (self.sdk.api.get_frame)(stream.handle.as_ptr(), &mut frame, timeout_ms) };
        match code {
            SUCCESS => {}
            ERROR_FRAME_TIMEOUT => return Ok(None),
            ERROR_NOT_SUPPORT => {
                return Err(ScanError::SdkUnauthorized(format!(
                    "cameraGetFrame: vendor error {code} ({}); the device is probably not \
                     authorised for SDK use",
                    error_name(code)
                )));
            }
            code => return Err(vendor_error("cameraGetFrame", code)),
        }
        let Some(frame) = NonNull::new(frame) else {
            return Ok(None);
        };
        let guard = FrameGuard {
            api: &self.sdk.api,
            stream: stream.handle,
            frame,
        };
        guard.to_depth_frame(stream).map(Some)
    }

    fn set_exposure_us(&mut self, us: u32) -> Result<()> {
        self.exposure_us = Some(us);
        if self.stream.is_some() {
            self.apply_exposure(us)?;
        }
        Ok(())
    }
}

impl Drop for RevopointDevice {
    fn drop(&mut self) {
        self.stop();
        self.sdk.disconnect(self.camera);
    }
}

#[cfg(test)]
mod tests {
    use std::mem::{align_of, offset_of, size_of};

    use super::*;

    /// Sizes and offsets of `hpp/Types.hpp` (SDK v3.2.229), as compiled by a
    /// C compiler for x86-64 (the layout is the same on Windows x64).
    #[test]
    fn layouts_match_vendor_headers() {
        assert_eq!(size_of::<c_int>(), 4);
        assert_eq!(size_of::<StreamInfo>(), 16);
        assert_eq!(align_of::<StreamInfo>(), 4);
        assert_eq!(offset_of!(StreamInfo, width), 4);
        assert_eq!(offset_of!(StreamInfo, height), 8);
        assert_eq!(offset_of!(StreamInfo, fps), 12);

        assert_eq!(size_of::<CameraInfo>(), 160);
        assert_eq!(offset_of!(CameraInfo, serial), 32);
        assert_eq!(offset_of!(CameraInfo, unique_id), 64);
        assert_eq!(offset_of!(CameraInfo, firmware_version), 96);
        assert_eq!(offset_of!(CameraInfo, algorithm_version), 128);

        assert_eq!(size_of::<Intrinsics>(), 40);
        assert_eq!(offset_of!(Intrinsics, height), 2);
        assert_eq!(offset_of!(Intrinsics, fx), 4);
        assert_eq!(offset_of!(Intrinsics, cx), 12);
        assert_eq!(offset_of!(Intrinsics, fy), 20);
        assert_eq!(offset_of!(Intrinsics, cy), 24);
        assert_eq!(offset_of!(Intrinsics, one22), 36);

        assert_eq!(size_of::<PropertyExtension>(), 1032);
        assert_eq!(align_of::<PropertyExtension>(), 8);
        assert_eq!(offset_of!(PropertyExtension, depth_scale), 0);
    }

    #[test]
    fn missing_library_is_sdk_missing() {
        let dir = tempfile::tempdir().unwrap();
        let backend = RevopointSdkBackend::with_library(dir.path().join("nope").join(LIBRARY_NAME));
        assert_eq!(backend.availability().unwrap_err().code(), "sdk-missing");
        assert!(backend.list().unwrap().is_empty());
        assert_eq!(backend.open("x").err().unwrap().code(), "sdk-missing");

        // A directory without the library, and a file that is no library.
        let empty = RevopointSdkBackend::with_library(dir.path());
        assert_eq!(empty.availability().unwrap_err().code(), "sdk-missing");
        let fake = dir.path().join(LIBRARY_NAME);
        std::fs::write(&fake, b"not a shared library").unwrap();
        let err = RevopointSdkBackend::with_library(&fake)
            .availability()
            .unwrap_err();
        assert_eq!(err.code(), "sdk-missing");
        assert!(err.to_string().contains("cannot load"), "{err}");
    }

    #[test]
    fn library_path_resolution() {
        let dir = tempfile::tempdir().unwrap();
        let lib = dir.path().join(LIBRARY_NAME);
        // Nothing installed yet.
        let err = resolve_library(None, Some(dir.path())).unwrap_err();
        assert_eq!(err.code(), "sdk-missing");
        assert!(err.to_string().contains(LIBRARY_NAME), "{err}");
        assert_eq!(
            resolve_library(None, None).unwrap_err().code(),
            "sdk-missing"
        );
        std::fs::write(&lib, b"").unwrap();
        // Per-user directory.
        assert_eq!(resolve_library(None, Some(dir.path())).unwrap(), lib);
        // Environment: the directory, the file, empty (= unset).
        let other = tempfile::tempdir().unwrap();
        assert_eq!(
            resolve_library(Some(dir.path().as_os_str()), Some(other.path())).unwrap(),
            lib
        );
        assert_eq!(
            resolve_library(Some(lib.as_os_str()), Some(other.path())).unwrap(),
            lib
        );
        assert_eq!(
            resolve_library(Some(OsStr::new("")), Some(dir.path())).unwrap(),
            lib
        );
        // The environment wins over the per-user directory.
        assert_eq!(
            resolve_library(Some(other.path().as_os_str()), Some(dir.path()))
                .unwrap_err()
                .code(),
            "sdk-missing"
        );
        if let Some(d) = default_sdk_dir() {
            assert!(d.ends_with(Path::new("dentalcraft").join("revopoint-sdk")));
            assert!(d.is_absolute());
        }
    }

    #[test]
    fn camera_info_to_device_info() {
        fn chars(s: &str) -> [c_char; 32] {
            let mut out: [c_char; 32] = [0; 32];
            for (o, b) in out.iter_mut().zip(s.bytes()) {
                *o = c_char::from_ne_bytes([b]);
            }
            out
        }
        let mut c = CameraInfo {
            name: chars("MINI2"),
            serial: chars("ABCDEFGHIJKLMNOPQRSTUVWXYZ012345"), // 32 chars, no NUL
            unique_id: chars("usb-1"),
            firmware_version: chars("1.0"),
            algorithm_version: chars("2.0"),
        };
        let d = device_info(&c);
        assert_eq!(d.backend, BACKEND_ID);
        assert_eq!(d.id, "ABCDEFGHIJKLMNOPQRSTUVWXYZ012345");
        assert_eq!(d.serial.as_deref(), Some(d.id.as_str()));
        assert_eq!(d.name, "Revopoint MINI2");
        c.serial = [0; 32];
        let d = device_info(&c);
        assert_eq!((d.id.as_str(), d.serial), ("usb-1", None));
    }

    #[test]
    fn profile_choice_prefers_z16_then_fps_then_size() {
        let p = |format, width, height, fps| StreamInfo {
            format,
            width,
            height,
            fps,
        };
        let profiles = [
            p(STREAM_FORMAT_Z16, 640, 400, 10.0),
            p(STREAM_FORMAT_Z16, 320, 200, 18.0),
            p(STREAM_FORMAT_Z16, 640, 400, 18.0),
            p(STREAM_FORMAT_Z16Y8Y8, 640, 400, 30.0),
            p(0x04, 1280, 800, 60.0), // PAIR: infrared only
        ];
        assert_eq!(pick_profile(&profiles), Some(profiles[2]));
        assert_eq!(pick_profile(&profiles[3..]), Some(profiles[3]));
        assert_eq!(pick_profile(&profiles[4..]), None);
    }

    #[test]
    fn depth_and_intrinsics_conversion() {
        let raw: Vec<u8> = [0_u16, 1, 1000, u16::MAX]
            .iter()
            .flat_map(|v| v.to_ne_bytes())
            .collect();
        assert_eq!(depth_to_mm(&raw, 0.1), [0.0, 0.1, 100.0, 6553.5]);

        let intr = Intrinsics {
            width: 1280,
            height: 800,
            fx: 1000.0,
            cx: 640.0,
            fy: 1002.0,
            cy: 400.0,
            one22: 1.0,
            ..Intrinsics::default()
        };
        let k = scale_intrinsics(&intr, 640, 400);
        assert_eq!((k.width, k.height), (640, 400));
        assert_eq!((k.fx, k.fy, k.cx, k.cy), (500.0, 501.0, 320.0, 200.0));
    }

    #[test]
    fn exposure_and_error_names() {
        assert_eq!(exposure_value(10_000, Some((3000.0, 60_000.0))), 10_000.0);
        assert_eq!(exposure_value(100_000, Some((3000.0, 60_000.0))), 60_000.0);
        assert_eq!(exposure_value(1000, None), 1000.0);
        // log2(0.0156 s) = -6
        assert_eq!(exposure_value(15_625, Some((-12.0, 2.0))), -6.0);

        assert_eq!(error_name(ERROR_FRAME_TIMEOUT), "ERROR_FRAME_TIMEOUT");
        assert_eq!(error_name(18), "ERROR_UNKNOW");
        assert_eq!(error_name(-1), "unknown");
        let e = vendor_error("cameraGetFrame", 4);
        assert_eq!(e.code(), "device");
        assert!(
            e.to_string().contains("vendor error 4 (ERROR_DEVICE_BUSY)"),
            "{e}"
        );
    }
}
