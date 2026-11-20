//! Linux platform layer of the USB backend: the `2207:110c` devices from
//! `nusb`, their V4L2 nodes from sysfs, streaming with [`super::v4l2`].

use std::path::{Path, PathBuf};

use nusb::MaybeFuture;
use rustix::io::Errno;

pub(super) use super::v4l2::{Capture, PIX_FMT_Y16};
use super::{BACKEND_ID, USB_PRODUCT_ID, USB_VENDOR_ID};
use crate::{DeviceInfo, Result, ScanError};

/// USB (bus, address) of one enumeration.
pub(super) type Address = (u8, u8);

/// The depth capture node: `/dev/videoN`.
pub(super) type Node = PathBuf;

/// V4L2 is part of the kernel: the backend is always usable.
pub(super) fn availability() -> Result<()> {
    Ok(())
}

/// A listed `2207:110c` device.
pub(super) struct Found {
    pub(super) info: DeviceInfo,
    /// USB (bus, address) of this enumeration.
    pub(super) address: Address,
    /// `/sys/bus/usb/devices/<port>`.
    sysfs: PathBuf,
}

impl Found {
    /// The depth capture node, when `uvcvideo` serves interface 0.
    pub(super) fn depth_node(&self) -> Option<Node> {
        nodes(&self.sysfs).map(|n| n.depth)
    }

    /// Detail of a device whose depth node is missing.
    pub(super) fn no_node(&self) -> String {
        no_node(&self.sysfs)
    }
}

/// Whether the device of `found` has the vendor extension unit (see
/// [`super::xu`]), read from its sysfs USB descriptors (no device I/O);
/// the reason when not. `_capture` is its open depth node, the transport.
pub(super) fn vendor_unit(found: &Found, _capture: &Capture) -> std::result::Result<(), String> {
    let path = found.sysfs.join("descriptors");
    let descriptors = std::fs::read(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    if super::xu::has_unit(&descriptors) {
        Ok(())
    } else {
        Err(super::xu::missing_unit("the USB descriptors"))
    }
}

/// The `2207:110c` devices on the bus.
pub(super) fn find_devices() -> Result<Vec<Found>> {
    let devices = nusb::list_devices()
        .wait()
        .map_err(|e| ScanError::Device(format!("USB enumeration: {e}")))?;
    Ok(devices
        .filter(|d| d.vendor_id() == USB_VENDOR_ID && d.product_id() == USB_PRODUCT_ID)
        .map(|d| {
            let serial = d
                .serial_number()
                .filter(|s| !s.is_empty())
                .map(str::to_owned);
            let port = d
                .sysfs_path()
                .file_name()
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_default();
            Found {
                info: DeviceInfo {
                    backend: BACKEND_ID.to_owned(),
                    id: serial.clone().unwrap_or_else(|| format!("usb-{port}")),
                    name: super::device_name(d.device_version()).to_owned(),
                    serial,
                },
                address: (d.busnum(), d.device_address()),
                sysfs: d.sysfs_path().to_owned(),
            }
        })
        .collect())
}

/// Opens the depth node. The error says whether it may be transient: the
/// node may not exist yet, or udev may not have applied the permissions
/// yet.
pub(super) fn open_capture(node: &Node) -> std::result::Result<Capture, (bool, ScanError)> {
    Capture::open(node)
        .map(|(capture, _)| capture)
        .map_err(|(errno, e)| {
            (
                matches!(
                    errno,
                    Errno::NOENT | Errno::ACCESS | Errno::PERM | Errno::NODEV
                ),
                e,
            )
        })
}

/// The V4L2 nodes of a device.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Nodes {
    /// Depth capture node (interface 0).
    depth: PathBuf,
    /// Colour capture node (interface 2), not streamed in this alpha.
    #[allow(dead_code, reason = "resolved for the colour stream to come")]
    color: Option<PathBuf>,
}

/// The capture node of USB interface `interface` of the device at
/// `sysfs`: the `video4linux` entry with `index` 0 (the others are
/// metadata nodes).
fn capture_node(sysfs: &Path, interface: u8) -> Option<PathBuf> {
    let port = sysfs.file_name()?.to_str()?;
    let video4linux = sysfs
        .join(format!("{port}:1.{interface}"))
        .join("video4linux");
    let mut nodes: Vec<String> = std::fs::read_dir(&video4linux)
        .ok()?
        .filter_map(|e| e.ok()?.file_name().into_string().ok())
        .filter(|n| n.starts_with("video"))
        .collect();
    nodes.sort_by_key(|n| n[5..].parse::<u32>().unwrap_or(u32::MAX));
    nodes
        .iter()
        .find(|n| {
            std::fs::read_to_string(video4linux.join(n).join("index"))
                .is_ok_and(|i| i.trim() == "0")
        })
        .or(nodes.first())
        .map(|n| Path::new("/dev").join(n))
}

fn nodes(sysfs: &Path) -> Option<Nodes> {
    Some(Nodes {
        depth: capture_node(sysfs, 0)?,
        color: capture_node(sysfs, 2),
    })
}

/// Detail of a device whose depth node is missing.
fn no_node(sysfs: &Path) -> String {
    let driver = std::fs::read_link(sysfs.join(format!(
        "{}:1.0/driver",
        sysfs.file_name().unwrap_or_default().to_string_lossy()
    )))
    .ok()
    .and_then(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
    .unwrap_or_else(|| "none".into());
    format!(
        "no V4L2 depth node under {} (interface 0 driver: {driver}); another program, \
         e.g. a virtual machine USB redirection, may hold the scanner",
        sysfs.display()
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capture_node_prefers_index_zero() {
        let dir = tempfile::tempdir().unwrap();
        let sysfs = dir.path().join("1-1");
        for (iface, node, index) in [(0, "video1", "1"), (0, "video0", "0"), (2, "video3", "0")] {
            let d = sysfs.join(format!("1-1:1.{iface}/video4linux/{node}"));
            std::fs::create_dir_all(&d).unwrap();
            std::fs::write(d.join("index"), format!("{index}\n")).unwrap();
        }
        assert_eq!(
            nodes(&sysfs),
            Some(Nodes {
                depth: PathBuf::from("/dev/video0"),
                color: Some(PathBuf::from("/dev/video3")),
            })
        );
        // no depth interface bound to uvcvideo: no node, a clear detail
        let other = dir.path().join("1-2");
        std::fs::create_dir_all(other.join("1-2:1.0")).unwrap();
        assert_eq!(nodes(&other), None);
        assert!(no_node(&other).contains("driver: none"));
    }
}
