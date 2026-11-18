//! Windows platform layer of the USB backend: the `2207:110c` devices from
//! `nusb` (serial, `bcdDevice`, USB address) and the Media Foundation video
//! source of their depth interface (interface 0, served by the inbox UVC
//! driver `usbvideo`), streamed with [`super::mf`].
//!
//! A video source is a device interface of the interface device node
//! (`USB\VID_2207&PID_110C&MI_00\...`); its parent node is the USB device
//! that `nusb` lists (`USB\VID_2207&PID_110C\<serial>`). The parent is found
//! with the Configuration Manager (`cfgmgr32`), which only reads the device
//! tree.

#![allow(unsafe_code)]
#![warn(clippy::undocumented_unsafe_blocks)]

use nusb::MaybeFuture;
use windows::Win32::Devices::DeviceAndDriverInstallation::{
    CM_Get_Device_IDW, CM_Get_Parent, CM_LOCATE_DEVNODE_NORMAL, CM_Locate_DevNodeW, CR_SUCCESS,
    MAX_DEVICE_ID_LEN,
};
use windows::core::PCWSTR;

pub(super) use super::mf::{Capture, PIX_FMT_Y16, availability, open_capture};
use super::{BACKEND_ID, USB_PRODUCT_ID, USB_VENDOR_ID};
use crate::{DeviceInfo, Result, ScanError};

/// USB (bus id, address) of one enumeration.
pub(super) type Address = (String, u8);

/// The depth capture node: the symbolic link of its video source.
pub(super) type Node = String;

/// A listed `2207:110c` device.
pub(super) struct Found {
    pub(super) info: DeviceInfo,
    /// USB (bus id, address) of this enumeration.
    pub(super) address: Address,
    /// Device instance id (`USB\VID_2207&PID_110C\<serial>`).
    instance_id: String,
    /// Symbolic link of the video source of interface 0.
    depth: Option<String>,
}

impl Found {
    /// The video source of the depth interface, when there is one.
    pub(super) fn depth_node(&self) -> Option<Node> {
        self.depth.clone()
    }

    /// Detail of a device whose depth source is missing.
    pub(super) fn no_node(&self) -> String {
        format!(
            "no Media Foundation video source for interface 0 of {}; the camera driver \
             (usbvideo) may be missing or disabled, or another program, e.g. a virtual \
             machine USB redirection, may hold the scanner",
            self.instance_id
        )
    }
}

/// The `2207:110c` devices on the bus, with the video source of their
/// depth interface.
pub(super) fn find_devices() -> Result<Vec<Found>> {
    let devices: Vec<nusb::DeviceInfo> = nusb::list_devices()
        .wait()
        .map_err(|e| ScanError::Device(format!("USB enumeration: {e}")))?
        .filter(|d| d.vendor_id() == USB_VENDOR_ID && d.product_id() == USB_PRODUCT_ID)
        .collect();
    if devices.is_empty() {
        return Ok(Vec::new());
    }
    // (parent instance id, symbolic link) of the depth sources
    let sources: Vec<(String, String)> = super::mf::video_sources()?
        .into_iter()
        .filter_map(|link| {
            let interface = depth_interface_instance_id(&link)?;
            Some((parent_instance_id(&interface)?, link))
        })
        .collect();
    Ok(devices
        .into_iter()
        .map(|d| {
            let serial = d
                .serial_number()
                .filter(|s| !s.is_empty())
                .map(str::to_owned);
            let instance_id = d.instance_id().to_string_lossy().into_owned();
            let depth = sources
                .iter()
                .find(|(parent, _)| parent.eq_ignore_ascii_case(&instance_id))
                .map(|(_, link)| link.clone());
            let port = d
                .port_chain()
                .iter()
                .map(u8::to_string)
                .collect::<Vec<_>>()
                .join(".");
            Found {
                info: DeviceInfo {
                    backend: BACKEND_ID.to_owned(),
                    id: serial
                        .clone()
                        .unwrap_or_else(|| format!("usb-{}-{port}", d.bus_id())),
                    name: super::device_name(d.device_version()).to_owned(),
                    serial,
                },
                address: (d.bus_id().to_owned(), d.device_address()),
                instance_id,
                depth,
            }
        })
        .collect())
}

/// Device instance id of the interface behind a video source symbolic
/// link, when it is interface 0 of a `2207:110c` device:
/// `\\?\usb#vid_2207&pid_110c&mi_00#7&1bc16135&0&0000#{guid}\global` gives
/// `USB\VID_2207&PID_110C&MI_00\7&1BC16135&0&0000`.
fn depth_interface_instance_id(link: &str) -> Option<String> {
    let mut parts = link.strip_prefix(r"\\?\")?.split('#');
    let (bus, hardware, instance) = (parts.next()?, parts.next()?, parts.next()?);
    let hardware = hardware.to_ascii_uppercase();
    let wanted = format!("VID_{USB_VENDOR_ID:04X}&PID_{USB_PRODUCT_ID:04X}&MI_00");
    (bus.eq_ignore_ascii_case("usb") && hardware == wanted)
        .then(|| format!("USB\\{hardware}\\{}", instance.to_ascii_uppercase()))
}

/// Instance id of the parent device node of `instance_id`.
fn parent_instance_id(instance_id: &str) -> Option<String> {
    let wide: Vec<u16> = instance_id.encode_utf16().chain([0]).collect();
    let mut node = 0u32;
    // SAFETY: `wide` is a NUL-terminated UTF-16 string alive for the call;
    // `node` is a valid out pointer.
    let located =
        unsafe { CM_Locate_DevNodeW(&mut node, PCWSTR(wide.as_ptr()), CM_LOCATE_DEVNODE_NORMAL) };
    if located != CR_SUCCESS {
        return None;
    }
    let mut parent = 0u32;
    // SAFETY: `parent` is a valid out pointer, `node` a located node.
    if unsafe { CM_Get_Parent(&mut parent, node, 0) } != CR_SUCCESS {
        return None;
    }
    let mut id = [0u16; MAX_DEVICE_ID_LEN as usize + 1];
    // SAFETY: the buffer holds the longest id and its NUL.
    if unsafe { CM_Get_Device_IDW(parent, &mut id, 0) } != CR_SUCCESS {
        return None;
    }
    let len = id.iter().position(|&c| c == 0).unwrap_or(id.len());
    Some(String::from_utf16_lossy(&id[..len]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interface_instance_id_from_link() {
        assert_eq!(
            depth_interface_instance_id(
                r"\\?\usb#vid_2207&pid_110c&mi_00#7&1bc16135&0&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global"
            )
            .as_deref(),
            Some(r"USB\VID_2207&PID_110C&MI_00\7&1BC16135&0&0000")
        );
        // the colour interface, another camera, a non-USB source
        for link in [
            r"\\?\usb#vid_2207&pid_110c&mi_02#7&1bc16135&0&0002#{e5323777-f976-4f5b-9b55-b94699c46e44}\global",
            r"\\?\usb#vid_046d&pid_0825&mi_00#7&abc&0&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global",
            r"\\?\root#camera#0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global",
            "",
        ] {
            assert_eq!(depth_interface_instance_id(link), None, "{link}");
        }
    }
}
