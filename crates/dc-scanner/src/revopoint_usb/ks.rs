//! The vendor extension unit through kernel streaming (Windows): the
//! transport of [`super::xu`] on the Media Foundation video source of
//! [`super::mf`].
//!
//! The inbox UVC driver (`usbvideo`) exposes each extension unit as a node
//! of type `KSNODETYPE_DEV_SPECIFIC` in the topology of the video source,
//! with a property set whose GUID is the unit's GUID and whose property ids
//! are the control selectors (Microsoft, "Extension Unit Plug-In
//! Architecture" and "Device Requirements for USB Video Class Extension
//! Units": the controls 1 to n before the first gap of `bmControls`; the
//! MINI 2 has 1 to 8, so the selectors 1, 2 and 7 of [`super::xu`]).
//!
//! The unit is looked up when the source is opened: the first such node
//! whose control answers the set-support query of [`UNIT_GUID`], through
//! the node's own `IKsControl` (`IKsTopologyInfo::CreateNodeInstance`) or,
//! for drivers that serve node properties on the filter only, the source's.
//! The query is answered by the driver from the descriptors it read at
//! start-up: nothing reaches the device. A source without that node gets
//! no request at all.
//!
//! Each request of [`super::xu`] is then one `IKsControl::KsProperty` with a
//! `KSP_NODE`: `Set` = [`UNIT_GUID`], `Id` = the selector, `Flags` =
//! [`XuRequest::ks_flags`], `NodeId` = that node, and
//!
//! * `SET_CUR`: the payload's 60 bytes;
//! * `GET_CUR`: the 60-byte reply buffer (a shorter reply is an error);
//! * `GET_LEN`: no buffer: the driver answers `ERROR_MORE_DATA` with the
//!   size of the control, as in Microsoft's extension-unit sample;
//! * `GET_INFO`: the basic-support query into a `KSPROPERTY_DESCRIPTION`
//!   (or, for a driver that only answers the whole description, a buffer of
//!   its size): its access flags hold `KSPROPERTY_TYPE_GET` and
//!   `KSPROPERTY_TYPE_SET`, the bits of `GET_INFO` at the same positions
//!   (the driver reads `GET_INFO` at start-up to know which requests a
//!   control takes).

#![allow(unsafe_code)]
#![warn(clippy::undocumented_unsafe_blocks)]

use std::ffi::c_void;
use std::ptr::null_mut;

use windows::Win32::Media::KernelStreaming::{
    IKsControl, IKsTopologyInfo, KSIDENTIFIER, KSIDENTIFIER_0, KSIDENTIFIER_0_0,
    KSNODETYPE_DEV_SPECIFIC, KSP_NODE, KSPROPERTY_DESCRIPTION, KSPROPERTY_TYPE_BASICSUPPORT,
    KSPROPERTY_TYPE_GET, KSPROPERTY_TYPE_SET, KSPROPERTY_TYPE_SETSUPPORT, KSPROPERTY_TYPE_TOPOLOGY,
};
use windows::Win32::Media::MediaFoundation::IMFMediaSource;
use windows::core::{GUID, HRESULT, IUnknown, Interface};

use super::mf::{hr_error, win32};
use super::xu::{
    INFO_GET, INFO_SET, KS_BASICSUPPORT, KS_GET, KS_SET, KS_TOPOLOGY, PAYLOAD_LEN, Selector, UNIT,
    UNIT_GUID, XuRequest, missing_unit,
};
use crate::{Result, ScanError};

/// `ERROR_MORE_DATA`: the answer of a size query.
const MORE_DATA: u32 = 234;
/// `ERROR_INSUFFICIENT_BUFFER`: a size query answered as a too small
/// buffer.
const INSUFFICIENT_BUFFER: u32 = 122;

// The flags of the shared layer are those of `ks.h`, and the layouts are
// those of `ks.h` (checked when compiling for Windows).
const _: () = assert!(
    KS_GET == KSPROPERTY_TYPE_GET
        && KS_SET == KSPROPERTY_TYPE_SET
        && KS_BASICSUPPORT == KSPROPERTY_TYPE_BASICSUPPORT
        && KS_TOPOLOGY == KSPROPERTY_TYPE_TOPOLOGY
);
const _: () = assert!(size_of::<KSP_NODE>() == 32 && size_of::<KSPROPERTY_DESCRIPTION>() == 40);

/// The vendor extension unit of an open video source: the topology node
/// that serves its property set, and the control that reaches it.
pub(super) struct VendorUnit {
    control: IKsControl,
    node: u32,
}

impl VendorUnit {
    /// Looks the unit up in the topology of `source` (see the module doc).
    ///
    /// # Errors
    /// Why there is none, for the notes of the backend.
    pub(super) fn find(source: &IMFMediaSource) -> std::result::Result<Self, String> {
        let topology: IKsTopologyInfo = source
            .cast()
            .map_err(|e| format!("the video source has no topology (IKsTopologyInfo: {e})"))?;
        // SAFETY: plain COM call on a live interface.
        let nodes = unsafe { topology.NumNodes() }
            .map_err(|e| format!("IKsTopologyInfo::NumNodes: {e}"))?;
        // the source's own control, for drivers that serve node properties
        // on the filter only
        let filter: Option<IKsControl> = source.cast().ok();
        for node in 0..nodes {
            // SAFETY: plain COM call on a live interface, `node` < `nodes`.
            if unsafe { topology.get_NodeType(node) }.ok() != Some(KSNODETYPE_DEV_SPECIFIC) {
                continue;
            }
            for control in [node_control(&topology, node), filter.clone()]
                .into_iter()
                .flatten()
            {
                if serves_unit(&control, node) {
                    return Ok(Self { control, node });
                }
            }
        }
        Err(missing_unit(&format!(
            "the {nodes} topology nodes of the video source"
        )))
    }

    /// Runs `request` on `selector` (see the module doc).
    pub(super) fn query(&self, selector: Selector, request: XuRequest<'_>) -> Result<()> {
        let property = node_property(u32::from(selector.id()), request.ks_flags(), self.node);
        let id = selector.id();
        match request {
            XuRequest::SetCur(payload) => {
                // `KsProperty` takes a mutable buffer; a set only reads it
                let mut bytes = *payload.as_bytes();
                // SAFETY: `bytes` is a live buffer of its length for the call.
                let (result, _) = unsafe {
                    ks_property(
                        &self.control,
                        &property,
                        bytes.as_mut_ptr().cast(),
                        bytes.len() as u32,
                    )
                };
                result.map_err(|e| hr_error("KsProperty (SET_CUR)", e.code()))
            }
            XuRequest::GetCur(reply) => {
                // SAFETY: `reply` is a live buffer of its length for the call.
                let (result, returned) = unsafe {
                    ks_property(
                        &self.control,
                        &property,
                        reply.as_mut_ptr().cast(),
                        reply.len() as u32,
                    )
                };
                result.map_err(|e| hr_error("KsProperty (GET_CUR)", e.code()))?;
                if returned as usize != PAYLOAD_LEN {
                    return Err(ScanError::Device(format!(
                        "extension unit {UNIT} selector {id}: GET_CUR returned {returned} bytes, \
                         not {PAYLOAD_LEN}"
                    )));
                }
                Ok(())
            }
            XuRequest::GetLen(len) => {
                // SAFETY: no buffer (a size query).
                let (result, returned) =
                    unsafe { ks_property(&self.control, &property, null_mut(), 0) };
                match result {
                    Ok(()) => {}
                    Err(e) if is_size_answer(e.code()) => {}
                    Err(e) => return Err(hr_error("KsProperty (GET_LEN)", e.code())),
                }
                *len = u16::try_from(returned)
                    .ok()
                    .filter(|n| *n > 0)
                    .ok_or_else(|| {
                        ScanError::Device(format!(
                            "extension unit {UNIT} selector {id}: GET_LEN answered {returned} bytes"
                        ))
                    })?;
                Ok(())
            }
            XuRequest::GetInfo(info) => {
                let header = size_of::<KSPROPERTY_DESCRIPTION>();
                let mut description = KSPROPERTY_DESCRIPTION::default();
                // SAFETY: `description` is a live value of the size passed.
                let (result, returned) = unsafe {
                    ks_property(
                        &self.control,
                        &property,
                        (&raw mut description).cast(),
                        header as u32,
                    )
                };
                let access_flags = match result {
                    Ok(()) if returned as usize >= size_of::<u32>() => description.AccessFlags,
                    // a driver that answers the whole description only:
                    // asked again with a buffer of its size (8-aligned)
                    Err(e) if is_size_answer(e.code()) && returned as usize >= header => {
                        let mut whole = vec![0u64; (returned as usize).div_ceil(8)];
                        // SAFETY: `whole` is a live buffer of at least
                        // `returned` bytes for the call.
                        let (result, n) = unsafe {
                            ks_property(
                                &self.control,
                                &property,
                                whole.as_mut_ptr().cast(),
                                returned,
                            )
                        };
                        result.map_err(|e| hr_error("KsProperty (GET_INFO)", e.code()))?;
                        if (n as usize) < size_of::<u32>() {
                            return Err(ScanError::Device(format!(
                                "extension unit {UNIT} selector {id}: no access flags ({n} bytes)"
                            )));
                        }
                        // the first member, `AccessFlags` (little-endian)
                        whole[0] as u32
                    }
                    Ok(()) => {
                        return Err(ScanError::Device(format!(
                            "extension unit {UNIT} selector {id}: no access flags ({returned} bytes)"
                        )));
                    }
                    Err(e) => return Err(hr_error("KsProperty (GET_INFO)", e.code())),
                };
                *info = info_bits(access_flags);
                Ok(())
            }
        }
    }
}

/// `true` for the answer of a property query whose buffer is too small (or
/// absent): the size needed comes with it.
fn is_size_answer(hr: HRESULT) -> bool {
    hr == win32(MORE_DATA) || hr == win32(INSUFFICIENT_BUFFER)
}

/// The `GET_INFO` bits of the access flags of a basic-support reply.
fn info_bits(access_flags: u32) -> u8 {
    let mut info = 0;
    if access_flags & KS_GET != 0 {
        info |= INFO_GET;
    }
    if access_flags & KS_SET != 0 {
        info |= INFO_SET;
    }
    info
}

/// The property `id` of the unit's set on topology node `node`.
fn node_property(id: u32, flags: u32, node: u32) -> KSP_NODE {
    KSP_NODE {
        Property: KSIDENTIFIER {
            Anonymous: KSIDENTIFIER_0 {
                Anonymous: KSIDENTIFIER_0_0 {
                    Set: GUID::from_u128(UNIT_GUID),
                    Id: id,
                    Flags: flags,
                },
            },
        },
        NodeId: node,
        Reserved: 0,
    }
}

/// One `IKsControl::KsProperty` call of `property` with `len` bytes at
/// `data`; returns its outcome and the byte count the driver reported (the
/// size needed, for a size query).
///
/// # Safety
/// `data` is null with `len` 0, or valid for reads and writes of `len`
/// bytes for the duration of the call.
unsafe fn ks_property(
    control: &IKsControl,
    property: &KSP_NODE,
    data: *mut c_void,
    len: u32,
) -> (windows::core::Result<()>, u32) {
    let mut returned = 0u32;
    // SAFETY: `property` is a live `KSP_NODE` of the size passed (it starts
    // with its `KSIDENTIFIER`); `data` and `len` as the caller guarantees;
    // `returned` is a valid out pointer.
    let result = unsafe {
        control.KsProperty(
            (&raw const *property).cast::<KSIDENTIFIER>(),
            size_of::<KSP_NODE>() as u32,
            data,
            len,
            &mut returned,
        )
    };
    (result, returned)
}

/// The control of topology node `node`, when the topology creates one.
fn node_control(topology: &IKsTopologyInfo, node: u32) -> Option<IKsControl> {
    let mut raw = null_mut();
    // SAFETY: a valid interface id and out pointer; on success `raw` owns
    // one reference to the node object.
    unsafe { topology.CreateNodeInstance(node, &IUnknown::IID, &mut raw) }.ok()?;
    if raw.is_null() {
        return None;
    }
    // SAFETY: a non-null owned `IUnknown` pointer.
    let unknown = unsafe { IUnknown::from_raw(raw) };
    unknown.cast().ok()
}

/// `true` when `control` serves the unit's property set on `node`.
fn serves_unit(control: &IKsControl, node: u32) -> bool {
    let probe = node_property(
        0,
        KSPROPERTY_TYPE_SETSUPPORT | KSPROPERTY_TYPE_TOPOLOGY,
        node,
    );
    // SAFETY: no buffer (a set-support query).
    unsafe { ks_property(control, &probe, null_mut(), 0) }
        .0
        .is_ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The property of a request addresses the unit's set, the selector and
    /// the node; the access flags of a basic-support reply give the
    /// `GET_INFO` bits.
    #[test]
    fn properties_and_access_flags() {
        let p = node_property(7, KS_SET | KS_TOPOLOGY, 3);
        // SAFETY: reading the member the property was built with.
        let id = unsafe { p.Property.Anonymous.Anonymous };
        assert_eq!(id.Set, GUID::from_u128(UNIT_GUID));
        assert_eq!((id.Id, id.Flags, p.NodeId), (7, 0x1000_0002, 3));
        assert_eq!(info_bits(KS_GET | KS_SET | KS_BASICSUPPORT), 0x03);
        assert_eq!(info_bits(KS_GET), INFO_GET);
        assert_eq!(info_bits(0), 0);
        assert!(is_size_answer(win32(234)) && is_size_answer(win32(122)));
        assert!(!is_size_answer(win32(1168)));
    }
}
