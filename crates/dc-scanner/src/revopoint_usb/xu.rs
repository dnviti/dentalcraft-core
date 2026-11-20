//! The vendor extension unit of the MINI 2: the closed list of requests the
//! backend may send, shared by every platform.
//!
//! **User decisions, 2026-11-18 21:36 and 21:43.** The scanner runs shell
//! lines and serves its files over its UVC extension unit [`UNIT`] (GUID
//! [`UNIT_GUID`]) of the video-control interface [`CONTROL_INTERFACE`]. The
//! user authorised exactly:
//!
//! * the fill light lines ([`super::light::FillLight`]) and the fast exposure
//!   lines ([`super::exposure::ExposureProfile`]), each one `SET_CUR` of
//!   [`PAYLOAD_LEN`] bytes on [`Selector::Shell`]: the ASCII line, NUL padded,
//!   as the vendor SDK sends them (`cs::Camera::ledAndLaserCtrl`, reverse
//!   engineered on the POP4 by BR1ZB3AR/All3DScan, `RE_NOTES.md`).
//!   [`ShellLine`] is that list; no other text can be built;
//! * the read-only file channel for the factory calibration
//!   ([`super::factory`]): [`read_file`], with the files of the closed list
//!   [`FactoryFile`] and the "open for reading" opcode only;
//! * the read-only standard requests `GET_INFO` and `GET_LEN` on those
//!   selectors, for the checks below.
//!
//! Before anything else the backend checks that the device has the unit
//! with that GUID (Linux and raw USB: the USB configuration descriptor,
//! [`has_unit`]; Windows: the node of the video source topology whose
//! property set is the GUID), then, before each batch, that the selector
//! takes the request (`GET_INFO`, [`INFO_GET`] / [`INFO_SET`]) and is
//! [`PAYLOAD_LEN`] bytes long (`GET_LEN`). A device that differs gets nothing
//! else.
//!
//! # Transports
//!
//! The requests are built here only. A platform carries them to the device
//! with an [`XuTransport`] and adds none of its own:
//!
//! * Linux: the `UVCIOC_CTRL_QUERY` ioctl of `uvcvideo` (`v4l2`): unit
//!   [`UNIT`], the selector, `query` = [`XuRequest::uvc_code`];
//! * Windows: the kernel-streaming properties of the inbox `usbvideo` driver
//!   on the Media Foundation source (`ks`): a `KSP_NODE` with `Set` =
//!   [`UNIT_GUID`], `Id` = the selector, `Flags` = [`XuRequest::ks_flags`],
//!   `NodeId` = the topology node of the unit;
//! * raw USB (a transport with no UVC driver in between, e.g. macOS): the
//!   class request on endpoint 0 of [`XuRequest::setup`].
//!
//! A `SET_CUR` payload is a [`Payload`], which only this module builds, so no
//! transport can send other bytes: every transport carries the same ones.

use super::factory::FactoryFile;
use crate::{Result, ScanError};

/// Id of the vendor extension unit (`bUnitID`).
pub const UNIT: u8 = 4;
/// GUID of the unit, `{c2211a6d-787e-3c4a-ac06-7e434b676f9a}`.
pub const UNIT_GUID: u128 = 0xc221_1a6d_787e_3c4a_ac06_7e43_4b67_6f9a;
/// [`UNIT_GUID`] in USB descriptor byte order: the first three fields
/// little-endian (the memory layout of a Windows `GUID`).
pub const UNIT_GUID_BYTES: [u8; 16] = guid_bytes(UNIT_GUID);
/// The video-control interface that holds the unit (the depth camera).
pub const CONTROL_INTERFACE: u8 = 0;
/// Bytes of every request of the selectors the backend uses.
pub const PAYLOAD_LEN: usize = 60;

/// [`UNIT_GUID`] as text: `c2211a6d-787e-3c4a-ac06-7e434b676f9a`.
pub fn unit_guid_text() -> String {
    let g = UNIT_GUID;
    format!(
        "{:08x}-{:04x}-{:04x}-{:04x}-{:012x}",
        g >> 96,
        (g >> 80) & 0xffff,
        (g >> 64) & 0xffff,
        (g >> 48) & 0xffff,
        g & 0xffff_ffff_ffff
    )
}

/// Why a device gets no request: no unit [`UNIT`] with [`UNIT_GUID`] in
/// `place` (where the platform looked for it).
pub fn missing_unit(place: &str) -> String {
    format!(
        "no extension unit {UNIT} with GUID {{{}}} in {place}",
        unit_guid_text()
    )
}

/// `GET_INFO` bit: the control takes `GET_CUR` (UVC 1.5, 4.1.2).
pub const INFO_GET: u8 = 0x01;
/// `GET_INFO` bit: the control takes `SET_CUR`.
pub const INFO_SET: u8 = 0x02;

/// `KSPROPERTY_TYPE_GET` (`ks.h`).
pub const KS_GET: u32 = 0x0000_0001;
/// `KSPROPERTY_TYPE_SET`.
pub const KS_SET: u32 = 0x0000_0002;
/// `KSPROPERTY_TYPE_BASICSUPPORT`: its reply starts with the access flags
/// ([`KS_GET`], [`KS_SET`]), the bits of `GET_INFO` at the same positions.
pub const KS_BASICSUPPORT: u32 = 0x0000_0200;
/// `KSPROPERTY_TYPE_TOPOLOGY`: the property of a topology node.
pub const KS_TOPOLOGY: u32 = 0x1000_0000;

/// Opcode "open for reading" of [`Selector::FileOpen`], the only one sent.
const OPEN_FOR_READ: u8 = 0x01;
/// Largest data of one file chunk (the payload less its length word).
const MAX_CHUNK: usize = PAYLOAD_LEN - 4;
/// Largest file accepted (the calibration files are a few KiB).
const MAX_FILE_BYTES: usize = 64 * 1024;

/// `guid` in USB descriptor byte order.
const fn guid_bytes(guid: u128) -> [u8; 16] {
    let b = guid.to_be_bytes();
    [
        b[3], b[2], b[1], b[0], b[5], b[4], b[7], b[6], b[8], b[9], b[10], b[11], b[12], b[13],
        b[14], b[15],
    ]
}

/// The selectors of the unit the backend uses: a closed list.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Selector {
    /// 1: opens (and closes) a file of the device for reading.
    FileOpen = 1,
    /// 2: returns the next chunk of the open file.
    FileRead = 2,
    /// 7: runs one shell line on the scanner's own Linux.
    Shell = 7,
}

impl Selector {
    /// The UVC control selector (the `KSPROPERTY` id on Windows).
    pub fn id(self) -> u8 {
        self as u8
    }

    /// The `GET_INFO` bit of the request the backend sends on it:
    /// `SET_CUR` on the open and shell selectors, `GET_CUR` on the read one.
    fn needs(self) -> u8 {
        match self {
            Self::FileOpen | Self::Shell => INFO_SET,
            Self::FileRead => INFO_GET,
        }
    }
}

/// The shell lines the user authorised, the only text [`Selector::Shell`]
/// receives (see [`super::light`] and [`super::exposure`] for what they do
/// and how they were measured).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShellLine {
    /// `echo s 0xb03 1 >/dev/rk_preisp`: first line of the fill light on.
    B03On,
    /// `echo s 0xb02 1 >/dev/rk_preisp`: second line of the fill light on.
    B02On,
    /// `echo s 0xb03 0 >/dev/rk_preisp`: the fill light off.
    B03Off,
    /// `echo s 0x912 0 >/dev/rk_preisp`: the device's auto exposure off.
    AutoExposureOff,
    /// `echo s 0x911 6000 >/dev/rk_preisp`: exposure 6000 µs.
    Exposure6000Us,
    /// `echo s 0x910 8000 >/dev/rk_preisp`: pattern time 8000 µs.
    FrameTime8000Us,
}

impl ShellLine {
    /// The line, as the scanner receives it.
    pub fn text(self) -> &'static str {
        match self {
            Self::B03On => "echo s 0xb03 1 >/dev/rk_preisp",
            Self::B02On => "echo s 0xb02 1 >/dev/rk_preisp",
            Self::B03Off => "echo s 0xb03 0 >/dev/rk_preisp",
            Self::AutoExposureOff => "echo s 0x912 0 >/dev/rk_preisp",
            Self::Exposure6000Us => "echo s 0x911 6000 >/dev/rk_preisp",
            Self::FrameTime8000Us => "echo s 0x910 8000 >/dev/rk_preisp",
        }
    }
}

/// A `SET_CUR` payload of the closed list. Only this module builds one, from
/// a [`ShellLine`] or a [`FactoryFile`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Payload([u8; PAYLOAD_LEN]);

impl Payload {
    /// Closes the open file: all zero (it ends the read session and writes
    /// nothing).
    const CLOSE: Self = Self([0; PAYLOAD_LEN]);

    /// `line`: ASCII, NUL padded (every line is shorter than the payload, so
    /// it is NUL terminated).
    fn shell(line: ShellLine) -> Self {
        let text = line.text().as_bytes();
        let mut p = [0; PAYLOAD_LEN];
        p[..text.len()].copy_from_slice(text);
        Self(p)
    }

    /// Opens `file` for reading: `[OPEN_FOR_READ, path, NUL]`, NUL padded.
    fn open(file: FactoryFile) -> Self {
        let path = file.device_path().as_bytes();
        let mut p = [0; PAYLOAD_LEN];
        p[0] = OPEN_FOR_READ;
        p[1..=path.len()].copy_from_slice(path);
        Self(p)
    }

    /// The bytes sent.
    pub fn as_bytes(&self) -> &[u8; PAYLOAD_LEN] {
        &self.0
    }
}

/// One request on a selector of the unit, as a transport carries it.
#[derive(Debug)]
pub enum XuRequest<'a> {
    /// `SET_CUR` of a payload of the closed list.
    SetCur(&'a Payload),
    /// `GET_CUR`: the reply fills the buffer.
    GetCur(&'a mut [u8; PAYLOAD_LEN]),
    /// `GET_LEN`: the length of the selector in bytes (2 bytes,
    /// little-endian, on the wire).
    GetLen(&'a mut u16),
    /// `GET_INFO`: the capability bits of the selector ([`INFO_GET`],
    /// [`INFO_SET`], ...).
    GetInfo(&'a mut u8),
}

/// The USB control transfer of one request on endpoint 0 (UVC 1.5, 4.2.1):
/// a class request to the video-control interface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlSetup {
    /// `bmRequestType`: 0x21 (host to device) for `SET_CUR`, 0xA1 for the
    /// GETs.
    pub request_type: u8,
    /// `bRequest`: [`XuRequest::uvc_code`].
    pub request: u8,
    /// `wValue`: the selector in the high byte.
    pub value: u16,
    /// `wIndex`: [`UNIT`] in the high byte, [`CONTROL_INTERFACE`] in the
    /// low byte.
    pub index: u16,
    /// `wLength`: [`XuRequest::length`].
    pub length: u16,
}

impl XuRequest<'_> {
    /// The UVC `bRequest` code: `SET_CUR` 0x01, `GET_CUR` 0x81, `GET_LEN`
    /// 0x85, `GET_INFO` 0x86 (also the `query` of `UVCIOC_CTRL_QUERY`).
    pub fn uvc_code(&self) -> u8 {
        match self {
            Self::SetCur(_) => 0x01,
            Self::GetCur(_) => 0x81,
            Self::GetLen(_) => 0x85,
            Self::GetInfo(_) => 0x86,
        }
    }

    /// Bytes of the request's data on the wire: the payload, 2 for
    /// `GET_LEN`, 1 for `GET_INFO`.
    pub fn length(&self) -> u16 {
        match self {
            Self::SetCur(_) | Self::GetCur(_) => PAYLOAD_LEN as u16,
            Self::GetLen(_) => 2,
            Self::GetInfo(_) => 1,
        }
    }

    /// The endpoint 0 control transfer of this request on `selector`, for a
    /// transport that sends the UVC class requests itself.
    pub fn setup(&self, selector: Selector) -> ControlSetup {
        ControlSetup {
            request_type: match self {
                Self::SetCur(_) => 0x21,
                _ => 0xa1,
            },
            request: self.uvc_code(),
            value: u16::from(selector.id()) << 8,
            index: (u16::from(UNIT) << 8) | u16::from(CONTROL_INTERFACE),
            length: self.length(),
        }
    }

    /// The `KSPROPERTY` flags of this request on the Windows UVC driver:
    /// `SET_CUR` is a set and `GET_CUR` a get of the property; `GET_LEN` is a
    /// get with no buffer (the driver answers the size); `GET_INFO` is the
    /// basic-support query (its access flags), all on the topology node.
    pub fn ks_flags(&self) -> u32 {
        KS_TOPOLOGY
            | match self {
                Self::SetCur(_) => KS_SET,
                Self::GetCur(_) | Self::GetLen(_) => KS_GET,
                Self::GetInfo(_) => KS_BASICSUPPORT,
            }
    }
}

/// Carries the requests of this module to the unit of one open device: one
/// implementation per platform (see the module doc). It sends exactly the
/// request it is given, nothing before or after, to the unit [`UNIT`] (GUID
/// [`UNIT_GUID`]) of the interface [`CONTROL_INTERFACE`].
pub trait XuTransport {
    /// Runs `request` on `selector`; the reply of a GET fills its buffer.
    ///
    /// # Errors
    /// The failure of the platform call: `device-lost` when the device left
    /// the bus, [`ScanError::Device`] otherwise (no such unit or selector, a
    /// refused or short request).
    fn query(&self, selector: Selector, request: XuRequest<'_>) -> Result<()>;
}

/// Checks that `selector` takes the request the backend sends on it
/// (`GET_INFO`) and is [`PAYLOAD_LEN`] bytes long (`GET_LEN`); `what` names
/// the command in the error.
fn check(t: &impl XuTransport, selector: Selector, what: &str) -> Result<()> {
    let id = selector.id();
    let mut info = 0;
    t.query(selector, XuRequest::GetInfo(&mut info))?;
    let needs = selector.needs();
    if info & needs != needs {
        let request = if needs == INFO_SET {
            "SET_CUR"
        } else {
            "GET_CUR"
        };
        return Err(ScanError::Device(format!(
            "extension unit {UNIT} selector {id} does not take {request} (GET_INFO \
             {info:#04x}): no {what} sent"
        )));
    }
    let mut len = 0;
    t.query(selector, XuRequest::GetLen(&mut len))?;
    if usize::from(len) != PAYLOAD_LEN {
        return Err(ScanError::Device(format!(
            "extension unit {UNIT} selector {id} is {len} bytes, not {PAYLOAD_LEN}: no {what} sent"
        )));
    }
    Ok(())
}

/// Runs `lines` on [`Selector::Shell`], in order, after its checks (`what`
/// names them in the errors).
///
/// # Errors
/// [`ScanError::Device`] when the selector does not take `SET_CUR`, has
/// another length or a request fails (`device-lost` when the device left).
pub fn send_lines(t: &impl XuTransport, lines: &[ShellLine], what: &str) -> Result<()> {
    check(t, Selector::Shell, what)?;
    for &line in lines {
        t.query(Selector::Shell, XuRequest::SetCur(&Payload::shell(line)))?;
    }
    Ok(())
}

/// Reads the factory file `file` over the read-only file channel: open for
/// reading, chunks until one of length 0, close (also after a failed read).
/// Call it with the stream idle.
///
/// A chunk is `[len as u32 LE (<= 56), data]`.
///
/// # Errors
/// [`ScanError::Device`] when a selector fails its checks, a request fails,
/// a reply is malformed or the file is empty.
pub fn read_file(t: &impl XuTransport, file: FactoryFile) -> Result<Vec<u8>> {
    check(t, Selector::FileOpen, "file read")?;
    check(t, Selector::FileRead, "file read")?;
    t.query(Selector::FileOpen, XuRequest::SetCur(&Payload::open(file)))?;
    let mut data = Vec::new();
    let read = (|| -> Result<()> {
        let mut reply = [0; PAYLOAD_LEN];
        loop {
            t.query(Selector::FileRead, XuRequest::GetCur(&mut reply))?;
            if !append_chunk(&reply, &mut data)? {
                return Ok(());
            }
        }
    })();
    // the session is closed whatever happened after it was opened
    let closed = t.query(Selector::FileOpen, XuRequest::SetCur(&Payload::CLOSE));
    read.and(closed)?;
    if data.is_empty() {
        return Err(ScanError::Device(format!(
            "{}: empty or missing on the device",
            file.device_path()
        )));
    }
    Ok(data)
}

/// Appends the data of one file chunk to `out`; `false` at the end of the
/// file (length 0).
///
/// # Errors
/// [`ScanError::Device`] for a length above [`MAX_CHUNK`] or a file larger
/// than [`MAX_FILE_BYTES`].
fn append_chunk(reply: &[u8; PAYLOAD_LEN], out: &mut Vec<u8>) -> Result<bool> {
    let len = u32::from_le_bytes([reply[0], reply[1], reply[2], reply[3]]) as usize;
    if len == 0 {
        return Ok(false);
    }
    if len > MAX_CHUNK {
        return Err(ScanError::Device(format!(
            "factory file read: chunk of {len} bytes (at most {MAX_CHUNK})"
        )));
    }
    if out.len() + len > MAX_FILE_BYTES {
        return Err(ScanError::Device(format!(
            "factory file read: more than {MAX_FILE_BYTES} bytes"
        )));
    }
    out.extend_from_slice(&reply[4..4 + len]);
    Ok(true)
}

/// `true` when the raw USB descriptors `descriptors` (a configuration
/// descriptor with its class-specific descriptors, or the sysfs
/// `descriptors` file of Linux) hold a video-control extension unit with id
/// [`UNIT`] and GUID [`UNIT_GUID`].
pub fn has_unit(descriptors: &[u8]) -> bool {
    // class-specific interface descriptor (0x24), VC_EXTENSION_UNIT (0x06):
    // bLength, bDescriptorType, bDescriptorSubtype, bUnitID, guid[16], ...
    let mut i = 0;
    while i + 2 <= descriptors.len() {
        let len = descriptors[i] as usize;
        if len < 2 || i + len > descriptors.len() {
            return false;
        }
        let d = &descriptors[i..i + len];
        if len >= 20 && d[1] == 0x24 && d[2] == 0x06 && d[3] == UNIT && d[4..20] == UNIT_GUID_BYTES
        {
            return true;
        }
        i += len;
    }
    false
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;

    use super::*;
    use crate::revopoint_usb::exposure::ExposureProfile;
    use crate::revopoint_usb::light::FillLight;

    /// What a fake transport saw: selector, UVC request code, the bytes of a
    /// `SET_CUR`.
    type Seen = (u8, u8, Vec<u8>);

    /// A device that records every request: selectors with capability bits
    /// `info` and `len` bytes long, file chunks served from `chunks`, a
    /// failure on the request number `fail_at`.
    struct Fake {
        info: u8,
        len: u16,
        chunks: RefCell<Vec<Vec<u8>>>,
        fail_at: Option<usize>,
        seen: RefCell<Vec<Seen>>,
    }

    impl Fake {
        fn new(len: u16) -> Self {
            Self {
                info: INFO_GET | INFO_SET,
                len,
                chunks: RefCell::default(),
                fail_at: None,
                seen: RefCell::default(),
            }
        }
    }

    impl XuTransport for Fake {
        fn query(&self, selector: Selector, request: XuRequest<'_>) -> Result<()> {
            let n = self.seen.borrow().len();
            let code = request.uvc_code();
            let bytes = match request {
                XuRequest::SetCur(p) => p.as_bytes().to_vec(),
                XuRequest::GetCur(reply) => {
                    let next = self.chunks.borrow_mut().pop().unwrap_or_default();
                    *reply = [0; PAYLOAD_LEN];
                    reply[..4].copy_from_slice(&(next.len() as u32).to_le_bytes());
                    reply[4..4 + next.len()].copy_from_slice(&next);
                    Vec::new()
                }
                XuRequest::GetLen(len) => {
                    *len = self.len;
                    Vec::new()
                }
                XuRequest::GetInfo(info) => {
                    *info = self.info;
                    Vec::new()
                }
            };
            self.seen.borrow_mut().push((selector.id(), code, bytes));
            if self.fail_at == Some(n) {
                return Err(ScanError::Device("refused".into()));
            }
            Ok(())
        }
    }

    /// `text` NUL padded to a payload.
    fn padded(text: &[u8]) -> Vec<u8> {
        let mut p = text.to_vec();
        p.resize(PAYLOAD_LEN, 0);
        p
    }

    const SET_CUR: u8 = 0x01;
    const GET_CUR: u8 = 0x81;
    const GET_LEN: u8 = 0x85;
    const GET_INFO: u8 = 0x86;

    /// The two checks of `selector`, as the fake sees them.
    fn checks(selector: u8) -> [Seen; 2] {
        [
            (selector, GET_INFO, Vec::new()),
            (selector, GET_LEN, Vec::new()),
        ]
    }

    #[test]
    fn unit_identity_and_request_codes() {
        // the GUID as the USB descriptor and a Windows GUID store it
        assert_eq!(
            UNIT_GUID_BYTES,
            [
                0x6d, 0x1a, 0x21, 0xc2, 0x7e, 0x78, 0x4a, 0x3c, 0xac, 0x06, 0x7e, 0x43, 0x4b, 0x67,
                0x6f, 0x9a
            ]
        );
        assert_eq!((UNIT, CONTROL_INTERFACE, PAYLOAD_LEN), (4, 0, 60));
        assert_eq!(unit_guid_text(), "c2211a6d-787e-3c4a-ac06-7e434b676f9a");
        assert_eq!(
            missing_unit("the USB descriptors"),
            "no extension unit 4 with GUID {c2211a6d-787e-3c4a-ac06-7e434b676f9a} in the USB \
             descriptors"
        );
        let ids = [Selector::FileOpen, Selector::FileRead, Selector::Shell].map(Selector::id);
        assert_eq!(ids, [1, 2, 7]);
        let (p, mut reply, mut len, mut info) = (Payload::CLOSE, [0; PAYLOAD_LEN], 0, 0);
        assert_eq!(XuRequest::SetCur(&p).uvc_code(), SET_CUR);
        assert_eq!(XuRequest::GetCur(&mut reply).uvc_code(), GET_CUR);
        assert_eq!(XuRequest::GetLen(&mut len).uvc_code(), GET_LEN);
        assert_eq!(XuRequest::GetInfo(&mut info).uvc_code(), GET_INFO);
    }

    /// The endpoint 0 transfers of a raw USB transport (UVC 1.5, 4.2.1) and
    /// the kernel-streaming flags of the Windows one (`ks.h`), for every
    /// kind of request.
    #[test]
    fn raw_and_kernel_streaming_forms_of_each_request() {
        let (p, mut reply, mut len, mut info) = (Payload::CLOSE, [0; PAYLOAD_LEN], 0, 0);
        let setup = |request_type, request, value, length| ControlSetup {
            request_type,
            request,
            value,
            index: 0x0400,
            length,
        };
        let shell = XuRequest::SetCur(&p);
        assert_eq!(shell.setup(Selector::Shell), setup(0x21, 0x01, 0x0700, 60));
        assert_eq!(shell.ks_flags(), 0x1000_0002);
        let read = XuRequest::GetCur(&mut reply);
        assert_eq!(
            read.setup(Selector::FileRead),
            setup(0xa1, 0x81, 0x0200, 60)
        );
        assert_eq!(read.ks_flags(), 0x1000_0001);
        let length = XuRequest::GetLen(&mut len);
        assert_eq!(
            length.setup(Selector::FileOpen),
            setup(0xa1, 0x85, 0x0100, 2)
        );
        assert_eq!(length.ks_flags(), 0x1000_0001);
        let capabilities = XuRequest::GetInfo(&mut info);
        assert_eq!(
            capabilities.setup(Selector::Shell),
            setup(0xa1, 0x86, 0x0700, 1)
        );
        assert_eq!(capabilities.ks_flags(), 0x1000_0200);
        // the access flags of the basic-support reply are the GET_INFO bits
        assert_eq!((KS_GET, KS_SET), (u32::from(INFO_GET), u32::from(INFO_SET)));
    }

    #[test]
    fn every_line_is_an_authorised_text_nul_terminated() {
        let lines = [
            (ShellLine::B03On, "echo s 0xb03 1 >/dev/rk_preisp"),
            (ShellLine::B02On, "echo s 0xb02 1 >/dev/rk_preisp"),
            (ShellLine::B03Off, "echo s 0xb03 0 >/dev/rk_preisp"),
            (ShellLine::AutoExposureOff, "echo s 0x912 0 >/dev/rk_preisp"),
            (
                ShellLine::Exposure6000Us,
                "echo s 0x911 6000 >/dev/rk_preisp",
            ),
            (
                ShellLine::FrameTime8000Us,
                "echo s 0x910 8000 >/dev/rk_preisp",
            ),
        ];
        for (line, text) in lines {
            assert_eq!(line.text(), text);
            assert!(text.is_ascii() && text.len() < PAYLOAD_LEN, "{text}");
            assert_eq!(
                Payload::shell(line).as_bytes().to_vec(),
                padded(text.as_bytes())
            );
        }
    }

    #[test]
    fn the_light_and_exposure_lines_are_the_exact_requests() {
        for (lines, texts) in [
            (
                FillLight::On.lines(),
                &[
                    "echo s 0xb03 1 >/dev/rk_preisp",
                    "echo s 0xb02 1 >/dev/rk_preisp",
                ][..],
            ),
            (
                FillLight::Off.lines(),
                &["echo s 0xb03 0 >/dev/rk_preisp"][..],
            ),
            (
                ExposureProfile::Fast.lines(),
                &[
                    "echo s 0x912 0 >/dev/rk_preisp",
                    "echo s 0x911 6000 >/dev/rk_preisp",
                    "echo s 0x910 8000 >/dev/rk_preisp",
                ][..],
            ),
        ] {
            let t = Fake::new(60);
            send_lines(&t, lines, "test").unwrap();
            let mut expected = checks(7).to_vec();
            expected.extend(texts.iter().map(|l| (7, SET_CUR, padded(l.as_bytes()))));
            assert_eq!(*t.seen.borrow(), expected);
        }
    }

    /// A `UVCIOC_CTRL_QUERY` of Linux: unit, selector, query, size, data.
    type LinuxQuery = (u8, u8, u8, u16, Vec<u8>);
    /// A `KSP_NODE` property of Windows: set, id, flags, data.
    type KsQuery = (u128, u32, u32, Vec<u8>);

    /// A recording transport of each platform built from the same request:
    /// the ioctl of Linux, the endpoint 0 transfer of a raw transport, the
    /// kernel-streaming property of Windows; all three carry the payload's
    /// own bytes.
    #[derive(Default)]
    struct Wire {
        linux: RefCell<Vec<LinuxQuery>>,
        raw: RefCell<Vec<(ControlSetup, Vec<u8>)>>,
        ks: RefCell<Vec<KsQuery>>,
    }

    impl XuTransport for Wire {
        fn query(&self, selector: Selector, request: XuRequest<'_>) -> Result<()> {
            let data = match &request {
                XuRequest::SetCur(p) => p.as_bytes().to_vec(),
                _ => Vec::new(),
            };
            let (code, len) = (request.uvc_code(), request.length());
            self.linux
                .borrow_mut()
                .push((UNIT, selector.id(), code, len, data.clone()));
            self.raw
                .borrow_mut()
                .push((request.setup(selector), data.clone()));
            self.ks.borrow_mut().push((
                UNIT_GUID,
                u32::from(selector.id()),
                request.ks_flags(),
                data,
            ));
            match request {
                XuRequest::GetInfo(info) => *info = INFO_GET | INFO_SET,
                XuRequest::GetLen(len) => *len = PAYLOAD_LEN as u16,
                // an empty chunk: the end of the file at once
                XuRequest::GetCur(reply) => *reply = [0; PAYLOAD_LEN],
                XuRequest::SetCur(_) => {}
            }
            Ok(())
        }
    }

    #[test]
    fn every_transport_carries_the_same_bytes() {
        let wire = Wire::default();
        send_lines(&wire, FillLight::On.lines(), "light command").unwrap();
        send_lines(&wire, FillLight::Off.lines(), "light command").unwrap();
        send_lines(&wire, ExposureProfile::Fast.lines(), "exposure command").unwrap();
        // the device serves an empty file here: an error after the close
        assert!(read_file(&wire, FactoryFile::Camparam).is_err());
        let (linux, raw, ks) = (wire.linux.take(), wire.raw.take(), wire.ks.take());
        // 2 checks + 2 lines, 2 + 1, 2 + 3, then 4 checks, open, read, close
        assert_eq!((linux.len(), raw.len(), ks.len()), (19, 19, 19));
        let sets: Vec<&[u8]> = linux
            .iter()
            .filter(|q| q.2 == SET_CUR)
            .map(|q| &q.4[..])
            .collect();
        let mut open = vec![0x01];
        open.extend_from_slice(b"/data/camparam/camparam.yaml");
        let texts: [&[u8]; 8] = [
            b"echo s 0xb03 1 >/dev/rk_preisp",
            b"echo s 0xb02 1 >/dev/rk_preisp",
            b"echo s 0xb03 0 >/dev/rk_preisp",
            b"echo s 0x912 0 >/dev/rk_preisp",
            b"echo s 0x911 6000 >/dev/rk_preisp",
            b"echo s 0x910 8000 >/dev/rk_preisp",
            &open,
            &[],
        ];
        let expected: Vec<Vec<u8>> = texts.iter().map(|t| padded(t)).collect();
        assert_eq!(sets, expected.iter().map(Vec::as_slice).collect::<Vec<_>>());
        for ((l, r), k) in linux.iter().zip(&raw).zip(&ks) {
            // same selector, same request, same data on every platform
            assert_eq!(l.0, UNIT);
            assert_eq!(r.0.value, u16::from(l.1) << 8);
            assert_eq!(r.0.index, 0x0400);
            assert_eq!(k.1, u32::from(l.1));
            assert_eq!(k.0, UNIT_GUID);
            assert_eq!(r.0.request, l.2);
            assert_eq!(r.0.length, l.3);
            assert_eq!((&l.4, &r.1), (&k.3, &k.3));
            let is_set = l.2 == SET_CUR;
            assert_eq!(r.0.request_type == 0x21, is_set);
            assert_eq!(k.2 & KS_SET != 0, is_set);
            assert_ne!(k.2 & KS_TOPOLOGY, 0);
        }
    }

    #[test]
    fn a_selector_that_differs_gets_nothing_else() {
        // another length
        let t = Fake::new(64);
        let e = send_lines(&t, FillLight::On.lines(), "light command").unwrap_err();
        assert_eq!(e.code(), "device");
        assert!(e.to_string().contains("64 bytes, not 60"), "{e}");
        assert_eq!(*t.seen.borrow(), checks(7));
        let e = read_file(&t, FactoryFile::Pl).unwrap_err();
        assert!(e.to_string().contains("no file read sent"), "{e}");
        assert_eq!(t.seen.borrow().len(), 4);
        // no SET_CUR on the shell selector: not even its length is asked
        let mut t = Fake::new(60);
        t.info = INFO_GET;
        let e = send_lines(&t, FillLight::Off.lines(), "light command").unwrap_err();
        assert!(
            e.to_string()
                .contains("does not take SET_CUR (GET_INFO 0x01): no light command sent"),
            "{e}"
        );
        assert_eq!(*t.seen.borrow(), [(7, GET_INFO, Vec::new())]);
        // no GET_CUR on the read selector: the file is never opened
        t.info = INFO_SET;
        t.seen.borrow_mut().clear();
        let e = read_file(&t, FactoryFile::Pl).unwrap_err();
        assert!(e.to_string().contains("does not take GET_CUR"), "{e}");
        let mut expected = checks(1).to_vec();
        expected.push((2, GET_INFO, Vec::new()));
        assert_eq!(*t.seen.borrow(), expected);
    }

    #[test]
    fn a_file_is_opened_read_in_chunks_and_closed() {
        let t = Fake::new(60);
        // served last first: 56 + 3 bytes, then the end
        *t.chunks.borrow_mut() = vec![Vec::new(), b"xyz".to_vec(), vec![7; 56]];
        let data = read_file(&t, FactoryFile::Camparam).unwrap();
        let mut file = vec![7; 56];
        file.extend_from_slice(b"xyz");
        assert_eq!(data, file);
        let mut open = vec![0x01];
        open.extend_from_slice(b"/data/camparam/camparam.yaml");
        let mut expected = checks(1).to_vec();
        expected.extend(checks(2));
        expected.extend([
            (1, SET_CUR, padded(&open)),
            (2, GET_CUR, Vec::new()),
            (2, GET_CUR, Vec::new()),
            (2, GET_CUR, Vec::new()),
            (1, SET_CUR, vec![0; PAYLOAD_LEN]),
        ]);
        assert_eq!(*t.seen.borrow(), expected);
        assert_eq!(
            Payload::open(FactoryFile::Pl).as_bytes()[..23],
            *b"\x01/data/camparam/Pl.bin\0"
        );
    }

    #[test]
    fn a_failed_read_still_closes_the_file() {
        let mut t = Fake::new(60);
        // the first GET_CUR, after the four checks and the open
        t.fail_at = Some(5);
        assert_eq!(read_file(&t, FactoryFile::Pl).unwrap_err().code(), "device");
        let seen = t.seen.borrow();
        assert_eq!(seen.len(), 7);
        assert_eq!(seen[6], (1, SET_CUR, vec![0; PAYLOAD_LEN]));
        // an empty file is an error too, after the close
        let t = Fake::new(60);
        let e = read_file(&t, FactoryFile::Pl).unwrap_err();
        assert!(e.to_string().contains("empty or missing"), "{e}");
        assert_eq!(t.seen.borrow().last().unwrap().1, SET_CUR);
    }

    #[test]
    fn chunks_are_appended_until_length_zero() {
        let mut reply = [0; PAYLOAD_LEN];
        reply[..4].copy_from_slice(&3u32.to_le_bytes());
        reply[4..7].copy_from_slice(b"abc");
        reply[7] = b'x';
        let mut out = Vec::new();
        assert!(append_chunk(&reply, &mut out).unwrap());
        assert_eq!(out, b"abc");
        assert!(!append_chunk(&[0; PAYLOAD_LEN], &mut out).unwrap());
        reply[..4].copy_from_slice(&57u32.to_le_bytes());
        assert_eq!(append_chunk(&reply, &mut out).unwrap_err().code(), "device");
        let mut full = vec![0; MAX_FILE_BYTES - 2];
        reply[..4].copy_from_slice(&3u32.to_le_bytes());
        assert!(append_chunk(&reply, &mut full).is_err());
    }

    #[test]
    fn the_unit_is_found_by_id_and_guid() {
        // configuration header, then the extension unit of the MINI 2 as
        // its descriptors hold it (14 controls, source 2, bmControls
        // ff a2 a7)
        let mut d = vec![9, 2, 0, 0, 1, 1, 0, 0x80, 0xfa];
        let mut xu = vec![28, 0x24, 0x06, 4];
        xu.extend_from_slice(&UNIT_GUID_BYTES);
        xu.extend_from_slice(&[14, 1, 2, 3, 0xff, 0xa2, 0xa7, 0]);
        d.extend_from_slice(&xu);
        assert!(has_unit(&d));
        // another unit id, another GUID, a truncated descriptor
        let mut other = d.clone();
        other[9 + 3] = 3;
        assert!(!has_unit(&other));
        let mut other = d.clone();
        other[9 + 4] ^= 1;
        assert!(!has_unit(&other));
        assert!(!has_unit(&d[..20]));
    }
}
