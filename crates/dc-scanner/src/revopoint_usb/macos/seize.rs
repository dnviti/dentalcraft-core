//! Taking the video-streaming interface from the system camera driver with
//! `USBInterfaceOpenSeize` (IOKit, `IOUSBLib`) when a plain open through
//! `nusb` is refused, and reading its bulk endpoint.
//!
//! Direct calls into the IOKit and CoreFoundation frameworks of the system
//! (nothing compiled from C). The interface is `IOUSBInterfaceInterface190`,
//! the first version with both `USBInterfaceOpenSeize` (183) and
//! `ClearPipeStallBothEnds` (190); the layouts and UUIDs are those of
//! `IOUSBLib.h`, `IOCFPlugIn.h` and `CFPlugInCOM.h`.
//!
//! The pipe is read with synchronous `ReadPipeTO` calls on the reader
//! thread. A timed-out read returns no data and may leave the data toggle
//! out of step (Apple, `IOUSBLib.h`), so the timeout ([`READ_TIMEOUT_MS`])
//! is longer than any pause between two frames (0.4 s at 2.7 frames per
//! second): only a stream that stalled times out, and a stop waits at most
//! that long for the read in progress.

#![allow(unsafe_code)]
#![warn(clippy::undocumented_unsafe_blocks)]

use std::ffi::{c_char, c_void};
use std::time::Duration;

use crate::revopoint_usb::uvc_bulk::{BulkPipe, Read, transfer_len};
use crate::{Result, ScanError};

/// `IOReturn` / `kern_return_t`.
type IoReturn = i32;
/// `io_object_t` (a Mach port name).
type IoObject = u32;

const SUCCESS: IoReturn = 0;

/// An `IOReturn` of `sub_iokit_common` (`IOReturn.h`).
const fn common(code: u32) -> IoReturn {
    (0xe000_0000 | code) as IoReturn
}

/// An `IOReturn` of `sub_iokit_usb` (`USB.h`).
const fn usb(code: u32) -> IoReturn {
    (0xe000_4000 | code) as IoReturn
}

const NO_DEVICE: IoReturn = common(0x2c0);
const NOT_PRIVILEGED: IoReturn = common(0x2c1);
const EXCLUSIVE_ACCESS: IoReturn = common(0x2c5);
const TIMEOUT: IoReturn = common(0x2d6);
const NOT_PERMITTED: IoReturn = common(0x2e2);
const ABORTED: IoReturn = common(0x2eb);
const NOT_RESPONDING: IoReturn = common(0x2ed);
const PIPE_STALLED: IoReturn = usb(0x4f);
const TRANSACTION_TIMEOUT: IoReturn = usb(0x51);

/// `kUSBIn`: direction of a device-to-host pipe.
const USB_IN: u8 = 1;
/// `kUSBBulk`: transfer type of a bulk pipe.
const USB_BULK: u8 = 2;

/// Longest wait of one read (see the module doc).
pub(super) const READ_TIMEOUT_MS: u32 = 1000;

/// `kIOUSBInterfaceUserClientTypeID`, `2D9786C6-9EF3-11D4-AD51-000A27052861`.
const INTERFACE_USER_CLIENT: [u8; 16] = [
    0x2d, 0x97, 0x86, 0xc6, 0x9e, 0xf3, 0x11, 0xd4, 0xad, 0x51, 0x00, 0x0a, 0x27, 0x05, 0x28, 0x61,
];
/// `kIOCFPlugInInterfaceID`, `C244E858-109C-11D4-91D4-0050E4C6426F`.
const PLUGIN_INTERFACE: [u8; 16] = [
    0xc2, 0x44, 0xe8, 0x58, 0x10, 0x9c, 0x11, 0xd4, 0x91, 0xd4, 0x00, 0x50, 0xe4, 0xc6, 0x42, 0x6f,
];
/// `kIOUSBInterfaceInterfaceID190`, `8FDB8455-74A6-11D6-97B1-003065D3608E`.
const INTERFACE_190: [u8; 16] = [
    0x8f, 0xdb, 0x84, 0x55, 0x74, 0xa6, 0x11, 0xd6, 0x97, 0xb1, 0x00, 0x30, 0x65, 0xd3, 0x60, 0x8e,
];

/// `CFUUIDBytes`, passed by value as a COM interface id (`REFIID`).
#[repr(C)]
#[derive(Clone, Copy)]
struct CfUuidBytes([u8; 16]);

/// A vtable slot this code never calls.
type Slot = *const c_void;

/// `IOCFPlugInInterface`: `IUNKNOWN_C_GUTS`, the rest unused.
#[repr(C)]
struct PlugInVtbl {
    _reserved: *mut c_void,
    query_interface: Option<
        unsafe extern "C" fn(this: *mut c_void, iid: CfUuidBytes, out: *mut *mut c_void) -> i32,
    >,
    _add_ref: Slot,
    release: Option<unsafe extern "C" fn(this: *mut c_void) -> u32>,
}

/// `IOUSBInterfaceInterface190` up to `ClearPipeStallBothEnds`.
#[repr(C)]
struct InterfaceVtbl {
    _reserved: *mut c_void,
    _query_interface: Slot,
    _add_ref: Slot,
    release: Option<unsafe extern "C" fn(this: *mut c_void) -> u32>,
    /// `CreateInterfaceAsyncEventSource` .. `GetInterfaceAsyncPort`.
    _async: [Slot; 4],
    _open: Slot,
    close: Option<unsafe extern "C" fn(this: *mut c_void) -> IoReturn>,
    /// `GetInterfaceClass` .. `GetConfigurationValue`.
    _identity: [Slot; 7],
    interface_number: Option<unsafe extern "C" fn(this: *mut c_void, number: *mut u8) -> IoReturn>,
    _alternate_setting: Slot,
    endpoint_count: Option<unsafe extern "C" fn(this: *mut c_void, count: *mut u8) -> IoReturn>,
    /// `GetLocationID` .. `ControlRequestAsync`.
    _device: [Slot; 6],
    pipe_properties: Option<
        unsafe extern "C" fn(
            this: *mut c_void,
            pipe: u8,
            direction: *mut u8,
            number: *mut u8,
            transfer_type: *mut u8,
            max_packet: *mut u16,
            interval: *mut u8,
        ) -> IoReturn,
    >,
    _pipe_status: Slot,
    _abort_pipe: Slot,
    /// `ResetPipe` .. `ControlRequestAsyncTO`.
    _pipes: [Slot; 10],
    read_pipe_to: Option<
        unsafe extern "C" fn(
            this: *mut c_void,
            pipe: u8,
            buf: *mut c_void,
            size: *mut u32,
            no_data_ms: u32,
            completion_ms: u32,
        ) -> IoReturn,
    >,
    /// `WritePipeTO` .. `USBInterfaceGetStringIndex`.
    _writes: [Slot; 4],
    open_seize: Option<unsafe extern "C" fn(this: *mut c_void) -> IoReturn>,
    clear_stall_both_ends: Option<unsafe extern "C" fn(this: *mut c_void, pipe: u8) -> IoReturn>,
}

// The slots of `IOUSBLib.h`: `IUNKNOWN_C_GUTS` (4), then the functions of
// versions 100, 182, 183 and 190 in order.
const _: () = {
    let p = size_of::<Slot>();
    assert!(std::mem::offset_of!(InterfaceVtbl, release) == 3 * p);
    assert!(std::mem::offset_of!(InterfaceVtbl, close) == 9 * p);
    assert!(std::mem::offset_of!(InterfaceVtbl, interface_number) == 17 * p);
    assert!(std::mem::offset_of!(InterfaceVtbl, endpoint_count) == 19 * p);
    assert!(std::mem::offset_of!(InterfaceVtbl, pipe_properties) == 26 * p);
    assert!(std::mem::offset_of!(InterfaceVtbl, read_pipe_to) == 39 * p);
    assert!(std::mem::offset_of!(InterfaceVtbl, open_seize) == 44 * p);
    assert!(std::mem::offset_of!(InterfaceVtbl, clear_stall_both_ends) == 45 * p);
    assert!(size_of::<CfUuidBytes>() == 16);
};

#[link(name = "CoreFoundation", kind = "framework")]
unsafe extern "C" {
    fn CFUUIDGetConstantUUIDWithBytes(
        allocator: *const c_void,
        b0: u8,
        b1: u8,
        b2: u8,
        b3: u8,
        b4: u8,
        b5: u8,
        b6: u8,
        b7: u8,
        b8: u8,
        b9: u8,
        b10: u8,
        b11: u8,
        b12: u8,
        b13: u8,
        b14: u8,
        b15: u8,
    ) -> *const c_void;
}

#[link(name = "IOKit", kind = "framework")]
unsafe extern "C" {
    fn IORegistryEntryIDMatching(entry_id: u64) -> *mut c_void;
    fn IOServiceGetMatchingService(main_port: u32, matching: *mut c_void) -> IoObject;
    fn IORegistryEntryGetChildIterator(
        entry: IoObject,
        plane: *const c_char,
        iterator: *mut IoObject,
    ) -> IoReturn;
    fn IOIteratorNext(iterator: IoObject) -> IoObject;
    fn IOObjectRelease(object: IoObject) -> IoReturn;
    fn IOCreatePlugInInterfaceForService(
        service: IoObject,
        plugin_type: *const c_void,
        interface_type: *const c_void,
        plugin: *mut *mut *mut PlugInVtbl,
        score: *mut i32,
    ) -> IoReturn;
}

/// The constant `CFUUID` of `b` (never released).
fn uuid(b: [u8; 16]) -> *const c_void {
    // SAFETY: a plain call with a null (default) allocator; it returns a
    // constant UUID object that lives for the whole process.
    unsafe {
        CFUUIDGetConstantUUIDWithBytes(
            std::ptr::null(),
            b[0],
            b[1],
            b[2],
            b[3],
            b[4],
            b[5],
            b[6],
            b[7],
            b[8],
            b[9],
            b[10],
            b[11],
            b[12],
            b[13],
            b[14],
            b[15],
        )
    }
}

/// `code` with its name when it is one of the expected ones.
fn describe(code: IoReturn) -> String {
    let name = match code {
        NO_DEVICE => "kIOReturnNoDevice",
        NOT_PRIVILEGED => "kIOReturnNotPrivileged",
        EXCLUSIVE_ACCESS => "kIOReturnExclusiveAccess",
        TIMEOUT => "kIOReturnTimeout",
        NOT_PERMITTED => "kIOReturnNotPermitted",
        ABORTED => "kIOReturnAborted",
        NOT_RESPONDING => "kIOReturnNotResponding",
        PIPE_STALLED => "kIOUSBPipeStalled",
        TRANSACTION_TIMEOUT => "kIOUSBTransactionTimeout",
        _ => return format!("IOReturn {:#010x}", code as u32),
    };
    format!("{name} ({:#010x})", code as u32)
}

/// Why the interface could not be taken.
#[derive(Debug)]
pub(super) enum SeizeError {
    /// Its holder kept it (or the system refused): no capture.
    Denied(String),
    /// The device left the bus.
    Lost(String),
    /// Something else failed.
    Failed(String),
}

/// An interface of the user client of `IOUSBLib`; closed (when opened) and
/// released on drop.
struct Interface {
    this: *mut *mut InterfaceVtbl,
    open: bool,
}

impl Interface {
    /// The interface of the USB interface service `service`; `None` for a
    /// child of the device that is no USB interface.
    fn of(service: IoObject) -> Option<Self> {
        let mut plugin: *mut *mut PlugInVtbl = std::ptr::null_mut();
        let mut score = 0;
        // SAFETY: a live service, two constant UUIDs and valid out
        // pointers; on success `plugin` is an owned plug-in.
        let created = unsafe {
            IOCreatePlugInInterfaceForService(
                service,
                uuid(INTERFACE_USER_CLIENT),
                uuid(PLUGIN_INTERFACE),
                &mut plugin,
                &mut score,
            )
        };
        if created != SUCCESS || plugin.is_null() {
            return None;
        }
        // SAFETY: `plugin` points to the object, whose first field points
        // to its vtable (`IUNKNOWN_C_GUTS`), live until released below.
        let vtbl = unsafe { &**plugin };
        let mut out: *mut c_void = std::ptr::null_mut();
        let found = vtbl.query_interface.map(|query| {
            // SAFETY: the plug-in's own `QueryInterface` with a valid out
            // pointer; on success `out` owns one reference.
            unsafe { query(plugin.cast(), CfUuidBytes(INTERFACE_190), &mut out) }
        });
        if let Some(release) = vtbl.release {
            // SAFETY: drops the plug-in reference created above (the
            // interface keeps its own).
            unsafe { release(plugin.cast()) };
        }
        (found == Some(0) && !out.is_null()).then(|| Self {
            this: out.cast(),
            open: false,
        })
    }

    fn vtbl(&self) -> &InterfaceVtbl {
        // SAFETY: `this` is a live interface object (until drop), whose
        // first field points to its vtable.
        unsafe { &**self.this }
    }

    /// `bInterfaceNumber`.
    fn number(&self) -> Option<u8> {
        let get = self.vtbl().interface_number?;
        let mut number = 0;
        // SAFETY: the interface's own method with a valid out pointer.
        (unsafe { get(self.this.cast(), &mut number) } == SUCCESS).then_some(number)
    }

    /// `USBInterfaceOpenSeize`.
    fn seize(&mut self) -> std::result::Result<(), SeizeError> {
        let seize = self
            .vtbl()
            .open_seize
            .ok_or_else(|| SeizeError::Failed("no USBInterfaceOpenSeize".into()))?;
        // SAFETY: the interface's own method.
        match unsafe { seize(self.this.cast()) } {
            SUCCESS => {
                self.open = true;
                Ok(())
            }
            NO_DEVICE => Err(SeizeError::Lost(format!(
                "USBInterfaceOpenSeize: {}",
                describe(NO_DEVICE)
            ))),
            e => Err(SeizeError::Denied(format!(
                "USBInterfaceOpenSeize: {}",
                describe(e)
            ))),
        }
    }
}

impl Drop for Interface {
    fn drop(&mut self) {
        let vtbl = self.vtbl();
        let (close, release) = (vtbl.close, vtbl.release);
        if self.open
            && let Some(close) = close
        {
            // SAFETY: the interface's own method; it was opened by this
            // value (errors: the device may be gone).
            let _ = unsafe { close(self.this.cast()) };
        }
        if let Some(release) = release {
            // SAFETY: drops the reference `of` obtained, once.
            unsafe { release(self.this.cast()) };
        }
    }
}

/// Interface `number` of the USB device with IORegistry entry id
/// `device`, seized from its holder (see the module doc).
///
/// # Errors
/// [`SeizeError::Denied`] when the holder keeps it, `Lost` when the device
/// left, `Failed` for the rest.
pub(super) fn seize(device: u64, number: u8) -> std::result::Result<Seized, SeizeError> {
    // SAFETY: a plain call; the dictionary it returns is consumed below.
    let matching = unsafe { IORegistryEntryIDMatching(device) };
    if matching.is_null() {
        return Err(SeizeError::Failed(
            "IORegistryEntryIDMatching failed".into(),
        ));
    }
    // SAFETY: a fresh matching dictionary, consumed by the call; port 0 is
    // the default main port.
    let service = unsafe { IOServiceGetMatchingService(0, matching) };
    if service == 0 {
        return Err(SeizeError::Lost(format!(
            "no USB device with registry entry id {device:#x}"
        )));
    }
    let mut children = 0;
    // SAFETY: a live service, a NUL-terminated plane name, an out pointer.
    let listed =
        unsafe { IORegistryEntryGetChildIterator(service, c"IOService".as_ptr(), &mut children) };
    // SAFETY: the service from `IOServiceGetMatchingService`, released once.
    unsafe { IOObjectRelease(service) };
    if listed != SUCCESS {
        return Err(SeizeError::Failed(format!(
            "IORegistryEntryGetChildIterator: {}",
            describe(listed)
        )));
    }
    let mut found = None;
    loop {
        // SAFETY: a live iterator.
        let child = unsafe { IOIteratorNext(children) };
        if child == 0 {
            break;
        }
        let interface = Interface::of(child);
        // SAFETY: an object from the iterator, released once.
        unsafe { IOObjectRelease(child) };
        if let Some(interface) = interface
            && interface.number() == Some(number)
        {
            found = Some(interface);
            break;
        }
    }
    // SAFETY: the iterator from `IORegistryEntryGetChildIterator`, released
    // once.
    unsafe { IOObjectRelease(children) };
    let mut interface = found
        .ok_or_else(|| SeizeError::Failed(format!("no interface {number} under the USB device")))?;
    interface.seize()?;
    Ok(Seized(interface))
}

/// A seized interface.
pub(super) struct Seized(Interface);

impl Seized {
    /// The bulk IN pipe of endpoint `address`.
    ///
    /// # Errors
    /// [`SeizeError::Failed`] when the interface has none.
    pub(super) fn into_pipe(self, address: u8) -> std::result::Result<SeizedPipe, SeizeError> {
        let vtbl = self.0.vtbl();
        let (Some(count), Some(properties)) = (vtbl.endpoint_count, vtbl.pipe_properties) else {
            return Err(SeizeError::Failed("no pipe functions".into()));
        };
        let mut pipes = 0;
        // SAFETY: the interface's own method with a valid out pointer.
        if unsafe { count(self.0.this.cast(), &mut pipes) } != SUCCESS {
            return Err(SeizeError::Failed("GetNumEndpoints failed".into()));
        }
        for pipe in 1..=pipes {
            let (mut direction, mut number, mut kind, mut packet, mut interval) = (0, 0, 0, 0, 0);
            // SAFETY: the interface's own method with valid out pointers.
            let read = unsafe {
                properties(
                    self.0.this.cast(),
                    pipe,
                    &mut direction,
                    &mut number,
                    &mut kind,
                    &mut packet,
                    &mut interval,
                )
            };
            if read == SUCCESS
                && direction == USB_IN
                && number == address & 0x0f
                && kind == USB_BULK
            {
                let packet = usize::from(packet);
                return Ok(SeizedPipe {
                    interface: self.0,
                    pipe,
                    packet,
                    buf: vec![0; transfer_len(0, packet)],
                });
            }
        }
        Err(SeizeError::Failed(format!(
            "no bulk IN pipe for endpoint {address:#04x}"
        )))
    }
}

/// The bulk IN pipe of a seized interface, read synchronously (see the
/// module doc).
pub(super) struct SeizedPipe {
    interface: Interface,
    /// `pipeRef` (1 to `GetNumEndpoints`).
    pipe: u8,
    /// `wMaxPacketSize`.
    packet: usize,
    buf: Vec<u8>,
}

// SAFETY: the user-client interface of IOKit may be called from any thread;
// the pipe is used by one thread at a time (the reader thread, then the
// thread that halts it after joining the reader).
unsafe impl Send for SeizedPipe {}

impl BulkPipe for SeizedPipe {
    fn set_payload_size(&mut self, max_payload: usize) {
        self.buf.resize(transfer_len(max_payload, self.packet), 0);
    }

    /// One `ReadPipeTO` of [`READ_TIMEOUT_MS`], whatever `_timeout` (see
    /// the module doc).
    fn read(&mut self, _timeout: Duration) -> Result<Option<Read<'_>>> {
        let read = self
            .interface
            .vtbl()
            .read_pipe_to
            .ok_or_else(|| ScanError::Device("no ReadPipeTO".into()))?;
        let mut size = u32::try_from(self.buf.len()).unwrap_or(u32::MAX);
        // SAFETY: the interface's own method; `buf` is valid for `size`
        // bytes for the whole (synchronous) call.
        let r = unsafe {
            read(
                self.interface.this.cast(),
                self.pipe,
                self.buf.as_mut_ptr().cast(),
                &mut size,
                READ_TIMEOUT_MS,
                2 * READ_TIMEOUT_MS,
            )
        };
        match r {
            SUCCESS => {
                let n = (size as usize).min(self.buf.len());
                Ok(Some(Read {
                    data: &self.buf[..n],
                    short: n < self.buf.len(),
                }))
            }
            TRANSACTION_TIMEOUT | TIMEOUT => Ok(None),
            NO_DEVICE | NOT_RESPONDING | ABORTED => Err(ScanError::DeviceLost(format!(
                "video stream read: {}",
                describe(r)
            ))),
            e => Err(ScanError::Device(format!(
                "video stream read: {}",
                describe(e)
            ))),
        }
    }

    /// `ClearPipeStallBothEnds`: `CLEAR_FEATURE(ENDPOINT_HALT)` to the
    /// device, the end of the stream (no read is queued: they are
    /// synchronous).
    fn halt(&mut self) -> Result<()> {
        let clear = self
            .interface
            .vtbl()
            .clear_stall_both_ends
            .ok_or_else(|| ScanError::Device("no ClearPipeStallBothEnds".into()))?;
        // SAFETY: the interface's own method.
        match unsafe { clear(self.interface.this.cast(), self.pipe) } {
            SUCCESS => Ok(()),
            NO_DEVICE => Err(ScanError::DeviceLost(format!(
                "CLEAR_FEATURE(ENDPOINT_HALT): {}",
                describe(NO_DEVICE)
            ))),
            e => Err(ScanError::Device(format!(
                "CLEAR_FEATURE(ENDPOINT_HALT): {}",
                describe(e)
            ))),
        }
    }
}
