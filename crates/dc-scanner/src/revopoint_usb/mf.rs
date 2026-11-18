//! Minimal Media Foundation video capture (Windows): the counterpart of
//! [`super::v4l2`]. Only standard capture calls of the inbox UVC driver:
//! enumerate the video capture sources, open one, pick one of its native
//! formats (subtype, size, frame rate), read samples, stop. Nothing
//! vendor-specific is sent.
//!
//! The source reader runs in asynchronous mode: one `ReadSample` request is
//! outstanding at a time and its callback, on a Media Foundation thread,
//! copies the frame into an inbox that [`Capture::next`] waits on with a
//! timeout (a synchronous `ReadSample` cannot time out).
//!
//! The Media Foundation entry points are resolved at run time from the
//! system directory: on Windows N editions (no Media Feature Pack) and
//! Windows Server without the Media Foundation feature the libraries may be
//! missing, and the application must still start. The backend then
//! reports `platform-unsupported`.

#![allow(unsafe_code)]
#![warn(clippy::undocumented_unsafe_blocks)]

use std::ffi::c_void;
use std::sync::{Arc, Condvar, LazyLock, Mutex, MutexGuard};
use std::time::Duration;

use libloading::os::windows::{LOAD_LIBRARY_SEARCH_SYSTEM32, Library};
use windows::Win32::Media::MediaFoundation::{
    IMFActivate, IMFAttributes, IMFMediaEvent, IMFMediaSource, IMFMediaType, IMFSample,
    IMFSourceReader, IMFSourceReaderCallback, IMFSourceReaderCallback_Impl,
    MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID,
    MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, MF_E_NO_MORE_TYPES,
    MF_E_VIDEO_RECORDING_DEVICE_INVALIDATED, MF_E_VIDEO_RECORDING_DEVICE_PREEMPTED,
    MF_MT_FRAME_RATE, MF_MT_FRAME_SIZE, MF_MT_SAMPLE_SIZE, MF_MT_SUBTYPE,
    MF_READWRITE_DISABLE_CONVERTERS, MF_SOURCE_READER_ALL_STREAMS, MF_SOURCE_READER_ASYNC_CALLBACK,
    MF_SOURCE_READER_FIRST_VIDEO_STREAM, MF_SOURCE_READERF_ENDOFSTREAM, MF_SOURCE_READERF_ERROR,
    MF_VERSION, MFSTARTUP_NOSOCKET,
};
use windows::Win32::System::Com::{CoIncrementMTAUsage, CoTaskMemFree};
use windows::core::{GUID, HRESULT, Interface, PWSTR, Ref, implement};

use crate::{Result, ScanError};

/// FOURCC of `Y16 `: 16-bit greyscale (the subtype is the FOURCC in the
/// Media Foundation base GUID).
pub(super) const PIX_FMT_Y16: u32 = u32::from_le_bytes(*b"Y16 ");

/// Longest wait for the source reader to drain on stop.
const FLUSH_TIMEOUT: Duration = Duration::from_secs(1);

// ----- errors --------------------------------------------------------------

/// `HRESULT_FROM_WIN32` of a Win32 error code.
const fn win32(code: u32) -> HRESULT {
    HRESULT((code | 0x8007_0000) as i32)
}

/// The device is not (or no longer) on the bus. `ERROR_INVALID_HANDLE` is
/// what a sample request returns when the device left right after it was
/// opened (seen when it re-enumerates).
const GONE: [HRESULT; 6] = [
    MF_E_VIDEO_RECORDING_DEVICE_INVALIDATED,
    win32(2),    // ERROR_FILE_NOT_FOUND
    win32(6),    // ERROR_INVALID_HANDLE
    win32(55),   // ERROR_DEV_NOT_EXIST
    win32(1167), // ERROR_DEVICE_NOT_CONNECTED
    win32(1617), // ERROR_DEVICE_REMOVED
];
/// The device is held by another program.
const BUSY: [HRESULT; 4] = [
    MF_E_VIDEO_RECORDING_DEVICE_PREEMPTED,
    HRESULT(0xC00D_3704_u32 as i32), // MF_E_HW_MFT_FAILED_START_STREAMING
    win32(32),                       // ERROR_SHARING_VIOLATION
    win32(170),                      // ERROR_BUSY
];
/// `E_ACCESSDENIED`: the camera privacy settings block desktop apps.
const ACCESS_DENIED: HRESULT = win32(5);
/// `ERROR_NOT_READY`: the device is still starting.
const NOT_READY: HRESULT = win32(21);

/// A failed call as a scanner error: a vanished device is `device-lost`.
fn hr_error(call: &str, hr: HRESULT) -> ScanError {
    let detail = format!("{call}: {} ({:#010x})", hr.message(), hr.0 as u32);
    if GONE.contains(&hr) {
        ScanError::DeviceLost(detail)
    } else if BUSY.contains(&hr) {
        ScanError::Device(format!("{detail} (another program is using the scanner)"))
    } else if hr == ACCESS_DENIED {
        ScanError::Device(format!(
            "{detail} (camera access is off in the Windows privacy settings: allow desktop \
             apps to access the camera)"
        ))
    } else {
        ScanError::Device(detail)
    }
}

// ----- run-time loaded entry points ----------------------------------------

type MfStartup = unsafe extern "system" fn(u32, u32) -> HRESULT;
type MfCreateAttributes = unsafe extern "system" fn(*mut *mut c_void, u32) -> HRESULT;
type MfEnumDeviceSources =
    unsafe extern "system" fn(*mut c_void, *mut *mut *mut c_void, *mut u32) -> HRESULT;
type MfCreateDeviceSource = unsafe extern "system" fn(*mut c_void, *mut *mut c_void) -> HRESULT;
type MfCreateSourceReaderFromMediaSource =
    unsafe extern "system" fn(*mut c_void, *mut c_void, *mut *mut c_void) -> HRESULT;

/// The Media Foundation functions this module calls, with their libraries
/// (loaded once, never unloaded).
struct Api {
    create_attributes: MfCreateAttributes,
    enum_device_sources: MfEnumDeviceSources,
    create_device_source: MfCreateDeviceSource,
    create_source_reader: MfCreateSourceReaderFromMediaSource,
    _libraries: [Library; 3],
}

impl Api {
    fn load() -> std::result::Result<Self, String> {
        // MTA for the whole process: Media Foundation objects are then
        // usable from any thread that did not join an STA (implicit MTA).
        // SAFETY: no precondition; the cookie is never released.
        unsafe { CoIncrementMTAUsage() }.map_err(|e| format!("CoIncrementMTAUsage: {e}"))?;
        let open = |name: &str| {
            // SAFETY: system libraries from System32 only; their
            // initialisers have no precondition.
            unsafe { Library::load_with_flags(name, LOAD_LIBRARY_SEARCH_SYSTEM32) }
                .map_err(|e| format!("{name}: {e}"))
        };
        let (mfplat, mf, mfreadwrite) = (
            open("mfplat.dll")?,
            open("mf.dll")?,
            open("mfreadwrite.dll")?,
        );
        // SAFETY: the symbols have the documented signatures of the type
        // aliases; the libraries stay loaded in `_libraries`.
        let api = unsafe {
            let startup: MfStartup = *mfplat.get(b"MFStartup\0").map_err(|e| e.to_string())?;
            startup(MF_VERSION, MFSTARTUP_NOSOCKET)
                .ok()
                .map_err(|e| format!("MFStartup: {e}"))?;
            Self {
                create_attributes: *mfplat
                    .get(b"MFCreateAttributes\0")
                    .map_err(|e| e.to_string())?,
                enum_device_sources: *mf
                    .get(b"MFEnumDeviceSources\0")
                    .map_err(|e| e.to_string())?,
                create_device_source: *mf
                    .get(b"MFCreateDeviceSource\0")
                    .map_err(|e| e.to_string())?,
                create_source_reader: *mfreadwrite
                    .get(b"MFCreateSourceReaderFromMediaSource\0")
                    .map_err(|e| e.to_string())?,
                _libraries: [mfplat, mf, mfreadwrite],
            }
        };
        Ok(api)
    }

    /// An attribute store holding `count` items.
    fn attributes(&self, count: u32) -> Result<IMFAttributes> {
        let mut raw = std::ptr::null_mut();
        // SAFETY: `raw` is a valid out pointer; on success it owns one
        // reference to an `IMFAttributes`.
        unsafe { (self.create_attributes)(&mut raw, count) }
            .ok()
            .map_err(|e| hr_error("MFCreateAttributes", e.code()))?;
        // SAFETY: a non-null owned `IMFAttributes` pointer.
        Ok(unsafe { IMFAttributes::from_raw(raw) })
    }

    /// Attributes selecting the video capture sources.
    fn video_capture(&self, count: u32) -> Result<IMFAttributes> {
        let attributes = self.attributes(count)?;
        // SAFETY: valid GUID pointers for the call.
        unsafe {
            attributes.SetGUID(
                &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
                &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID,
            )
        }
        .map_err(|e| hr_error("IMFAttributes::SetGUID", e.code()))?;
        Ok(attributes)
    }
}

/// Media Foundation, started once per process.
fn api() -> Result<&'static Api> {
    static API: LazyLock<std::result::Result<Api, String>> = LazyLock::new(Api::load);
    API.as_ref().map_err(|e| {
        ScanError::PlatformUnsupported(format!(
            "Media Foundation is not available ({e}); on Windows N install the Media Feature \
             Pack, on Windows Server the Media Foundation feature"
        ))
    })
}

/// Whether Media Foundation can be used.
pub(super) fn availability() -> Result<()> {
    api().map(|_| ())
}

/// Symbolic links of the video capture sources.
pub(super) fn video_sources() -> Result<Vec<String>> {
    let api = api()?;
    let attributes = api.video_capture(1)?;
    let mut array: *mut *mut c_void = std::ptr::null_mut();
    let mut count = 0u32;
    // SAFETY: valid out pointers; on success `array` is a CoTaskMem array of
    // `count` owned `IMFActivate` pointers.
    unsafe { (api.enum_device_sources)(attributes.as_raw(), &mut array, &mut count) }
        .ok()
        .map_err(|e| hr_error("MFEnumDeviceSources", e.code()))?;
    let mut links = Vec::with_capacity(count as usize);
    for i in 0..count as usize {
        // SAFETY: `i < count`; every entry is an owned non-null pointer,
        // released when `activate` drops.
        let activate = unsafe { IMFActivate::from_raw(*array.add(i)) };
        let mut text = PWSTR::null();
        let mut len = 0u32;
        // SAFETY: valid out pointers; the string is CoTaskMem allocated.
        let got = unsafe {
            activate.GetAllocatedString(
                &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK,
                &mut text,
                &mut len,
            )
        };
        if got.is_ok() {
            // SAFETY: a NUL-terminated string from Media Foundation, freed
            // once right after.
            unsafe {
                if let Ok(link) = text.to_string() {
                    links.push(link);
                }
                CoTaskMemFree(Some(text.0 as *const c_void));
            }
        }
    }
    // SAFETY: the array came from `MFEnumDeviceSources`, its entries are
    // released above.
    unsafe { CoTaskMemFree(Some(array as *const c_void)) };
    Ok(links)
}

/// Opens the source behind `link`. The error says whether it may be
/// transient (the device is not, or not yet, on the bus).
pub(super) fn open_capture(link: &str) -> std::result::Result<Capture, (bool, ScanError)> {
    Capture::open(link).map_err(|(hr, e)| (GONE.contains(&hr) || hr == NOT_READY, e))
}

// ----- capture -------------------------------------------------------------

/// A dequeued frame, copied out of the sample.
#[derive(Clone, Copy, Debug)]
pub(super) struct Dequeued {
    /// Bytes of the frame written into the output vector.
    pub bytes: usize,
    /// Sample time, microseconds.
    pub timestamp_us: u64,
}

/// What a `ReadSample` request produced.
enum Ready {
    /// A frame of `bytes` in [`State::data`].
    Frame { bytes: usize, timestamp_us: u64 },
    /// No sample (a gap in the stream, or a flush).
    Nothing,
    /// The stream failed.
    Failed(ScanError),
}

#[derive(Default)]
struct State {
    /// Bytes of the latest frame.
    data: Vec<u8>,
    ready: Option<Ready>,
    /// A `ReadSample` request is outstanding.
    pending: bool,
    /// The last flush completed.
    flushed: bool,
}

/// Shared by the capture and the reader callback.
#[derive(Default)]
struct Inbox {
    state: Mutex<State>,
    changed: Condvar,
}

impl Inbox {
    fn lock(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(|e| e.into_inner())
    }
}

/// The source reader callback: hands each result over to the inbox.
#[implement(IMFSourceReaderCallback)]
struct Callback {
    inbox: Arc<Inbox>,
}

impl IMFSourceReaderCallback_Impl for Callback_Impl {
    fn OnReadSample(
        &self,
        hrstatus: HRESULT,
        _stream: u32,
        flags: u32,
        timestamp: i64,
        sample: Ref<IMFSample>,
    ) -> windows::core::Result<()> {
        let mut state = self.inbox.lock();
        let ready = if hrstatus.is_err() {
            Ready::Failed(hr_error("ReadSample", hrstatus))
        } else if flags & MF_SOURCE_READERF_ERROR.0 as u32 != 0 {
            Ready::Failed(ScanError::DeviceLost("the video stream failed".into()))
        } else if flags & MF_SOURCE_READERF_ENDOFSTREAM.0 as u32 != 0 {
            Ready::Failed(ScanError::DeviceLost("the video stream ended".into()))
        } else if let Some(sample) = sample.as_ref() {
            match copy_sample(sample, &mut state.data) {
                Ok(bytes) => Ready::Frame {
                    bytes,
                    // 100 ns units
                    timestamp_us: u64::try_from(timestamp).unwrap_or(0) / 10,
                },
                Err(e) => Ready::Failed(e),
            }
        } else {
            Ready::Nothing
        };
        state.pending = false;
        state.ready = Some(ready);
        self.inbox.changed.notify_all();
        Ok(())
    }

    fn OnFlush(&self, _stream: u32) -> windows::core::Result<()> {
        self.inbox.lock().flushed = true;
        self.inbox.changed.notify_all();
        Ok(())
    }

    fn OnEvent(&self, _stream: u32, _event: Ref<IMFMediaEvent>) -> windows::core::Result<()> {
        Ok(())
    }
}

/// Copies the bytes of `sample` into `out`; returns their count.
fn copy_sample(sample: &IMFSample, out: &mut Vec<u8>) -> Result<usize> {
    // SAFETY: plain COM calls on a live sample.
    let buffer = unsafe { sample.ConvertToContiguousBuffer() }
        .map_err(|e| hr_error("ConvertToContiguousBuffer", e.code()))?;
    let mut data = std::ptr::null_mut();
    let mut len = 0u32;
    // SAFETY: valid out pointers; the buffer stays locked until `Unlock`.
    unsafe { buffer.Lock(&mut data, None, Some(&mut len)) }
        .map_err(|e| hr_error("IMFMediaBuffer::Lock", e.code()))?;
    out.clear();
    if !data.is_null() {
        // SAFETY: a locked buffer of `len` valid bytes at `data`.
        out.extend_from_slice(unsafe { std::slice::from_raw_parts(data, len as usize) });
    }
    // SAFETY: the buffer was locked above.
    unsafe { buffer.Unlock() }.map_err(|e| hr_error("IMFMediaBuffer::Unlock", e.code()))?;
    Ok(out.len())
}

/// A native format of the source.
struct Native {
    media_type: IMFMediaType,
    subtype: GUID,
    width: u32,
    height: u32,
    /// Frames per second (0 when unknown).
    fps: f64,
    /// Bytes of one image.
    sample_size: usize,
}

/// The Media Foundation subtype of a FOURCC.
fn fourcc_subtype(fourcc: u32) -> GUID {
    GUID::from_values(
        fourcc,
        0x0000,
        0x0010,
        [0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71],
    )
}

const VIDEO_STREAM: u32 = MF_SOURCE_READER_FIRST_VIDEO_STREAM.0 as u32;
const ALL_STREAMS: u32 = MF_SOURCE_READER_ALL_STREAMS.0 as u32;

/// An open video capture source, streaming once [`Capture::start`]
/// succeeded.
pub(super) struct Capture {
    source: IMFMediaSource,
    reader: IMFSourceReader,
    inbox: Arc<Inbox>,
    /// Subtype and size set by [`Capture::set_format`].
    format: Option<(GUID, u32, u32)>,
    streaming: bool,
}

// SAFETY: the media source and the asynchronous source reader are
// free-threaded Media Foundation objects living in the process MTA; they
// are only used through `&mut self`.
unsafe impl Send for Capture {}

impl Capture {
    /// Opens the video capture source with symbolic link `link` and an
    /// asynchronous source reader on it.
    ///
    /// # Errors
    /// The failing call with its `HRESULT`.
    fn open(link: &str) -> std::result::Result<Self, (HRESULT, ScanError)> {
        let fail = |call: &str, e: windows::core::Error| (e.code(), hr_error(call, e.code()));
        let api = api().map_err(|e| (HRESULT(0), e))?;
        let attributes = api.video_capture(2).map_err(|e| (HRESULT(0), e))?;
        let wide: Vec<u16> = link.encode_utf16().chain([0]).collect();
        // SAFETY: a NUL-terminated string alive for the call (copied).
        unsafe {
            attributes.SetString(
                &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK,
                windows::core::PCWSTR(wide.as_ptr()),
            )
        }
        .map_err(|e| fail("IMFAttributes::SetString", e))?;
        let mut raw = std::ptr::null_mut();
        // SAFETY: valid attributes and out pointer; on success `raw` owns
        // one reference to an `IMFMediaSource`.
        unsafe { (api.create_device_source)(attributes.as_raw(), &mut raw) }
            .ok()
            .map_err(|e| fail("MFCreateDeviceSource", e))?;
        // SAFETY: a non-null owned `IMFMediaSource` pointer.
        let source = unsafe { IMFMediaSource::from_raw(raw) };
        let shutdown = |e| {
            // SAFETY: shuts the source down on the error path.
            let _ = unsafe { source.Shutdown() };
            e
        };
        let inbox = Arc::new(Inbox::default());
        let callback: IMFSourceReaderCallback = Callback {
            inbox: inbox.clone(),
        }
        .into();
        let reader_attributes = api.attributes(2).map_err(|e| shutdown((HRESULT(0), e)))?;
        // SAFETY: valid GUID pointer and a live callback.
        unsafe {
            reader_attributes
                .SetUnknown(&MF_SOURCE_READER_ASYNC_CALLBACK, &callback)
                .and_then(|()| reader_attributes.SetUINT32(&MF_READWRITE_DISABLE_CONVERTERS, 1))
        }
        .map_err(|e| shutdown(fail("IMFAttributes::SetUnknown", e)))?;
        let mut raw = std::ptr::null_mut();
        // SAFETY: live source and attributes, valid out pointer; on success
        // `raw` owns one reference to an `IMFSourceReader`.
        unsafe {
            (api.create_source_reader)(source.as_raw(), reader_attributes.as_raw(), &mut raw)
        }
        .ok()
        .map_err(|e| shutdown(fail("MFCreateSourceReaderFromMediaSource", e)))?;
        // SAFETY: a non-null owned `IMFSourceReader` pointer.
        let reader = unsafe { IMFSourceReader::from_raw(raw) };
        // only the video stream
        // SAFETY: plain COM calls on a live reader.
        unsafe {
            reader
                .SetStreamSelection(ALL_STREAMS, false)
                .and_then(|()| reader.SetStreamSelection(VIDEO_STREAM, true))
        }
        .map_err(|e| shutdown(fail("IMFSourceReader::SetStreamSelection", e)))?;
        Ok(Self {
            source,
            reader,
            inbox,
            format: None,
            streaming: false,
        })
    }

    /// The native formats of the video stream.
    fn native_types(&self) -> Result<Vec<Native>> {
        let mut types = Vec::new();
        for index in 0.. {
            // SAFETY: plain COM call on a live reader.
            let media_type = match unsafe { self.reader.GetNativeMediaType(VIDEO_STREAM, index) } {
                Ok(t) => t,
                Err(e) if e.code() == MF_E_NO_MORE_TYPES => break,
                Err(e) => return Err(hr_error("GetNativeMediaType", e.code())),
            };
            // SAFETY: valid GUID pointers on a live media type.
            let (subtype, size, rate, sample_size) = unsafe {
                (
                    media_type.GetGUID(&MF_MT_SUBTYPE),
                    media_type.GetUINT64(&MF_MT_FRAME_SIZE),
                    media_type.GetUINT64(&MF_MT_FRAME_RATE).unwrap_or(0),
                    media_type.GetUINT32(&MF_MT_SAMPLE_SIZE).ok(),
                )
            };
            let (Ok(subtype), Ok(size)) = (subtype, size) else {
                continue;
            };
            let (width, height) = ((size >> 32) as u32, size as u32);
            let (num, den) = ((rate >> 32) as u32, rate as u32);
            types.push(Native {
                media_type,
                subtype,
                width,
                height,
                fps: if den == 0 {
                    0.0
                } else {
                    f64::from(num) / f64::from(den)
                },
                // 16-bit formats: two bytes per pixel when not stated
                sample_size: sample_size
                    .map_or(width as usize * height as usize * 2, |s| s as usize),
            });
        }
        Ok(types)
    }

    fn set_type(&mut self, native: &Native) -> Result<()> {
        // SAFETY: plain COM call on a live reader with a native type.
        unsafe {
            self.reader
                .SetCurrentMediaType(VIDEO_STREAM, None, &native.media_type)
        }
        .map_err(|e| hr_error("SetCurrentMediaType", e.code()))
    }

    /// Sets a `width x height` native format of `fourcc` (the fastest of
    /// its frame rates); returns the image size in bytes. `Ok(None)` when
    /// the source does not offer it.
    pub fn set_format(&mut self, width: u32, height: u32, fourcc: u32) -> Result<Option<usize>> {
        let subtype = fourcc_subtype(fourcc);
        let types = self.native_types()?;
        let Some(best) = types
            .iter()
            .filter(|t| t.subtype == subtype && t.width == width && t.height == height)
            .max_by(|a, b| a.fps.total_cmp(&b.fps))
        else {
            return Ok(None);
        };
        self.set_type(best)?;
        self.format = Some((subtype, width, height));
        Ok(Some(best.sample_size))
    }

    /// Picks, among the native types of the format set by
    /// [`Capture::set_format`], the frame rate closest to `fps`.
    pub fn set_fps(&mut self, fps: u32) -> Result<()> {
        let Some((subtype, width, height)) = self.format else {
            return Ok(());
        };
        if fps == 0 {
            return Ok(());
        }
        let types = self.native_types()?;
        let wanted = f64::from(fps);
        if let Some(best) = types
            .iter()
            .filter(|t| t.subtype == subtype && t.width == width && t.height == height)
            .min_by(|a, b| (a.fps - wanted).abs().total_cmp(&(b.fps - wanted).abs()))
        {
            self.set_type(best)?;
        }
        Ok(())
    }

    /// Starts streaming (the first sample request). Media Foundation
    /// manages the driver buffers itself: `_buffers` is not used.
    pub fn start(&mut self, _buffers: u32) -> Result<()> {
        if self.streaming {
            return Ok(());
        }
        self.request()?;
        self.streaming = true;
        Ok(())
    }

    /// Issues a sample request unless one is outstanding or unread.
    fn request(&self) -> Result<()> {
        {
            let mut state = self.inbox.lock();
            if state.pending || state.ready.is_some() {
                return Ok(());
            }
            state.pending = true;
        }
        // SAFETY: plain COM call on a live asynchronous reader (all out
        // parameters null, as asynchronous mode requires).
        if let Err(e) = unsafe {
            self.reader
                .ReadSample(VIDEO_STREAM, 0, None, None, None, None)
        } {
            self.inbox.lock().pending = false;
            return Err(hr_error("ReadSample", e.code()));
        }
        Ok(())
    }

    /// Waits at most `timeout` for a frame and moves it into `out`.
    /// `Ok(None)` when no frame came in time.
    ///
    /// # Errors
    /// `device-lost` when the device is gone, [`ScanError::Device`] for
    /// other failures.
    pub fn next(&mut self, timeout: Duration, out: &mut Vec<u8>) -> Result<Option<Dequeued>> {
        if !self.streaming {
            return Err(ScanError::Device("the video stream is not started".into()));
        }
        self.request()?;
        let state = self.inbox.lock();
        let (mut state, _) = self
            .inbox
            .changed
            .wait_timeout_while(state, timeout, |s| s.ready.is_none())
            .unwrap_or_else(|e| e.into_inner());
        match state.ready.take() {
            None | Some(Ready::Nothing) => Ok(None),
            Some(Ready::Failed(e)) => Err(e),
            Some(Ready::Frame {
                bytes,
                timestamp_us,
            }) => {
                std::mem::swap(out, &mut state.data);
                Ok(Some(Dequeued {
                    bytes,
                    timestamp_us,
                }))
            }
        }
    }

    /// Stops streaming: drains the reader (errors ignored: the device may
    /// already be gone). The source is shut down on drop.
    pub fn stop(&mut self) {
        if !self.streaming {
            return;
        }
        self.streaming = false;
        self.inbox.lock().flushed = false;
        // SAFETY: plain COM call on a live asynchronous reader.
        if unsafe { self.reader.Flush(ALL_STREAMS) }.is_ok() {
            let state = self.inbox.lock();
            let _ = self
                .inbox
                .changed
                .wait_timeout_while(state, FLUSH_TIMEOUT, |s| !s.flushed);
        }
    }
}

impl Drop for Capture {
    fn drop(&mut self) {
        self.stop();
        // SAFETY: shuts the device source down (closes the camera); the
        // reader is released afterwards with the fields.
        let _ = unsafe { self.source.Shutdown() };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn y16_subtype_and_error_mapping() {
        assert_eq!(
            fourcc_subtype(PIX_FMT_Y16),
            GUID::from_u128(0x2036_3159_0000_0010_8000_00aa_0038_9b71)
        );
        assert_eq!(
            hr_error("x", MF_E_VIDEO_RECORDING_DEVICE_INVALIDATED).code(),
            "device-lost"
        );
        assert_eq!(hr_error("x", win32(1167)).code(), "device-lost");
        // seen when the device left right after it was opened
        assert_eq!(hr_error("x", win32(6)).code(), "device-lost");
        let busy = hr_error("x", MF_E_VIDEO_RECORDING_DEVICE_PREEMPTED).to_string();
        assert!(busy.contains("another program"), "{busy}");
        let denied = hr_error("x", ACCESS_DENIED).to_string();
        assert!(denied.contains("privacy"), "{denied}");
        assert_eq!(win32(5), HRESULT(0x8007_0005_u32 as i32));
    }
}
