//! The scanner device, shared by the platforms: discovery by serial, open,
//! streaming, stop and reopening after the re-enumeration (see the module
//! doc of [`super`]). The platform layer ([`super::platform`]) finds the
//! devices and their depth capture node and streams that node.

use std::time::{Duration, Instant};

use super::calib::{Calibration, load_calibration};
use super::frame::{self, StreamMode};
#[cfg(target_os = "linux")]
use super::light::FillLight;
use super::platform::{self, Address, Capture, PIX_FMT_Y16, find_devices, open_capture};
use super::{BACKEND_ID, REENUMERATION_GRACE, UsbOptions};
use crate::{
    DepthFrame, DeviceInfo, Result, RevopointUsbBackend, ScanError, ScannerBackend, ScannerDevice,
};

/// Longest sleep of one reconnection step inside `next_frame`.
const RECONNECT_STEP: Duration = Duration::from_millis(100);

impl ScannerBackend for RevopointUsbBackend {
    fn id(&self) -> &'static str {
        BACKEND_ID
    }

    fn availability(&self) -> Result<()> {
        platform::availability()
    }

    fn list(&self) -> Result<Vec<DeviceInfo>> {
        Ok(find_devices()?
            .into_iter()
            .filter(|f| f.depth_node().is_some())
            .map(|f| f.info)
            .collect())
    }

    fn open(&self, device_id: &str) -> Result<Box<dyn ScannerDevice>> {
        let found = find_devices()?
            .into_iter()
            .find(|f| f.info.id == device_id)
            .ok_or_else(|| ScanError::DeviceNotFound(device_id.to_owned()))?;
        let node = found
            .depth_node()
            .ok_or_else(|| ScanError::Device(found.no_node()))?;
        let (calibration, _) = load_calibration()?;
        // the node must open; it is closed again until `start`
        let idle = open_capture(&node).map_err(|(_, e)| e)?;
        // idle, the device blinks its LEDs: off until a stream turns them on
        #[cfg(target_os = "linux")]
        if found.has_shell_unit() {
            let _ = idle.set_fill_light(FillLight::Off);
        }
        drop(idle);
        Ok(Box::new(RevopointUsbDevice {
            info: found.info,
            options: self.options.clone(),
            calibration,
            address: found.address,
            capture: None,
            mode: self.options.stream_mode(),
            restart: None,
            reconnect_deadline: None,
            buf: Vec::new(),
            spare: Vec::new(),
            latest_ir: None,
            fill_light: true,
            shell_unit: false,
            light_on: false,
            light_note: None,
        }))
    }
}

/// A stream that ended: the device re-enumerates before it can stream
/// again.
#[derive(Clone, Debug)]
struct Ended {
    /// When the stream ended.
    at: Instant,
    /// USB address of the enumeration that streamed.
    address: Address,
}

/// A Revopoint scanner streaming over USB video (see the module doc).
pub struct RevopointUsbDevice {
    info: DeviceInfo,
    options: UsbOptions,
    calibration: Calibration,
    /// USB address of the current enumeration.
    address: Address,
    capture: Option<Capture>,
    /// The negotiated mode of `capture`.
    mode: StreamMode,
    /// Set when a stream ended: the next start waits for the device.
    restart: Option<Ended>,
    /// Set while a start waits for the device to come back.
    reconnect_deadline: Option<Instant>,
    buf: Vec<u8>,
    /// Second frame buffer, to drain the queue down to the newest frame.
    spare: Vec<u8>,
    latest_ir: Option<Vec<u8>>,
    /// Turn the fill light on while streaming (see [`super::light`]).
    fill_light: bool,
    /// The current enumeration has the vendor shell extension unit.
    shell_unit: bool,
    /// The fill light was turned on for the current stream.
    light_on: bool,
    /// Why the fill light was not driven, once.
    light_note: Option<String>,
}

impl std::fmt::Debug for RevopointUsbDevice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RevopointUsbDevice")
            .field("info", &self.info)
            .field("streaming", &self.capture.is_some())
            .field("mode", &self.mode)
            .field("fill_light", &self.fill_light)
            .field("shell_unit", &self.shell_unit)
            .field("light_on", &self.light_on)
            .finish_non_exhaustive()
    }
}

/// Why a reopen attempt did not stream yet.
enum NotYet {
    /// The device is not back (or not ready): try again.
    Wait,
    /// A real failure.
    Fail(ScanError),
}

impl RevopointUsbDevice {
    /// The calibration in use (loaded when the device was opened).
    pub fn calibration(&self) -> &Calibration {
        &self.calibration
    }

    /// The negotiated stream mode (the requested one until a stream ran).
    pub fn mode(&self) -> StreamMode {
        self.mode
    }

    /// The infrared pair of the latest frame (`first` then `second`
    /// image, 800x600 each), when [`UsbOptions::keep_ir`] is set and the
    /// mode carries it.
    pub fn latest_ir(&self) -> Option<frame::IrPair<'_>> {
        let ir = self.latest_ir.as_deref()?;
        let (first, second) = ir.split_at(ir.len() / 2);
        Some(frame::IrPair { first, second })
    }

    /// Finds the device, opens its depth node and starts streaming.
    fn try_open(&mut self) -> std::result::Result<(), NotYet> {
        let found = find_devices()
            .map_err(NotYet::Fail)?
            .into_iter()
            .find(|f| f.info.id == self.info.id)
            .ok_or(NotYet::Wait)?;
        if let Some(ended) = &self.restart
            && found.address == ended.address
            && ended.at.elapsed() < REENUMERATION_GRACE
        {
            // the old enumeration, about to leave the bus
            return Err(NotYet::Wait);
        }
        let node = found.depth_node().ok_or(NotYet::Wait)?;
        let mut capture = open_capture(&node).map_err(|(transient, e)| {
            if transient {
                NotYet::Wait
            } else {
                NotYet::Fail(e)
            }
        })?;
        let mode = self.negotiate(&mut capture).map_err(NotYet::Fail)?;
        capture
            .set_fps(self.options.fps)
            .and_then(|()| capture.start(self.options.buffers.max(2)))
            .map_err(NotYet::Fail)?;
        self.mode = mode;
        self.address = found.address;
        self.capture = Some(capture);
        self.restart = None;
        self.reconnect_deadline = None;
        #[cfg(target_os = "linux")]
        {
            self.shell_unit = found.has_shell_unit();
            let light = if self.fill_light {
                FillLight::On
            } else {
                FillLight::Off
            };
            self.light_on = self.send_light(light) && self.fill_light;
        }
        Ok(())
    }

    /// Sends `light` on the open stream when the device has the vendor
    /// shell unit; a refusal is noted once.
    #[cfg(target_os = "linux")]
    fn send_light(&mut self, light: FillLight) -> bool {
        let result = match (&self.capture, self.shell_unit) {
            (Some(capture), true) => capture.set_fill_light(light),
            (Some(_), false) => Err(ScanError::Device(
                "no extension unit 4 with the expected GUID: fill light left alone".into(),
            )),
            (None, _) => return false,
        };
        match result {
            Ok(()) => true,
            Err(e) => {
                self.light_note.get_or_insert_with(|| e.to_string());
                false
            }
        }
    }

    /// Why the fill light was not driven, when it was not (the first
    /// reason only).
    pub fn fill_light_note(&self) -> Option<&str> {
        self.light_note.as_deref()
    }

    /// Sets the requested mode, else [`StreamMode::Depth`].
    fn negotiate(&self, capture: &mut Capture) -> Result<StreamMode> {
        for mode in [self.options.stream_mode(), StreamMode::Depth] {
            let (w, h) = mode.size();
            if let Some(size) = capture.set_format(w, h, PIX_FMT_Y16)?
                && size >= mode.frame_bytes()
            {
                return Ok(mode);
            }
        }
        Err(ScanError::Device(
            "the depth node offers no Y16 800x600 format".into(),
        ))
    }

    /// The stream ended (stopped or lost): forget it, the fill light off
    /// first (left on, or blinking, it would stay so after the scan).
    fn end_stream(&mut self) {
        #[cfg(target_os = "linux")]
        if self.capture.is_some() && self.shell_unit {
            // a device that left the bus refuses it: nothing to do then
            self.send_light(FillLight::Off);
        }
        self.light_on = false;
        if let Some(mut capture) = self.capture.take() {
            capture.stop();
            #[allow(
                clippy::clone_on_copy,
                reason = "the address is `Copy` on Linux, not on Windows"
            )]
            let address = self.address.clone();
            self.restart = Some(Ended {
                at: Instant::now(),
                address,
            });
        }
        self.reconnect_deadline = None;
    }

    /// One reconnection step: waits at most `timeout`.
    fn reconnect(&mut self, timeout: Duration, deadline: Instant) -> Result<()> {
        match self.try_open() {
            Ok(()) => Ok(()),
            Err(NotYet::Fail(e)) => {
                self.reconnect_deadline = None;
                Err(e)
            }
            Err(NotYet::Wait) if Instant::now() >= deadline => {
                self.reconnect_deadline = None;
                Err(ScanError::DeviceNotFound(format!(
                    "{} did not come back on USB within {:.0} s after the stream \
                     stopped",
                    self.info.id,
                    self.options.reconnect_timeout.as_secs_f32()
                )))
            }
            Err(NotYet::Wait) => {
                std::thread::sleep(timeout.min(RECONNECT_STEP));
                Ok(())
            }
        }
    }

    fn depth_frame(&mut self, driver_timestamp_us: u64) -> Result<DepthFrame> {
        let planes = frame::split_planes(&self.buf, self.mode)?;
        if self.options.keep_ir {
            self.latest_ir = planes.ir.map(|ir| [ir.first, ir.second].concat());
        }
        Ok(DepthFrame {
            depth_mm: frame::depth_to_mm(planes.depth, self.calibration.depth_scale_mm),
            intrinsics: self.calibration.intrinsics,
            timestamp_us: planes
                .header
                .map_or(driver_timestamp_us, |h| u64::from(h.timestamp_ms) * 1000),
            color: None,
            imu_orientation: planes.header.and_then(|h| h.orientation),
        })
    }
}

impl ScannerDevice for RevopointUsbDevice {
    fn info(&self) -> DeviceInfo {
        self.info.clone()
    }

    fn start(&mut self) -> Result<()> {
        if self.capture.is_some() || self.reconnect_deadline.is_some() {
            return Ok(());
        }
        if self.restart.is_some() {
            // the device is re-enumerating: `next_frame` reopens it
            self.reconnect_deadline = Some(Instant::now() + self.options.reconnect_timeout);
            return Ok(());
        }
        match self.try_open() {
            Ok(()) => Ok(()),
            Err(NotYet::Fail(e)) => Err(e),
            Err(NotYet::Wait) => Err(ScanError::DeviceNotFound(format!(
                "{}: the scanner or its depth node is not ready",
                self.info.id
            ))),
        }
    }

    fn stop(&mut self) {
        self.end_stream();
    }

    fn set_fill_light(&mut self, on: bool) -> Result<()> {
        self.fill_light = on;
        #[cfg(target_os = "linux")]
        if self.capture.is_some() && on != self.light_on {
            let sent = self.send_light(if on { FillLight::On } else { FillLight::Off });
            self.light_on = on && sent;
        }
        Ok(())
    }

    fn next_frame(&mut self, timeout: Duration) -> Result<Option<DepthFrame>> {
        if let Some(deadline) = self.reconnect_deadline {
            self.reconnect(timeout, deadline)?;
            return Ok(None);
        }
        let Some(capture) = self.capture.as_mut() else {
            return Err(ScanError::Device("the depth stream is not started".into()));
        };
        let full = self.mode.frame_bytes();
        match capture.next(timeout, &mut self.buf) {
            Ok(Some(dq)) => {
                if dq.bytes < full {
                    // a short (incomplete) frame: skip it
                    return Ok(None);
                }
                // frames queued behind it are newer: keep the newest, so
                // the frame and the turntable angle read now stay close
                let mut timestamp_us = dq.timestamp_us;
                for _ in 0..self.options.buffers {
                    match capture.next(Duration::ZERO, &mut self.spare) {
                        Ok(Some(next)) if next.bytes >= full => {
                            std::mem::swap(&mut self.buf, &mut self.spare);
                            timestamp_us = next.timestamp_us;
                        }
                        Ok(Some(_)) => {}
                        // a failure surfaces at the next call
                        Ok(None) | Err(_) => break,
                    }
                }
                self.depth_frame(timestamp_us).map(Some)
            }
            Ok(None) => Ok(None),
            Err(e) => {
                if e.code() == "device-lost" {
                    self.end_stream();
                }
                Err(e)
            }
        }
    }

    fn latency(&self) -> Duration {
        self.options.latency
    }
}

impl Drop for RevopointUsbDevice {
    fn drop(&mut self) {
        self.end_stream();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn listing_never_fails_without_devices() {
        // whatever is plugged in, listing works and only lists 2207:110c
        let list = RevopointUsbBackend::default().list().unwrap();
        assert!(list.iter().all(|d| d.backend == BACKEND_ID));
        assert_eq!(
            RevopointUsbBackend::default()
                .open("no-such-serial")
                .err()
                .unwrap()
                .code(),
            "device-not-found"
        );
    }
}
