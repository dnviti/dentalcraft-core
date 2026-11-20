//! The scanner device, shared by the platforms: discovery by serial, open,
//! streaming, stop and reopening after the re-enumeration (see the module
//! doc of [`super`]). The platform layer ([`super::platform`]) finds the
//! devices and their depth capture node and streams that node.

use std::time::{Duration, Instant};

use super::calib::{Calibration, CalibrationSource, load_calibration};
use super::exposure::ExposureProfile;
use super::factory;
use super::frame::{self, StreamMode};
use super::light::FillLight;
use super::platform::{self, Address, Capture, PIX_FMT_Y16, find_devices, open_capture};
use super::xu::{self, ShellLine};
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
        let (mut calibration, mut calibration_source) = load_calibration()?;
        // the node must open; it is closed again until `start`
        let idle = open_capture(&node).map_err(|(_, e)| e)?;
        // the vendor unit, checked before any request reaches it
        let unit = platform::vendor_unit(&found, &idle);
        let mut calibration_note = None;
        // no user file: the factory calibration, cached or read now (the
        // stream is idle, as the file channel requires)
        if calibration_source == CalibrationSource::Defaults {
            match found.info.serial.as_deref().and_then(factory::cache_dir) {
                Some(dir) => {
                    let read = |file| match &unit {
                        Ok(()) => xu::read_file(&idle, file),
                        Err(why) => Err(ScanError::Device(format!(
                            "{why}: the factory calibration is not read"
                        ))),
                    };
                    match factory::load_or_read(&dir, read) {
                        Ok((c, path)) => {
                            calibration = c;
                            calibration_source = CalibrationSource::Factory(path);
                        }
                        Err(e) => calibration_note = Some(e.to_string()),
                    }
                }
                None => {
                    calibration_note =
                        Some("no serial number or data directory to cache the factory file".into());
                }
            }
        }
        // idle, the device blinks its LEDs: off until a stream turns them on
        if unit.is_ok() {
            let _ = xu::send_lines(&idle, FillLight::Off.lines(), "light command");
        }
        drop(idle);
        Ok(Box::new(RevopointUsbDevice {
            info: found.info,
            options: self.options.clone(),
            calibration,
            calibration_source,
            calibration_note,
            address: found.address,
            capture: None,
            mode: self.options.stream_mode(),
            restart: None,
            reconnect_deadline: None,
            buf: Vec::new(),
            spare: Vec::new(),
            latest_ir: None,
            fill_light: true,
            unit,
            light_on: false,
            light_note: None,
            fast_exposure: false,
            exposure_note: None,
            last_frame_us: None,
            period: None,
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
    /// Where `calibration` comes from.
    calibration_source: CalibrationSource,
    /// Why the factory calibration is not in use, when it is not.
    calibration_note: Option<String>,
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
    /// The current enumeration has the vendor extension unit (checked by
    /// the platform, see [`super::xu`]); why not, when it has not.
    unit: std::result::Result<(), String>,
    /// The fill light was turned on for the current stream.
    light_on: bool,
    /// Why the fill light was not driven, once.
    light_note: Option<String>,
    /// Apply [`ExposureProfile::Fast`] when a stream starts.
    fast_exposure: bool,
    /// Why the exposure profile was not applied, once.
    exposure_note: Option<String>,
    /// Driver timestamp of the latest complete frame dequeued (µs).
    last_frame_us: Option<u64>,
    /// Smoothed frame period of the stream, once two frames came.
    period: Option<Duration>,
}

/// Latency of a frame beyond its period, and in frame periods: a MINI 2
/// frame shows the scene `LATENCY_BASE + LATENCY_PERIODS × period` before
/// it arrives (against the turntable's own angle estimate). Fitted on the
/// rig: 0.7 to 1.1 s at 2.74 frames per second (2026-11-18), 0.35 to 0.40 s
/// at 11.2 (fast exposure, 2026-11-19).
const LATENCY_BASE: Duration = Duration::from_millis(200);
const LATENCY_PERIODS: f32 = 1.9;

/// The latency of a frame of a stream with frame period `period` (see
/// [`LATENCY_BASE`]); `fallback` before the period is known.
fn latency_of(period: Option<Duration>, fallback: Duration) -> Duration {
    period.map_or(fallback, |p| LATENCY_BASE + p.mul_f32(LATENCY_PERIODS))
}

/// Updates the smoothed frame `period` with a frame dequeued at
/// `timestamp_us` (driver clock), `last_us` being the previous one; gaps
/// outside 5 ms to 1 s (a restart, a stall) do not count.
fn note_frame(last_us: &mut Option<u64>, period: &mut Option<Duration>, timestamp_us: u64) {
    if let Some(dt) = last_us
        .and_then(|last| timestamp_us.checked_sub(last))
        .filter(|dt| (5_000..=1_000_000).contains(dt))
    {
        let dt = Duration::from_micros(dt);
        *period = Some(period.map_or(dt, |p| p.mul_f32(0.9) + dt.mul_f32(0.1)));
    }
    *last_us = Some(timestamp_us);
}

impl std::fmt::Debug for RevopointUsbDevice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RevopointUsbDevice")
            .field("info", &self.info)
            .field("streaming", &self.capture.is_some())
            .field("mode", &self.mode)
            .field("fill_light", &self.fill_light)
            .field("fast_exposure", &self.fast_exposure)
            .field("vendor_unit", &self.unit.is_ok())
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

    /// Where the calibration in use comes from.
    pub fn calibration_source(&self) -> &CalibrationSource {
        &self.calibration_source
    }

    /// Why the factory calibration is not in use, when there is no user
    /// file and it could not be read or parsed.
    pub fn calibration_note(&self) -> Option<&str> {
        self.calibration_note.as_deref()
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
        let unit = platform::vendor_unit(&found, &capture);
        self.mode = mode;
        self.address = found.address;
        self.capture = Some(capture);
        self.restart = None;
        self.reconnect_deadline = None;
        self.unit = unit;
        let light = if self.fill_light {
            FillLight::On
        } else {
            FillLight::Off
        };
        self.light_on = self.send_light(light) && self.fill_light;
        if self.fast_exposure
            && let Err(e) = self.send_lines(ExposureProfile::Fast.lines(), "exposure command")
        {
            self.exposure_note.get_or_insert_with(|| e.to_string());
        }
        Ok(())
    }

    /// Sends `lines` on the open stream when the device has the vendor unit
    /// (`what` names them in the errors).
    fn send_lines(&self, lines: &[ShellLine], what: &str) -> Result<()> {
        let Some(capture) = &self.capture else {
            return Err(ScanError::Device("the depth stream is not started".into()));
        };
        match &self.unit {
            Ok(()) => xu::send_lines(capture, lines, what),
            Err(why) => Err(ScanError::Device(format!("{why}: no {what} sent"))),
        }
    }

    /// Sends `light` on the open stream; a refusal is noted once.
    fn send_light(&mut self, light: FillLight) -> bool {
        if self.capture.is_none() {
            return false;
        }
        match self.send_lines(light.lines(), "light command") {
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

    /// Why the exposure profile was not applied, when it was not (the
    /// first reason only).
    pub fn exposure_note(&self) -> Option<&str> {
        self.exposure_note.as_deref()
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
        if self.capture.is_some() && self.unit.is_ok() {
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
        // the next stream may run at another rate
        self.last_frame_us = None;
        self.period = None;
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
            imu_up: planes.header.and_then(|h| h.up()),
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
        if self.capture.is_some() && on != self.light_on {
            let sent = self.send_light(if on { FillLight::On } else { FillLight::Off });
            self.light_on = on && sent;
        }
        Ok(())
    }

    fn set_fast_exposure(&mut self, on: bool) -> Result<()> {
        // applied when the next stream starts (the registers stay set
        // across restarts until the scanner is unplugged)
        self.fast_exposure = on;
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
                note_frame(&mut self.last_frame_us, &mut self.period, timestamp_us);
                for _ in 0..self.options.buffers {
                    match capture.next(Duration::ZERO, &mut self.spare) {
                        Ok(Some(next)) if next.bytes >= full => {
                            std::mem::swap(&mut self.buf, &mut self.spare);
                            timestamp_us = next.timestamp_us;
                            note_frame(&mut self.last_frame_us, &mut self.period, timestamp_us);
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
        latency_of(self.period, self.options.latency)
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

    #[test]
    fn latency_follows_the_frame_period_measured_on_the_rig() {
        let fallback = UsbOptions::default().latency;
        assert_eq!(latency_of(None, fallback), fallback);
        let at = |fps: f64| latency_of(Some(Duration::from_secs_f64(1.0 / fps)), fallback);
        // 0.7 to 1.1 s at 2.74 fps, 0.35 to 0.40 s at 11.2 fps (fast exposure)
        assert!(
            (0.7..=1.1).contains(&at(2.74).as_secs_f64()),
            "{:?}",
            at(2.74)
        );
        assert!(
            (0.35..=0.40).contains(&at(11.2).as_secs_f64()),
            "{:?}",
            at(11.2)
        );

        // the period comes from consecutive frames; a stall does not count
        let (mut last, mut period) = (None, None);
        for k in 0..20u64 {
            note_frame(&mut last, &mut period, 1_000_000 + k * 89_000);
        }
        note_frame(&mut last, &mut period, 1_000_000 + 19 * 89_000 + 3_000_000);
        let p = period.unwrap().as_secs_f64();
        assert!((p - 0.089).abs() < 1e-4, "{p}");
    }
}
