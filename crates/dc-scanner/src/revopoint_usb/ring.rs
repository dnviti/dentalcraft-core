//! The newest frames of a stream whose driver hands them over by callback
//! (Media Foundation, [`super::mf`]): a queue of at most `capacity` frames
//! in which the oldest one makes room for a new one, so that the caller,
//! draining it as it drains the V4L2 queue, ends on the newest frame.
//! Buffers are reused: once the stream runs, no frame allocates.

use std::collections::VecDeque;

/// Frames handed over and not taken yet, oldest first.
#[derive(Debug, Default)]
pub(super) struct FrameRing {
    /// (bytes, timestamp in µs) of each frame.
    frames: VecDeque<(Vec<u8>, u64)>,
    /// Buffers of taken or dropped frames.
    spare: Vec<Vec<u8>>,
    capacity: usize,
}

impl FrameRing {
    /// Keeps at most `capacity` frames (at least one).
    pub(super) fn set_capacity(&mut self, capacity: usize) {
        self.capacity = capacity.max(1);
        while self.frames.len() > self.capacity {
            self.drop_oldest();
        }
    }

    /// Adds the frame of time `timestamp_us` that `fill` writes into a
    /// reused buffer, after dropping the oldest frame when the ring is full;
    /// nothing is added when `fill` fails.
    pub(super) fn push_with<E>(
        &mut self,
        timestamp_us: u64,
        fill: impl FnOnce(&mut Vec<u8>) -> Result<(), E>,
    ) -> Result<(), E> {
        if self.frames.len() >= self.capacity.max(1) {
            self.drop_oldest();
        }
        let mut data = self.spare.pop().unwrap_or_default();
        match fill(&mut data) {
            Ok(()) => {
                self.frames.push_back((data, timestamp_us));
                Ok(())
            }
            Err(e) => {
                self.spare.push(data);
                Err(e)
            }
        }
    }

    /// Moves the oldest frame into `out` (the buffer `out` held is kept for
    /// reuse); the frame's timestamp, `None` when there is no frame.
    pub(super) fn pop_into(&mut self, out: &mut Vec<u8>) -> Option<u64> {
        let (data, timestamp_us) = self.frames.pop_front()?;
        let taken = std::mem::replace(out, data);
        if self.spare.len() < self.capacity {
            self.spare.push(taken);
        }
        Some(timestamp_us)
    }

    /// `true` when no frame waits.
    pub(super) fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }

    /// Drops every frame (their buffers are kept).
    pub(super) fn clear(&mut self) {
        while !self.frames.is_empty() {
            self.drop_oldest();
        }
    }

    fn drop_oldest(&mut self) {
        if let Some((data, _)) = self.frames.pop_front() {
            self.spare.push(data);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn push(ring: &mut FrameRing, t: u64) {
        ring.push_with(t, |b: &mut Vec<u8>| -> Result<(), ()> {
            b.clear();
            b.extend_from_slice(&t.to_le_bytes());
            Ok(())
        })
        .unwrap();
    }

    #[test]
    fn the_oldest_frame_makes_room_and_buffers_are_reused() {
        let mut ring = FrameRing::default();
        ring.set_capacity(2);
        let mut out = Vec::new();
        assert_eq!(ring.pop_into(&mut out), None);
        // a slow reader: frames 1 and 2 are dropped for 3 and 4
        for t in 1..=4 {
            push(&mut ring, t);
        }
        assert_eq!(ring.pop_into(&mut out), Some(3));
        assert_eq!(out, 3u64.to_le_bytes());
        assert_eq!(ring.pop_into(&mut out), Some(4));
        assert_eq!(out, 4u64.to_le_bytes());
        assert!(ring.is_empty());
        // steady state: the buffers circulate, nothing is allocated
        let held = out.as_ptr();
        push(&mut ring, 5);
        push(&mut ring, 6);
        assert_eq!(ring.pop_into(&mut out), Some(5));
        assert_eq!(ring.pop_into(&mut out), Some(6));
        push(&mut ring, 7);
        push(&mut ring, 8);
        ring.pop_into(&mut out);
        ring.pop_into(&mut out);
        let seen: Vec<*const u8> = (9..=12)
            .map(|t| {
                push(&mut ring, t);
                ring.pop_into(&mut out);
                out.as_ptr()
            })
            .collect();
        assert!(seen.contains(&held), "the first buffer comes back");
        assert!(ring.spare.len() <= 2 && ring.frames.is_empty());
    }

    #[test]
    fn a_failed_frame_adds_nothing_and_clear_keeps_the_buffers() {
        let mut ring = FrameRing::default();
        ring.set_capacity(3);
        push(&mut ring, 1);
        assert_eq!(ring.push_with(2, |_| Err("short")), Err("short"));
        push(&mut ring, 3);
        ring.clear();
        assert!(ring.is_empty());
        assert_eq!(ring.spare.len(), 2);
        // a smaller capacity drops the oldest frames
        for t in 4..=6 {
            push(&mut ring, t);
        }
        ring.set_capacity(1);
        let mut out = Vec::new();
        assert_eq!(ring.pop_into(&mut out), Some(6));
        assert!(ring.is_empty());
    }
}
