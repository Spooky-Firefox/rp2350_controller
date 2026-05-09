//! Inter-core communication types and SIO FIFO primitives.
#![allow(unsafe_code)] // FifoTx/FifoRx use direct PAC register access; see safety comments.
//!
//! # Protocol
//!
//! Data is exchanged via `heapless::spsc` queues.  The SIO FIFO carries only
//! 1-word [`IpcSignal`] values that tell the receiving core which queue has
//! new data.
//!
//! | Direction       | Signal                    | Payload queue        |
//! |-----------------|---------------------------|----------------------|
//! | Core 0 → Core 1 | `IpcSignal::SensorReady`  | `SensorEvent` SPSC   |
//! | Core 1 → Core 0 | `IpcSignal::ControlReady` | `ControlOutput` SPSC |
//! | Core 1 → Core 0 | `IpcSignal::LogReady`     | Core 1 log SPSC      |
//!
//! The 32-bit timestamp in [`SensorEvent`] wraps every ~71.6 min; use
//! [`TimeExtender`] on the receiver to reconstruct a monotonic `u64`.
//!
//! ## Why `FifoTx` / `FifoRx` are split
//!
//! On Core 0, `FifoTx` and `FifoRx` live in separate RTIC shared resources so
//! they get different priority ceilings:
//! - `FifoTx` ceiling = 3 (shared with `gpio_interrupt` / `sensor_timeout`)
//! - `FifoRx` ceiling = 4 (exclusive to `sio_interrupt`)
//!
//! This ensures `sio_interrupt` (priority 4) is never ceiling-blocked by a
//! lower-priority task that holds the TX lock, preventing inter-core FIFO
//! deadlocks.

use rp235x_hal::sio::SioFifo;
use rp235x_pac;

// ─────────────────────────────────────────────────────────────────────────────
// IPC signal words
// ─────────────────────────────────────────────────────────────────────────────

/// One-word signal sent over the SIO FIFO to notify the other core that new
/// data is available in the corresponding SPSC queue.
#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IpcSignal {
    /// Core 0 → Core 1: a [`SensorEvent`] has been pushed to the sensor queue.
    SensorReady = 1,
    /// Core 1 → Core 0: a [`ControlOutput`] has been pushed to the control queue.
    ControlReady = 2,
    /// Core 1 → Core 0: a log item has been pushed to the Core 1 log queue.
    LogReady = 3,
}

impl IpcSignal {
    pub fn from_u32(w: u32) -> Option<Self> {
        match w {
            1 => Some(IpcSignal::SensorReady),
            2 => Some(IpcSignal::ControlReady),
            3 => Some(IpcSignal::LogReady),
            _ => None,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Sensor event (Core 0 → Core 1 via SPSC)
// ─────────────────────────────────────────────────────────────────────────────

/// Sensor snapshot produced by Core 0 and consumed by Core 1.
#[derive(Clone, Copy, Debug)]
pub struct SensorEvent {
    /// 32-bit microsecond timestamp (lower 32 bits of `MainMono::now()`).
    pub t32_us: u32,
    /// Speed setpoint [m/s] for the straight-line speed controller.
    pub setpoint_mps: f32,
    /// Up to 4 sensor values.  Unused slots are `f32::INFINITY`.
    pub values: [f32; 4],
}

impl SensorEvent {
    pub fn rpm_and_steer(
        timestamp_us: u64,
        setpoint_mps: f32,
        steer: f32,
        rpm_period_us: f32,
    ) -> Self {
        Self {
            t32_us: timestamp_us as u32,
            setpoint_mps,
            values: [steer, rpm_period_us, f32::INFINITY, f32::INFINITY],
        }
    }

    pub fn steer_only_timeout(timestamp_us: u64, setpoint_mps: f32, steer: f32) -> Self {
        Self {
            t32_us: timestamp_us as u32,
            setpoint_mps,
            values: [steer, f32::INFINITY, f32::INFINITY, f32::INFINITY],
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Control output (Core 1 → Core 0 via SPSC)
// ─────────────────────────────────────────────────────────────────────────────

/// Control output produced by Core 1 and consumed by Core 0 to drive the PWM.
#[derive(Clone, Copy, Debug)]
pub struct ControlOutput {
    pub steer_pwm_us: u16,
    pub power_pwm_us: u16,
}

// ─────────────────────────────────────────────────────────────────────────────
// 32-bit → 64-bit timestamp extension
// ─────────────────────────────────────────────────────────────────────────────

/// Reconstructs a monotonic `u64` µs timestamp from 32-bit wrapped values.
///
/// Works correctly as long as consecutive events are less than ~71.6 min apart.
#[derive(Clone, Copy)]
pub struct TimeExtender {
    initialized: bool,
    last_t32: u32,
    t64: u64,
}

impl TimeExtender {
    pub const fn new() -> Self {
        Self {
            initialized: false,
            last_t32: 0,
            t64: 0,
        }
    }

    /// Feed a 32-bit timestamp; returns the monotonic u64 µs value.
    pub fn extend(&mut self, t32: u32) -> u64 {
        if !self.initialized {
            self.initialized = true;
            self.last_t32 = t32;
            self.t64 = t32 as u64;
            return self.t64;
        }
        let dt = t32.wrapping_sub(self.last_t32);
        self.last_t32 = t32;
        self.t64 += dt as u64;
        self.t64
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Core 0 split FIFO halves
// ─────────────────────────────────────────────────────────────────────────────

/// Core 0 write-only FIFO half: Core 0 → Core 1.
///
/// Shared only with tasks at priority ≤ 3, giving it an RTIC priority ceiling
/// of 3.  `sio_interrupt` (priority 4) can always preempt any task that holds
/// this lock, keeping Core 1's reply path unblocked.
pub struct FifoTx;

impl FifoTx {
    fn write_word_blocking(value: u32) {
        // SAFETY: Only Core 0 writes to fifo_wr; no concurrent writers exist.
        let sio = unsafe { &(*rp235x_pac::SIO::ptr()) };
        while !sio.fifo_st().read().rdy().bit_is_set() {
            cortex_m::asm::nop();
        }
        sio.fifo_wr().write(|w| unsafe { w.bits(value) });
        cortex_m::asm::sev();
    }

    /// Send a single [`IpcSignal`] word to Core 1 (blocking until FIFO has space).
    pub fn signal(&mut self, sig: IpcSignal) {
        Self::write_word_blocking(sig as u32);
    }
}

/// Core 0 read-only FIFO half: Core 1 → Core 0.
///
/// Used exclusively by `sio_interrupt` (priority 4).  Its RTIC priority
/// ceiling is therefore 4, which allows `sio_interrupt` to preempt any task
/// holding `FifoTx` (ceiling 3).
pub struct FifoRx;

impl FifoRx {
    /// Try to read one word without blocking.  Returns `None` if the FIFO is
    /// empty.
    pub fn try_read(&mut self) -> Option<u32> {
        // SAFETY: Only Core 0 reads from fifo_rd; no concurrent readers.
        let sio = unsafe { &(*rp235x_pac::SIO::ptr()) };
        if sio.fifo_st().read().vld().bit_is_set() {
            Some(sio.fifo_rd().read().bits())
        } else {
            None
        }
    }

    /// Discard all words currently in the RX FIFO.
    pub fn drain(&mut self) {
        // SAFETY: Only Core 0 reads from fifo_rd.
        let sio = unsafe { &(*rp235x_pac::SIO::ptr()) };
        while sio.fifo_st().read().vld().bit_is_set() {
            let _ = sio.fifo_rd().read().bits();
        }
    }
}

/// Consume the Core 0 [`SioFifo`] and produce separate [`FifoTx`] / [`FifoRx`]
/// halves that live in independent RTIC shared resources, giving them distinct
/// priority ceilings.
pub fn split_fifo(fifo: SioFifo) -> (FifoTx, FifoRx) {
    // The HAL SioFifo is consumed and dropped; FifoTx/FifoRx access the same
    // hardware via direct PAC register reads/writes.
    drop(fifo);
    (FifoTx, FifoRx)
}

impl Default for TimeExtender {
    fn default() -> Self {
        Self::new()
    }
}
