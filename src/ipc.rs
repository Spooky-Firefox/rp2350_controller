//! Inter-core communication types and SIO FIFO primitives.
#![allow(unsafe_code)] // FifoTx/FifoRx use direct PAC register access; see safety comments.
//!
//! # SensorEvent (Core 0 → Core 1): 6 × 32-bit FIFO words
//!
//! | Word | Content |
//! |------|---------|
//! | 0    | `timestamp_us` (lower 32 bits from `MainMono::now()`) |
//! | 1    | `setpoint_mps` as `f32::to_bits()` |
//! | 2    | `values[0]` as `f32::to_bits()` |
//! | 3    | `values[1]` as `f32::to_bits()` |
//! | 4    | `values[2]` as `f32::to_bits()` |
//! | 5    | `values[3]` as `f32::to_bits()` |
//!
//! Unused value slots are set to `f32::INFINITY`.
//! The 32-bit timestamp wraps every ~71.6 min; use [`TimeExtender`] on the
//! receiver to reconstruct a monotonic `u64`.
//!
//! # ControlEvent (Core 1 → Core 0): 1 × 32-bit FIFO word
//!
//! | Word | Content |
//! |------|---------|
//! | 0    | `[steer_pwm_us: u16 | power_pwm_us: u16]` |

use rp235x_hal::sio::SioFifo;
use rp235x_pac;

/// Sensor event packed as 6 × u32 words for SIO FIFO transfer.
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

    /// Encode into 6 FIFO words.
    pub fn to_words(&self) -> [u32; 6] {
        [
            self.t32_us,
            self.setpoint_mps.to_bits(),
            self.values[0].to_bits(),
            self.values[1].to_bits(),
            self.values[2].to_bits(),
            self.values[3].to_bits(),
        ]
    }

    /// Decode from 6 FIFO words.
    pub fn from_words(words: [u32; 6]) -> Self {
        Self {
            t32_us: words[0],
            setpoint_mps: f32::from_bits(words[1]),
            values: [
                f32::from_bits(words[2]),
                f32::from_bits(words[3]),
                f32::from_bits(words[4]),
                f32::from_bits(words[5]),
            ],
        }
    }
}

/// Control output from Core 1 → Core 0: two PWM on-times packed into one word.
#[derive(Clone, Copy, Debug)]
pub enum ControlEvent {
    // control signal
    Control {
        steer_pwm_us: u16,
        power_pwm_us: u16,
    },
    Pid {
        error: f32,
        proportional: f32,
        integral: f32,
        derivative: f32,
    },
    KalmanDebug {
        x: [f32; 4],
    },
}

impl ControlEvent {
    pub fn to_words(&self) -> heapless::Vec<u32, 6> {
        match self {
            ControlEvent::Control {
                steer_pwm_us,
                power_pwm_us,
            } => heapless::Vec::from_slice(&[
                1,
                (*steer_pwm_us as u32) << 16 | (*power_pwm_us as u32),
            ])
            .unwrap(),
            ControlEvent::Pid {
                error,
                proportional,
                integral,
                derivative,
            } => heapless::Vec::from_slice(&[
                2,
                error.to_bits(),
                proportional.to_bits(),
                integral.to_bits(),
                derivative.to_bits(),
            ])
            .unwrap(),
            ControlEvent::KalmanDebug { x } => heapless::Vec::from_slice(&[
                3,
                x[0].to_bits(),
                x[1].to_bits(),
                x[2].to_bits(),
                x[3].to_bits(),
            ])
            .unwrap(),
        }
    }

    pub fn from_fifo_chanel_blocking(channel: &mut FifoChannel) -> Self {
        Self::read_words(|| channel.fifo.read_blocking())
    }

    /// Receive a [`ControlEvent`] from Core 1 using the split [`FifoRx`] half.
    /// Used by `sio_interrupt` on Core 0.
    pub fn from_fifo_rx_blocking(rx: &mut FifoRx) -> Self {
        Self::read_words(|| rx.read_word_blocking())
    }

    fn read_words(mut read: impl FnMut() -> u32) -> Self {
        match read() {
            1 => {
                let w = read();
                let steer_pwm_us = ((w >> 16) & 0xFFFF) as u16;
                let power_pwm_us = (w & 0xFFFF) as u16;
                ControlEvent::Control {
                    steer_pwm_us,
                    power_pwm_us,
                }
            }
            2 => {
                let error = f32::from_bits(read());
                let proportional = f32::from_bits(read());
                let integral = f32::from_bits(read());
                let derivative = f32::from_bits(read());
                ControlEvent::Pid {
                    error,
                    proportional,
                    integral,
                    derivative,
                }
            }
            3 => {
                let x0 = f32::from_bits(read());
                let x1 = f32::from_bits(read());
                let x2 = f32::from_bits(read());
                let x3 = f32::from_bits(read());
                ControlEvent::KalmanDebug {
                    x: [x0, x1, x2, x3],
                }
            }
            _ => panic!("Unknown control event type received from Core 1"),
        }
    }
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
/// Shared only with tasks that send sensor events (max priority 3), so its
/// RTIC priority ceiling is 3. That means `sio_interrupt` (priority 4) can
/// always preempt a task holding this lock — keeping Core 1's reply path
/// unblocked and preventing the inter-core FIFO deadlock.
pub struct FifoTx;

impl FifoTx {
    fn is_write_ready() -> bool {
        // SAFETY: read-only status register, no aliasing concerns.
        let sio = unsafe { &(*rp235x_pac::SIO::ptr()) };
        sio.fifo_st().read().rdy().bit_is_set()
    }

    fn write_word(value: u32) {
        // SAFETY: Only Core 0 writes to fifo_wr; no concurrent writers exist.
        let sio = unsafe { &(*rp235x_pac::SIO::ptr()) };
        sio.fifo_wr().write(|w| unsafe { w.bits(value) });
        cortex_m::asm::sev();
    }

    fn write_word_blocking(value: u32) {
        while !Self::is_write_ready() {
            cortex_m::asm::nop();
        }
        Self::write_word(value);
    }

    /// Send a [`SensorEvent`] (6 blocking writes) to Core 1.
    ///
    /// It is safe to block here because `sio_interrupt` (priority 4) can
    /// preempt any task holding `FifoTx` (ceiling 3) to drain the
    /// Core1 → Core0 FIFO, keeping Core 1 unblocked.
    pub fn send_sensor_event(&mut self, event: &SensorEvent) {
        for &w in &event.to_words() {
            Self::write_word_blocking(w);
        }
    }
}

/// Core 0 read-only FIFO half: Core 1 → Core 0.
///
/// Used exclusively by `sio_interrupt` (priority 4). Its RTIC priority
/// ceiling is therefore 4, which is what allows `sio_interrupt` to preempt
/// any task holding `FifoTx` (ceiling 3).
pub struct FifoRx {
    rx_count: usize,
}

impl FifoRx {
    fn new() -> Self {
        Self {
            rx_count: 0,
        }
    }

    pub fn have_data(&mut self) -> bool {
        // SAFETY: read-only status register.
        let sio = unsafe { &(*rp235x_pac::SIO::ptr()) };
        sio.fifo_st().read().vld().bit_is_set()
    }

    pub fn read_word_blocking(&mut self) -> u32 {
        loop {
            // SAFETY: Only Core 0 reads from fifo_rd; no concurrent readers.
            let sio = unsafe { &(*rp235x_pac::SIO::ptr()) };
            if sio.fifo_st().read().vld().bit_is_set() {
                return sio.fifo_rd().read().bits();
            }
            cortex_m::asm::nop();
        }
    }

    pub fn drain(&mut self) {
        loop {
            // SAFETY: Only Core 0 reads from fifo_rd.
            let sio = unsafe { &(*rp235x_pac::SIO::ptr()) };
            if sio.fifo_st().read().vld().bit_is_set() {
                let _ = sio.fifo_rd().read().bits();
            } else {
                break;
            }
        }
        self.rx_count = 0;
    }
}

/// Consume the Core 0 [`SioFifo`] and produce separate [`FifoTx`] / [`FifoRx`]
/// halves that live in independent RTIC shared resources, giving them distinct
/// priority ceilings and breaking the potential inter-core FIFO deadlock.
pub fn split_fifo(fifo: SioFifo) -> (FifoTx, FifoRx) {
    // The HAL SioFifo is consumed and dropped; FifoTx/FifoRx access the same
    // hardware via direct PAC register reads/writes.
    drop(fifo);
    (FifoTx, FifoRx::new())
}

impl Default for TimeExtender {
    fn default() -> Self {
        Self::new()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Owned SIO FIFO channel (no unsafe, uses rp235x-hal)
// ─────────────────────────────────────────────────────────────────────────────

/// Owns the SIO FIFO and provides typed inter-core message passing.
///
/// Hardware routes reads/writes based on the executing core, so each core
/// constructs its own `FifoChannel` from its `SioFifo` handle.
pub struct FifoChannel {
    fifo: SioFifo,
    rx_buf: [u32; 6],
    rx_count: usize,
}

impl FifoChannel {
    pub fn new(fifo: SioFifo) -> Self {
        Self {
            fifo,
            rx_buf: [0u32; 6],
            rx_count: 0,
        }
    }
    pub fn have_data(&mut self) -> bool {
        self.fifo.is_read_ready()
    }

    /// Discard all data currently in the RX FIFO.
    pub fn drain(&mut self) {
        self.fifo.drain();
    }



    /// Send a complete [`ControlEvent`] (1 blocking write).
    pub fn send_control_event_blocking(&mut self, event: &ControlEvent) {
        for &w in &event.to_words() {
            self.fifo.write_blocking(w);
        }
    }

    /// Accumulate FIFO words into the internal 6-word buffer.
    /// Returns `Some` when all 6 words have been collected.
    pub fn try_recv_sensor_event(&mut self) -> Option<SensorEvent> {
        while self.rx_count < 6 {
            match self.fifo.read() {
                Some(w) => {
                    self.rx_buf[self.rx_count] = w;
                    self.rx_count += 1;
                }
                None => return None,
            }
        }
        self.rx_count = 0;
        Some(SensorEvent::from_words(self.rx_buf))
    }
}
