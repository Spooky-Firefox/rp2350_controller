//! Inter-core communication types and SIO FIFO primitives.
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

use rp235x_pac as pac;

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
    pub fn rpm_and_steer(timestamp_us: u64, setpoint_mps: f32, steer: f32, rpm_period_us: f32) -> Self {
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
pub struct ControlEvent {
    /// Steering servo on-time [µs], typically 1000–2000.
    pub steer_pwm_us: u16,
    /// Power/throttle servo on-time [µs], typically 1000–2000.
    pub power_pwm_us: u16,
}

impl ControlEvent {
    pub fn new(steer_pwm_us: u16, power_pwm_us: u16) -> Self {
        Self {
            steer_pwm_us,
            power_pwm_us,
        }
    }

    pub fn to_word(&self) -> u32 {
        ((self.steer_pwm_us as u32) << 16) | (self.power_pwm_us as u32)
    }

    pub fn from_word(word: u32) -> Self {
        Self {
            steer_pwm_us: (word >> 16) as u16,
            power_pwm_us: word as u16,
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
// Raw SIO FIFO access (hardware routes based on executing core)
// ─────────────────────────────────────────────────────────────────────────────

/// RX FIFO has at least one word available.
#[inline]
pub fn fifo_has_data() -> bool {
    let sio = unsafe { &*pac::SIO::ptr() };
    sio.fifo_st().read().vld().bit_is_set()
}

/// TX FIFO has space for at least one word.
#[inline]
fn fifo_has_space() -> bool {
    let sio = unsafe { &*pac::SIO::ptr() };
    sio.fifo_st().read().rdy().bit_is_set()
}

/// Write one word to the TX FIFO (blocks until space available, then SEV).
pub fn fifo_write_blocking(val: u32) {
    while !fifo_has_space() {
        cortex_m::asm::nop();
    }
    let sio = unsafe { &*pac::SIO::ptr() };
    sio.fifo_wr().write(|w| unsafe { w.bits(val) });
    cortex_m::asm::sev();
}

/// Read one word from the RX FIFO (non-blocking).
pub fn fifo_read() -> Option<u32> {
    if !fifo_has_data() {
        return None;
    }
    let sio = unsafe { &*pac::SIO::ptr() };
    Some(sio.fifo_rd().read().bits())
}

/// Discard all data currently in the RX FIFO.
pub fn fifo_drain() {
    while fifo_has_data() {
        let _ = fifo_read();
    }
}

/// Send a complete [`SensorEvent`] (6 blocking writes).
pub fn send_sensor_event(event: &SensorEvent) {
    for &w in &event.to_words() {
        fifo_write_blocking(w);
    }
}

/// Send a complete [`ControlEvent`] (1 blocking write).
pub fn send_control_event(event: &ControlEvent) {
    fifo_write_blocking(event.to_word());
}

/// Accumulate FIFO words into a 6-word buffer.
/// Returns `Some` when all 6 words have been collected (and resets `count`).
pub fn try_recv_sensor_event(buf: &mut [u32; 6], count: &mut usize) -> Option<SensorEvent> {
    while *count < 6 {
        match fifo_read() {
            Some(w) => {
                buf[*count] = w;
                *count += 1;
            }
            None => return None,
        }
    }
    *count = 0;
    Some(SensorEvent::from_words(*buf))
}

/// Read one complete control event.
pub fn try_recv_control_event() -> Option<ControlEvent> {
    fifo_read().map(ControlEvent::from_word)
}
