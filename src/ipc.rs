//! Inter-core communication types and SIO FIFO primitives.
#![allow(unsafe_code)] // FifoTx/FifoRx use direct PAC register access; see safety comments.
//!
//! # Protocol
//!
//! Data is exchanged via `heapless::spsc` queues. The SIO FIFO carries only
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

use core::convert::TryFrom;
use rp235x_hal::sio::SioFifo;
use rp235x_pac;

#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IpcSignal {
    SensorReady = 1,
    ControlReady = 2,
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

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Constants {
    SpeedSetpoint = 0,
    SpeedKp = 1,
    SpeedKi = 2,
    SpeedKd = 3,
    SteeringKp = 4,
    SteeringKi = 5,
    SteeringKd = 6,
    ObserverProcessNoise = 7,
    ObserverMeasurementNoise = 8,
    ObserverCovarianceInit = 9,
}

impl TryFrom<&str> for Constants {
    type Error = ();

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        match value {
            "speed" | "setpoint" | "speed_setpoint" => Ok(Constants::SpeedSetpoint),
            "speed_kp" => Ok(Constants::SpeedKp),
            "speed_ki" => Ok(Constants::SpeedKi),
            "speed_kd" => Ok(Constants::SpeedKd),
            "steering_kp" => Ok(Constants::SteeringKp),
            "steering_ki" => Ok(Constants::SteeringKi),
            "steering_kd" => Ok(Constants::SteeringKd),
            "observer_q" => Ok(Constants::ObserverProcessNoise),
            "observer_r" => Ok(Constants::ObserverMeasurementNoise),
            "observer_p0" => Ok(Constants::ObserverCovarianceInit),
            _ => Err(()),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum SensorKind {
    Encoder {
        rpm_period_us: f32,
    },
    EncoderTimeout,
    Distances {
        left_cm: f32,
        center_cm: f32,
        right_cm: f32,
    },
    ConstantUpdate {
        constant: Constants,
        value: f32,
    },
    CameraAlign {
        angle: f32,
        confidence: f32,
    },
}

#[derive(Clone, Copy, Debug)]
pub struct SensorEvent {
    pub t32_us: u32,
    pub kind: SensorKind,
}

impl SensorEvent {
    pub fn encoder(timestamp_us: u64, rpm_period_us: f32) -> Self {
        Self {
            t32_us: timestamp_us as u32,
            kind: SensorKind::Encoder { rpm_period_us },
        }
    }

    pub fn encoder_timeout(timestamp_us: u64) -> Self {
        Self {
            t32_us: timestamp_us as u32,
            kind: SensorKind::EncoderTimeout,
        }
    }

    pub fn distances(timestamp_us: u64, left_cm: f32, center_cm: f32, right_cm: f32) -> Self {
        Self {
            t32_us: timestamp_us as u32,
            kind: SensorKind::Distances {
                left_cm,
                center_cm,
                right_cm,
            },
        }
    }

    pub fn constant_update(timestamp_us: u64, constant: Constants, value: f32) -> Self {
        Self {
            t32_us: timestamp_us as u32,
            kind: SensorKind::ConstantUpdate { constant, value },
        }
    }

    pub fn camera_align(timestamp_us: u64, angle: f32, confidence: f32) -> Self {
        Self {
            t32_us: timestamp_us as u32,
            kind: SensorKind::CameraAlign { angle, confidence },
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ControlOutput {
    pub steer_pwm_us: u16,
    pub power_pwm_us: u16,
}

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

pub struct FifoTx;

impl FifoTx {
    fn write_word_blocking(value: u32) {
        let sio = unsafe { &(*rp235x_pac::SIO::ptr()) };
        while !sio.fifo_st().read().rdy().bit_is_set() {
            cortex_m::asm::nop();
        }
        sio.fifo_wr().write(|w| unsafe { w.bits(value) });
        cortex_m::asm::sev();
    }

    pub fn signal(&mut self, sig: IpcSignal) {
        Self::write_word_blocking(sig as u32);
    }
}

pub struct FifoRx;

impl FifoRx {
    pub fn try_read(&mut self) -> Option<u32> {
        let sio = unsafe { &(*rp235x_pac::SIO::ptr()) };
        if sio.fifo_st().read().vld().bit_is_set() {
            Some(sio.fifo_rd().read().bits())
        } else {
            None
        }
    }

    pub fn drain(&mut self) {
        let sio = unsafe { &(*rp235x_pac::SIO::ptr()) };
        while sio.fifo_st().read().vld().bit_is_set() {
            let _ = sio.fifo_rd().read().bits();
        }
    }
}

pub fn split_fifo(_fifo: SioFifo) -> (FifoTx, FifoRx) {
    (FifoTx, FifoRx)
}

impl Default for TimeExtender {
    fn default() -> Self {
        Self::new()
    }
}
