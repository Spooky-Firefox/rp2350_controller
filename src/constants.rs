use core::f32::consts::PI;

use fugit::MicrosDurationU32;

pub const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// CPU clock speed (150 MHz). All timers and PWM are derived from this.
pub const SYS_CLOCK_HZ: u32 = 150_000_000;
pub const PWM_DIV_INT: u8 = 64;
pub const PWM_TIMER_HZ: u32 = SYS_CLOCK_HZ / PWM_DIV_INT as u32;

/// Arc length per encoder pulse [m].
pub const LENGTH_PER_ENCODER_PULSE_METERS: f32 = 13.0 * PI / 300.0;

// Standard RC-servo frame period (50 Hz).
pub const PWM_PERIOD: MicrosDurationU32 = MicrosDurationU32::from_ticks(20_000);
pub const PWM_DEFAULT_ON_TIME: MicrosDurationU32 = MicrosDurationU32::from_ticks(1_500);
