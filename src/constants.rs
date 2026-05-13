use fugit::MicrosDurationU32;

pub const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// CPU clock speed (150 MHz). All timers and PWM are derived from this.
pub const SYS_CLOCK_HZ: u32 = 150_000_000;
pub const PWM_DIV_INT: u8 = 64;
pub const PWM_TIMER_HZ: u32 = SYS_CLOCK_HZ / PWM_DIV_INT as u32;

/// Arc length per encoder pulse [m].
pub const LENGTH_PER_ENCODER_PULSE_METERS: f32 = 3.03 / 100.0;

// Standard RC-servo frame period (50 Hz).
pub const PWM_PERIOD: MicrosDurationU32 = MicrosDurationU32::from_ticks(20_000);
pub const PWM_DEFAULT_ON_TIME: MicrosDurationU32 = MicrosDurationU32::from_ticks(1_500);

// --- Heading Observer (Bicycle Model) Constants ---

/// Wheelbase of the vehicle [m].
/// Typical RC car: 80–150 mm. Adjust based on actual vehicle geometry.
pub const OBSERVER_WHEELBASE_METERS: f32 = 0.12;

/// Steering angle per PWM microsecond from neutral [deg/µs].
/// Relates servo PWM command to front-wheel steering angle.
/// Default: ±50° range over ±500 µs from 1500 µs neutral = 0.1 deg/µs
pub const OBSERVER_STEERING_ANGLE_PER_US_DEG: f32 = 0.1;

/// Neutral steering PWM [µs] — center position (straight ahead).
pub const OBSERVER_NEUTRAL_STEERING_PWM_US: u16 = 1500;
