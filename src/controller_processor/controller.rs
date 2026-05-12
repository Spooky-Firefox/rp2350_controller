use crate::ipc::Constants;

pub const NEUTRAL_STEERING_PWM_US: u16 = 1500;
pub const MIN_STEERING_PWM_US: f32 = 1200.0;
pub const MAX_STEERING_PWM_US: f32 = 1800.0;

pub const STRAIGHT_THROTTLE_PWM_US: u16 = 1620;
pub const TURN_THROTTLE_PWM_US: u16 = 1600;
pub const TURN_STEERING_PWM_US: u16 = 1720;

pub const DISTANCE_JUMP_TRIGGER_CM: f32 = 100.0;
pub const TURN_ANGLE_IGNORE_US: u64 = 500_000;
pub const FORCE_STRAIGHT_MODE: bool = false;
pub const STARTUP_HOLD_US: u64 = 1_000_000;
pub const STARTUP_THROTTLE_PWM_US: u16 = 1500;

pub const ANGLE_PID_KP: f32 = 12.0;
pub const ANGLE_PID_KI: f32 = 0.8;
pub const ANGLE_PID_KD: f32 = 0.5;
pub const ANGLE_PID_INTEGRAL_LIMIT: f32 = 100.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DriveMode {
    Straight,
    Turning {
        entered_at_us: u64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OutputMode {
    Startup,
    Straight,
    Turning,
}

pub struct RaceController {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    previous_error: f32,
    integral_limit: f32,
    last_error: f32,
    mode: DriveMode,
    latest_angle_deg: Option<f32>,
    prev_left_cm: Option<f32>,
    prev_right_cm: Option<f32>,
    startup_started_at_us: Option<u64>,
}

impl RaceController {
    pub fn new() -> Self {
        Self {
            kp: ANGLE_PID_KP,
            ki: ANGLE_PID_KI,
            kd: ANGLE_PID_KD,
            integral: 0.0,
            previous_error: 0.0,
            integral_limit: ANGLE_PID_INTEGRAL_LIMIT,
            last_error: 0.0,
            mode: DriveMode::Straight,
            latest_angle_deg: None,
            prev_left_cm: None,
            prev_right_cm: None,
            startup_started_at_us: None,
        }
    }

    pub fn apply_constant_update(&mut self, constant: Constants, value: f32) {
        match constant {
            Constants::SpeedKp => self.kp = value,
            Constants::SpeedKi => self.ki = value,
            Constants::SpeedKd => self.kd = value,
            Constants::SpeedSetpoint | Constants::SteeringKp | Constants::SteeringKi | Constants::SteeringKd => {
                let _ = value;
            }
        }
    }

    pub fn on_camera_align(&mut self, angle: f32, t_us: u64) {
        self.latest_angle_deg = Some(angle);

        if let DriveMode::Turning { entered_at_us } = self.mode
            && t_us.saturating_sub(entered_at_us) >= TURN_ANGLE_IGNORE_US
        {
            self.mode = DriveMode::Straight;
            self.reset_pid();
        }
    }

    pub fn on_distances(&mut self, left_cm: f32, right_cm: f32, t_us: u64) {
        if !FORCE_STRAIGHT_MODE
            && matches!(self.mode, DriveMode::Straight)
            && (has_steep_increase(self.prev_left_cm, left_cm)
                || has_steep_increase(self.prev_right_cm, right_cm)
                || is_distance_timeout(left_cm)
                || is_distance_timeout(right_cm))
        {
            self.mode = DriveMode::Turning { entered_at_us: t_us };
        }

        self.prev_left_cm = Some(left_cm);
        self.prev_right_cm = Some(right_cm);
    }

    pub fn output(&mut self, t_us: u64, dt_s: f32) -> (u16, u16, f32, OutputMode) {
        let startup_t0 = self.startup_started_at_us.get_or_insert(t_us);
        if t_us.saturating_sub(*startup_t0) < STARTUP_HOLD_US {
            self.last_error = 0.0;
            return (
                NEUTRAL_STEERING_PWM_US,
                STARTUP_THROTTLE_PWM_US,
                self.last_error,
                OutputMode::Startup,
            );
        }

        match self.mode {
            DriveMode::Straight => {
                let steer_pwm = if let Some(angle_deg) = self.latest_angle_deg {
                    self.update_angle_pid(angle_deg, dt_s)
                } else {
                    self.last_error = 0.0;
                    NEUTRAL_STEERING_PWM_US
                };
                (
                    steer_pwm,
                    STRAIGHT_THROTTLE_PWM_US,
                    self.last_error,
                    OutputMode::Straight,
                )
            }
            DriveMode::Turning { .. } => {
                (TURN_STEERING_PWM_US, TURN_THROTTLE_PWM_US, 0.0, OutputMode::Turning)
            }
        }
    }

    fn update_angle_pid(&mut self, angle_deg: f32, dt_s: f32) -> u16 {
        let error = -angle_deg;

        if dt_s > 0.0 {
            self.integral += error * dt_s;
            self.integral = self
                .integral
                .clamp(-self.integral_limit, self.integral_limit);
        }

        let derivative = if dt_s > 0.0 {
            (error - self.previous_error) / dt_s
        } else {
            0.0
        };
        self.previous_error = error;
        self.last_error = error;

        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        (NEUTRAL_STEERING_PWM_US as f32 + output).clamp(MIN_STEERING_PWM_US, MAX_STEERING_PWM_US)
            as u16
    }

    fn reset_pid(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
        self.last_error = 0.0;
    }
}

impl Default for RaceController {
    fn default() -> Self {
        Self::new()
    }
}

fn has_steep_increase(prev_cm: Option<f32>, current_cm: f32) -> bool {
    if !(current_cm.is_finite() && current_cm > 0.0) {
        return false;
    }

    match prev_cm {
        Some(prev) if prev.is_finite() && prev > 0.0 => {
            (current_cm - prev) >= DISTANCE_JUMP_TRIGGER_CM
        }
        _ => false,
    }
}

fn is_distance_timeout(distance_cm: f32) -> bool {
    !distance_cm.is_finite() || distance_cm <= 0.0
}
