use defmt::warn;

pub trait Controller {
    fn update(&mut self, speed_mps: f32, dt_s: f32) -> [u16; 2];
    fn set_speed_setpoint(&mut self, setpoint_mps: f32);
}

pub struct StraightLineSpeedController {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub speed_setpoint_mps: f32,
    pub integral_error: f32,
    pub previous_error: f32,
    pub integral_limit: f32,
    pub steering_pwm_us: u16,
    pub neutral_power_pwm_us: u16,
    /// Last computed PID terms — populated by `update`, read for telemetry.
    pub last_error: f32,
    pub last_proportional: f32,
    pub last_integral: f32,
    pub last_derivative: f32,
}

impl Controller for StraightLineSpeedController {
    fn update(&mut self, speed_mps: f32, dt_s: f32) -> [u16; 2] {
        let error = self.speed_setpoint_mps - speed_mps;
        warn!(
            "Speed: {} m/s, Setpoint: {} m/s, Error: {} m/s",
            speed_mps, self.speed_setpoint_mps, error
        );
        if dt_s > 0.0 {
            self.integral_error += error * dt_s;
            self.integral_error = self
                .integral_error
                .clamp(-self.integral_limit, self.integral_limit);
        }

        let derivative = if dt_s > 0.0 {
            (error - self.previous_error) / dt_s
        } else {
            0.0
        };
        self.previous_error = error;

        self.last_error = error;
        self.last_proportional = self.kp * error;
        self.last_integral = self.ki * self.integral_error;
        self.last_derivative = self.kd * derivative;

        let power_pwm = (self.neutral_power_pwm_us as f32
            + self.last_proportional
            + self.last_integral
            + self.last_derivative)
            .clamp(1200.0, 1800.0) as u16;

        [self.steering_pwm_us, power_pwm]
    }

    fn set_speed_setpoint(&mut self, setpoint_mps: f32) {
        self.speed_setpoint_mps = setpoint_mps;
    }
}
