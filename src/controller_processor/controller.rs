use defmt::warn;

pub trait Controller {
    fn update(&mut self, speed_mps: f32, dt_s: f32) -> [u16; 2];
    fn set_speed_setpoint(&mut self, setpoint_mps: f32);
}

pub struct StraightLineSpeedController {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub distance_setpoint_m: f32,
    pub measured_distance_m: f32,
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
        if dt_s > 0.0 {
            self.measured_distance_m += speed_mps.abs() * dt_s;
        }

        let error = self.distance_setpoint_m - self.measured_distance_m;
        warn!(
            "Speed: {} m/s, Distance: {} m, Setpoint: {} m, Error: {} m",
            speed_mps,
            self.measured_distance_m,
            self.distance_setpoint_m,
            error
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
            .clamp(1500.0, 2000.0) as u16;

        [self.steering_pwm_us, power_pwm]
    }

    fn set_speed_setpoint(&mut self, setpoint_mps: f32) {
        self.distance_setpoint_m = setpoint_mps;
    }
}
