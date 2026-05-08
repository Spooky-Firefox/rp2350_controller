use defmt::warn;

pub trait Controller {
    fn update(&mut self, distance_increment_m: f32, dt_s: f32) -> [u16; 2];
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
    fn update(&mut self, distance_increment_m: f32, dt_s: f32) -> [u16; 2] {
        self.measured_distance_m += distance_increment_m;

        let error = self.distance_setpoint_m - self.measured_distance_m;
        warn!(
            "Distance inc: {} m, Distance: {} m, Setpoint: {} m, Error: {} m",
            distance_increment_m, self.measured_distance_m, self.distance_setpoint_m, error
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

/// PID controller for steering based on distance sensor input.
/// Uses three HC-SR04 sensors (left, center, right) to steer around obstacles.
/// Computes error as the difference between left and right distances.
pub struct SteeringDistanceController {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral_error: f32,
    pub previous_error: f32,
    pub integral_limit: f32,
    pub neutral_steering_pwm_us: u16,
    /// Activate steering only when center sensor reads below this threshold [cm].
    pub min_distance_cm: f32,
    /// Last computed PID terms — populated by `update`, read for telemetry.
    pub last_error: f32,
    pub last_proportional: f32,
    pub last_integral: f32,
    pub last_derivative: f32,
}

impl SteeringDistanceController {
    /// Update steering based on distance sensor readings.
    /// Takes distances from left, center, and right HC-SR04 sensors [cm].
    /// Returns steering PWM value [µs] (1200–1800 range).
    pub fn update(
        &mut self,
        dist_left_cm: f32,
        dist_center_cm: f32,
        dist_right_cm: f32,
        dt_s: f32,
    ) -> u16 {
        let center_valid = dist_center_cm.is_finite() && dist_center_cm > 0.0;

        if !center_valid || dist_center_cm > self.min_distance_cm {
            self.last_error = 0.0;
            self.last_proportional = 0.0;
            self.last_integral = 0.0;
            self.last_derivative = 0.0;
            return self.neutral_steering_pwm_us;
        }

        // Error: positive → steer right (obstacle on left), negative → steer left.
        let left_valid = dist_left_cm.is_finite() && dist_left_cm > 0.0;
        let right_valid = dist_right_cm.is_finite() && dist_right_cm > 0.0;
        let error = if left_valid && right_valid {
            dist_right_cm - dist_left_cm
        } else if left_valid {
            50.0
        } else if right_valid {
            -50.0
        } else {
            0.0
        };

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

        let steering_output = self.last_proportional + self.last_integral + self.last_derivative;
        (self.neutral_steering_pwm_us as f32 + steering_output).clamp(1200.0, 1800.0) as u16
    }

    /// Reset controller state.
    pub fn reset(&mut self) {
        self.integral_error = 0.0;
        self.previous_error = 0.0;
        self.last_error = 0.0;
        self.last_proportional = 0.0;
        self.last_integral = 0.0;
        self.last_derivative = 0.0;
    }
}
