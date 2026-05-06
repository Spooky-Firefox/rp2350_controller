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
    /// Minimum distance threshold [cm] - steer if any sensor reads below this
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
    /// Returns steering PWM value [us] (1200-1800 range).
    pub fn update(&mut self, dist_left_cm: f32, dist_center_cm: f32, dist_right_cm: f32, dt_s: f32) -> u16 {
        // Only apply steering control if center sensor detects something close
        let center_valid = dist_center_cm.is_finite() && dist_center_cm > 0.0;
        
        if !center_valid {
            // No obstacle detected, return neutral steering
            self.last_error = 0.0;
            self.last_proportional = 0.0;
            self.last_integral = 0.0;
            self.last_derivative = 0.0;
            return self.neutral_steering_pwm_us;
        }

        // If obstacle is too far, don't steer
        if dist_center_cm > self.min_distance_cm {
            self.last_error = 0.0;
            self.last_proportional = 0.0;
            self.last_integral = 0.0;
            self.last_derivative = 0.0;
            return self.neutral_steering_pwm_us;
        }

        // Error is the difference between left and right distances:
        // If left is closer (obstacle on left), error is positive → steer right
        // If right is closer (obstacle on right), error is negative → steer left
        let left_valid = dist_left_cm.is_finite() && dist_left_cm > 0.0;
        let right_valid = dist_right_cm.is_finite() && dist_right_cm > 0.0;
        
        let error = if left_valid && right_valid {
            dist_right_cm - dist_left_cm
        } else if left_valid {
            50.0 // Default to steer right if right sensor failed
        } else if right_valid {
            -50.0 // Default to steer left if left sensor failed
        } else {
            0.0 // Both failed, no steering
        };

        // PID calculation
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
        
        // Clamp steering PWM to valid range (1200-1800 μs, center at 1500)
        let steering_pwm = (self.neutral_steering_pwm_us as f32 + steering_output)
            .clamp(1200.0, 1800.0) as u16;

        steering_pwm
    }

    /// Reset controller state (typically after obstacle is clear)
    pub fn reset(&mut self) {
        self.integral_error = 0.0;
        self.previous_error = 0.0;
        self.last_error = 0.0;
        self.last_proportional = 0.0;
        self.last_integral = 0.0;
        self.last_derivative = 0.0;
    }
}

