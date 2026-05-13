use defmt::warn;

pub trait Controller {
    fn update(&mut self, distance_increment_m: f32, dt_s: f32) -> [u16; 2];
    fn set_speed_setpoint(&mut self, setpoint_mps: f32);
}

#[derive(Clone, Copy, Debug)]
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral_limit: f32,
    pub integral_error: f32,
    pub previous_error: f32,
    pub last_error: f32,
    pub last_proportional: f32,
    pub last_integral: f32,
    pub last_derivative: f32,
}

impl Pid {
    pub const fn new(kp: f32, ki: f32, kd: f32, integral_limit: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_limit,
            integral_error: 0.0,
            previous_error: 0.0,
            last_error: 0.0,
            last_proportional: 0.0,
            last_integral: 0.0,
            last_derivative: 0.0,
        }
    }

    pub fn update(&mut self, error: f32, step: f32) -> f32 {
        if step > 0.0 {
            self.integral_error += error * step;
            self.integral_error = self
                .integral_error
                .clamp(-self.integral_limit, self.integral_limit);
        }

        let derivative = if step > 0.0 {
            (error - self.previous_error) / step
        } else {
            0.0
        };
        self.previous_error = error;

        self.last_error = error;
        self.last_proportional = self.kp * error;
        self.last_integral = self.ki * self.integral_error;
        self.last_derivative = self.kd * derivative;

        self.output()
    }

    pub fn output(&self) -> f32 {
        self.last_proportional + self.last_integral + self.last_derivative
    }

    pub fn reset(&mut self) {
        self.integral_error = 0.0;
        self.previous_error = 0.0;
        self.last_error = 0.0;
        self.last_proportional = 0.0;
        self.last_integral = 0.0;
        self.last_derivative = 0.0;
    }
}

pub struct StraightLineSpeedController {
    pub pid: Pid,
    pub distance_setpoint_m: f32,
    pub measured_distance_m: f32,
    pub steering_pwm_us: u16,
    pub neutral_power_pwm_us: u16,
}

impl Controller for StraightLineSpeedController {
    fn update(&mut self, distance_increment_m: f32, dt_s: f32) -> [u16; 2] {
        self.measured_distance_m += distance_increment_m;

        let error = self.distance_setpoint_m - self.measured_distance_m;
        warn!(
            "Distance inc: {} m, Distance: {} m, Setpoint: {} m, Error: {} m",
            distance_increment_m, self.measured_distance_m, self.distance_setpoint_m, error
        );
        let pid_output = self.pid.update(error, dt_s);

        let power_pwm =
            (self.neutral_power_pwm_us as f32 + pid_output).clamp(1500.0, 2000.0) as u16;

        [self.steering_pwm_us, power_pwm]
    }

    fn set_speed_setpoint(&mut self, setpoint_mps: f32) {
        self.distance_setpoint_m = setpoint_mps;
    }
}

/// PID controller for steering-angle tracking.
///
/// The current setpoint is fixed at straight-ahead (`0.0` deg). A future
/// wall-following or path planner can set `angle_setpoint_deg` instead of
/// directly commanding PWM.
pub struct SteeringAngleController {
    pub pid: Pid,
    pub angle_setpoint_deg: f32,
    pub neutral_steering_pwm_us: u16,
    pub min_steering_pwm_us: u16,
    pub max_steering_pwm_us: u16,
}

impl SteeringAngleController {
    /// Update steering using traveled distance as PID step size.
    pub fn update(&mut self, measured_angle_deg: f32, distance_increment_m: f32) -> u16 {
        let error = self.angle_setpoint_deg - measured_angle_deg;
        let steering_output = self.pid.update(error, distance_increment_m);

        (self.neutral_steering_pwm_us as f32 - steering_output).clamp(
            self.min_steering_pwm_us as f32,
            self.max_steering_pwm_us as f32,
        ) as u16
    }

    /// Reset controller state.
    pub fn reset(&mut self) {
        self.pid.reset();
    }
}

/// Recursive Least Squares (RLS) filter for camera angle estimation.
///
/// Tracks the camera angle with adaptive filtering that rejects noise
/// while adapting to gradual changes in the true angle.
#[derive(Clone, Copy, Debug)]
pub struct RlsAngleFilter {
    /// Estimated angle [deg]
    pub angle_estimate_deg: f32,
    /// Covariance (confidence in estimate)
    pub covariance: f32,
    /// Forgetting factor: 0.95..1.0 (lower = faster adaptation, higher = more stable)
    pub lambda: f32,
    /// Measurement noise variance (tuning parameter)
    pub measurement_noise_variance: f32,
}

impl RlsAngleFilter {
    /// Create a new RLS filter with default tuning.
    pub const fn new() -> Self {
        Self {
            angle_estimate_deg: 0.0,
            covariance: 1.0,
            lambda: 0.98,
            measurement_noise_variance: 0.5,
        }
    }

    /// Create a new RLS filter with custom forgetting factor and noise variance.
    pub const fn with_params(lambda: f32, measurement_noise_variance: f32) -> Self {
        Self {
            angle_estimate_deg: 0.0,
            covariance: 1.0,
            lambda,
            measurement_noise_variance,
        }
    }

    /// Update the filter with a new angle measurement.
    ///
    /// Returns the filtered (estimated) angle.
    pub fn update(&mut self, measured_angle_deg: f32) -> f32 {
        if !measured_angle_deg.is_finite() {
            return self.angle_estimate_deg;
        }

        // RLS scalar update:
        // - Innovation (measurement residual)
        let innovation = measured_angle_deg - self.angle_estimate_deg;

        // - Kalman gain
        let denominator = self.lambda + self.covariance;
        let kalman_gain = self.covariance / denominator;

        // - Update estimate
        self.angle_estimate_deg += kalman_gain * innovation;

        // - Update covariance (discounted by lambda for forgetting)
        self.covariance = (self.covariance - kalman_gain * self.covariance) / self.lambda;
        self.covariance = self.covariance.max(0.01);

        self.angle_estimate_deg
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.angle_estimate_deg = 0.0;
        self.covariance = 1.0;
    }
}

/// Heading observer using bicycle-model predict and camera-aligned correct (scalar Kalman filter).
///
/// Predicts heading change from steering angle and encoder distance using the kinematic bicycle model:
///   dθ = (ds / L) * tan(δ)  [rad]
/// where ds is distance traveled [m], L is wheelbase [m], and δ is steering angle [rad].
///
/// Corrects predicted heading toward the camera-measured angle using a scalar Kalman-style update:
///   innovation = z - θ_pred
///   gain = P / (P + R)
///   θ = θ_pred + gain * innovation
///   P = (1 - gain) * P + Q
#[derive(Clone, Copy, Debug)]
pub struct HeadingObserver {
    /// Estimated heading [deg]
    pub heading_estimate_deg: f32,
    /// Covariance (estimate uncertainty) [deg²]
    pub covariance: f32,
    /// Process noise (prediction uncertainty) [deg²]
    pub process_noise: f32,
    /// Measurement noise (camera angle uncertainty) [deg²]
    pub measurement_noise: f32,
}

impl HeadingObserver {
    /// Create a new observer with default tuning.
    pub const fn new() -> Self {
        Self {
            heading_estimate_deg: 0.0,
            covariance: 1.0,
            process_noise: 0.01,  // Q: trust the bicycle model strongly
            measurement_noise: 0.5, // R: camera angle is ~7 FPS, higher noise
        }
    }

    /// Create observer with custom tuning parameters.
    pub const fn with_params(process_noise: f32, measurement_noise: f32) -> Self {
        Self {
            heading_estimate_deg: 0.0,
            covariance: 1.0,
            process_noise,
            measurement_noise,
        }
    }

    /// Predict step: advance heading estimate using bicycle kinematics.
    ///
    /// # Arguments
    /// - `distance_increment_m` — distance traveled since last prediction [m]
    /// - `steering_pwm_us` — current steering command [microseconds]
    /// - `wheelbase_m` — vehicle wheelbase [m]
    /// - `steering_angle_per_us_deg` — steering angle sensitivity [deg/µs]
    /// - `neutral_steering_pwm_us` — neutral steering PWM [µs]
    ///
    /// Updates `heading_estimate_deg` and increases covariance by `process_noise`.
    pub fn predict(
        &mut self,
        distance_increment_m: f32,
        steering_pwm_us: u16,
        wheelbase_m: f32,
        steering_angle_per_us_deg: f32,
        neutral_steering_pwm_us: u16,
    ) {
        if distance_increment_m <= 0.0 || !wheelbase_m.is_finite() || wheelbase_m <= 0.0 {
            // No motion or invalid wheelbase: covariance grows but heading is unchanged
            self.covariance += self.process_noise;
            return;
        }

        // Compute steering angle from PWM command
        let pwm_error_us = (steering_pwm_us as i32) - (neutral_steering_pwm_us as i32);
        let steering_angle_deg = (pwm_error_us as f32) * steering_angle_per_us_deg;
        let steering_angle_rad = steering_angle_deg.to_radians();

        // Bicycle model: dθ = (ds / L) * tan(δ)
        // Use libm::tanf for no_std compatibility
        let tan_delta = libm::tanf(steering_angle_rad);
        let heading_change_rad = (distance_increment_m / wheelbase_m) * tan_delta;
        let heading_change_deg = heading_change_rad.to_degrees();

        // Update estimate and increase uncertainty
        self.heading_estimate_deg += heading_change_deg;
        self.covariance += self.process_noise;
    }

    /// Correct step: fuse camera-measured angle using scalar Kalman gain.
    ///
    /// Updates `heading_estimate_deg` toward the measured angle, weighted by current covariance
    /// relative to measurement noise. Also decreases covariance.
    pub fn correct(&mut self, measured_angle_deg: f32) {
        if !measured_angle_deg.is_finite() {
            // Invalid measurement: no update
            return;
        }

        // Innovation
        let innovation = measured_angle_deg - self.heading_estimate_deg;

        // Kalman gain: K = P / (P + R)
        let denominator = self.covariance + self.measurement_noise;
        let kalman_gain = self.covariance / denominator;

        // Update estimate
        self.heading_estimate_deg += kalman_gain * innovation;

        // Update covariance: P = (1 - K) * P
        self.covariance = (1.0 - kalman_gain) * self.covariance;
        self.covariance = self.covariance.max(0.01); // Prevent zero uncertainty
    }

    /// Reset observer state to initial values.
    pub fn reset(&mut self) {
        self.heading_estimate_deg = 0.0;
        self.covariance = 1.0;
    }
}
