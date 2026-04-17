//! Extended Kalman Filter — kinematic bicycle model, dead-reckoning variant.
//!
//! Mirrors the math in `controll/kalman_dead_reckoning.m`.
//!
//! # State
//! `x = [X, Y, theta, v]`
//! - `X`, `Y`  : global position [m]
//! - `theta`   : heading [rad]  (0 = +X, CCW positive)
//! - `v`       : longitudinal speed [m/s]  (+ forward)
//!
//! # API
//! | Function | When to call | What it does |
//! |---|---|---|
//! | [`EkfFilter::on_speed_sample`] | Hall pulse interrupt | predict(dt) → update with v |
//! | [`EkfFilter::on_timeout`]    | watchdog task, no RPM arrived | predict(dt) only |
//! | [`EkfFilter::set_control`]   | whenever control changes | store δ, a_long |
//!
//! Both time-driven functions accept a `TimerInstantU64<1_000_000>`.
//! The filter tracks `t_last` internally so `dt` is always correct regardless
//! of whether RPM is faster or slower than any fixed tick rate.

use core::f32::consts::PI;
use fugit::TimerInstantU64;

// ─────────────────────────────────────────────────────────────────────────────
// Types
// ─────────────────────────────────────────────────────────────────────────────

/// Alias kept short for readability.
type Instant = TimerInstantU64<1_000_000>;

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Tuning constants for the EKF.  All fields are public so they can be filled
/// in from a `const` block or computed from sensor specs at init time.
#[derive(Clone, Copy)]
pub struct EkfConst {
    /// Wheelbase [m].
    pub l: f32,

    // --- Process noise (continuous-time std-dev, scaled by dt inside predict) ---
    /// Position noise [m/s].  Higher = more model uncertainty in X, Y.
    pub q_pos: f32,
    /// Heading noise [rad/s].
    pub q_theta: f32,
    /// Velocity noise [m/s²].  Set high if `a_long` input is unreliable.
    pub q_v: f32,

    // --- Measurement noise ---
    /// Longitudinal speed measurement noise std-dev [m/s].
    pub r_speed: f32,

    // --- Numerics ---
    /// Smoothing term to avoid |v| Jacobian singularity near v = 0 [m/s].
    /// Typical: 1e-3.
    pub eps_v: f32,
    /// Maximum believable dt [µs].  Gaps larger than this are clamped so a
    /// long idle period does not produce huge covariance jumps.
    pub dt_max_us: u64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Filter
// ─────────────────────────────────────────────────────────────────────────────

/// Event-driven Extended Kalman Filter.
///
/// Holds state, covariance, last timestamp, and current control inputs.
/// No heap allocation; all matrices are fixed-size `[f32; N]` arrays.
pub struct EkfFilter {
    /// State estimate `[X, Y, theta, v]`.
    x: [f32; 4],
    /// Covariance 4×4 stored row-major (`p[i*4 + j]` = row i, col j).
    p: [f32; 16],
    /// Timestamp of the last filter step.
    t_last: Instant,
    /// Current control inputs: `[delta (rad), a_long (m/s²)]`.
    u: [f32; 2],
    /// Filter constants.
    c: EkfConst,
}

impl EkfFilter {
    /// Initialise the filter.
    ///
    /// - `c`  : tuning constants
    /// - `x0` : initial state `[X, Y, theta, v]`
    /// - `p0` : initial covariance (4×4 row-major)
    /// - `t0` : timestamp at which the initial state is valid
    pub fn new(c: EkfConst, x0: [f32; 4], p0: [f32; 16], t0: Instant) -> Self {
        Self {
            x: x0,
            p: p0,
            t_last: t0,
            u: [0.0; 2],
            c,
        }
    }

    // ── Control ──────────────────────────────────────────────────────────────

    /// Store the current control inputs.  Call whenever steering or throttle
    /// command changes.
    ///
    /// - `delta`  : front-wheel steering angle [rad]
    /// - `a_long` : longitudinal acceleration [m/s²]  (0 if not available)
    pub fn set_control(&mut self, delta: f32, a_long: f32) {
        self.u = [delta, a_long];
    }

    // ── Event entry points ───────────────────────────────────────────────────

    /// **Call from the hall pulse interrupt** each time a new speed sample arrives.
    ///
    /// Runs `predict(dt)` from the last filter time to `now`, then corrects
    /// the speed state with the new linear speed measurement.
    ///
    /// - `speed_meas` : absolute longitudinal speed [m/s]  (≥ 0, no sign)
    /// - `now`        : interrupt timestamp from `MainMono::now()`
    pub fn on_speed_sample(&mut self, speed_meas: f32, now: Instant) {
        let dt_s = self.dt_seconds(now);
        self.t_last = now;
        if dt_s > 0.0 {
            self.predict(dt_s);
        }
        self.update_speed(speed_meas);
    }

    /// **Call from a watchdog / periodic task** when no RPM sample has arrived
    /// within the expected window.
    ///
    /// Runs `predict(dt)` only.  Covariance grows according to process noise
    /// until the next measurement arrives.
    ///
    /// - `now` : current timestamp from `MainMono::now()`
    pub fn on_timeout(&mut self, now: Instant) {
        let dt_s = self.dt_seconds(now);
        self.t_last = now;
        if dt_s > 0.0 {
            self.predict(dt_s);
        }
    }

    // ── State accessors ──────────────────────────────────────────────────────

    /// Full state estimate `[X, Y, theta, v]`.
    pub fn state(&self) -> [f32; 4] {
        self.x
    }

    /// Estimated longitudinal speed [m/s].
    pub fn speed(&self) -> f32 {
        self.x[3]
    }

    /// Estimated heading [rad].
    pub fn heading(&self) -> f32 {
        self.x[2]
    }

    /// Diagonal of the covariance matrix `[var_X, var_Y, var_theta, var_v]`.
    pub fn covariance_diag(&self) -> [f32; 4] {
        [self.p[0], self.p[5], self.p[10], self.p[15]]
    }

    // ── Private ──────────────────────────────────────────────────────────────

    /// Compute dt [s] from `t_last` to `now`, clamped to `dt_max_us`.
    /// Returns 0.0 if `now` is not strictly after `t_last` (stale/same tick).
    fn dt_seconds(&self, now: Instant) -> f32 {
        if now <= self.t_last {
            return 0.0;
        }
        let dur = now - self.t_last;
        let dt_us = dur.ticks().min(self.c.dt_max_us);
        dt_us as f32 * 1e-6_f32
    }

    /// EKF prediction step — bicycle kinematic model.
    ///
    /// Matches `ekf_predict()` in `kalman_dead_reckoning.m`.
    fn predict(&mut self, dt: f32) {
        // --- Capture pre-update values (Jacobian must use old state) ---
        let theta = self.x[2];
        let v = self.x[3];
        let delta = self.u[0];
        let a_lon = self.u[1];
        let l = self.c.l;

        let ct = libm::cosf(theta);
        let st = libm::sinf(theta);
        let td = libm::tanf(delta);

        // --- State propagation ---
        // X_{k+1} = X_k + v*cos(theta)*dt
        // Y_{k+1} = Y_k + v*sin(theta)*dt
        // theta_{k+1} = theta_k + (v/L)*tan(delta)*dt
        // v_{k+1}     = v_k + a_lon*dt
        self.x[0] += v * ct * dt;
        self.x[1] += v * st * dt;
        self.x[2] = wrap_pi(self.x[2] + (v / l) * td * dt);
        self.x[3] += a_lon * dt;

        // --- Jacobian F = df/dx  (4×4, sparse off-diagonal terms) ---
        //
        //   F = [ 1,  0,  -v*st*dt,   ct*dt      ]
        //       [ 0,  1,   v*ct*dt,   st*dt      ]
        //       [ 0,  0,   1,         td/L*dt    ]
        //       [ 0,  0,   0,         1          ]
        let f02 = -v * st * dt;
        let f03 = ct * dt;
        let f12 = v * ct * dt;
        let f13 = st * dt;
        let f23 = td / l * dt;

        // --- Process noise Q (diagonal, scaled by dt²) ---
        let q0 = (self.c.q_pos * dt) * (self.c.q_pos * dt);     // X, Y
        let q2 = (self.c.q_theta * dt) * (self.c.q_theta * dt); // theta
        let q3 = (self.c.q_v * dt) * (self.c.q_v * dt);         // v

        // --- Covariance update: P = F*P*F' + Q ---
        self.p = fpfT_plus_q(&self.p, f02, f03, f12, f13, f23, q0, q2, q3);
    }

    /// EKF measurement update — absolute longitudinal speed (no sign).
    ///
    /// Measurement model: `z = |v| + noise`.
    fn update_speed(&mut self, speed_meas: f32) {
        let v = self.x[3];
        let eps = self.c.eps_v;

        // Smoothed |v| to avoid singularity at v = 0
        let v_abs = libm::sqrtf(v * v + eps * eps);

        // Predicted measurement: h(x) = |v|
        let h_pred = v_abs;

        // Jacobian H = [0, 0, 0, dh/dv]  (1×4)
        let dh_dv = v / v_abs;

        // Innovation
        let y_inn = speed_meas - h_pred;

        // S = H*P*H' + R  (scalar, since H is 1×4 with only index-3 nonzero)
        //   = dh_dv² * P[3][3] + r_speed²
        let r = self.c.r_speed * self.c.r_speed;
        let p33 = self.p[3 * 4 + 3];
        let s = dh_dv * dh_dv * p33 + r;

        if s <= 0.0 {
            return; // numerically degenerate — skip update
        }

        // Kalman gain K = P*H' / S  (4×1)
        //   (P*H')[i] = P[i][3] * dh_dv
        let mut k = [0.0_f32; 4];
        for i in 0..4 {
            k[i] = self.p[i * 4 + 3] * dh_dv / s;
        }

        // State update: x += K * y_inn
        for i in 0..4 {
            self.x[i] += k[i] * y_inn;
        }
        self.x[2] = wrap_pi(self.x[2]); // keep heading in [-π, π]

        // Covariance update — Joseph form: P = (I-KH)*P*(I-KH)' + K*R*K'
        self.p = joseph_update(&self.p, &k, dh_dv, r);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Matrix helpers  (4×4, row-major, no heap)
// ─────────────────────────────────────────────────────────────────────────────

/// Compute `F * P * F' + Q` for the sparse bicycle-model Jacobian F and
/// diagonal Q.
///
/// F is identity plus five nonzero off-diagonal terms:
///   `F[0][2] = f02`,  `F[0][3] = f03`
///   `F[1][2] = f12`,  `F[1][3] = f13`
///   `F[2][3] = f23`
///
/// Q is diagonal: `diag(q0, q0, q2, q3)`.
#[allow(non_snake_case)]
fn fpfT_plus_q(
    p: &[f32; 16],
    f02: f32,
    f03: f32,
    f12: f32,
    f13: f32,
    f23: f32,
    q0: f32,
    q2: f32,
    q3: f32,
) -> [f32; 16] {
    // Build F explicitly
    let mut f = [0.0_f32; 16];
    f[0 * 4 + 0] = 1.0;
    f[1 * 4 + 1] = 1.0;
    f[2 * 4 + 2] = 1.0;
    f[3 * 4 + 3] = 1.0;
    f[0 * 4 + 2] = f02;
    f[0 * 4 + 3] = f03;
    f[1 * 4 + 2] = f12;
    f[1 * 4 + 3] = f13;
    f[2 * 4 + 3] = f23;

    // tmp = F * P
    let tmp = mat4_mul(&f, p);

    // F' (transpose of F)
    let mut ft = [0.0_f32; 16];
    for i in 0..4 {
        for j in 0..4 {
            ft[i * 4 + j] = f[j * 4 + i];
        }
    }

    // result = tmp * F' = F*P*F'
    let mut result = mat4_mul(&tmp, &ft);

    // Add Q diagonal
    result[0 * 4 + 0] += q0;
    result[1 * 4 + 1] += q0;
    result[2 * 4 + 2] += q2;
    result[3 * 4 + 3] += q3;

    result
}

/// Covariance update in Joseph form: `(I-KH)*P*(I-KH)' + K*R*K'`.
///
/// `H = [0, 0, 0, dh_dv]` — only index 3 is nonzero, so only column 3 of
/// `(I-KH)` differs from the identity.
fn joseph_update(p: &[f32; 16], k: &[f32; 4], dh_dv: f32, r: f32) -> [f32; 16] {
    // Build (I - K*H)
    let mut imkh = [0.0_f32; 16];
    imkh[0 * 4 + 0] = 1.0;
    imkh[1 * 4 + 1] = 1.0;
    imkh[2 * 4 + 2] = 1.0;
    imkh[3 * 4 + 3] = 1.0;
    // Subtract K*H — only column 3 is affected
    for i in 0..4 {
        imkh[i * 4 + 3] -= k[i] * dh_dv;
    }

    // tmp = (I-KH) * P
    let tmp = mat4_mul(&imkh, p);

    // (I-KH)'
    let mut imkh_t = [0.0_f32; 16];
    for i in 0..4 {
        for j in 0..4 {
            imkh_t[i * 4 + j] = imkh[j * 4 + i];
        }
    }

    // result = tmp * (I-KH)' = (I-KH)*P*(I-KH)'
    let mut result = mat4_mul(&tmp, &imkh_t);

    // Add K * R * K'  (outer product, scaled by R)
    for i in 0..4 {
        for j in 0..4 {
            result[i * 4 + j] += k[i] * r * k[j];
        }
    }

    result
}

/// Standard 4×4 matrix multiply `A * B`, both row-major.
fn mat4_mul(a: &[f32; 16], b: &[f32; 16]) -> [f32; 16] {
    let mut c = [0.0_f32; 16];
    for i in 0..4 {
        for k in 0..4 {
            let aik = a[i * 4 + k];
            for j in 0..4 {
                c[i * 4 + j] += aik * b[k * 4 + j];
            }
        }
    }
    c
}

/// Wrap angle to `[-π, π]`.
fn wrap_pi(a: f32) -> f32 {
    let mut a = a % (2.0 * PI);
    if a > PI {
        a -= 2.0 * PI;
    } else if a < -PI {
        a += 2.0 * PI;
    }
    a
}
