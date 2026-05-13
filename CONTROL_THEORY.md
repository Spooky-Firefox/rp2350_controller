# Control Theory Notes

This note explains what the control-side code is doing in plain engineering terms.
Relevant source files:

- `src/controller_processor/controller.rs` — Heading observer (bicycle predict-correct), PID, and control logic
- `src/controller_processor/controller_processor_loop.rs` — Core 1 event loop and drive-mode logic

## Big Picture

Core 1 does two jobs:

1. **Estimate** the vehicle heading using a bicycle-model predict-correct observer.
   - **Predict** on encoder ticks: advance heading from steering command and distance traveled.
   - **Correct** on camera updates: fuse low-rate camera angle measurements.
2. **Turn that estimate into actuator commands** according to the current drive mode.

Throttle is open-loop (fixed PWM), and steering is closed-loop on the observer-estimated heading.

The data flow is:

```text
SensorEvent::Encoder { rpm_period_us }
            |
            v
  distance_increment_m = LENGTH_PER_ENCODER_PULSE_METERS
  HeadingObserver.predict(distance, steering_pwm, wheelbase, ...)
            → heading_estimate_deg (advanced via bicycle model)
            |
            v
  SteeringAngleController.update(heading_estimate, distance_increment)
            → steer_pwm_us

SensorEvent::CameraAlign { angle, confidence }
            |
            v
  HeadingObserver.correct(angle) → heading_estimate_deg (fused with camera)

SensorEvent::Distances { left_cm, center_cm, right_cm }
            |
            v
  Turn-trigger detection
  → may switch DriveMode to Turning { direction }

SensorEvent::EncoderTimeout
  → distance_increment_m = 0.0 (steering PID does not step; observer predict skipped)

SensorEvent::ConstantUpdate { constant, value }
  → may update observer tuning (process noise Q, measurement noise R, initial covariance P0)
```

---

## Drive Modes

Core 1 maintains a `DriveMode` state machine with three states:

```text
   STARTUP  ──5 s──►  STRAIGHT  ──turn trigger──►  TURNING
                         ▲                              │
                         └──── angle flip ◄─────────────┘
```

### STARTUP

Outputs neutral PWM (1500 µs) for both steering and throttle for the first 5 seconds after boot.
This gives the vehicle time to settle on the ground before the controller takes over.

### STRAIGHT

- **Throttle**: fixed at 1600 µs.
- **Steering**: driven by `SteeringAngleController` using the latest RLS-filtered camera angle.

### TURNING (currently disabled via `ENABLE_TURNING_MODE = false`)

Fixed open-loop PWM values are applied:
- Throttle: 1600 µs
- Steering left: 1325 µs / Steering right: 1675 µs

Turn entry is triggered when an HC-SR04 left or right sensor shows a large distance jump (≥ 100 cm) or
returns no-value readings for 3 consecutive sweeps — indicating the vehicle has passed the end of a wall.

Turn exit is triggered when the camera angle flips sign beyond ±30° from the sign at turn entry — indicating
the vehicle has completed the corner.

---

## Heading Observer (Bicycle Predict-Correct)

The vehicle heading is estimated using a lightweight **bicycle-model predict-correct observer** that handles sparse camera measurements (~7 FPS) by predicting heading changes between updates.

### Predict Step (on encoder distance increments)

The **kinematic bicycle model** predicts heading change from steering angle and distance traveled:

$$\Delta\theta = \frac{ds}{L} \tan(\delta)$$

where:

- $\Delta\theta$ is the heading change [rad]
- $ds$ is distance traveled since last update [m] — from encoder increments
- $L$ is wheelbase [m]
- $\delta$ is front-wheel steering angle [rad]

The steering angle is derived from the PWM command:

$$\delta = (\text{steering\_pwm\_us} - \text{neutral\_pwm\_us}) \times \text{steering\_angle\_per\_us\_deg} \times \frac{\pi}{180}$$

After prediction, covariance increases by the **process noise** $Q$ to reflect growing uncertainty.

### Correct Step (on camera alignment)

Camera angle measurements are fused using a **scalar Kalman-style correction**:

$$K = \frac{P^-}{P^- + R}$$

$$\hat{\theta} = \hat{\theta}^- + K\,(z - \hat{\theta}^-)$$

$$P = (1 - K) P^-$$

where:

- $\hat{\theta}^-$ is the predicted heading (before correction)
- $z$ is the camera-measured angle
- $K$ is the Kalman gain (0 to 1)
- $P^-$ is the prediction-step covariance (uncertainty)
- $R$ is measurement noise (camera angle uncertainty)
- $P$ is the corrected covariance

**Intuition**: if the camera is noisy ($R$ high), $K$ is small → measurement has little effect. If the model is uncertain ($P^-$ high), $K$ is large → measurement pulls the estimate strongly toward the camera angle.

### Compile-Time Vehicle Geometry Constants

Located in `src/constants.rs`:

| Constant | Default | Typical Range | Notes |
| -------- | ------- | ------------- | ----- |
| `OBSERVER_WHEELBASE_METERS` | 0.12 m | 0.08–0.15 m | Distance between front and rear axles |
| `OBSERVER_STEERING_ANGLE_PER_US_DEG` | 0.1 deg/µs | 0.05–0.15 deg/µs | Servo response: angle change per PWM microsecond from neutral |
| `OBSERVER_NEUTRAL_STEERING_PWM_US` | 1500 µs | 1500 µs | Neutral servo position (straight ahead) |

These are fixed vehicle geometry and do not change at runtime. Measure or estimate them from your platform.

### Runtime Tuning Constants

Three observer parameters can be adjusted at runtime via `const` commands:

| Parameter | USB Command | Range | Default | Effect |
| --------- | ----------- | ----- | ------- | ------ |
| Process noise $Q$ | `const observer_q <value>` | 0.001–1.0 | 0.01 | Higher → trust predictions less, require more camera feedback; lower → smooth predictions dominate |
| Measurement noise $R$ | `const observer_r <value>` | 0.01–2.0 | 0.5 | Higher → trust camera less, rely on model; lower → camera pulls estimate strongly |
| Covariance $P_0$ | `const observer_p0 <value>` | 0.1–5.0 | 1.0 | Initial uncertainty at startup; affects gain weighting in early corrections |

### Tuning Guidance

**Oscillation (steering left-right):** Increase `observer_r` (trust camera less, model more) so camera noise doesn't pull heading estimate around.

**Sluggish response:** Decrease `observer_r` (trust camera more) or increase `observer_q` (lower prediction trust) so camera corrections pull estimate faster.

**Drift between camera frames:** Decrease `observer_q` (higher model trust) so predicted heading changes accumulate; or check wheelbase/steering-angle calibration.

**Noisy camera angle:** Increase `observer_r` to dampen camera noise; the observer will rely more on the prediction from steering+motion.

---

## Steering Angle PID

`SteeringAngleController` runs a standard PID on observer-estimated heading error:

$$e = \theta_{setpoint} - \hat{\theta}_{observer}$$

$$u_{steer} = u_{neutral} - \left( K_p\,e + K_i \int e\,ds + K_d \frac{de}{ds} \right)$$

Note the **integral and derivative step** is the **encoder distance increment** $ds$
(`LENGTH_PER_ENCODER_PULSE_METERS = 13\pi/600` m per Hall rise), not wall-clock time.
This means the PID only integrates while the vehicle is moving.

Output is clamped to [1250, 1750] µs.

| Parameter | Default value | Effect |
| --------- | ------------- | ------ |
| `kp` | 50.0 | Immediate response to angle error |
| `ki` | 0.0 | Steady-state bias correction (currently off) |
| `kd` | 5.0 | Damping — resists fast angle changes |
| `integral_limit` | 100.0 | Anti-windup clamp |
| `angle_setpoint_deg` | 0.0 | Target: drive straight ahead |
| `neutral_steering_pwm_us` | 1500 | Center position |

### Tuning Guidance

- Too much `kp` → steering oscillates left/right at high frequency.
- Too much `kd` → amplifies noise in the observer heading estimate; feels twitchy.
- `ki` is currently 0 because the observer already removes slow drift via predict-correct; re-enable if persistent
  steady-state offset is observed (e.g., systematic camera bias).
- Increase `observer_r` (reduce measurement noise weight) to smooth PID input; decrease to make corrections tighter.

---

## Control Modes (Manual / Auto)

Core 0 holds a `ControlModes` struct with **independent** flags for steering and throttle:

```rust
pub struct ControlModes {
    pub steering: ControlMode,  // Manual or Auto
    pub throttle: ControlMode,  // Manual or Auto
}
```

When a `ControlOutput` arrives from Core 1 via the SIO FIFO:

- If `steering == Auto`, Core 0 applies `steer_pwm_us` to PWM channel A (GPIO 16).
- If `throttle == Auto`, Core 0 applies `power_pwm_us` to PWM channel B (GPIO 17).
- If a channel is `Manual`, the channel is **not overwritten** — it retains whatever was set
  by the most recent `pwm-a` / `pwm-b` USB command.

This lets you, for example, test the steering PID at a fixed throttle:

```
mode throttle manual   ← freeze throttle
pwm-b 1600             ← set throttle manually
mode steering auto     ← let Core 1 control steering
```

---

## Why Bicycle Predict-Correct Instead of EKF?

An Extended Kalman Filter (EKF) was present in earlier revisions for speed estimation and heading tracking.
A scalar predict-correct observer was chosen instead because:

1. **Simplicity and latency**: Bicycle kinematics + scalar Kalman gain is lightweight and runs in microseconds. EKF would require matrix operations.
2. **Camera-only heading need**: We only need to estimate heading, not speed or position. Throttle is open-loop (fixed PWM).
3. **Sparse measurements**: At ~7 FPS, a predict step is crucial to handle gaps between camera updates. The bicycle model elegantly captures steering+motion effects between frames.
4. **Tuning**: Three scalar parameters (process noise, measurement noise, covariance) are easier to reason about than full-state EKF covariance matrices.

A full EKF can be reintroduced if absolute position tracking or speed-closed-loop control becomes a requirement.

---

## Reading the Code

Start in `controller_processor_loop.rs`:

- `core1_task()` — entry point, initialises state, runs the blocking read loop
- `handle_sensor_event()` — dispatches per-event state updates, drives mode transitions, calls `emit_control_output`
- `Core1State::maybe_enter_turning_mode()` — turn-trigger logic using HC-SR04 left/right readings
- `Core1State::maybe_exit_turning_mode()` — turn-exit logic using observer heading flip

Then look in `controller.rs`:

- `HeadingObserver` — bicycle predict and Kalman-style correct methods
- `SteeringAngleController` — PID wrapper with clamped output
- `Pid` — general PID building block
