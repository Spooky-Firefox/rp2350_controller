# Control Theory Notes

This note explains what the control-side code is doing in plain engineering terms.
Relevant source files:

- `src/controller_processor/controller.rs` — PID and RLS filter structs
- `src/controller_processor/controller_processor_loop.rs` — Core 1 event loop and drive-mode logic

## Big Picture

Core 1 does two jobs:

1. **Estimate** the camera-derived steering angle (via RLS filtering).
2. **Turn that estimate into actuator commands** according to the current drive mode.

There is no Kalman filter. Throttle is open-loop (fixed PWM), and steering is closed-loop on camera angle.

The data flow is:

```text
SensorEvent::CameraAlign { angle, confidence }
            |
            v
  RlsAngleFilter.update(angle) → filtered_angle_deg
            |
            v
  SteeringAngleController.update(filtered_angle, distance_increment)
            → steer_pwm_us

SensorEvent::Encoder { rpm_period_us }
            |
            v
  distance_increment_m = LENGTH_PER_HAL_RISE_METERS
  (used as the PID step size for the steering controller above)

SensorEvent::Distances { left_cm, center_cm, right_cm }
            |
            v
  Turn-trigger detection
  → may switch DriveMode to Turning { direction }

SensorEvent::EncoderTimeout
  → distance_increment_m = 0.0 (steering PID does not step)
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

## RLS Angle Filter

The camera angle arriving over USB can be noisy. The **Recursive Least Squares (RLS)** filter smooths it
adaptively while still tracking genuine changes.

The scalar update equations are:

$$K_n = \frac{P_{n-1}}{\lambda + P_{n-1}}$$

$$\hat{\theta}_n = \hat{\theta}_{n-1} + K_n\,(z_n - \hat{\theta}_{n-1})$$

$$P_n = \frac{P_{n-1} - K_n P_{n-1}}{\lambda}$$

where:

- $z_n$ is the raw measured angle
- $\hat{\theta}_n$ is the filtered angle estimate
- $P_n$ is the estimate covariance (confidence)
- $K_n$ is the adaptive gain (Kalman-style)
- $\lambda \in (0,1]$ is the **forgetting factor**

Tuning knobs (set in `controller_processor_loop.rs`):

| Parameter | Value | Effect |
| --------- | ----- | ------ |
| `lambda` | 0.98 | Close to 1 → stable, slow to adapt; lower → faster but noisier |
| `measurement_noise_variance` | 0.5 | Higher → trust model more, smoother output |

When $\lambda = 1$ the filter is equivalent to a simple running average with no forgetting.
The current value of 0.98 means measurements older than ~50 samples contribute negligibly.

---

## Steering Angle PID

`SteeringAngleController` runs a standard PID on camera-angle error:

$$e = \theta_{setpoint} - \theta_{filtered}$$

$$u_{steer} = u_{neutral} - \left( K_p\,e + K_i \int e\,ds + K_d \frac{de}{ds} \right)$$

Note the **integral and derivative step** is the **encoder distance increment** $ds$
(`LENGTH_PER_HAL_RISE_METERS = 13\pi/600` m per Hall rise), not wall-clock time.
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
- Too much `kd` → amplifies noise in the camera angle; feels twitchy.
- `ki` is currently 0 because the RLS filter already removes slow drift; re-enable if persistent
  steady-state offset is observed.
- Increase `lambda` toward 1.0 for smoother angle tracking on a clean surface; decrease toward 0.95
  for faster adaptation when the camera angle source is high-latency.

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

## Why No Kalman Filter?

An Extended Kalman Filter (EKF) was present in earlier revisions for speed estimation and heading tracking.
It was removed because:

1. Throttle is now open-loop — there is no speed setpoint to track via EKF.
2. Camera angle from the `align` command already provides heading directly; dead reckoning is not needed.
3. The RLS scalar filter is simpler to tune and sufficient for the current noise level.

The EKF infrastructure may be reintroduced if absolute position tracking or speed-closed-loop control
becomes a requirement.

---

## Reading the Code

Start in `controller_processor_loop.rs`:

- `core1_task()` — entry point, initialises state, runs the blocking read loop
- `handle_sensor_event()` — dispatches per-event state updates, drives mode transitions, calls `emit_control_output`
- `Core1State::maybe_enter_turning_mode()` — turn-trigger logic using HC-SR04 left/right readings
- `Core1State::maybe_exit_turning_mode()` — turn-exit logic using camera angle flip

Then look in `controller.rs`:

- `SteeringAngleController` — PID wrapper with clamped output
- `RlsAngleFilter` — scalar RLS estimator
- `Pid` — general PID building block
