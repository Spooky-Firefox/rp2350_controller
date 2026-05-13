# Changelog

This file tracks notable repository changes.

## spooky-firefox 2026-05-13

### Heading Observer (Bicycle Predict-Correct)

- Implemented a lightweight bicycle-model predict-correct observer to handle low-rate camera updates (~7 FPS).
- **Predict step**: Uses kinematic bicycle model on encoder distance increments to advance heading estimate.
- **Correct step**: Fuses camera angle measurements using scalar Kalman-style gain weighted by covariance and measurement noise.
- Replaced the measurement-only RLS filter with the dual-step observer; heading estimate is now continuous even without camera frames.
- Added three new runtime-tunable constants: `observer_q` (process noise), `observer_r` (measurement noise), `observer_p0` (initial covariance).
- Added compile-time vehicle geometry constants: `OBSERVER_WHEELBASE_METERS`, `OBSERVER_STEERING_ANGLE_PER_US_DEG`, `OBSERVER_NEUTRAL_STEERING_PWM_US`.

### Documentation Updates

- Updated `README.md`: replaced stale EKF references with observer-based steering estimation explanation; updated source layout and USB command table.
- Updated `HOW_IT_WORKS.md`: added "Heading Estimation: Bicycle-Model Predict-Correct Observer" section and revised Core 1 event-flow table to reflect observer predict/correct steps.
- Updated `CONTROL_THEORY.md`: replaced RLS-only filter theory with complete bicycle-model observer equations, compile-time and runtime tuning constants, and updated "Why No Kalman Filter?" rationale.
- Updated `CHANGELOG.md` (this file): documented observer implementation and documentation corrections.

Files changed:

- `src/controller_processor/controller.rs` — added `HeadingObserver` struct with `predict()`, `correct()`, and `reset()` methods
- `src/controller_processor/controller_processor_loop.rs` — replaced RLS filter with observer; updated event handlers for predict (Encoder) and correct (CameraAlign); added observer constant updates
- `src/ipc.rs` — extended `Constants` enum with `ObserverProcessNoise`, `ObserverMeasurementNoise`, `ObserverCovarianceInit`
- `src/constants.rs` — added observer-related compile-time constants
- `README.md`, `HOW_IT_WORKS.md`, `CONTROL_THEORY.md`, `CHANGELOG.md` — documentation updates

## 2026-05-12

### svenviktorm - `d8d73906bcc2b5f53880276d7f97c33e86768a1c`

- Added an RLS-based angle filter for camera alignment smoothing.
- Updated the controller loop to use and reset the new filter during mode transitions.
- Removed the previous Kalman filter module.
- Added camera alignment data and a MATLAB plotting script for analysis.

Files changed:

- `controll/data/camera_alignment_1778588231549(1).csv`
- `controll/plot_camera_degrees.m`
- `src/controller_processor.rs`
- `src/controller_processor/controller.rs`
- `src/controller_processor/controller_processor_loop.rs`
- `src/controller_processor/kalman_filter.rs` (deleted)
- `src/main.rs`
