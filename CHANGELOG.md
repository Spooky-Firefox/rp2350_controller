# Changelog

This file tracks notable repository changes.

## Unreleased

- Restored distance-based steering PID stepping: steering integral/derivative now step on encoder distance increments again (`LENGTH_PER_HAL_RISE_METERS`) instead of `dt_s`.
- Updated steering controller interface to take `distance_increment_m` explicitly.

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
