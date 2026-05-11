//! Core 1 event loop for sensor processing and control.
#![deny(unsafe_code)]

use crate::controller_processor::controller::{Controller, StraightLineSpeedController, SteeringDistanceController};
use crate::controller_processor::kalman_filter;
use crate::ipc::{self, IpcSignal, SensorKind, TimeExtender};
use crate::logging::LogData;

use core::f32::consts::PI;
use fugit::TimerInstantU64;
use heapless::spsc::{Consumer, Producer};
use rp235x_hal as hal;

/// Arc length per encoder magnet pulse [m].
const LENGTH_PER_HAL_RISE_METERS: f32 = 13.0 * PI / 600.0;

/// Entry point for Core 1.  Called from the RTIC `init` on Core 0 via
/// `core.spawn(...)`.  Runs a blocking event loop that never returns.
///
/// This module does not deny unsafe, as it must steal the PAC SIO peripheral.
pub fn core1_task(
    mut sensor_q_rx: Consumer<'static, ipc::SensorEvent, 4>,
    mut control_q_tx: Producer<'static, ipc::ControlOutput, 4>,
    mut log_q_tx: Producer<'static, LogData, 128>,
) -> ! {
    // Core 1 steals the PAC peripheral (Peripherals::steal()). This is the standard
    // pattern on RP2350 because SIO is per-core hardware: each core has its own SIO view.
    // Safe because: (1) PAC prevents multi-core data races (SIO regs are not shared),
    // (2) Core 0 doesn't access SIO (uses HAL singletons), (3) Core 1 accesses only its
    // own SIO address space, mapped by the hardware to the executing core.
    #[allow(unsafe_code)]
    let sio = hal::Sio::new(unsafe { rp235x_pac::Peripherals::steal() }.SIO);
    let mut fifo = sio.fifo;

    // Drain any stale FIFO data left over from boot / previous run.
    fifo.drain();

    // ── Kalman filter constants ──────────────────────────────────────────
    let kalman_const = kalman_filter::EkfConst {
        l: 0.2,
        q_pos: 0.01,
        q_theta: 0.01,
        q_v: 0.01,
        r_speed: 0.05,
        eps_v: 0.01,
        dt_max_us: 20000,
    };

    let x0: [f32; 4] = [0.0; 4];

    let p0: [f32; 16] = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];

    // ── Controller ───────────────────────────────────────────────────────
    let mut controller = StraightLineSpeedController {
        kp: 75.0,
        ki: 5.0,
        kd: 0.0,
        distance_setpoint_m: 0.5,
        measured_distance_m: 0.0,
        integral_error: 0.0,
        previous_error: 0.0,
        integral_limit: 50.0,
        steering_pwm_us: 1500,
        neutral_power_pwm_us: 1500,
        last_error: 0.0,
        last_proportional: 0.0,
        last_integral: 0.0,
        last_derivative: 0.0,
    };

    // ── Steering Distance Controller ─────────────────────────────────────
    let mut distance_controller = SteeringDistanceController {
        kp: 50.0,
        ki: 2.0,
        kd: 5.0,
        integral_error: 0.0,
        previous_error: 0.0,
        integral_limit: 100.0,
        neutral_steering_pwm_us: 1500,
        min_distance_cm: 30.0,
        last_error: 0.0,
        last_proportional: 0.0,
        last_integral: 0.0,
        last_derivative: 0.0,
    };

    // Filter is lazily initialised on the first event so that t0 is correct.
    let mut filter: Option<kalman_filter::EkfFilter> = None;
    let mut time_ext = TimeExtender::new();
    let mut last_control_time_us: Option<u64> = None;
    let mut current_setpoint_mps: f32 = 0.0;
    let mut last_distance_pwm_us: u16 = 1500;

    // ── Main event loop ──────────────────────────────────────────────────
    loop {
        // Block until Core 0 signals that new data is available.
        let word = fifo.read_blocking();
        if let Some(IpcSignal::SensorReady) = IpcSignal::from_u32(word)
            && let Some(event) = sensor_q_rx.dequeue()
        {
            let t_us = time_ext.extend(event.t32_us);
            let now = TimerInstantU64::<1_000_000>::from_ticks(t_us);
            let dt_s = last_control_time_us
                .map(|last| t_us.saturating_sub(last) as f32 * 1e-6)
                .unwrap_or(0.0);
            last_control_time_us = Some(t_us);

            match event.kind {
                SensorKind::SetpointUpdate { setpoint_mps } => {
                    // ── Handle Setpoint Change ────────────────────────────────
                    current_setpoint_mps = setpoint_mps;
                    controller.set_speed_setpoint(setpoint_mps);
                }
                SensorKind::Distances { left_cm, center_cm, right_cm } => {
                    // ── Handle Distance Sensor Event ──────────────────────────
                    last_distance_pwm_us =
                        distance_controller.update(left_cm, center_cm, right_cm, dt_s);
                }
                SensorKind::Encoder { steer, rpm_period_us } => {
                    // ── Handle Encoder Pulse ──────────────────────────────────
                    let filt = filter
                        .get_or_insert_with(|| kalman_filter::EkfFilter::new(kalman_const, x0, p0, now));

                    let measured_speed_mps =
                        process_encoder_event(filt, steer, Some(rpm_period_us), now);
                    let [_steer_pwm_us, power_pwm_us] =
                        controller.update(LENGTH_PER_HAL_RISE_METERS, dt_s);
                    let steer_pwm_us = last_distance_pwm_us;

                    log_q_tx
                        .enqueue(LogData {
                            timestamp: now,
                            steer_value_ms: steer_pwm_us,
                            throttle_value_ms: power_pwm_us,
                            setpoint_value: current_setpoint_mps,
                            error_value: controller.last_error,
                            speed_value: measured_speed_mps,
                            kalman_values: filt.state(),
                        })
                        .ok();
                    fifo.write_blocking(IpcSignal::LogReady as u32);

                    control_q_tx
                        .enqueue(ipc::ControlOutput { steer_pwm_us, power_pwm_us })
                        .ok();
                    fifo.write_blocking(IpcSignal::ControlReady as u32);
                }
                SensorKind::EncoderTimeout { steer } => {
                    // ── Handle Encoder Timeout (vehicle stopped) ──────────────
                    let filt = filter
                        .get_or_insert_with(|| kalman_filter::EkfFilter::new(kalman_const, x0, p0, now));

                    let measured_speed_mps =
                        process_encoder_event(filt, steer, None, now);
                    let [_steer_pwm_us, power_pwm_us] = controller.update(0.0, dt_s);
                    let steer_pwm_us = last_distance_pwm_us;

                    log_q_tx
                        .enqueue(LogData {
                            timestamp: now,
                            steer_value_ms: steer_pwm_us,
                            throttle_value_ms: power_pwm_us,
                            setpoint_value: current_setpoint_mps,
                            error_value: controller.last_error,
                            speed_value: measured_speed_mps,
                            kalman_values: filt.state(),
                        })
                        .ok();
                    fifo.write_blocking(IpcSignal::LogReady as u32);

                    control_q_tx
                        .enqueue(ipc::ControlOutput { steer_pwm_us, power_pwm_us })
                        .ok();
                    fifo.write_blocking(IpcSignal::ControlReady as u32);
                }
            }
        }
    }
}

/// Update the EKF for an encoder event.
/// `rpm_period_us` is `Some(period)` for a real pulse, `None` for a timeout.
/// Returns the measured longitudinal speed [m/s] (0.0 if no pulse).
fn process_encoder_event(
    filter: &mut kalman_filter::EkfFilter,
    steer: f32,
    rpm_period_us: Option<f32>,
    now: TimerInstantU64<1_000_000>,
) -> f32 {
    filter.set_control(steer, 0.0);
    match rpm_period_us {
        Some(period_us) => {
            let speed_mps = LENGTH_PER_HAL_RISE_METERS / (period_us * 1e-6);
            filter.on_speed_sample(speed_mps, now);
            speed_mps
        }
        None => {
            filter.on_timeout(now);
            0.0
        }
    }
}
