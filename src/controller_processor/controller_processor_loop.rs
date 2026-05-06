//! Core 1 event loop for sensor processing and control.
#![deny(unsafe_code)]

use crate::controller_processor::controller::{Controller, StraightLineSpeedController, SteeringDistanceController};
use crate::controller_processor::kalman_filter;
use crate::ipc::{self, ControlEvent, SensorEvent, TimeExtender};

use core::f32::consts::PI;
use defmt::info;
use fugit::TimerInstantU64;
use rp235x_hal as hal;

/// Arc length per encoder magnet pulse [m].
const LENGTH_PER_HAL_RISE_METERS: f32 = 13.0 * PI / 300.0;

/// Entry point for Core 1.  Called from the RTIC `init` on Core 0 via
/// `core.spawn(...)`.  Runs a blocking event loop that never returns.
///
/// This module does not deny unsafe, as it must steal the PAC SIO peripheral.
pub fn core1_task() -> ! {
    // Core 1 steals the PAC peripheral (Peripherals::steal()). This is the standard
    // pattern on RP2350 because SIO is per-core hardware: each core has its own SIO view.
    // Safe because: (1) PAC prevents multi-core data races (SIO regs are not shared),
    // (2) Core 0 doesn't access SIO (uses HAL singletons), (3) Core 1 accesses only its
    // own SIO address space, mapped by the hardware to the executing core.
    #[allow(unsafe_code)]
    let sio = hal::Sio::new(unsafe { rp235x_pac::Peripherals::steal() }.SIO);
    let mut channel = ipc::FifoChannel::new(sio.fifo);

    // Drain any stale FIFO data left over from boot / previous run.
    channel.drain();

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
        speed_setpoint_mps: 0.5,
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
        min_distance_cm: 30.0, // Activate steering if center sensor < 30cm
        last_error: 0.0,
        last_proportional: 0.0,
        last_integral: 0.0,
        last_derivative: 0.0,
    };

    // Filter is lazily initialised on the first event so that t0 is correct.
    let mut filter: Option<kalman_filter::EkfFilter> = None;
    let mut time_ext = TimeExtender::new();
    let mut last_control_time_us: Option<u64> = None;
    let mut last_setpoint_bits = f32::INFINITY.to_bits();
    let mut last_distance_pwm_us: u16 = 1500;

    // ── Main event loop ──────────────────────────────────────────────────
    loop {
        if let Some(event) = channel.try_recv_sensor_event() {
            let t_us = time_ext.extend(event.t32_us);
            let now = TimerInstantU64::<1_000_000>::from_ticks(t_us);
            let dt_s = last_control_time_us
                .map(|last| t_us.saturating_sub(last) as f32 * 1e-6)
                .unwrap_or(0.0);
            last_control_time_us = Some(t_us);

            if event.setpoint_mps.to_bits() != last_setpoint_bits {
                controller.set_speed_setpoint(event.setpoint_mps);
                last_setpoint_bits = event.setpoint_mps.to_bits();
            }

            // Check if this is a distance sensor event or a speed/encoder event
            // Distance events: values[2] is finite (3 distance measurements)
            // Speed events: values[2] is INFINITY
            let is_distance_event = event.values[2].is_finite();

            if is_distance_event {
                // ── Handle Distance Sensor Event ──────────────────────────────
                let dist_left_cm = event.values[0];
                let dist_center_cm = event.values[1];
                let dist_right_cm = event.values[2];

                // Run the distance-based steering controller
                let steering_pwm = distance_controller.update(dist_left_cm, dist_center_cm, dist_right_cm, dt_s);
                last_distance_pwm_us = steering_pwm;

                info!(
                    "Distance steering: L={} cm, C={} cm, R={} cm, PWM={} us, error={}",
                    dist_left_cm as i32,
                    dist_center_cm as i32,
                    dist_right_cm as i32,
                    steering_pwm,
                    distance_controller.last_error as i32
                );

                // Send steering telemetry
                channel.send_control_event_blocking(&ControlEvent::Pid {
                    error: distance_controller.last_error,
                    proportional: distance_controller.last_proportional,
                    integral: distance_controller.last_integral,
                    derivative: distance_controller.last_derivative,
                });
            } else {
                // ── Handle Speed/Encoder Event ────────────────────────────────
                // Lazy-init filter on first event.
                let filt = filter
                    .get_or_insert_with(|| kalman_filter::EkfFilter::new(kalman_const, x0, p0, now));

                process_event(filt, &event, now);
                let speed = LENGTH_PER_HAL_RISE_METERS / (event.values[1] * 1e-6); // Convert encoder period → speed for controller input.
                // Run the controller and send result back to Core 0.
                let [_steer_pwm_us, power_pwm_us] = controller.update(speed, dt_s);

                // Use steering PWM from distance controller if available, otherwise neutral
                let steer_pwm_us = last_distance_pwm_us;

                channel.send_control_event_blocking(&ControlEvent::Pid {
                    error: controller.last_error,
                    proportional: controller.last_proportional,
                    integral: controller.last_integral,
                    derivative: controller.last_derivative,
                });
                channel.send_control_event_blocking(&ControlEvent::Control {
                    steer_pwm_us,
                    power_pwm_us,
                });
            }
        } else {
            // No data available — sleep until the other core signals via SEV.
            cortex_m::asm::wfe();
        }
    }
}

/// Dispatch a [`SensorEvent`] to the appropriate EKF entry point.
fn process_event(
    filter: &mut kalman_filter::EkfFilter,
    event: &SensorEvent,
    now: TimerInstantU64<1_000_000>,
) {
    let steer = event.values[0];
    let rpm_period_us = event.values[1];

    filter.set_control(steer, 0.0);
    // Temporary fix to differentiate between "no data" and "data" events
    if rpm_period_us.is_finite() && rpm_period_us > 0.0 {
        // Convert encoder edge period [µs] → longitudinal speed [m/s].
        let period_s = rpm_period_us * 1e-6;
        let speed_mps = LENGTH_PER_HAL_RISE_METERS / period_s;
        filter.on_speed_sample(speed_mps, now);
    } else {
        filter.on_timeout(now);
    }
}
