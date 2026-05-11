//! Core 1 event loop for sensor processing and control.
#![deny(unsafe_code)]

use crate::controller_processor::controller::{
    Controller, Pid, SteeringAngleController, StraightLineSpeedController,
};
use crate::controller_processor::kalman_filter;
use crate::ipc::{self, Constants, IpcSignal, SensorKind, TimeExtender};
use crate::logging::{LogData, LogEvent};

use core::f32::consts::PI;
use fugit::TimerInstantU64;
use heapless::spsc::{Consumer, Producer};
use rp235x_hal as hal;

type Instant = TimerInstantU64<1_000_000>;

/// Arc length per encoder magnet pulse [m].
const LENGTH_PER_HAL_RISE_METERS: f32 = 13.0 * PI / 600.0;

struct Core1State {
    speed_controller: StraightLineSpeedController,
    steering_controller: SteeringAngleController,
    filter: Option<kalman_filter::EkfFilter>,
    kalman_const: kalman_filter::EkfConst,
    kalman_x0: [f32; 4],
    kalman_p0: [f32; 16],
    current_setpoint_mps: f32,
    latest_camera_angle_rad: Option<f32>,
}

struct Core1Io<'a> {
    control_q_tx: &'a mut Producer<'static, ipc::ControlOutput, 4>,
    log_q_tx: &'a mut Producer<'static, LogData, 128>,
    fifo: &'a mut hal::sio::SioFifo,
}

impl Core1State {
    fn filter_mut(&mut self, now: Instant) -> &mut kalman_filter::EkfFilter {
        self.filter.get_or_insert_with(|| {
            kalman_filter::EkfFilter::new(self.kalman_const, self.kalman_x0, self.kalman_p0, now)
        })
    }

    fn steering_measurement_rad(&self, filtered_heading_rad: f32) -> f32 {
        self.latest_camera_angle_rad.unwrap_or(filtered_heading_rad)
    }
}

pub fn core1_task(
    mut sensor_q_rx: Consumer<'static, ipc::SensorEvent, 4>,
    mut control_q_tx: Producer<'static, ipc::ControlOutput, 4>,
    mut log_q_tx: Producer<'static, LogData, 128>,
) -> ! {
    #[allow(unsafe_code)]
    let sio = hal::Sio::new(unsafe { rp235x_pac::Peripherals::steal() }.SIO);
    let mut fifo = sio.fifo;

    fifo.drain();

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

    let mut state = Core1State {
        speed_controller: StraightLineSpeedController {
            pid: Pid::new(75.0, 5.0, 0.0, 50.0),
            distance_setpoint_m: 0.5,
            measured_distance_m: 0.0,
            steering_pwm_us: 1500,
            neutral_power_pwm_us: 1500,
        },
        steering_controller: SteeringAngleController {
            pid: Pid::new(50.0, 0.0, 5.0, 100.0),
            angle_setpoint_rad: 0.0,
            neutral_steering_pwm_us: 1500,
            min_steering_pwm_us: 1250,
            max_steering_pwm_us: 1750,
        },
        filter: None,
        kalman_const,
        kalman_x0: x0,
        kalman_p0: p0,
        current_setpoint_mps: 0.0,
        latest_camera_angle_rad: None,
    };

    let mut time_ext = TimeExtender::new();
    let mut last_control_time_us: Option<u64> = None;

    loop {
        let word = fifo.read_blocking();
        if !matches!(IpcSignal::from_u32(word), Some(IpcSignal::SensorReady)) {
            continue;
        }

        let Some(event) = sensor_q_rx.dequeue() else {
            continue;
        };

        let t_us = time_ext.extend(event.t32_us);
        let now = Instant::from_ticks(t_us);
        let dt_s = last_control_time_us
            .map(|last| t_us.saturating_sub(last) as f32 * 1e-6)
            .unwrap_or(0.0);
        last_control_time_us = Some(t_us);

        let mut io = Core1Io {
            control_q_tx: &mut control_q_tx,
            log_q_tx: &mut log_q_tx,
            fifo: &mut fifo,
        };

        handle_sensor_event(&mut state, event.kind, now, dt_s, &mut io);
    }
}

fn handle_sensor_event(
    state: &mut Core1State,
    event: SensorKind,
    now: Instant,
    dt_s: f32,
    io: &mut Core1Io<'_>,
) {
    let distance_increment_m = match event {
        SensorKind::Encoder { rpm_period_us } => {
            let filter = state.filter_mut(now);
            let _measured_speed_mps = process_encoder_event(filter, Some(rpm_period_us), now);
            LENGTH_PER_HAL_RISE_METERS
        }
        SensorKind::EncoderTimeout => {
            let filter = state.filter_mut(now);
            let _measured_speed_mps = process_encoder_event(filter, None, now);
            0.0
        }
        _ => {
            state.filter_mut(now).on_timeout(now);
            0.0
        }
    };

    match event {
        SensorKind::ConstantUpdate { constant, value } => match constant {
            Constants::SpeedSetpoint => {
                state.current_setpoint_mps = value;
                state.speed_controller.set_speed_setpoint(value);
            }
            Constants::SpeedKp => state.speed_controller.pid.kp = value,
            Constants::SpeedKi => state.speed_controller.pid.ki = value,
            Constants::SpeedKd => state.speed_controller.pid.kd = value,
            Constants::SteeringKp => state.steering_controller.pid.kp = value,
            Constants::SteeringKi => state.steering_controller.pid.ki = value,
            Constants::SteeringKd => state.steering_controller.pid.kd = value,
        },
        SensorKind::CameraAlign { angle, confidence } => {
            let _ = confidence;
            state.latest_camera_angle_rad = Some(angle);
        }
        SensorKind::Distances {
            left_cm,
            center_cm,
            right_cm,
        } => {
            let _ = (left_cm, center_cm, right_cm);
            // Wall-distance logic will eventually map these values into
            // `steering_controller.angle_setpoint_rad`.
        }
        SensorKind::Encoder { .. } | SensorKind::EncoderTimeout => {}
    }

    let (filtered_heading_rad, kalman_values) = {
        let filter = state.filter_mut(now);
        (filter.heading(), filter.state())
    };

    let [_steer_pwm_us, power_pwm_us] = state.speed_controller.update(distance_increment_m, dt_s);
    let measured_angle_rad = state.steering_measurement_rad(filtered_heading_rad);
    let steer_pwm_us = state
        .steering_controller
        .update(measured_angle_rad, distance_increment_m);

    emit_control_output(state, now, steer_pwm_us, power_pwm_us, kalman_values, io);
}

fn emit_control_output(
    state: &Core1State,
    now: Instant,
    steer_pwm_us: u16,
    power_pwm_us: u16,
    kalman_values: [f32; 4],
    io: &mut Core1Io<'_>,
) {
    io.log_q_tx
        .enqueue(LogData {
            timestamp: now,
            event: LogEvent::Controller {
                steer_value_us: steer_pwm_us,
                throttle_value_us: power_pwm_us,
                setpoint_value_mps: state.current_setpoint_mps,
                error_value: state.speed_controller.pid.last_error,
                kalman_values,
            },
        })
        .ok();
    io.fifo.write_blocking(IpcSignal::LogReady as u32);

    io.control_q_tx
        .enqueue(ipc::ControlOutput {
            steer_pwm_us,
            power_pwm_us,
        })
        .ok();
    io.fifo.write_blocking(IpcSignal::ControlReady as u32);
}

fn process_encoder_event(
    filter: &mut kalman_filter::EkfFilter,
    rpm_period_us: Option<f32>,
    now: Instant,
) -> f32 {
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
