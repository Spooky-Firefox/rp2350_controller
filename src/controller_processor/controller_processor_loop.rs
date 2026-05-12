//! Core 1 event loop for sensor processing and control.
#![deny(unsafe_code)]

use crate::controller_processor::controller::RaceController;
use crate::ipc::{self, Constants, IpcSignal, SensorKind, TimeExtender};
use crate::logging::{LogData, LogEvent};

use defmt::info;
use fugit::TimerInstantU64;
use heapless::spsc::{Consumer, Producer};
use rp235x_hal as hal;

pub fn core1_task(
    mut sensor_q_rx: Consumer<'static, ipc::SensorEvent, 4>,
    mut control_q_tx: Producer<'static, ipc::ControlOutput, 4>,
    mut log_q_tx: Producer<'static, LogData, 128>,
) -> ! {
    #[allow(unsafe_code)]
    let sio = hal::Sio::new(unsafe { rp235x_pac::Peripherals::steal() }.SIO);
    let mut fifo = sio.fifo;

    fifo.drain();

    let mut controller = RaceController::new();
    let mut time_ext = TimeExtender::new();
    let mut last_control_time_us: Option<u64> = None;

    loop {
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
                SensorKind::ConstantUpdate { constant, value } => {
                    info!(
                        "core1 rx constant: kind={:?} value={}",
                        defmt::Debug2Format(&constant),
                        value
                    );
                    if matches!(
                        constant,
                        Constants::SpeedSetpoint
                            | Constants::SpeedKp
                            | Constants::SpeedKi
                            | Constants::SpeedKd
                            | Constants::SteeringKp
                            | Constants::SteeringKi
                            | Constants::SteeringKd
                    ) {
                        controller.apply_constant_update(constant, value);
                    }
                }
                SensorKind::CameraAlign { angle, confidence } => {
                    info!("core1 rx camera: angle={} conf={}", angle, confidence);
                    controller.on_camera_align(angle, t_us);
                }
                SensorKind::Distances {
                    left_cm,
                    center_cm,
                    right_cm,
                } => {
                    info!(
                        "core1 rx distances: left_cm={} center_cm={} right_cm={}",
                        left_cm,
                        center_cm,
                        right_cm
                    );
                    controller.on_distances(left_cm, right_cm, t_us);
                }
                SensorKind::Encoder { steer, rpm_period_us } => {
                    info!(
                        "core1 rx encoder: steer={} period_us={}",
                        steer,
                        rpm_period_us
                    );
                    let (steer_pwm_us, power_pwm_us, error_value, output_mode) =
                        controller.output(t_us, dt_s);

                    log_q_tx
                        .enqueue(LogData {
                            timestamp: now,
                            event: LogEvent::Controller {
                                steer_value_us: steer_pwm_us,
                                throttle_value_us: power_pwm_us,
                                setpoint_value_mps: 0.0,
                                error_value,
                                kalman_values: [0.0; 4],
                            },
                        })
                        .ok();
                    fifo.write_blocking(IpcSignal::LogReady as u32);

                    control_q_tx
                        .enqueue(ipc::ControlOutput {
                            steer_pwm_us,
                            power_pwm_us,
                        })
                        .ok();
                    fifo.write_blocking(IpcSignal::ControlReady as u32);
                    info!(
                        "core1 tx control: mode={:?} steer_us={} throttle_us={} err={}",
                        defmt::Debug2Format(&output_mode),
                        steer_pwm_us,
                        power_pwm_us,
                        error_value
                    );
                }
                SensorKind::EncoderTimeout { steer } => {
                    info!("core1 rx encoder-timeout: steer={}", steer);
                    let (steer_pwm_us, power_pwm_us, error_value, output_mode) =
                        controller.output(t_us, dt_s);

                    log_q_tx
                        .enqueue(LogData {
                            timestamp: now,
                            event: LogEvent::Controller {
                                steer_value_us: steer_pwm_us,
                                throttle_value_us: power_pwm_us,
                                setpoint_value_mps: 0.0,
                                error_value,
                                kalman_values: [0.0; 4],
                            },
                        })
                        .ok();
                    fifo.write_blocking(IpcSignal::LogReady as u32);

                    control_q_tx
                        .enqueue(ipc::ControlOutput {
                            steer_pwm_us,
                            power_pwm_us,
                        })
                        .ok();
                    fifo.write_blocking(IpcSignal::ControlReady as u32);
                    info!(
                        "core1 tx control(timeout): mode={:?} steer_us={} throttle_us={} err={}",
                        defmt::Debug2Format(&output_mode),
                        steer_pwm_us,
                        power_pwm_us,
                        error_value
                    );
                }
            }
        }
    }
}
