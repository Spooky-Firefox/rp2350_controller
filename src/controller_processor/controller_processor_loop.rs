//! Core 1 event loop for sensor processing and control.
#![deny(unsafe_code)]

use crate::controller_processor::controller::{Pid, RlsAngleFilter, SteeringAngleController};
use crate::ipc::{self, Constants, IpcSignal, SensorKind, TimeExtender};
use crate::logging::{LogData, LogEvent};

use defmt::info;
use fugit::TimerInstantU64;
use heapless::spsc::{Consumer, Producer};
use rp235x_hal as hal;

type Instant = TimerInstantU64<1_000_000>;

const NEUTRAL_PWM_US: u16 = 1500;
const STARTUP_DURATION_US: u64 = 5_000_000;

// Straight driving mode outputs.
const STRAIGHT_THROTTLE_PWM_US: u16 = 1600;

// Turning mode outputs.
const TURN_THROTTLE_PWM_US: u16 = 1600;
const TURN_LEFT_STEER_PWM_US: u16 = 1325;
const TURN_RIGHT_STEER_PWM_US: u16 = 1675;

// Turn trigger and exit thresholds.
const TURN_TRIGGER_INCREASE_CM: f32 = 100.0;
const TURN_TRIGGER_NO_VALUE_STREAK: u8 = 3;
const ANGLE_FLIP_THRESHOLD_DEG: f32 = 30.0;

// Set to `false` to permanently disable turning mode.
const ENABLE_TURNING_MODE: bool = false;

#[derive(Clone, Copy)]
enum TurnDirection {
    Left,
    Right,
}

#[derive(Clone, Copy)]
enum DriveMode {
    Startup,
    Straight,
    Turning {
        direction: TurnDirection,
        start_angle_sign: i8,
    },
}

struct Core1State {
    steering_controller: SteeringAngleController,
    rls_angle_filter: RlsAngleFilter,
    mode: DriveMode,
    startup_until_us: Option<u64>,
    latest_camera_angle_deg: Option<f32>,
    last_left_cm: Option<f32>,
    last_right_cm: Option<f32>,
    left_no_value_streak: u8,
    right_no_value_streak: u8,
}

struct Core1Io<'a> {
    control_q_tx: &'a mut Producer<'static, ipc::ControlOutput, 4>,
    log_q_tx: &'a mut Producer<'static, LogData, 128>,
    fifo: &'a mut hal::sio::SioFifo,
}

impl Core1State {
    fn active_angle_deg(&self) -> f32 {
        self.latest_camera_angle_deg.unwrap_or(0.0)
    }

    fn maybe_enter_turning_mode(&mut self, left_cm: f32, right_cm: f32) {
        let left_no_value = !left_cm.is_finite();
        let right_no_value = !right_cm.is_finite();

        if left_no_value {
            self.left_no_value_streak = self.left_no_value_streak.saturating_add(1);
        } else {
            self.left_no_value_streak = 0;
        }

        if right_no_value {
            self.right_no_value_streak = self.right_no_value_streak.saturating_add(1);
        } else {
            self.right_no_value_streak = 0;
        }

        let left_jump = self
            .last_left_cm
            .map(|last| {
                left_cm.is_finite() && last.is_finite() && left_cm - last >= TURN_TRIGGER_INCREASE_CM
            })
            .unwrap_or(false);
        let right_jump = self
            .last_right_cm
            .map(|last| {
                right_cm.is_finite()
                    && last.is_finite()
                    && right_cm - last >= TURN_TRIGGER_INCREASE_CM
            })
            .unwrap_or(false);

        let left_no_value_trigger = self.left_no_value_streak >= TURN_TRIGGER_NO_VALUE_STREAK;
        let right_no_value_trigger = self.right_no_value_streak >= TURN_TRIGGER_NO_VALUE_STREAK;

        let left_trigger = left_jump || left_no_value_trigger;
        let right_trigger = right_jump || right_no_value_trigger;

        self.last_left_cm = Some(left_cm);
        self.last_right_cm = Some(right_cm);

        let direction = match (left_trigger, right_trigger) {
            (true, false) => Some(TurnDirection::Left),
            (false, true) => Some(TurnDirection::Right),
            (true, true) => {
                if self.left_no_value_streak > self.right_no_value_streak {
                    Some(TurnDirection::Left)
                } else if self.right_no_value_streak > self.left_no_value_streak {
                    Some(TurnDirection::Right)
                } else if left_cm >= right_cm {
                    Some(TurnDirection::Left)
                } else {
                    Some(TurnDirection::Right)
                }
            }
            (false, false) => None,
        };

        let Some(direction) = direction else {
            return;
        };

        info!(
            "Turn trigger detected: left_cm={} right_cm={} left_jump={} right_jump={} left_no_value_streak={} right_no_value_streak={} direction={} enable_turning={}",
            left_cm,
            right_cm,
            left_jump,
            right_jump,
            self.left_no_value_streak,
            self.right_no_value_streak,
            turn_direction_name(direction),
            ENABLE_TURNING_MODE
        );

        if !ENABLE_TURNING_MODE {
            info!(
                "Turning mode disabled; would switch to TURNING({}) but staying in {}",
                turn_direction_name(direction),
                drive_mode_name(self.mode)
            );
            return;
        }

        if !matches!(self.mode, DriveMode::Straight) {
            return;
        }

        let start_angle_sign = self
            .latest_camera_angle_deg
            .map(|angle| if angle >= 0.0 { 1 } else { -1 })
            .unwrap_or(match direction {
                TurnDirection::Left => 1,
                TurnDirection::Right => -1,
            });

        self.steering_controller.reset();
        self.rls_angle_filter.reset();
        self.mode = DriveMode::Turning {
            direction,
            start_angle_sign,
        };
        info!(
            "Mode switch: {} -> TURNING({})",
            drive_mode_name(DriveMode::Straight),
            turn_direction_name(direction)
        );
    }

    fn maybe_exit_turning_mode(&mut self) {
        let DriveMode::Turning {
            start_angle_sign, ..
        } = self.mode
        else {
            return;
        };

        let Some(angle) = self.latest_camera_angle_deg else {
            return;
        };

        let flipped = if start_angle_sign >= 0 {
            angle <= -ANGLE_FLIP_THRESHOLD_DEG
        } else {
            angle >= ANGLE_FLIP_THRESHOLD_DEG
        };

        if flipped {
            info!(
                "Turn exit condition met: angle={} start_sign={} threshold={} (would switch to STRAIGHT)",
                angle,
                start_angle_sign,
                ANGLE_FLIP_THRESHOLD_DEG
            );
            self.steering_controller.reset();
            self.rls_angle_filter.reset();
            self.mode = DriveMode::Straight;
            info!("Mode switch: TURNING -> STRAIGHT");
        }
    }
}

fn turn_direction_name(direction: TurnDirection) -> &'static str {
    match direction {
        TurnDirection::Left => "LEFT",
        TurnDirection::Right => "RIGHT",
    }
}

fn drive_mode_name(mode: DriveMode) -> &'static str {
    match mode {
        DriveMode::Startup => "STARTUP",
        DriveMode::Straight => "STRAIGHT",
        DriveMode::Turning { .. } => "TURNING",
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

    let mut state = Core1State {
        steering_controller: SteeringAngleController {
            pid: Pid::new(50.0, 0.0, 5.0, 100.0),
            angle_setpoint_deg: 0.0,
            neutral_steering_pwm_us: NEUTRAL_PWM_US,
            min_steering_pwm_us: 1250,
            max_steering_pwm_us: 1750,
        },
        rls_angle_filter: RlsAngleFilter::with_params(0.98, 0.5),
        mode: DriveMode::Startup,
        startup_until_us: None,
        latest_camera_angle_deg: None,
        last_left_cm: None,
        last_right_cm: None,
        left_no_value_streak: 0,
        right_no_value_streak: 0,
    };

    let mut time_ext = TimeExtender::new();
    let mut last_step_time_us: Option<u64> = None;

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
        let dt_s = last_step_time_us
            .map(|last| t_us.saturating_sub(last) as f32 * 1e-6)
            .unwrap_or(0.0);
        last_step_time_us = Some(t_us);

        if state.startup_until_us.is_none() {
            state.startup_until_us = Some(t_us.saturating_add(STARTUP_DURATION_US));
        }

        let mut io = Core1Io {
            control_q_tx: &mut control_q_tx,
            log_q_tx: &mut log_q_tx,
            fifo: &mut fifo,
        };

        handle_sensor_event(&mut state, event.kind, now, t_us, dt_s, &mut io);
    }
}

fn handle_sensor_event(
    state: &mut Core1State,
    event: SensorKind,
    now: Instant,
    t_us: u64,
    dt_s: f32,
    io: &mut Core1Io<'_>,
) {
    info!(
        "Received sensor event at t_us={}: {}",
        t_us,
        defmt::Debug2Format(&event)
    );

    match event {
        SensorKind::ConstantUpdate { constant, value } => {
            info!(
                "ConstantUpdate values: constant={} value={}",
                defmt::Debug2Format(&constant),
                value
            );
            match constant {
                Constants::SpeedSetpoint
                | Constants::SpeedKp
                | Constants::SpeedKi
                | Constants::SpeedKd => {}
                Constants::SteeringKp => state.steering_controller.pid.kp = value,
                Constants::SteeringKi => state.steering_controller.pid.ki = value,
                Constants::SteeringKd => state.steering_controller.pid.kd = value,
            }
        }
        SensorKind::CameraAlign { angle, confidence } => {
            let _ = confidence;
            // Apply RLS filter to smooth the camera angle measurement
            let filtered_angle = state.rls_angle_filter.update(angle);
            state.latest_camera_angle_deg = Some(filtered_angle);
            info!(
                "CameraAlign values: raw_angle_deg={} filtered_angle_deg={} confidence={} rls_covariance={}",
                angle,
                filtered_angle,
                confidence,
                state.rls_angle_filter.covariance
            );
        }
        SensorKind::Distances {
            left_cm,
            center_cm,
            right_cm,
        } => {
            info!(
                "Distances values: left_cm={} center_cm={} right_cm={}",
                left_cm,
                center_cm,
                right_cm
            );
            state.maybe_enter_turning_mode(left_cm, right_cm)
        }
        SensorKind::Encoder { rpm_period_us } => {
            info!("Encoder value: rpm_period_us={}", rpm_period_us);
        }
        SensorKind::EncoderTimeout => {
            info!("Encoder timeout event");
        }
    }

    if matches!(state.mode, DriveMode::Startup)
        && state
            .startup_until_us
            .is_some_and(|startup_until| t_us >= startup_until)
    {
        state.steering_controller.reset();
        state.rls_angle_filter.reset();
        state.mode = DriveMode::Straight;
        info!("Mode switch: STARTUP -> STRAIGHT");
    }

    state.maybe_exit_turning_mode();

    let (steer_pwm_us, power_pwm_us) = match state.mode {
        DriveMode::Startup => (NEUTRAL_PWM_US, NEUTRAL_PWM_US),
        DriveMode::Straight => {
            let measured_angle_deg = state.active_angle_deg();
            let steer_pwm_us = state.steering_controller.update(measured_angle_deg, dt_s);
            (steer_pwm_us, STRAIGHT_THROTTLE_PWM_US)
        }
        DriveMode::Turning { direction, .. } => {
            let steer_pwm_us = match direction {
                TurnDirection::Left => TURN_LEFT_STEER_PWM_US,
                TurnDirection::Right => TURN_RIGHT_STEER_PWM_US,
            };
            (steer_pwm_us, TURN_THROTTLE_PWM_US)
        }
    };

    emit_control_output(state, now, steer_pwm_us, power_pwm_us, io);
}

fn emit_control_output(
    state: &Core1State,
    now: Instant,
    steer_pwm_us: u16,
    power_pwm_us: u16,
    io: &mut Core1Io<'_>,
) {
    let mode_setpoint = match state.mode {
        DriveMode::Startup => 0.0,
        DriveMode::Straight => state.steering_controller.angle_setpoint_deg,
        DriveMode::Turning { .. } => 0.0,
    };

    io.log_q_tx
        .enqueue(LogData {
            timestamp: now,
            event: LogEvent::Controller {
                steer_value_us: steer_pwm_us,
                throttle_value_us: power_pwm_us,
                setpoint_value_mps: mode_setpoint,
                error_value: state.steering_controller.pid.last_error,
                kalman_values: [0.0; 4],
            },
        })
        .ok();
    io.fifo.write_blocking(IpcSignal::LogReady as u32);

    info!(
        "Control output at t_us={}: mode={} steer_pwm_us={} throttle_pwm_us={}",
        now.ticks(),
        drive_mode_name(state.mode),
        steer_pwm_us,
        power_pwm_us
    );

    io.control_q_tx
        .enqueue(ipc::ControlOutput {
            steer_pwm_us,
            power_pwm_us,
        })
        .ok();
    io.fifo.write_blocking(IpcSignal::ControlReady as u32);
}
