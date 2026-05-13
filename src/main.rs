#![no_std]
#![no_main]
#![deny(unsafe_code)]

use defmt_rtt as _;
use embedded_hal::digital::StatefulOutputPin;
use fugit::{MicrosDurationU32, TimerDurationU32};
use hal::gpio;
use hal::multicore::Stack;
use panic_probe as _;
use rp235x_hal as hal;
use rp2350_controller::constants::*;
use rp2350_controller::entry::entry;
use rp2350_controller::ipc;
use rp2350_controller::logging::LogData;
use rp2350_controller::usb_serial::{ControlMode, ControlModes, ControlTarget, init_usb_serial};
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::UsbDevice;
use usbd_serial::SerialPort;

type MyUsbBus = hal::usb::UsbBus;

/// Required by the RP2350 bootrom to identify and validate the image.
#[allow(unsafe_code)]
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

static CORE1_STACK: Stack<4096> = Stack::new();

// Compile-time interrupt gates.
// Set to `false` to disable handling for the corresponding interrupt source.
const ENABLE_USB_INTERRUPT: bool = true;
const ENABLE_CORE1_INTERRUPT: bool = true;

// Shorthand for GPIO25 configured as a digital output (LED pin).
type LedPin = gpio::Pin<gpio::bank0::Gpio25, gpio::FunctionSioOutput, gpio::PullDown>;

// Shorthand for GPIO13 configured as a digital input with pull-up (encoder pin).
type EncoderPin = gpio::Pin<gpio::bank0::Gpio18, gpio::FunctionSioInput, gpio::PullUp>;

type MotorPwmSlice = hal::pwm::Slice<hal::pwm::Pwm0, hal::pwm::FreeRunning>;

type MotorPwmPinA = gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionPwm, gpio::PullDown>;

type MotorPwmPinB = gpio::Pin<gpio::bank0::Gpio17, gpio::FunctionPwm, gpio::PullDown>;

use rp2350_controller::hc_sr04;
// HC-SR04 sensor 0: trig GPIO14, echo GPIO15 (Pwm7 B-channel gate).
type UsTrig0 = gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionSioOutput, gpio::PullDown>;
// HC-SR04 sensor 1: trig GPIO12, echo GPIO13 (Pwm6 B-channel gate).
type UsTrig1 = gpio::Pin<gpio::bank0::Gpio12, gpio::FunctionSioOutput, gpio::PullDown>;
// HC-SR04 sensor 2: trig GPIO10, echo GPIO11 (Pwm5 B-channel gate).
type UsTrig2 = gpio::Pin<gpio::bank0::Gpio10, gpio::FunctionSioOutput, gpio::PullDown>;
type UsMeasure0 = hc_sr04::HcSr04Measure<'static, UsTrig0, hal::pwm::Pwm7>;
type UsMeasure1 = hc_sr04::HcSr04Measure<'static, UsTrig1, hal::pwm::Pwm6>;
type UsMeasure2 = hc_sr04::HcSr04Measure<'static, UsTrig2, hal::pwm::Pwm5>;
type UsOnIrq0 = hc_sr04::HcSr04OnInterrupt<'static, gpio::bank0::Gpio15, gpio::PullDown>;
type UsOnIrq1 = hc_sr04::HcSr04OnInterrupt<'static, gpio::bank0::Gpio13, gpio::PullDown>;
type UsOnIrq2 = hc_sr04::HcSr04OnInterrupt<'static, gpio::bank0::Gpio11, gpio::PullDown>;

#[rtic::app(device = rp235x_pac, peripherals = true, dispatchers = [DMA_IRQ_0, DMA_IRQ_1])]
mod app {

    use defmt::trace;
    use embedded_hal::pwm::SetDutyCycle;
    use fugit::{ExtU64, MicrosDurationU32, TimerInstantU64};
    use rp2350_controller::hc_sr04;
    use rp2350_controller::usb_serial::{UsbCommand, process_usb_command};
    use rtic_monotonics::Monotonic;

    use super::*;
    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, MyUsbBus>,
        serial: SerialPort<'static, MyUsbBus>,
        pwm: MotorPwmSlice,
        measured_speed_mps: f32,
        last_sensor_irq_us: u64,
        fifo_tx: ipc::FifoTx,
        fifo_rx: ipc::FifoRx,
        sensor_q_tx: heapless::spsc::Producer<'static, ipc::SensorEvent, 4>,
        log_q_tx_core0: heapless::spsc::Producer<'static, LogData, 128>,
        control_mode: ControlModes,
        power: u16,
    }

    #[local]
    struct Local {
        encoder: EncoderPin,
        encoder_last_time: TimerInstantU64<1_000_000>,
        led: LedPin,
        _pwm_a_pin: MotorPwmPinA,
        _pwm_b_pin: MotorPwmPinB,
        log_data_consumer_core0: heapless::spsc::Consumer<'static, LogData, 128>,
        log_data_consumer_core1: heapless::spsc::Consumer<'static, LogData, 128>,
        control_q_rx: heapless::spsc::Consumer<'static, ipc::ControlOutput, 4>,
        us0: UsMeasure0,
        us1: UsMeasure1,
        us2: UsMeasure2,
        us0_irq: UsOnIrq0,
        us1_irq: UsOnIrq1,
        us2_irq: UsOnIrq2,
    }

    rtic_monotonics::rp235x_timer_monotonic!(MainMono);

    #[init(local = [
        usb_bus: Option<UsbBusAllocator<MyUsbBus>> = None,
        core0_log_q: heapless::spsc::Queue<LogData, 128> = heapless::spsc::Queue::new(),
        core1_log_q: heapless::spsc::Queue<LogData, 128> = heapless::spsc::Queue::new(),
        sensor_q: heapless::spsc::Queue<ipc::SensorEvent, 4> = heapless::spsc::Queue::new(),
        control_q: heapless::spsc::Queue<ipc::ControlOutput, 4> = heapless::spsc::Queue::new(),
        us0_shared: hc_sr04::HcSr04Shared = hc_sr04::HcSr04Shared::new(),
        us1_shared: hc_sr04::HcSr04Shared = hc_sr04::HcSr04Shared::new(),
        us2_shared: hc_sr04::HcSr04Shared = hc_sr04::HcSr04Shared::new(),
    ])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Perform early hardware setup (spinlocks, co-processor enable).
        // This calls entry::entry() which is a copy of hal::entry().
        // Safe because we're in single-threaded init context and this must run early.
        #[allow(unsafe_code)]
        unsafe {
            entry()
        };

        // Grab our singleton objects
        let mut pac = ctx.device;

        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            XTAL_FREQ_HZ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .unwrap();

        let (usb_dev, serial) = init_usb_serial(
            pac.USB,
            pac.USB_DPRAM,
            clocks.usb_clock,
            &mut pac.RESETS,
            ctx.local.usb_bus,
        );

        let mut sio = hal::Sio::new(pac.SIO);
        let pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let onboard_led = pins.gpio25.into_push_pull_output();

        // Configure PWM peripheral for servo/motor control (20 ms period = 50 Hz).
        let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
        let mut pwm = pwm_slices.pwm0;
        pwm.set_div_int(PWM_DIV_INT); // Clock divider to reach desired PWM frequency.
        let period_ticks: fugit::TimerDurationU32<PWM_TIMER_HZ> = PWM_PERIOD.convert();
        pwm.set_top(period_ticks.ticks().saturating_sub(1) as u16); // Set 20 ms period.
        pwm.enable(); // Start the PWM counter.
        // Attach PWM channels to pins and set initial position (1500 µs = center).
        let pwm_a_pin = pwm.channel_a.output_to(pins.gpio16);
        let pwm_b_pin = pwm.channel_b.output_to(pins.gpio17);
        let _ = pwm
            .channel_a
            .set_duty_cycle(micros_to_pwm_ticks(PWM_DEFAULT_ON_TIME));
        let _ = pwm
            .channel_b
            .set_duty_cycle(micros_to_pwm_ticks(PWM_DEFAULT_ON_TIME));
        pwm.channel_a.set_enabled(true);
        pwm.channel_b.set_enabled(true);

        // Configure encoder input and enable interrupt on rising edges (magnet passes sensor).
        let encoder = pins.gpio18.into_pull_up_input();
        encoder.set_interrupt_enabled(rp235x_hal::gpio::Interrupt::EdgeHigh, true);

        // Configure HC-SR04 ultrasonic sensors.
        // Each PWM slice runs at 1 MHz (div_int=150 @ 150 MHz), so 1 tick = 1 µs.
        // Echo pulses up to ~38 ms fit within the u16 counter (max 65 535 µs).
        let mut us_pwm5 = pwm_slices.pwm5;
        us_pwm5.set_div_int(150);
        let mut us_pwm6 = pwm_slices.pwm6;
        us_pwm6.set_div_int(150);
        let mut us_pwm7 = pwm_slices.pwm7;
        us_pwm7.set_div_int(150);
        let (us0, us0_irq) =
            ctx.local
                .us0_shared
                .split(pins.gpio14.into_push_pull_output(), pins.gpio15, us_pwm7);
        let (us1, us1_irq) =
            ctx.local
                .us1_shared
                .split(pins.gpio12.into_push_pull_output(), pins.gpio13, us_pwm6);
        let (us2, us2_irq) =
            ctx.local
                .us2_shared
                .split(pins.gpio10.into_push_pull_output(), pins.gpio11, us_pwm5);

        let (sensor_q_tx, sensor_q_rx) = ctx.local.sensor_q.split();
        let (control_q_tx, control_q_rx) = ctx.local.control_q.split();
        let (log_q_tx_core0, log_q_rx_core0) = ctx.local.core0_log_q.split();
        let (log_q_tx_core1, log_q_rx_core1) = ctx.local.core1_log_q.split();

        // Spawn Core 1 for controller processing.
        {
            let mut mc = hal::multicore::Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
            let cores = mc.cores();
            let core1 = &mut cores[1];
            let stack = CORE1_STACK.take().unwrap();
            let _ = core1.spawn(stack, move || {
                rp2350_controller::controller_processor::controller_processor_loop::core1_task(
                    sensor_q_rx,
                    control_q_tx,
                    log_q_tx_core1,
                );
            });
        }

        let (fifo_tx, fifo_rx) = ipc::split_fifo(sio.fifo);

        MainMono::start(pac.TIMER0, &pac.RESETS);
        toggle_led::spawn().unwrap();
        sensor_timeout::spawn().unwrap();
        periodic_drain_log_data::spawn().unwrap();
        ultrasound_scan::spawn().unwrap();
        (
            Shared {
                usb_dev,
                serial,
                pwm,
                measured_speed_mps: 0.0,
                last_sensor_irq_us: 0,
                fifo_tx,
                fifo_rx,
                sensor_q_tx,
                log_q_tx_core0,
                control_mode: if ENABLE_USB_INTERRUPT {
                    ControlModes {
                        steering: ControlMode::Manual,
                        throttle: ControlMode::Manual,
                    }
                } else {
                    ControlModes {
                        steering: ControlMode::Auto,
                        throttle: ControlMode::Auto,
                    }
                },
                power: 0,
            },
            Local {
                led: onboard_led,
                encoder,
                encoder_last_time: TimerInstantU64::<1_000_000>::from_ticks(0),
                _pwm_a_pin: pwm_a_pin,
                _pwm_b_pin: pwm_b_pin,
                log_data_consumer_core0: log_q_rx_core0,
                log_data_consumer_core1: log_q_rx_core1,
                control_q_rx,
                us0,
                us1,
                us2,
                us0_irq,
                us1_irq,
                us2_irq,
            },
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfe();
        }
    }

    #[task(local = [led], priority = 1)]
    async fn toggle_led(ctx: toggle_led::Context) -> ! {
        loop {
            MainMono::delay(250u64.millis()).await;
            ctx.local.led.toggle().unwrap();
        }
    }

    #[task(binds = IO_IRQ_BANK0, local = [cnt: u32 = 0, encoder, encoder_last_time, us0_irq, us1_irq, us2_irq], shared = [measured_speed_mps, last_sensor_irq_us, fifo_tx, sensor_q_tx, log_q_tx_core0], priority = 3)]
    fn gpio_interrupt(mut ctx: gpio_interrupt::Context) {
        trace!("gpio interrupt");
        *ctx.local.cnt += 1;
        if ctx
            .local
            .encoder
            .interrupt_status(hal::gpio::Interrupt::EdgeHigh)
        {
            trace!("encoder edge cnt {}", ctx.local.cnt);

            let now = MainMono::now();
            let previous = *ctx.local.encoder_last_time;
            *ctx.local.encoder_last_time = now;
            ctx.shared
                .last_sensor_irq_us
                .lock(|last| *last = now.ticks());

            if previous.ticks() != 0 {
                let time_diff = now - previous;
                ctx.shared.log_q_tx_core0.lock(|q| {
                    if q.enqueue(LogData {
                        timestamp: now,
                        event: rp2350_controller::logging::LogEvent::HallDeltaT {
                            delta_t_us: time_diff.ticks() as u32,
                        },
                    })
                    .is_err()
                    {
                        defmt::warn!("core0 log queue full, hall delta-t sample dropped");
                    }
                });
                let measured_speed_mps =
                    LENGTH_PER_ENCODER_PULSE_METERS / (time_diff.ticks() as f32 * 1e-6);
                ctx.shared
                    .measured_speed_mps
                    .lock(|speed| *speed = measured_speed_mps);

                let event = ipc::SensorEvent::encoder(now.ticks(), time_diff.ticks() as f32);
                ctx.shared.sensor_q_tx.lock(|q| {
                    if q.enqueue(event).is_err() {
                        defmt::warn!("sensor_q full, encoder event dropped");
                    }
                });
                ctx.shared
                    .fifo_tx
                    .lock(|tx| tx.signal(ipc::IpcSignal::SensorReady));
            }

            ctx.local
                .encoder
                .clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
        }

        // Forward any HC-SR04 echo-fall interrupts to the sensor drivers.
        ctx.local.us0_irq.on_interrupt();
        ctx.local.us1_irq.on_interrupt();
        ctx.local.us2_irq.on_interrupt();
    }

    #[task(shared = [measured_speed_mps, last_sensor_irq_us, fifo_tx, sensor_q_tx], priority = 2)]
    async fn sensor_timeout(mut ctx: sensor_timeout::Context) -> ! {
        loop {
            MainMono::delay(100u64.millis()).await;

            let now = MainMono::now().ticks();
            let last_sensor_irq_us = ctx.shared.last_sensor_irq_us.lock(|last| *last);

            if now.saturating_sub(last_sensor_irq_us) >= 100_000 {
                ctx.shared.measured_speed_mps.lock(|speed| *speed = 0.0);
                let event = ipc::SensorEvent::encoder_timeout(now);
                ctx.shared.sensor_q_tx.lock(|q| {
                    if q.enqueue(event).is_err() {
                        defmt::warn!("sensor_q full, timeout event dropped");
                    }
                });
                ctx.shared
                    .fifo_tx
                    .lock(|tx| tx.signal(ipc::IpcSignal::SensorReady));
            }
        }
    }

    /// Periodic logging hook that processes queued log data and sends to USB serial.
    #[task(local = [log_data_consumer_core0, log_data_consumer_core1, drain_count: u32 = 0], shared = [usb_dev, serial], priority = 1)]
    async fn log_data(mut ctx: log_data::Context) {
        let mut data_count = 0;
        while let Some(data) = ctx.local.log_data_consumer_core0.dequeue() {
            data_count += 1;
            trace!("Dequeued core0 log data #{}", data_count);
            rp2350_controller::logging::write_log_data(
                data,
                &mut ctx.shared.usb_dev,
                &mut ctx.shared.serial,
            );
        }
        while let Some(data) = ctx.local.log_data_consumer_core1.dequeue() {
            data_count += 1;
            trace!("Dequeued core1 log data #{}", data_count);
            rp2350_controller::logging::write_log_data(
                data,
                &mut ctx.shared.usb_dev,
                &mut ctx.shared.serial,
            );
        }
        if data_count > 0 {
            trace!("Logged {} data points", data_count);
        }
    }

    #[task(priority = 1)]
    async fn periodic_drain_log_data(_ctx: periodic_drain_log_data::Context) -> ! {
        let mut cycle_count = 0u32;
        loop {
            cycle_count = cycle_count.wrapping_add(1);
            trace!("periodic_drain_log_data cycle {}", cycle_count);
            log_data::spawn().ok(); // Trigger log processing task.
            MainMono::delay(100u64.millis()).await;
        }
    }

    /// Receives control signals from Core 1 and applies PWM outputs.
    #[task(binds = SIO_IRQ_FIFO, local = [control_q_rx], shared = [pwm, fifo_rx, power, control_mode], priority = 4)]
    fn sio_interrupt(ctx: sio_interrupt::Context) {
        let mut pwm = ctx.shared.pwm;
        let mut fifo_rx = ctx.shared.fifo_rx;
        let mut power = ctx.shared.power;
        let mut control_mode = ctx.shared.control_mode;

        if !ENABLE_CORE1_INTERRUPT {
            // Drain FIFO and matching queue items to avoid backpressure deadlocks
            // when Core 1 keeps signaling while this handler is disabled.
            while let Some(word) = fifo_rx.lock(|rx| rx.try_read()) {
                if matches!(
                    ipc::IpcSignal::from_u32(word),
                    Some(ipc::IpcSignal::ControlReady)
                ) {
                    let _ = ctx.local.control_q_rx.dequeue();
                }
            }
            return;
        }

        while let Some(word) = fifo_rx.lock(|rx| rx.try_read()) {
            match ipc::IpcSignal::from_u32(word) {
                Some(ipc::IpcSignal::ControlReady) => {
                    if let Some(output) = ctx.local.control_q_rx.dequeue() {
                        let (steering_auto, throttle_auto) = control_mode.lock(|mode| {
                            (
                                mode.steering == ControlMode::Auto,
                                mode.throttle == ControlMode::Auto,
                            )
                        });
                        if steering_auto || throttle_auto {
                            let steer_on =
                                MicrosDurationU32::from_ticks(output.steer_pwm_us as u32)
                                    .min(PWM_PERIOD);
                            let power_on =
                                MicrosDurationU32::from_ticks(output.power_pwm_us as u32)
                                    .min(PWM_PERIOD);
                            pwm.lock(|pwm| {
                                if steering_auto {
                                    let _ =
                                        pwm.channel_a.set_duty_cycle(micros_to_pwm_ticks(steer_on));
                                }
                                if throttle_auto {
                                    let _ =
                                        pwm.channel_b.set_duty_cycle(micros_to_pwm_ticks(power_on));
                                }
                            });
                            if throttle_auto {
                                power.lock(|p| *p = output.power_pwm_us);
                            }
                            trace!(
                                "ctrl auto: steer={} us [{}] power={} us [{}]",
                                output.steer_pwm_us,
                                if steering_auto { "auto" } else { "manual" },
                                output.power_pwm_us,
                                if throttle_auto { "auto" } else { "manual" }
                            );
                        } else {
                            trace!(
                                "ctrl ignored in manual mode: steer={} us [{}] power={} us [{}]",
                                output.steer_pwm_us,
                                if steering_auto { "auto" } else { "manual" },
                                output.power_pwm_us,
                                if throttle_auto { "auto" } else { "manual" }
                            );
                        }
                    }
                }
                Some(ipc::IpcSignal::LogReady) => {
                    log_data::spawn().ok();
                }
                _ => {}
            }
        }
    }

    #[task(binds = USBCTRL_IRQ, local = [buff: [u8; 64] = [0; 64], buff_len: usize = 0], shared = [usb_dev, serial, pwm, fifo_tx, sensor_q_tx, control_mode], priority = 5)]
    fn usb_interrupt(mut ctx: usb_interrupt::Context) {
        trace!("USBCTRL_IRQ fired");
        let usb_dev = ctx.shared.usb_dev;
        let serial = ctx.shared.serial;
        let mut pwm = ctx.shared.pwm;

        let buff = ctx.local.buff;
        let buff_len = ctx.local.buff_len;

        if !ENABLE_USB_INTERRUPT {
            // Still poll/drain USB state so the interrupt line can clear.
            let _ = process_usb_command(usb_dev, serial, buff, buff_len);
            return;
        }

        match process_usb_command(usb_dev, serial, buff, buff_len) {
            Some(UsbCommand::SetPwmA { on_time_us }) => {
                let on_time = MicrosDurationU32::from_ticks(on_time_us).min(PWM_PERIOD);
                let ticks = micros_to_pwm_ticks(on_time);
                pwm.lock(|pwm| {
                    let _ = pwm.channel_a.set_duty_cycle(ticks);
                });
            }
            Some(UsbCommand::SetPwmB { on_time_us }) => {
                let on_time = MicrosDurationU32::from_ticks(on_time_us).min(PWM_PERIOD);
                let ticks = micros_to_pwm_ticks(on_time);
                pwm.lock(|pwm| {
                    let _ = pwm.channel_b.set_duty_cycle(ticks);
                });
            }
            Some(UsbCommand::SetConstant { constant, value }) => {
                let now_us = MainMono::now().ticks();
                let event = ipc::SensorEvent::constant_update(now_us, constant, value);
                ctx.shared.sensor_q_tx.lock(|q| {
                    if q.enqueue(event).is_err() {
                        defmt::warn!("sensor_q full, constant event dropped");
                    }
                });
                ctx.shared
                    .fifo_tx
                    .lock(|tx| tx.signal(ipc::IpcSignal::SensorReady));
            }
            Some(UsbCommand::SendAlign { angle, confidence }) => {
                let now_us = MainMono::now().ticks();
                let event = ipc::SensorEvent::camera_align(now_us, angle, confidence);
                ctx.shared.sensor_q_tx.lock(|q| {
                    if q.enqueue(event).is_err() {
                        defmt::warn!("sensor_q full, align event dropped");
                    }
                });
                ctx.shared
                    .fifo_tx
                    .lock(|tx| tx.signal(ipc::IpcSignal::SensorReady));
            }
            Some(UsbCommand::SetControlMode { target, mode }) => {
                ctx.shared.control_mode.lock(|current| match target {
                    ControlTarget::Steering => current.steering = mode,
                    ControlTarget::Throttle => current.throttle = mode,
                    ControlTarget::Both => {
                        current.steering = mode;
                        current.throttle = mode;
                    }
                });
            }
            None => {}
        }
    }

    /// Polls all three HC-SR04 sensors in sequence with a 60 ms gap between each.
    ///
    /// Uses `delay_until` with an absolute deadline so the inter-sensor spacing
    /// does not drift over time regardless of how long each measurement takes.
    /// RTT log shows the raw counter value (µs, since div_int=150 @ 150 MHz).
    #[task(local = [us0, us1, us2], shared = [log_q_tx_core0, sensor_q_tx, fifo_tx], priority = 1)]
    async fn ultrasound_scan(mut ctx: ultrasound_scan::Context) -> ! {
        use defmt::{info, warn};
        const TIMEOUT_US: u64 = 50_000;
        const INTER_SENSOR_DELAY_US: u64 = 60_000;
        let mut next = MainMono::now();
        loop {
            let us0_cm = match ctx.local.us0.measure::<MainMono>(TIMEOUT_US.micros()).await {
                Ok(ticks) => {
                    let distance_cm = ticks as u32 / 58;
                    info!("us0: {} us (~{} cm)", ticks, distance_cm);
                    Some(distance_cm)
                }
                Err(_) => {
                    warn!("us0: timeout");
                    None
                }
            };
            next += INTER_SENSOR_DELAY_US.micros();
            MainMono::delay_until(next).await;

            let us1_cm = match ctx.local.us1.measure::<MainMono>(TIMEOUT_US.micros()).await {
                Ok(ticks) => {
                    let distance_cm = ticks as u32 / 58;
                    info!("us1: {} us (~{} cm)", ticks, distance_cm);
                    Some(distance_cm)
                }
                Err(_) => {
                    warn!("us1: timeout");
                    None
                }
            };
            next += INTER_SENSOR_DELAY_US.micros();
            MainMono::delay_until(next).await;

            let us2_cm = match ctx.local.us2.measure::<MainMono>(TIMEOUT_US.micros()).await {
                Ok(ticks) => {
                    let distance_cm = ticks as u32 / 58;
                    info!("us2: {} us (~{} cm)", ticks, distance_cm);
                    Some(distance_cm)
                }
                Err(_) => {
                    warn!("us2: timeout");
                    None
                }
            };

            let log_data = LogData {
                timestamp: MainMono::now(),
                event: rp2350_controller::logging::LogEvent::Ultrasound {
                    distance0_cm: us0_cm,
                    distance1_cm: us1_cm,
                    distance2_cm: us2_cm,
                },
            };
            ctx.shared.log_q_tx_core0.lock(|q| {
                if q.enqueue(log_data).is_err() {
                    defmt::warn!("core0 log queue full, ultrasound sample dropped");
                }
            });

            let event = ipc::SensorEvent::distances(
                MainMono::now().ticks(),
                us0_cm.map(|value| value as f32).unwrap_or(f32::INFINITY),
                us1_cm.map(|value| value as f32).unwrap_or(f32::INFINITY),
                us2_cm.map(|value| value as f32).unwrap_or(f32::INFINITY),
            );
            ctx.shared.sensor_q_tx.lock(|q| {
                if q.enqueue(event).is_err() {
                    warn!("sensor_q full, distance event dropped");
                }
            });
            ctx.shared
                .fifo_tx
                .lock(|tx| tx.signal(ipc::IpcSignal::SensorReady));

            next += INTER_SENSOR_DELAY_US.micros();
            MainMono::delay_until(next).await;
        }
    }
}

// Convert microseconds to PWM counter ticks at the PWM timer frequency.
fn micros_to_pwm_ticks(on_time: MicrosDurationU32) -> u16 {
    let pwm_ticks: TimerDurationU32<PWM_TIMER_HZ> = on_time.convert();
    pwm_ticks.ticks().min(u16::MAX as u32) as u16 // Clamp to 16-bit register width.
}
