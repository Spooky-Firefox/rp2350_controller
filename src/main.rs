#![no_std]
#![no_main]
#![deny(unsafe_code)]

use defmt_rtt as _;
use embedded_hal::digital::StatefulOutputPin;
use fugit::{MicrosDurationU32, TimerDurationU32};
use rp235x_hal as hal;
use hal::gpio;
use hal::multicore::Stack;
use panic_probe as _;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::UsbDevice;
use usbd_serial::SerialPort;
use rp2350_controller::constants::*;
use rp2350_controller::entry::entry;
use rp2350_controller::ipc;
use rp2350_controller::logging::LogData;
use rp2350_controller::usb_serial::init_usb_serial;

type MyUsbBus = hal::usb::UsbBus;

/// Required by the RP2350 bootrom to identify and validate the image.
#[allow(unsafe_code)]
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

static CORE1_STACK: Stack<4096> = Stack::new();

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
    use rp2350_controller::usb_serial::process_usb_command;
    use rtic_monotonics::Monotonic;

    use super::*;
    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, MyUsbBus>,
        serial: SerialPort<'static, MyUsbBus>,
        pwm: MotorPwmSlice,
        speed_setpoint_mps: f32,
        measured_speed_mps: f32,
        last_sensor_irq_us: u64,
        fifo_tx: ipc::FifoTx,
        fifo_rx: ipc::FifoRx,
        power: u16,
    }

    #[local]
    struct Local {
        encoder: EncoderPin,
        encoder_last_time: TimerInstantU64<1_000_000>,
        led: LedPin,
        _pwm_a_pin: MotorPwmPinA,
        _pwm_b_pin: MotorPwmPinB,
        log_data_producer: heapless::spsc::Producer<'static, LogData, 128>,
        log_data_consumer: heapless::spsc::Consumer<'static, LogData, 128>,
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
        dataQueue: heapless::spsc::Queue<LogData, 128> = heapless::spsc::Queue::new(),
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
        let (us0, us0_irq) = ctx.local.us0_shared.split(
            pins.gpio14.into_push_pull_output(),
            pins.gpio15,
            us_pwm7,
        );
        let (us1, us1_irq) = ctx.local.us1_shared.split(
            pins.gpio12.into_push_pull_output(),
            pins.gpio13,
            us_pwm6,
        );
        let (us2, us2_irq) = ctx.local.us2_shared.split(
            pins.gpio10.into_push_pull_output(),
            pins.gpio11,
            us_pwm5,
        );

        // Spawn Core 1 for controller processing.
        {
            let mut mc = hal::multicore::Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
            let cores = mc.cores();
            let core1 = &mut cores[1];
            let stack = CORE1_STACK.take().unwrap();
            let _ = core1.spawn(stack, move || {
                rp2350_controller::controller_processor::controller_processor_loop::core1_task();
            });
        }

        let (fifo_tx, fifo_rx) = ipc::split_fifo(sio.fifo);
        delay_update_setpoint::spawn().unwrap();

        MainMono::start(pac.TIMER0, &pac.RESETS);
        toggle_led::spawn().unwrap();
        sensor_timeout::spawn().unwrap();
        periodic_drain_log_data::spawn().unwrap();
        ultrasound_scan::spawn().unwrap();
        let (log_data_producer, log_data_consumer) = ctx.local.dataQueue.split();
        (
            Shared {
                usb_dev,
                serial,
                pwm,
                speed_setpoint_mps: 0.0,
                measured_speed_mps: 0.0,
                last_sensor_irq_us: 0,
                fifo_tx,
                fifo_rx,
                power: 0,
            },
            Local {
                led: onboard_led,
                encoder,
                encoder_last_time: TimerInstantU64::<1_000_000>::from_ticks(0),
                _pwm_a_pin: pwm_a_pin,
                _pwm_b_pin: pwm_b_pin,
                log_data_producer,
                log_data_consumer,
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

    #[task(shared = [speed_setpoint_mps], priority = 1)]
    async fn delay_update_setpoint(mut ctx: delay_update_setpoint::Context) -> ! {
        loop {
            MainMono::delay(10.secs()).await;
            ctx.shared
                .speed_setpoint_mps
                .lock(|setpoint| *setpoint = if *setpoint == 0.0 { 10.0 } else { 0.0 });
        }
    }

    #[task(local = [led], priority = 1)]
    async fn toggle_led(ctx: toggle_led::Context) -> ! {
        loop {
            MainMono::delay(250u64.millis()).await;
            ctx.local.led.toggle().unwrap();
        }
    }

    #[task(binds = IO_IRQ_BANK0, local = [cnt: u32 = 0, encoder, encoder_last_time, us0_irq, us1_irq, us2_irq], shared = [speed_setpoint_mps, measured_speed_mps, last_sensor_irq_us, fifo_tx], priority = 3)]
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
                let setpoint_mps = ctx.shared.speed_setpoint_mps.lock(|setpoint| *setpoint);
                let measured_speed_mps =
                    LENGTH_PER_ENCODER_PULSE_METERS / (time_diff.ticks() as f32 * 1e-6);
                ctx.shared
                    .measured_speed_mps
                    .lock(|speed| *speed = measured_speed_mps);

                // Steering is stubbed to 0.0 for now.
                let event = ipc::SensorEvent::rpm_and_steer(
                    now.ticks(),
                    setpoint_mps,
                    0.0,
                    time_diff.ticks() as f32,
                );
                ctx.shared.fifo_tx.lock(|tx| tx.send_sensor_event(&event));
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

    #[task(shared = [speed_setpoint_mps, measured_speed_mps, last_sensor_irq_us, fifo_tx], priority = 2)]
    async fn sensor_timeout(mut ctx: sensor_timeout::Context) -> ! {
        loop {
            MainMono::delay(100u64.millis()).await;

            let now = MainMono::now().ticks();
            let last_sensor_irq_us = ctx.shared.last_sensor_irq_us.lock(|last| *last);

            if now.saturating_sub(last_sensor_irq_us) >= 100_000 {
                let setpoint_mps = ctx.shared.speed_setpoint_mps.lock(|setpoint| *setpoint);
                ctx.shared.measured_speed_mps.lock(|speed| *speed = 0.0);
                let event = ipc::SensorEvent::steer_only_timeout(now, setpoint_mps, 0.0);
                ctx.shared.fifo_tx.lock(|tx| tx.send_sensor_event(&event));
            }
        }
    }

    /// Periodic logging hook that processes queued log data and sends to USB serial.
    #[task(local = [log_data_consumer, drain_count: u32 = 0], shared = [usb_dev, serial], priority = 1)]
    async fn log_data(mut ctx: log_data::Context) {
        let mut data_count = 0;
        while let Some(data) = ctx.local.log_data_consumer.dequeue() {
            data_count += 1;
            trace!("Dequeued log data #{}", data_count);
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

    /// Receives control events from Core 1 and applies PWM outputs.
    #[task(binds = SIO_IRQ_FIFO, local = [log_data_producer],shared = [pwm, fifo_rx, power,measured_speed_mps,speed_setpoint_mps], priority = 4)]
    fn sio_interrupt(ctx: sio_interrupt::Context) {
        let mut pwm = ctx.shared.pwm;
        let mut fifo_rx = ctx.shared.fifo_rx;
        let mut power = ctx.shared.power;
        let mut measured_speed_mps = ctx.shared.measured_speed_mps;
        let mut speed_setpoint_mps = ctx.shared.speed_setpoint_mps;

        let mut log_data = LogData {
            timestamp: MainMono::now(),
            steer_value_ms: 0,
            throttle_value_ms: 0,
            setpoint_value: speed_setpoint_mps.lock(|s| *s),
            error_value: 0.0,
            speed_value: measured_speed_mps.lock(|s| *s),
            kalman_values: [0.0; 4],
        };
        while fifo_rx.lock(|rx| rx.have_data()) {
            match fifo_rx.lock(ipc::ControlEvent::from_fifo_rx_blocking) {
                ipc::ControlEvent::Control {
                    steer_pwm_us,
                    power_pwm_us,
                } => {
                    let steer_on =
                        MicrosDurationU32::from_ticks(steer_pwm_us as u32).min(PWM_PERIOD);
                    let power_on =
                        MicrosDurationU32::from_ticks(power_pwm_us as u32).min(PWM_PERIOD);
                    log_data.steer_value_ms = steer_on.ticks() as u16;
                    log_data.throttle_value_ms = power_on.ticks() as u16;
                    pwm.lock(|pwm| {
                        let _ = pwm.channel_a.set_duty_cycle(micros_to_pwm_ticks(steer_on));
                        let _ = pwm.channel_b.set_duty_cycle(micros_to_pwm_ticks(power_on));
                    });
                    power.lock(|p| *p = power_pwm_us);
                    trace!("ctrl: steer={} us  power={} us", steer_pwm_us, power_pwm_us);
                }
                ipc::ControlEvent::Pid {
                    error,
                    proportional: _,
                    integral: _,
                    derivative: _,
                } => {
                    log_data.error_value = error;
                }
                ipc::ControlEvent::KalmanDebug { x } => {
                    log_data.kalman_values = x;
                }
            }
        }
        ctx.local.log_data_producer.enqueue(log_data).ok(); // Ignore enqueue failures if the log queue is full.
        if ctx.local.log_data_producer.capacity() == 0 {
            log_data::spawn().ok(); // Trigger log processing if we have data to send but the consumer might be idle.
        }
    }

    #[task(binds = USBCTRL_IRQ, local = [buff: [u8; 64] = [0; 64], buff_len: usize = 0], shared = [usb_dev, serial, pwm, speed_setpoint_mps], priority = 5)]
    fn usb_interrupt(ctx: usb_interrupt::Context) {
        trace!("USBCTRL_IRQ fired");
        let usb_dev = ctx.shared.usb_dev;
        let serial = ctx.shared.serial;
        let pwm = ctx.shared.pwm;
        let speed_setpoint = ctx.shared.speed_setpoint_mps;

        let buff = ctx.local.buff;
        let buff_len = ctx.local.buff_len;

        process_usb_command(usb_dev, serial, pwm, buff, buff_len, speed_setpoint);
    }

    /// Polls all three HC-SR04 sensors in sequence with a 60 ms gap between each.
    ///
    /// Uses `delay_until` with an absolute deadline so the inter-sensor spacing
    /// does not drift over time regardless of how long each measurement takes.
    /// RTT log shows the raw counter value (µs, since div_int=150 @ 150 MHz).
    #[task(local = [us0, us1, us2], priority = 1)]
    async fn ultrasound_scan(ctx: ultrasound_scan::Context) -> ! {
        use defmt::{info, warn};
        // 50 ms timeout covers the maximum HC-SR04 echo pulse (~38 ms at ~6.5 m).
        const TIMEOUT_US: u64 = 50_000;
        let mut next = MainMono::now();
        loop {
            match ctx.local.us0.measure::<MainMono>(TIMEOUT_US.micros()).await {
                Ok(ticks) => info!("us0: {} us (~{} cm)", ticks, ticks as u32 / 58),
                Err(_) => warn!("us0: timeout"),
            }
            next = next + 60_000u64.micros();
            MainMono::delay_until(next).await;

            match ctx.local.us1.measure::<MainMono>(TIMEOUT_US.micros()).await {
                Ok(ticks) => info!("us1: {} us (~{} cm)", ticks, ticks as u32 / 58),
                Err(_) => warn!("us1: timeout"),
            }
            next = next + 60_000u64.micros();
            MainMono::delay_until(next).await;

            match ctx.local.us2.measure::<MainMono>(TIMEOUT_US.micros()).await {
                Ok(ticks) => info!("us2: {} us (~{} cm)", ticks, ticks as u32 / 58),
                Err(_) => warn!("us2: timeout"),
            }
            next = next + 60_000u64.micros();
            MainMono::delay_until(next).await;
        }
    }
}

// Convert microseconds to PWM counter ticks at the PWM timer frequency.
fn micros_to_pwm_ticks(on_time: MicrosDurationU32) -> u16 {
    let pwm_ticks: TimerDurationU32<PWM_TIMER_HZ> = on_time.convert();
    pwm_ticks.ticks().min(u16::MAX as u32) as u16 // Clamp to 16-bit register width.
}
