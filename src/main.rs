#![no_std]
#![no_main]
#![deny(unsafe_code)]

use defmt_rtt as _;
use embedded_hal::digital::StatefulOutputPin;
use fugit::{ExtU64, MicrosDurationU32, TimerDurationU32};
use hal::gpio;
use hal::multicore::Stack;
use panic_probe as _;
use rp235x_hal as hal;
use rp235x_pac as pac;
use rp2350_controller::constants::*;
use rp2350_controller::entry::entry;
use rp2350_controller::ipc;
use rp2350_controller::logging::LogData;
use rp2350_controller::usb_serial::{MyUsbBus, init_usb_serial};
use usb_device::{class_prelude::UsbBusAllocator, device::UsbDevice};
use usbd_serial::SerialPort;
// 2^16 = 65536 bytes = 64 KB stack size for core 1 (adjust as needed).
static mut CORE1_STACK: Stack<65536> = Stack::new();

/// Required by the RP2350 bootrom to identify and validate the image.
#[allow(unsafe_code)]
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

// Shorthand for GPIO25 configured as a digital output (LED pin).
type LedPin = gpio::Pin<gpio::bank0::Gpio25, gpio::FunctionSioOutput, gpio::PullDown>;

// Shorthand for GPIO13 configured as a digital input with pull-up (encoder pin).
type EncoderPin = gpio::Pin<gpio::bank0::Gpio18, gpio::FunctionSioInput, gpio::PullUp>;

type MotorPwmSlice = hal::pwm::Slice<hal::pwm::Pwm0, hal::pwm::FreeRunning>;

type MotorPwmPinA = gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionPwm, gpio::PullDown>;

type MotorPwmPinB = gpio::Pin<gpio::bank0::Gpio17, gpio::FunctionPwm, gpio::PullDown>;
#[rtic::app(device = crate::pac, peripherals = true, dispatchers = [DMA_IRQ_0, DMA_IRQ_1])]
mod app {

    use defmt::trace;
    use embedded_hal::pwm::SetDutyCycle;
    use fugit::{MicrosDurationU32, TimerInstantU64};
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
    }

    rtic_monotonics::rp235x_timer_monotonic!(MainMono);

    #[init(local = [usb_bus: Option<UsbBusAllocator<MyUsbBus>> = None, dataQueue: heapless::spsc::Queue<LogData, 128> = heapless::spsc::Queue::new()])]
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

        // Set up the watchdog driver - needed by the clock setup code
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

        // Configure the clocks
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

        let pads_bank = pac.PADS_BANK0;
        let io_bank = pac.IO_BANK0;
        let mut sio = hal::Sio::new(pac.SIO);
        let pins = hal::gpio::Pins::new(io_bank, pads_bank, sio.gpio_bank0, &mut pac.RESETS);

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

        // Spawn Core 1 for controller processing.
        {
            let mut mc = hal::multicore::Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
            let cores = mc.cores();
            let core1 = &mut cores[1];
            let stack;
            // Taking from a static mutable is unsafe, but wrapped in a critical section.
            // RTIC's internals protect this with a spinlock, making concurrent access safe.
            // This is the standard pattern for handing stacks to spawn tasks in RTIC.
            #[allow(unsafe_code, static_mut_refs)]
            unsafe {
                stack = CORE1_STACK.take().unwrap();
            }
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

    #[task(binds = IO_IRQ_BANK0, local = [cnt: u32 = 0,encoder, encoder_last_time], shared = [speed_setpoint_mps, measured_speed_mps, last_sensor_irq_us, fifo_tx], priority = 3)]
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
            log_data::spawn(); // Trigger log processing if we have data to send but the consumer might be idle.
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
}

// Convert microseconds to PWM counter ticks at the PWM timer frequency.
fn micros_to_pwm_ticks(on_time: MicrosDurationU32) -> u16 {
    let pwm_ticks: TimerDurationU32<PWM_TIMER_HZ> = on_time.convert();
    pwm_ticks.ticks().min(u16::MAX as u32) as u16 // Clamp to 16-bit register width.
}
