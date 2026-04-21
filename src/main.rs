#![no_std]
#![no_main]
#![deny(unsafe_code)]

use defmt_rtt as _;
use embedded_hal::digital::StatefulOutputPin;
use fugit::{ExtU64, MicrosDurationU32, TimerDurationU32};
use hal::gpio;
use heapless::String;
use panic_probe as _;
use rp235x_hal as hal;
use rp235x_pac as pac;
use rp2350_controller::entry::entry;
use rp2350_controller::ipc;
use rp2350_controller::usb_serial::{MyUsbBus, init_usb_serial};
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceState},
};
use usbd_serial::SerialPort;

use core::f32::consts::PI;
use core::fmt::Write as _;

use hal::multicore::Stack;
// 2^16 = 65536 bytes = 64 KB stack size for core 1 (adjust as needed).
static mut CORE1_STACK: Stack<65536> = Stack::new();

/// Required by the RP2350 bootrom to identify and validate the image.
#[allow(unsafe_code)]
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// CPU clock speed (150 MHz). All timers and PWM are derived from this.
const SYS_CLOCK_HZ: u32 = 150_000_000;
const PWM_DIV_INT: u8 = 64;
const PWM_TIMER_HZ: u32 = SYS_CLOCK_HZ / PWM_DIV_INT as u32;

/// Arc length per encoder pulse [m].
const LENGTH_PER_ENCODER_PULSE_METERS: f32 = 13.0 * PI / 300.0;

// Standard RC-servo frame period (50 Hz).
const PWM_PERIOD: MicrosDurationU32 = MicrosDurationU32::from_ticks(20_000);
const PWM_DEFAULT_ON_TIME: MicrosDurationU32 = MicrosDurationU32::from_ticks(1_500);

// Shorthand for GPIO25 configured as a digital output (LED pin).
type LedPin = gpio::Pin<gpio::bank0::Gpio25, gpio::FunctionSioOutput, gpio::PullDown>;

// Shorthand for GPIO13 configured as a digital input with pull-up (encoder pin).
type EncoderPin = gpio::Pin<gpio::bank0::Gpio18, gpio::FunctionSioInput, gpio::PullUp>;

type MotorPwmSlice = hal::pwm::Slice<hal::pwm::Pwm0, hal::pwm::FreeRunning>;

type MotorPwmPinA = gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionPwm, gpio::PullDown>;

type MotorPwmPinB = gpio::Pin<gpio::bank0::Gpio17, gpio::FunctionPwm, gpio::PullDown>;
#[rtic::app(device = crate::pac, peripherals = true, dispatchers = [DMA_IRQ_0, DMA_IRQ_1])]
mod app {

    use core::task;

    use defmt::{debug, info, trace, warn};
    use embedded_hal::pwm::SetDutyCycle;
    use fugit::{MicrosDurationU32, TimerInstantU64};
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
        fifo: ipc::FifoChannel,
    }

    #[local]
    struct Local {
        encoder: EncoderPin,
        encoder_last_time: TimerInstantU64<1_000_000>,
        led: LedPin,
        _pwm_a_pin: MotorPwmPinA,
        _pwm_b_pin: MotorPwmPinB,
    }

    rtic_monotonics::rp235x_timer_monotonic!(MainMono);

    #[init(local = [usb_bus: Option<UsbBusAllocator<MyUsbBus>> = None])]
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

        let fifo = ipc::FifoChannel::new(sio.fifo);
        delay_update_setpoint::spawn().unwrap();

        MainMono::start(pac.TIMER0, &pac.RESETS);
        toggle_led::spawn().unwrap();
        sensor_timeout::spawn().unwrap();
        log_speed::spawn().unwrap();
        (
            Shared {
                usb_dev,
                serial,
                pwm,
                speed_setpoint_mps: 0.0,
                measured_speed_mps: 0.0,
                last_sensor_irq_us: 0,
                fifo,
            },
            Local {
                led: onboard_led,
                encoder,
                encoder_last_time: TimerInstantU64::<1_000_000>::from_ticks(0),
                _pwm_a_pin: pwm_a_pin,
                _pwm_b_pin: pwm_b_pin,
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
                .lock(|setpoint| *setpoint = if *setpoint == 0.0 { 1.0 } else { 0.0 });
        }
    }

    #[task(local = [led], priority = 1)]
    async fn toggle_led(ctx: toggle_led::Context) -> ! {
        loop {
            MainMono::delay(250u64.millis()).await;
            ctx.local.led.toggle().unwrap();
        }
    }

    #[task(binds = IO_IRQ_BANK0, local = [cnt: u32 = 0,encoder, encoder_last_time], shared = [speed_setpoint_mps, measured_speed_mps, last_sensor_irq_us, fifo], priority = 3)]
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
                ctx.shared.fifo.lock(|fifo| fifo.send_sensor_event(&event));
            }

            ctx.local
                .encoder
                .clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
        }
    }

    #[task(shared = [speed_setpoint_mps, measured_speed_mps, last_sensor_irq_us, fifo], priority = 2)]
    async fn sensor_timeout(mut ctx: sensor_timeout::Context) -> ! {
        loop {
            MainMono::delay(100u64.millis()).await;

            let now = MainMono::now().ticks();
            let last_sensor_irq_us = ctx.shared.last_sensor_irq_us.lock(|last| *last);

            if now.saturating_sub(last_sensor_irq_us) >= 100_000 {
                let setpoint_mps = ctx.shared.speed_setpoint_mps.lock(|setpoint| *setpoint);
                ctx.shared.measured_speed_mps.lock(|speed| *speed = 0.0);
                let event = ipc::SensorEvent::steer_only_timeout(now, setpoint_mps, 0.0);
                ctx.shared.fifo.lock(|fifo| fifo.send_sensor_event(&event));
            }
        }
    }

    /// Periodic telemetry logging hook. Easy place to later mirror data to USB serial.
    #[task(shared = [usb_dev, serial, speed_setpoint_mps, measured_speed_mps, last_sensor_irq_us], priority = 1)]
    async fn log_speed(mut ctx: log_speed::Context) -> ! {
        loop {
            MainMono::delay(250u64.millis()).await;

            let now_us = MainMono::now().ticks();
            let setpoint = ctx.shared.speed_setpoint_mps.lock(|v| *v);
            let speed = ctx.shared.measured_speed_mps.lock(|v| *v);
            let age_us = ctx
                .shared
                .last_sensor_irq_us
                .lock(|last| now_us.saturating_sub(*last));

            info!(
                "speed: measured={} m/s setpoint={} m/s age={} us",
                speed, setpoint, age_us
            );

            // VS Code Serial Plotter line format: ">name:value,name:value\r\n"
            let mut line: String<96> = String::new();
            let _ = write!(
                &mut line,
                ">speed_mps:{:.4},setpoint_mps:{:.4},age_us:{}\r\n",
                speed, setpoint, age_us
            );

            (&mut ctx.shared.usb_dev, &mut ctx.shared.serial).lock(|usb_dev, serial| {
                // Only emit plot data when the device is configured and host has opened the port.
                if usb_dev.state() == UsbDeviceState::Configured && serial.dtr() {
                    let _ = usb_dev.poll(&mut [serial]);
                    let r = serial.write(line.as_bytes());
                    if let Err(e) = r {
                        warn!("USB serial write error: {:?}", defmt::Debug2Format(&e));
                    }
                }
            });
        }
    }

    /// Receives control events from Core 1 and applies PWM outputs.
    #[task(binds = SIO_IRQ_FIFO, shared = [pwm, fifo], priority = 2)]
    fn sio_interrupt(ctx: sio_interrupt::Context) {
        let mut pwm = ctx.shared.pwm;
        let mut fifo = ctx.shared.fifo;

        while let Some(ctrl) = fifo.lock(|f| f.try_recv_control_event()) {
            let steer_on = MicrosDurationU32::from_ticks(ctrl.steer_pwm_us as u32).min(PWM_PERIOD);
            let power_on = MicrosDurationU32::from_ticks(ctrl.power_pwm_us as u32).min(PWM_PERIOD);
            pwm.lock(|pwm| {
                let _ = pwm.channel_a.set_duty_cycle(micros_to_pwm_ticks(steer_on));
                let _ = pwm.channel_b.set_duty_cycle(micros_to_pwm_ticks(power_on));
            });
            trace!(
                "ctrl: steer={} us  power={} us",
                ctrl.steer_pwm_us, ctrl.power_pwm_us
            );
        }
    }

    #[task(binds = USBCTRL_IRQ, local = [buff: [u8; 64] = [0; 64], buff_len: usize = 0], shared = [usb_dev, serial, pwm, speed_setpoint_mps], priority = 2)]
    fn usb_interrupt(mut ctx: usb_interrupt::Context) {
        info!("usb interrupt");
        let mut usb_dev = ctx.shared.usb_dev;
        let mut serial = ctx.shared.serial;
        let mut pwm = ctx.shared.pwm;

        let buff = ctx.local.buff;
        let buff_len = ctx.local.buff_len;

        // read data from usb serial and store it in the local buffer. The buffer length is stored in a local variable as well.
        let mut bytes_read = 0;
        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
            if !usb_dev.poll(&mut [serial]) {
                return;
            }

            while *buff_len < buff.len() {
                let Ok(count) = serial.read(&mut buff[*buff_len..]) else {
                    break;
                };
                if count == 0 {
                    break;
                }
                bytes_read += count;
                *buff_len += count;
            }

            // Echo received data back to USB if feature is enabled (skip CR/LF to avoid cursor jumping)
            #[cfg(feature = "echo_usb")]
            if bytes_read > 0 {
                for &byte in &buff[*buff_len - bytes_read..*buff_len] {
                    if byte != b'\r' && byte != b'\n' {
                        let _ = serial.write(&[byte]);
                    }
                }
                usb_dev.poll(&mut [serial]);
            }
        });

        if *buff_len == buff.len() {
            info!("USB command too long, dropping buffer");
            *buff_len = 0;
            return;
        }
        if *buff_len > 0 && (buff[*buff_len - 1] == b'\n' || buff[*buff_len - 1] == b'\r') {
            // Newline-terminated command framing.
            let command = core::str::from_utf8(&buff[..*buff_len]).unwrap_or("<invalid utf-8>");
            let command = command.trim();
            info!("Received command: {}", command);
            *buff_len = 0;

            if let Some(rest) = command.strip_prefix("pwm-a ") {
                if let Some(on_time_us) = parse_on_time_us(rest) {
                    let on_time = MicrosDurationU32::from_ticks(on_time_us).min(PWM_PERIOD);
                    let ticks = micros_to_pwm_ticks(on_time);
                    pwm.lock(|pwm| {
                        let _ = pwm.channel_a.set_duty_cycle(ticks);
                    });
                    info!(
                        "Set PWM A on-time to {} us ({} ticks)",
                        on_time.ticks(),
                        ticks
                    );

                    // Echo success if feature is enabled
                    #[cfg(feature = "echo_usb")]
                    (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
                        let _ = serial.write(b"\r\nOK\r\n");
                        usb_dev.poll(&mut [serial]);
                    });
                } else {
                    info!("Invalid pwm-a value: {}", rest);
                }
            } else if let Some(rest) = command.strip_prefix("speed ") {
                if let Some(setpoint_mps) = parse_speed_setpoint(rest) {
                    ctx.shared
                        .speed_setpoint_mps
                        .lock(|setpoint| *setpoint = setpoint_mps);
                    info!("Set speed setpoint to {} m/s", setpoint_mps);

                    #[cfg(feature = "echo_usb")]
                    (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
                        let _ = serial.write(b"\r\nOK\r\n");
                        usb_dev.poll(&mut [serial]);
                    });
                } else {
                    info!("Invalid speed value: {}", rest);
                }
            } else if let Some(rest) = command.strip_prefix("pwm-b ") {
                if let Some(on_time_us) = parse_on_time_us(rest) {
                    let on_time = MicrosDurationU32::from_ticks(on_time_us).min(PWM_PERIOD);
                    let ticks = micros_to_pwm_ticks(on_time);
                    pwm.lock(|pwm| {
                        let _ = pwm.channel_b.set_duty_cycle(ticks);
                    });
                    info!(
                        "Set PWM B on-time to {} us ({} ticks)",
                        on_time.ticks(),
                        ticks
                    );

                    // Echo success if feature is enabled
                    #[cfg(feature = "echo_usb")]
                    (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
                        let _ = serial.write(b"\r\nOK\r\n");
                        usb_dev.poll(&mut [serial]);
                    });
                } else {
                    info!("Invalid pwm-b value: {}", rest);
                }
            } else {
                info!("Unknown command: {}", command);
                #[cfg(feature = "echo_usb")]
                (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
                    let _ = serial.write(b"\r\nERR: unknown/malformed command\r\n");
                    usb_dev.poll(&mut [serial]);
                });
            }
        }
    }
}

// Parse USB command argument as microseconds. Returns None if not a valid number.
fn parse_on_time_us(arg: &str) -> Option<u32> {
    arg.trim().parse::<u32>().ok()
}

fn parse_speed_setpoint(arg: &str) -> Option<f32> {
    arg.trim().parse::<f32>().ok()
}

// Convert microseconds to PWM counter ticks at the PWM timer frequency.
fn micros_to_pwm_ticks(on_time: MicrosDurationU32) -> u16 {
    let pwm_ticks: TimerDurationU32<PWM_TIMER_HZ> = on_time.convert();
    pwm_ticks.ticks().min(u16::MAX as u32) as u16 // Clamp to 16-bit register width.
}
