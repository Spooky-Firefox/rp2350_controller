#![no_std]
#![no_main]
#![deny(unsafe_code)]

use cortex_m::asm;
use embedded_hal::digital::StatefulOutputPin;
use fugit::{ExtU64, MicrosDurationU32, TimerDurationU32};
use panic_halt as _;
use rp235x_hal as hal;
use rp235x_hal::pac;

// ────────────── Constants ──────────────
const XTAL_FREQ_HZ: u32 = 12_000_000;
const SYS_CLOCK_HZ: u32 = 150_000_000;
const PWM_DIV_INT: u8 = 64;
const PWM_TIMER_HZ: u32 = SYS_CLOCK_HZ / (1 + PWM_DIV_INT as u32);
const PWM_PERIOD: MicrosDurationU32 = MicrosDurationU32::from_ticks(20_000);
const PWM_DEFAULT_ON_TIME: MicrosDurationU32 = MicrosDurationU32::from_ticks(1_500);
/// Arc length per encoder pulse [m]: 13π/300
const LENGTH_PER_ENCODER_PULSE_METERS: f32 = 0.13613568;

// ────────────── Type aliases ──────────────
type LedPin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio25, hal::gpio::FunctionSioOutput, hal::gpio::PullDown>;
type EncoderPin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio18, hal::gpio::FunctionSioInput, hal::gpio::PullUp>;
type MotorPwmSlice = hal::pwm::Slice<hal::pwm::Pwm0, hal::pwm::FreeRunning>;
type MotorPwmPinA =
    hal::gpio::Pin<hal::gpio::bank0::Gpio16, hal::gpio::FunctionPwm, hal::gpio::PullDown>;
type MotorPwmPinB =
    hal::gpio::Pin<hal::gpio::bank0::Gpio17, hal::gpio::FunctionPwm, hal::gpio::PullDown>;
type MyUsbBus = hal::usb::UsbBus;

/// Required by the RP2350 bootrom to identify and validate the image.
#[allow(unsafe_code)]
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

// ────────────── RTIC app ──────────────
#[rtic::app(device = crate::pac, peripherals = true, dispatchers = [DMA_IRQ_0, DMA_IRQ_1])]
mod app {
    use core::fmt::Write as _;
    use embedded_hal::pwm::SetDutyCycle as _;
    use fugit::{MicrosDurationU32, TimerDurationU32, TimerInstantU64};
    use heapless::String;
    use rp235x_hal as hal;
    use rtic_monotonics::Monotonic;
    use usb_device::{
        class_prelude::UsbBusAllocator,
        device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid},
    };
    use usbd_serial::{SerialPort, USB_CLASS_CDC};

    use super::*;

    // ── Shared resources ──
    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, MyUsbBus>,
        serial: SerialPort<'static, MyUsbBus>,
        pwm: MotorPwmSlice,
        steer_us: u16,
        throttle_us: u16,
        measured_speed_mps: f32,
        /// Timestamp of the last encoder pulse in µs. 0 = no pulse seen yet.
        last_encoder_us: u64,
    }

    // ── Local resources ──
    #[local]
    struct Local {
        encoder: EncoderPin,
        encoder_last_time: TimerInstantU64<1_000_000>,
        led: LedPin,
        _pwm_a_pin: MotorPwmPinA,
        _pwm_b_pin: MotorPwmPinB,
        cmd_buf: [u8; 64],
        cmd_len: usize,
    }

    rtic_monotonics::rp235x_timer_monotonic!(MainMono);

    #[init(local = [usb_bus: Option<UsbBusAllocator<MyUsbBus>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        #[allow(unsafe_code)]
        unsafe {
            early_init()
        };

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

        // ── USB CDC serial ──
        let usb_bus = hal::usb::UsbBus::new(
            pac.USB,
            pac.USB_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        );
        *ctx.local.usb_bus = Some(UsbBusAllocator::new(usb_bus));
        let bus_ref = ctx.local.usb_bus.as_ref().unwrap();
        let serial = SerialPort::new(bus_ref);
        let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[StringDescriptors::default()
                .manufacturer("rp2350_simple")
                .product("CDC Serial")
                .serial_number("001")])
            .expect("USB strings valid")
            .device_class(USB_CLASS_CDC)
            .build();

        // ── GPIO / PWM ──
        let sio = hal::Sio::new(pac.SIO);
        let pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let led = pins.gpio25.into_push_pull_output();

        let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
        let mut pwm = pwm_slices.pwm0;
        pwm.set_div_int(PWM_DIV_INT);
        let period_ticks: TimerDurationU32<PWM_TIMER_HZ> = PWM_PERIOD.convert();
        pwm.set_top(period_ticks.ticks().saturating_sub(1) as u16);
        pwm.enable();
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

        // ── Encoder ──
        let encoder = pins.gpio18.into_pull_up_input();
        encoder.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);

        // ── Timer (monotonic) ──
        MainMono::start(pac.TIMER0, &pac.RESETS);

        toggle_led::spawn().unwrap();
        sensor_timeout::spawn().unwrap();
        periodic_log::spawn().unwrap();
        send_header::spawn().unwrap();

        (
            Shared {
                usb_dev,
                serial,
                pwm,
                steer_us: 1500,
                throttle_us: 1500,
                measured_speed_mps: 0.0,
                last_encoder_us: 0,
            },
            Local {
                encoder,
                encoder_last_time: TimerInstantU64::<1_000_000>::from_ticks(0),
                led,
                _pwm_a_pin: pwm_a_pin,
                _pwm_b_pin: pwm_b_pin,
                cmd_buf: [0; 64],
                cmd_len: 0,
            },
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            asm::wfe();
        }
    }

    // ── Heartbeat LED (500 ms blink) ──
    #[task(local = [led], priority = 1)]
    async fn toggle_led(ctx: toggle_led::Context) -> ! {
        loop {
            MainMono::delay(500u64.millis()).await;
            let _ = ctx.local.led.toggle();
        }
    }

    // ── Send CSV header once USB enumerated ──
    #[task(shared = [usb_dev, serial], priority = 1)]
    async fn send_header(mut ctx: send_header::Context) -> ! {
        MainMono::delay(2000u64.millis()).await;
        loop {
            let connected = (&mut ctx.shared.usb_dev, &mut ctx.shared.serial)
                .lock(|dev, _s| dev.state() == UsbDeviceState::Configured);
            if connected {
                (&mut ctx.shared.usb_dev, &mut ctx.shared.serial).lock(|dev, serial| {
                    let _ = serial.write(b"# timestamp_us,steer_us,throttle_us,speed_mps\r\n");
                    dev.poll(&mut [serial]);
                });
                return;
            }
            MainMono::delay(500u64.millis()).await;
        }
    }

    // ── Encoder rising-edge interrupt → compute speed + emit one CSV row per pulse ──
    #[task(
        binds = IO_IRQ_BANK0,
        local = [encoder, encoder_last_time],
        shared = [measured_speed_mps, last_encoder_us, usb_dev, serial, steer_us, throttle_us],
        priority = 3
    )]
    fn gpio_interrupt(mut ctx: gpio_interrupt::Context) {
        if ctx
            .local
            .encoder
            .interrupt_status(hal::gpio::Interrupt::EdgeHigh)
        {
            let now = MainMono::now();
            let previous = *ctx.local.encoder_last_time;
            *ctx.local.encoder_last_time = now;
            let now_us = now.ticks();

            ctx.shared.last_encoder_us.lock(|t| *t = now_us);

            if previous.ticks() != 0 {
                let dt_us = (now - previous).ticks() as f32;
                if dt_us > 0.0 {
                    let speed = LENGTH_PER_ENCODER_PULSE_METERS / (dt_us * 1e-6);
                    ctx.shared.measured_speed_mps.lock(|s| *s = speed);

                    let steer = ctx.shared.steer_us.lock(|v| *v);
                    let throttle = ctx.shared.throttle_us.lock(|v| *v);

                    let mut line: String<128> = String::new();
                    let _ = write!(
                        &mut line,
                        "{},{},{},{:.4}\r\n",
                        now_us, steer, throttle, speed
                    );

                    (&mut ctx.shared.usb_dev, &mut ctx.shared.serial).lock(|dev, serial| {
                        if dev.state() == UsbDeviceState::Configured {
                            let _ = serial.write(line.as_bytes());
                            dev.poll(&mut [serial]);
                        }
                    });
                }
            }

            ctx.local
                .encoder
                .clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
        }
    }

    // ── Emit one zero-speed row when no encoder pulse for 200 ms ──
    #[task(
        shared = [measured_speed_mps, last_encoder_us, usb_dev, serial, steer_us, throttle_us],
        priority = 2
    )]
    async fn sensor_timeout(mut ctx: sensor_timeout::Context) -> ! {
        const TIMEOUT_US: u64 = 200_000;
        loop {
            MainMono::delay(50u64.millis()).await;

            let now_us = MainMono::now().ticks();
            let last = ctx.shared.last_encoder_us.lock(|v| *v);

            // No pulses yet, or timeout not elapsed
            if last == 0 || now_us.saturating_sub(last) < TIMEOUT_US {
                continue;
            }

            // Already zeroed — avoid emitting duplicate stopped rows
            let speed = ctx.shared.measured_speed_mps.lock(|v| *v);
            if speed == 0.0 {
                continue;
            }

            // Timeout elapsed and speed is non-zero: car has stopped
            ctx.shared.measured_speed_mps.lock(|s| *s = 0.0);
            let steer = ctx.shared.steer_us.lock(|v| *v);
            let throttle = ctx.shared.throttle_us.lock(|v| *v);

            let mut line: String<128> = String::new();
            let _ = write!(&mut line, "{},{},{},0.0000\r\n", now_us, steer, throttle);

            (&mut ctx.shared.usb_dev, &mut ctx.shared.serial).lock(|dev, serial| {
                if dev.state() == UsbDeviceState::Configured {
                    let _ = serial.write(line.as_bytes());
                    dev.poll(&mut [serial]);
                }
            });
        }
    }

    // ── USB IRQ: poll device and parse commands ──
    #[task(
        binds = USBCTRL_IRQ,
        local = [cmd_buf, cmd_len],
        shared = [usb_dev, serial, pwm, steer_us, throttle_us],
        priority = 5
    )]
    fn usb_interrupt(ctx: usb_interrupt::Context) {
        let cmd_buf = ctx.local.cmd_buf;
        let cmd_len = ctx.local.cmd_len;

        let mut usb_dev = ctx.shared.usb_dev;
        let mut serial = ctx.shared.serial;
        let mut pwm = ctx.shared.pwm;
        let mut steer_us = ctx.shared.steer_us;
        let mut throttle_us = ctx.shared.throttle_us;

        (&mut usb_dev, &mut serial).lock(|dev, serial| {
            if !dev.poll(&mut [serial]) {
                return;
            }
            while *cmd_len < cmd_buf.len() {
                match serial.read(&mut cmd_buf[*cmd_len..]) {
                    Ok(0) | Err(_) => break,
                    Ok(n) => *cmd_len += n,
                }
            }
        });

        // Buffer overflow — discard and return
        if *cmd_len == cmd_buf.len() {
            *cmd_len = 0;
            return;
        }

        if *cmd_len == 0 {
            return;
        }

        // Wait for newline terminator
        let last = cmd_buf[*cmd_len - 1];
        if last != b'\n' && last != b'\r' {
            return;
        }

        let cmd_str = core::str::from_utf8(&cmd_buf[..*cmd_len])
            .unwrap_or("")
            .trim();
        *cmd_len = 0;

        let response: &[u8] = if let Some(rest) = cmd_str.strip_prefix("pwm-a ") {
            if let Some(us) = parse_us(rest) {
                let clamped = us.clamp(1000, 2000);
                let on_time = MicrosDurationU32::from_ticks(clamped as u32).min(PWM_PERIOD);
                pwm.lock(|p| {
                    let _ = p.channel_a.set_duty_cycle(micros_to_pwm_ticks(on_time));
                });
                steer_us.lock(|v| *v = clamped);
                b"\r\nOK\r\n"
            } else {
                b"\r\nERR: bad value\r\n"
            }
        } else if let Some(rest) = cmd_str.strip_prefix("pwm-b ") {
            if let Some(us) = parse_us(rest) {
                let clamped = us.clamp(1000, 2000);
                let on_time = MicrosDurationU32::from_ticks(clamped as u32).min(PWM_PERIOD);
                pwm.lock(|p| {
                    let _ = p.channel_b.set_duty_cycle(micros_to_pwm_ticks(on_time));
                });
                throttle_us.lock(|v| *v = clamped);
                b"\r\nOK\r\n"
            } else {
                b"\r\nERR: bad value\r\n"
            }
        } else {
            b"\r\nERR: unknown command\r\n"
        };

        (&mut usb_dev, &mut serial).lock(|dev, serial| {
            let _ = serial.write(response);
            dev.poll(&mut [serial]);
        });
    }

    // ── Periodic baseline log at 1 Hz (data even when car is still) ──
    #[task(
        shared = [usb_dev, serial, steer_us, throttle_us, measured_speed_mps],
        priority = 1
    )]
    async fn periodic_log(mut ctx: periodic_log::Context) -> ! {
        loop {
            MainMono::delay(1000u64.millis()).await;

            let now_us = MainMono::now().ticks();
            let steer = ctx.shared.steer_us.lock(|v| *v);
            let throttle = ctx.shared.throttle_us.lock(|v| *v);
            let speed = ctx.shared.measured_speed_mps.lock(|v| *v);

            let mut line: String<128> = String::new();
            let _ = write!(
                &mut line,
                "{},{},{},{:.4}\r\n",
                now_us, steer, throttle, speed
            );

            (&mut ctx.shared.usb_dev, &mut ctx.shared.serial).lock(|dev, serial| {
                if dev.state() == UsbDeviceState::Configured {
                    let _ = serial.write(line.as_bytes());
                    dev.poll(&mut [serial]);
                }
            });
        }
    }
}

// ── Free functions ──

fn micros_to_pwm_ticks(on_time: MicrosDurationU32) -> u16 {
    let ticks: TimerDurationU32<PWM_TIMER_HZ> = on_time.convert();
    ticks.ticks().min(u16::MAX as u32) as u16
}

fn parse_us(s: &str) -> Option<u16> {
    s.trim().parse::<u16>().ok()
}

/// Early hardware init: reset spinlocks and enable co-processors.
/// Mirrors what rp235x-hal's `#[entry]` macro injects automatically.
///
/// # Safety
/// Must be called once, single-threaded, before any other hardware access.
#[allow(unsafe_code)]
unsafe fn early_init() {
    const SIO_BASE: u32 = 0xd000_0000;
    const SPINLOCK0_PTR: *mut u32 = (SIO_BASE + 0x100) as *mut u32;
    for i in 0..32usize {
        // Safety: valid RP2350 register addresses, called once at boot.
        unsafe { SPINLOCK0_PTR.wrapping_add(i).write_volatile(1) };
    }
    #[cfg(target_arch = "arm")]
    {
        const SCB_CPACR_PTR: *mut u32 = 0xE000_ED88 as *mut u32;
        const FULL: u32 = 0b11;
        // Safety: valid Cortex-M33 CPACR address, called once at boot.
        let mut val = unsafe { SCB_CPACR_PTR.read_volatile() };
        val |= FULL << (4 * 2); // DCP co-processor (CP4)
        #[allow(clippy::erasing_op, clippy::identity_op)]
        {
            val |= FULL << (0 * 2); // GPIO co-processor (CP0)
        }
        unsafe { SCB_CPACR_PTR.write_volatile(val) };
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}
