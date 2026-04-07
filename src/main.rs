#![no_std]
#![no_main]

use core::f32::consts::PI;

use defmt_rtt as _;
use embedded_hal::digital::StatefulOutputPin;
use fugit::ExtU64;
use panic_probe as _;
use rp235x_hal as hal;
use rp235x_pac as pac;
use rp2350_controller::entry::entry;
use rp2350_controller::usb_serial::{MyUsbBus, init_usb_serial};
use usb_device::{class_prelude::UsbBusAllocator, device::UsbDevice};
use usbd_serial::SerialPort;

/// Required by the RP2350 bootrom to identify and validate the image.
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const XTAL_FREQ_HZ: u32 = 12_000_000u32;
const LENGTH_PER_HAL_RISE_METERS: f32 = 13.0 * PI / 300.0;

type LedPin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio25, hal::gpio::FunctionSioOutput, hal::gpio::PullDown>;

type EncoderPin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio13, hal::gpio::FunctionSioInput, hal::gpio::PullUp>;

#[rtic::app(device = crate::pac, peripherals = true, dispatchers = [DMA_IRQ_0])]
mod app {

    use cortex_m::asm::delay;
    use defmt::{info, trace};
    use rp235x_pac::usb;
    use rtic_monotonics::Monotonic;

    use super::*;

    #[shared]
    struct Shared {
        encoder_time_diff: fugit::Duration<u64, 1, 1_000_000>,
        usb_dev: UsbDevice<'static, MyUsbBus>,
        serial: SerialPort<'static, MyUsbBus>,
    }

    #[local]
    struct Local {
        encoder: EncoderPin,
        encoder_last_time: fugit::Instant<u64, 1, 1_000_000>,
        led: LedPin,
    }

    rtic_monotonics::rp235x_timer_monotonic!(MainMono);

    #[init(local = [usb_bus: Option<UsbBusAllocator<MyUsbBus>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // do init stuff that are in hal::entry but left out when using rtic which uses the cortex-m-rt entry point.
        unsafe { entry() };

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
        let sio = hal::Sio::new(pac.SIO);
        let pins = hal::gpio::Pins::new(io_bank, pads_bank, sio.gpio_bank0, &mut pac.RESETS);

        let onboard_led = pins.gpio25.into_push_pull_output();

        toggle_led::spawn().unwrap();

        let encoder = pins.gpio13.into_pull_up_input();
        encoder.set_interrupt_enabled(rp235x_hal::gpio::Interrupt::EdgeHigh, true);

        MainMono::start(pac.TIMER0, &pac.RESETS);
        (
            Shared {
                encoder_time_diff: fugit::Duration::<u64, 1, 1_000_000>::from_ticks(0),
                usb_dev,
                serial,
            },
            Local {
                led: onboard_led,
                encoder,
                encoder_last_time: fugit::Instant::<u64, 1, 1000000>::from_ticks(0),
            },
        )
    }

    #[idle(shared = [encoder_time_diff])]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
            //info!("sleeping");
            trace!(
                "secs measured: {}",
                (ctx.shared.encoder_time_diff.lock(|diff| *diff).ticks() as f32) / 1_000_000.0
            );
            info!(
                "encoder speed : {} m/s",
                calculate_speed(ctx.shared.encoder_time_diff.lock(|diff| *diff))
            );
            delay(100_000_000);
        }
    }

    #[task(local = [led], priority = 1)]
    async fn toggle_led(ctx: toggle_led::Context) -> ! {
        loop {
            MainMono::delay(1u64.secs()).await;
            ctx.local.led.toggle().unwrap();
        }
    }

    #[task(binds = IO_IRQ_BANK0, local = [cnt: u32 = 0,encoder, encoder_last_time], shared = [encoder_time_diff], priority = 2)]
    fn gpio_interrupt(mut ctx: gpio_interrupt::Context) {
        trace!("gpio interrupt");
        *ctx.local.cnt += 1;
        if ctx
            .local
            .encoder
            .interrupt_status(hal::gpio::Interrupt::EdgeHigh)
        {
            trace!("encoder edge high cnt {}", ctx.local.cnt);

            // Calculate the time difference between this edge and the last edge, and store it in the shared resource.
            let now = MainMono::now();
            let time_diff = now - *ctx.local.encoder_last_time;
            *ctx.local.encoder_last_time = now;
            ctx.shared.encoder_time_diff.lock(|diff| *diff = time_diff);
            ctx.local
                .encoder
                .clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
        }
    }

    #[task(binds = USBCTRL_IRQ, shared = [usb_dev, serial], priority = 2)]
    fn usb_interrupt(mut ctx: usb_interrupt::Context) {
        let usb_dev = ctx.shared.usb_dev;
        let serial = ctx.shared.serial;

        (usb_dev, serial).lock(|usb_dev, serial| {
            if !usb_dev.poll(&mut [serial]) {
                return;
            }

            let mut buf = [0u8; 64];
            if let Ok(count) = serial.read(&mut buf)
                && count > 0
            {
                let _ = serial.write(&buf[..count]);
            }
        });
    }
}

fn calculate_speed(magnet_speed: fugit::Duration<u64, 1, 1_000_000>) -> f32 {
    let dur_sec = magnet_speed.ticks() as f32 / 1_000_000.0;

    LENGTH_PER_HAL_RISE_METERS / dur_sec
}
