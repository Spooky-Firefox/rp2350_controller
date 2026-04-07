#![no_std]
#![no_main]

use defmt_rtt as _;
use embedded_hal::digital::StatefulOutputPin;
use fugit::ExtU64;
use panic_probe as _;
use rp235x_hal as hal;
use rp235x_pac as pac;
use rp2350_controller::entry::entry;

/// Required by the RP2350 bootrom to identify and validate the image.
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

type LedPin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio25, hal::gpio::FunctionSioOutput, hal::gpio::PullDown>;

type EncoderPin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio13, hal::gpio::FunctionSioInput, hal::gpio::PullUp>;

#[rtic::app(device = crate::pac, peripherals = true, dispatchers = [DMA_IRQ_0])]
mod app {

    use cortex_m::asm::delay;
    use defmt::info;
    use embedded_hal::digital::InputPin;
    use rtic_monotonics::Monotonic;

    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        encoder: EncoderPin,
        led: LedPin,
    }

    rtic_monotonics::rp235x_timer_monotonic!(MainMono);

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // do init stuff that are in hal::entry but left out when using rtic which uses the cortex-m-rt entry point.
        unsafe { entry() };

        // Grab our singleton objects
        let mut pac = ctx.device;

        // Set up the watchdog driver - needed by the clock setup code
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

        // Configure the clocks
        let _clocks = hal::clocks::init_clocks_and_plls(
            XTAL_FREQ_HZ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .unwrap();

        let pads_bank = pac.PADS_BANK0;
        let io_bank = pac.IO_BANK0;
        let sio = hal::Sio::new(pac.SIO);
        let pins = hal::gpio::Pins::new(io_bank, pads_bank, sio.gpio_bank0, &mut pac.RESETS);

        let onboard_led = pins.gpio25.into_push_pull_output();

        toggle_led::spawn().unwrap();

        let mut encoder = pins.gpio13.into_pull_up_input();
        // Clear any latched edge status before enabling IRQ sources.
        encoder.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
        encoder.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
        encoder.set_interrupt_enabled(rp235x_hal::gpio::Interrupt::EdgeHigh, true);
        encoder.set_interrupt_enabled(rp235x_hal::gpio::Interrupt::EdgeLow, true);

        MainMono::start(pac.TIMER0, &pac.RESETS);
        (
            Shared {},
            Local {
                led: onboard_led,
                encoder,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            //info!("sleeping");
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

    #[task(binds = IO_IRQ_BANK0, local = [cnt: u32 = 0,encoder], priority = 2)]
    fn gpio_interrupt(ctx: gpio_interrupt::Context) {
        info!("gpio interrupt");
        *ctx.local.cnt += 1;
        if ctx
            .local
            .encoder
            .interrupt_status(hal::gpio::Interrupt::EdgeHigh)
        {
            info!("encoder edge high cnt {}", ctx.local.cnt);
            ctx.local
                .encoder
                .clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
        }
        if ctx
            .local
            .encoder
            .interrupt_status(hal::gpio::Interrupt::EdgeLow)
        {
            info!("encoder edge low cnt {}", ctx.local.cnt);
            ctx.local
                .encoder
                .clear_interrupt(hal::gpio::Interrupt::EdgeLow);
        }
    }
}
