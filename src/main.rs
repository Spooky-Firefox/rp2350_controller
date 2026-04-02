#![no_std]
#![no_main]

use cortex_m::asm::delay;
use defmt_rtt as _;
use embedded_hal::digital::StatefulOutputPin;
use panic_probe as _;
use rp235x_hal as hal;
use rp235x_pac as pac;

/// Required by the RP2350 bootrom to identify and validate the image.
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

type LedPin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio25, hal::gpio::FunctionSioOutput, hal::gpio::PullDown>;

#[rtic::app(device = crate::pac, peripherals = true, dispatchers = [DMA_IRQ_0])]
mod app {

    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: LedPin,
    }

    rtic_monotonics::rp235x_timer_monotonic!(MainMono);

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
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
        let onboard_led = hal::gpio::Pins::new(io_bank, pads_bank, sio.gpio_bank0, &mut pac.RESETS)
            .gpio25
            .into_push_pull_output();

        MainMono::start(pac.TIMER0, &pac.RESETS);
        (Shared {}, Local { led: onboard_led })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            defmt::info!("RTT alive");
            delay(150_000_000);
        }
    }

    #[task(local = [led], priority = 1)]
    async fn toggle_led(ctx: toggle_led::Context) {
        ctx.local.led.toggle().unwrap();
    }
}
