#![no_std]
#![no_main]
#![deny(unsafe_code)]

use defmt_rtt as _;
use embedded_hal::digital::{InputPin as _, OutputPin as _};
use fugit::ExtU64;
use panic_probe as _;
use rp235x_hal as hal;
use rp235x_hal::gpio;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

const XTAL_FREQ_HZ: u32 = 12_000_000;
const TRIGGER_HIGH_US: u64 = 10;
const TRIGGER_PERIOD_MS: u64 = 60;
const CSV_PERIOD_MS: u64 = 60;
const ECHO_TIMEOUT_US: u64 = 35_000;
const ECHO_MIN_US: u64 = 100;
const ECHO_MAX_US: u64 = 25_000;

type MyUsbBus = hal::usb::UsbBus;
type TxPin = gpio::Pin<gpio::bank0::Gpio10, gpio::FunctionSioOutput, gpio::PullDown>;
type RxPin = gpio::Pin<gpio::bank0::Gpio11, gpio::FunctionSioInput, gpio::PullDown>;

/// Required by the RP2350 bootrom to identify and validate the image.
#[allow(unsafe_code)]
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[rtic::app(device = rp235x_pac, peripherals = true, dispatchers = [DMA_IRQ_0, DMA_IRQ_1])]
mod app {
	use defmt::{info, warn};
	use rtic_monotonics::Monotonic;

	use super::*;

	#[shared]
	struct Shared {
		tx_pin: TxPin,
		rx_pin: RxPin,
		usb_dev: UsbDevice<'static, MyUsbBus>,
		serial: SerialPort<'static, MyUsbBus>,
		echo_start_us: Option<u64>,
		last_distance_cm_x10000: Option<u32>,
		csv_header_sent: bool,
	}

	#[local]
	struct Local {}

	rtic_monotonics::rp235x_timer_monotonic!(Mono);

	#[init(local = [usb_bus: Option<UsbBusAllocator<MyUsbBus>> = None])]
	fn init(ctx: init::Context) -> (Shared, Local) {
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

		let sio = hal::Sio::new(pac.SIO);
		let pins = hal::gpio::Pins::new(
			pac.IO_BANK0,
			pac.PADS_BANK0,
			sio.gpio_bank0,
			&mut pac.RESETS,
		);

		let tx_pin = pins.gpio10.into_push_pull_output();
		let mut rx_pin = pins.gpio11.into_pull_down_input();
		rx_pin.clear_interrupt(gpio::Interrupt::EdgeHigh);
		rx_pin.clear_interrupt(gpio::Interrupt::EdgeLow);
		rx_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
		rx_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);

		let mut tx_pin = tx_pin;
		let _ = tx_pin.set_low();
		let _ = clocks;
		info!(
			"Starting: trigger {} us every {} ms, echo on GPIO11, CSV over USB",
			TRIGGER_HIGH_US,
			TRIGGER_PERIOD_MS
		);
		Mono::start(pac.TIMER0, &pac.RESETS);
		send_trigger::spawn().ok();
		stream_csv_over_usb::spawn().ok();

		(
			Shared {
				tx_pin,
				rx_pin,
				usb_dev,
				serial,
				echo_start_us: None,
				last_distance_cm_x10000: None,
				csv_header_sent: false,
			},
			Local {},
		)
	}

	#[idle]
	fn idle(_ctx: idle::Context) -> ! {
		loop {
			cortex_m::asm::wfe();
		}
	}

	#[task(shared = [tx_pin, rx_pin, echo_start_us], priority = 2)]
	async fn send_trigger(mut ctx: send_trigger::Context) -> ! {
		loop {
			let now_us = Mono::now().ticks();
			ctx.shared.echo_start_us.lock(|start| {
				if let Some(start_us) = *start {
					if now_us.saturating_sub(start_us) > ECHO_TIMEOUT_US {
						*start = None;
						warn!("Echo timeout: stale pulse start cleared");
					}
				}
			});

			// If echo is still high, previous measurement did not finish cleanly.
			let rx_is_high = ctx.shared.rx_pin.lock(|pin| pin.is_high().unwrap_or(false));
			if rx_is_high {
				warn!("Echo pin still HIGH before trigger; skipping cycle");
				Mono::delay(TRIGGER_PERIOD_MS.millis()).await;
				continue;
			}

			ctx.shared.tx_pin.lock(|pin| {
				let _ = pin.set_high();
			});
			Mono::delay(TRIGGER_HIGH_US.micros()).await;
			ctx.shared.tx_pin.lock(|pin| {
				let _ = pin.set_low();
			});

			Mono::delay(TRIGGER_PERIOD_MS.millis()).await;
		}
	}

	#[task(shared = [usb_dev, serial, last_distance_cm_x10000, csv_header_sent], priority = 1)]
	async fn stream_csv_over_usb(mut ctx: stream_csv_over_usb::Context) -> ! {
		loop {
			let distance_cm_x10000 = ctx.shared.last_distance_cm_x10000.lock(|d| *d);

			(&mut ctx.shared.usb_dev, &mut ctx.shared.serial).lock(|usb_dev, serial| {
				if !usb_dev.poll(&mut [serial]) {
					return;
				}

				if usb_dev.state() != UsbDeviceState::Configured || !serial.dtr() {
					return;
				}

				ctx.shared.csv_header_sent.lock(|header_sent| {
					if !*header_sent {
						let _ = serial.write(b"cm\r\n");
						*header_sent = true;
					}
				});

				if let Some(cm_x10000) = distance_cm_x10000 {
					let mut row = [0u8; 32];
					let n = u32_fixed4_to_ascii_line(cm_x10000, &mut row);
					let _ = serial.write(&row[..n]);
				}
			});

			Mono::delay(CSV_PERIOD_MS.millis()).await;
		}
	}

	#[task(binds = IO_IRQ_BANK0, shared = [rx_pin, echo_start_us, last_distance_cm_x10000], priority = 3)]
	fn gpio11_echo_irq(mut ctx: gpio11_echo_irq::Context) {
		let now_us = Mono::now().ticks();
		let (saw_rise, saw_fall) = ctx.shared.rx_pin.lock(|pin| {
			let rising = pin.interrupt_status(gpio::Interrupt::EdgeHigh);
			let falling = pin.interrupt_status(gpio::Interrupt::EdgeLow);

			if rising {
				pin.clear_interrupt(gpio::Interrupt::EdgeHigh);
			}
			if falling {
				pin.clear_interrupt(gpio::Interrupt::EdgeLow);
			}

			(rising, falling)
		});

		if saw_rise {
			ctx.shared.echo_start_us.lock(|start| {
				if start.is_none() {
					*start = Some(now_us);
				} else {
					warn!("Ignored extra rising edge while waiting for falling edge");
				}
			});
		}

		if saw_fall {
			ctx.shared.echo_start_us.lock(|start| {
				if let Some(start_us) = *start {
					let pulse_us = now_us.saturating_sub(start_us);
					if (ECHO_MIN_US..=ECHO_MAX_US).contains(&pulse_us) {
						let distance_cm = (pulse_us as f32) / 56.0;
						let distance_cm_x10000 = (distance_cm * 10_000.0 + 0.5) as u32;
						ctx.shared.last_distance_cm_x10000.lock(|last| {
							*last = Some(distance_cm_x10000);
						});
						let whole = distance_cm_x10000 / 10_000;
						let frac = distance_cm_x10000 % 10_000;
						info!("Echo: {} us -> {}.{=u32:04} cm", pulse_us, whole, frac);
					} else {
						warn!("Discarded out-of-range echo pulse: {} us", pulse_us);
					}
				} else {
					warn!("Falling edge without recorded rising edge");
				}
				*start = None;
			});
		}
	}

	fn u32_to_ascii(mut value: u32, out: &mut [u8; 10]) -> usize {
		let mut digits = [0u8; 20];
		let mut len = 0usize;

		if value == 0 {
			digits[0] = b'0';
			len = 1;
		} else {
			while value > 0 {
				digits[len] = b'0' + (value % 10) as u8;
				len += 1;
				value /= 10;
			}
		}

		for i in 0..len {
			out[i] = digits[len - 1 - i];
		}
		len
	}

	fn u32_fixed4_to_ascii_line(value_x10000: u32, out: &mut [u8; 32]) -> usize {
		let whole = value_x10000 / 10_000;
		let frac = value_x10000 % 10_000;

		let mut idx = 0usize;
		let mut whole_buf = [0u8; 10];
		let whole_len = u32_to_ascii(whole, &mut whole_buf);
		out[..whole_len].copy_from_slice(&whole_buf[..whole_len]);
		idx += whole_len;

		out[idx] = b'.';
		idx += 1;

		out[idx] = b'0' + ((frac / 1000) % 10) as u8;
		out[idx + 1] = b'0' + ((frac / 100) % 10) as u8;
		out[idx + 2] = b'0' + ((frac / 10) % 10) as u8;
		out[idx + 3] = b'0' + (frac % 10) as u8;
		idx += 4;

		out[idx] = b'\r';
		out[idx + 1] = b'\n';
		idx + 2
	}

	fn init_usb_serial(
		usb: rp235x_pac::USB,
		usb_dpram: rp235x_pac::USB_DPRAM,
		usb_clock: hal::clocks::UsbClock,
		resets: &mut rp235x_pac::RESETS,
		usb_bus_slot: &'static mut Option<UsbBusAllocator<MyUsbBus>>,
	) -> (UsbDevice<'static, MyUsbBus>, SerialPort<'static, MyUsbBus>) {
		let usb_bus = MyUsbBus::new(usb, usb_dpram, usb_clock, true, resets);
		*usb_bus_slot = Some(UsbBusAllocator::new(usb_bus));

		let bus_ref = usb_bus_slot
			.as_ref()
			.expect("USB bus allocator must be initialized");

		let serial = SerialPort::new(bus_ref);
		let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
			.strings(&[StringDescriptors::default()
				.manufacturer("rp2350_controller")
				.product("HC-SR04 CSV Stream")
				.serial_number("HC-SR04-CSV")])
			.expect("USB string descriptors must be valid")
			.device_class(USB_CLASS_CDC)
			.build();

		(usb_dev, serial)
	}
}
