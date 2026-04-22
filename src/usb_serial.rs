//! USB serial initialization and communication.
#![deny(unsafe_code)]

use crate::constants::PWM_PERIOD;
use crate::utils::micros_to_pwm_ticks;
use defmt::{info, trace};
use embedded_hal::pwm::SetDutyCycle as _;
use fugit::MicrosDurationU32;
use rp235x_hal as hal;
use rp235x_hal::{clocks::UsbClock, pac, usb::UsbBus};
use rtic::Mutex;
use rtic::mutex_prelude::*;
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
pub type MyUsbBus = UsbBus;

pub fn init_usb_serial(
    usb: pac::USB,
    usb_dpram: pac::USB_DPRAM,
    usb_clock: UsbClock,
    resets: &mut pac::RESETS,
    usb_bus_slot: &'static mut Option<UsbBusAllocator<MyUsbBus>>,
) -> (UsbDevice<'static, MyUsbBus>, SerialPort<'static, MyUsbBus>) {
    let usb_bus = UsbBus::new(usb, usb_dpram, usb_clock, true, resets);
    *usb_bus_slot = Some(UsbBusAllocator::new(usb_bus));

    let bus_ref = usb_bus_slot
        .as_ref()
        .expect("USB bus allocator must be initialized");

    let serial = SerialPort::new(bus_ref);
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("rp2350_controller")
            .product("CDC Serial")
            .serial_number("TEST")])
        .expect("USB string descriptors must be valid")
        .device_class(USB_CLASS_CDC)
        .build();

    (usb_dev, serial)
}

pub fn process_usb_command(
    mut usb_dev: impl Mutex<T = UsbDevice<'static, MyUsbBus>>,
    mut serial: impl Mutex<T = SerialPort<'static, MyUsbBus>>,
    mut pwm: impl Mutex<T = hal::pwm::Slice<hal::pwm::Pwm0, hal::pwm::FreeRunning>>,
    command_buffer: &mut [u8; 64],
    buffer_len: &mut usize,
    mut speed_setpoint: impl Mutex<T = f32>,
) {
    let mut bytes_read = 0;
    (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
        trace!("USB interrupt - polling device. State: {:?}", defmt::Debug2Format(&usb_dev.state()));
        if !usb_dev.poll(&mut [serial]) {
            trace!("No USB events to process");
            return;
        }
        trace!("USB event detected");

        while *buffer_len < command_buffer.len() {
            let Ok(count) = serial.read(&mut command_buffer[*buffer_len..]) else {
                break;
            };
            if count == 0 {
                break;
            }
            bytes_read += count;
            *buffer_len += count;
        }

        // Echo received data back to USB if feature is enabled (skip CR/LF to avoid cursor jumping)
        #[cfg(feature = "echo_usb")]
        if bytes_read > 0 {
            for &byte in &command_buffer[*buffer_len - bytes_read..*buffer_len] {
                if byte != b'\r' && byte != b'\n' {
                    let _ = serial.write(&[byte]);
                }
            }
            usb_dev.poll(&mut [serial]);
        }
    });

    if *buffer_len == command_buffer.len() {
        info!("USB command too long, dropping buffer");
        *buffer_len = 0;
        return;
    }
    if *buffer_len > 0
        && (command_buffer[*buffer_len - 1] == b'\n' || command_buffer[*buffer_len - 1] == b'\r')
    {
        // Newline-terminated command framing.
        let command =
            core::str::from_utf8(&command_buffer[..*buffer_len]).unwrap_or("<invalid utf-8>");
        let command = command.trim();
        info!("Received command: {}", command);
        *buffer_len = 0;

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
                speed_setpoint.lock(|setpoint| *setpoint = setpoint_mps);
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

// Parse USB command argument as microseconds. Returns None if not a valid number.
fn parse_on_time_us(arg: &str) -> Option<u32> {
    arg.trim().parse::<u32>().ok()
}

fn parse_speed_setpoint(arg: &str) -> Option<f32> {
    arg.trim().parse::<f32>().ok()
}
