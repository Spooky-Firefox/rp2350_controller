//! USB serial initialization and communication.
#![deny(unsafe_code)]

use crate::ipc::Constants;
use defmt::{info, trace};
use rp235x_hal::{clocks::UsbClock, pac, usb::UsbBus};
use rtic::Mutex;
use rtic::mutex_prelude::*;
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

pub type MyUsbBus = UsbBus;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlMode {
    Manual,
    Auto,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlModes {
    pub steering: ControlMode,
    pub throttle: ControlMode,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlTarget {
    Steering,
    Throttle,
    Both,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum UsbCommand {
    SetPwmA {
        on_time_us: u32,
    },
    SetPwmB {
        on_time_us: u32,
    },
    SetConstant {
        constant: Constants,
        value: f32,
    },
    SendAlign {
        angle: f32,
        confidence: f32,
    },
    SetControlMode {
        target: ControlTarget,
        mode: ControlMode,
    },
}

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
    command_buffer: &mut [u8; 64],
    buffer_len: &mut usize,
) -> Option<UsbCommand> {
    let mut bytes_read = 0;
    (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
        trace!(
            "USB interrupt - polling device. State: {:?}",
            defmt::Debug2Format(&usb_dev.state())
        );
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
        return None;
    }
    if *buffer_len > 0
        && (command_buffer[*buffer_len - 1] == b'\n' || command_buffer[*buffer_len - 1] == b'\r')
    {
        let command =
            core::str::from_utf8(&command_buffer[..*buffer_len]).unwrap_or("<invalid utf-8>");
        let command = command.trim();
        info!("Received command: {}", command);
        *buffer_len = 0;

        if let Some(rest) = command.strip_prefix("pwm-a ") {
            if let Some(on_time_us) = parse_on_time_us(rest) {
                send_ok(&mut usb_dev, &mut serial);
                return Some(UsbCommand::SetPwmA { on_time_us });
            } else {
                info!("Invalid pwm-a value: {}", rest);
            }
        } else if let Some(rest) = command.strip_prefix("pwm-b ") {
            if let Some(on_time_us) = parse_on_time_us(rest) {
                send_ok(&mut usb_dev, &mut serial);
                return Some(UsbCommand::SetPwmB { on_time_us });
            } else {
                info!("Invalid pwm-b value: {}", rest);
            }
        } else if let Some(rest) = command.strip_prefix("const ") {
            if let Some((constant, value)) = parse_constant_update(rest) {
                send_ok(&mut usb_dev, &mut serial);
                return Some(UsbCommand::SetConstant { constant, value });
            } else {
                info!("Invalid const value: {}", rest);
            }
        } else if let Some(rest) = command.strip_prefix("align ") {
            let mut parts = rest.split_ascii_whitespace();
            let angle = parts.next().and_then(|value| value.parse::<f32>().ok());
            let confidence = parts.next().and_then(|value| value.parse::<f32>().ok());
            if let (Some(angle), Some(confidence), None) = (angle, confidence, parts.next()) {
                send_ok(&mut usb_dev, &mut serial);
                return Some(UsbCommand::SendAlign { angle, confidence });
            } else {
                info!("Invalid align value: {}", rest);
            }
        } else if let Some(rest) = command.strip_prefix("mode ") {
            if let Some((target, mode)) = parse_control_mode(rest) {
                send_ok(&mut usb_dev, &mut serial);
                return Some(UsbCommand::SetControlMode { target, mode });
            } else {
                info!("Invalid control mode: {}", rest);
            }
        } else {
            info!("Unknown command: {}", command);
            send_error(
                &mut usb_dev,
                &mut serial,
                b"\r\nERR: unknown/malformed command\r\n",
            );
        }

        send_error(&mut usb_dev, &mut serial, b"\r\nERR: malformed command\r\n");
    }

    None
}

fn parse_on_time_us(arg: &str) -> Option<u32> {
    arg.trim().parse::<u32>().ok()
}

fn parse_constant_update(arg: &str) -> Option<(Constants, f32)> {
    let mut parts = arg.split_ascii_whitespace();
    let constant = Constants::try_from(parts.next()?).ok()?;
    let value = parts.next()?.parse::<f32>().ok()?;
    if parts.next().is_some() {
        return None;
    }
    Some((constant, value))
}

fn parse_control_mode(arg: &str) -> Option<(ControlTarget, ControlMode)> {
    let mut parts = arg.split_ascii_whitespace();
    let first = parts.next()?;
    let second = parts.next();

    if parts.next().is_some() {
        return None;
    }

    let (target, mode_text) = match second {
        Some(mode_text) => (
            match first {
                "steering" => ControlTarget::Steering,
                "throttle" => ControlTarget::Throttle,
                "both" => ControlTarget::Both,
                _ => return None,
            },
            mode_text,
        ),
        None => (ControlTarget::Both, first),
    };

    let mode = match mode_text {
        "manual" => ControlMode::Manual,
        "auto" => ControlMode::Auto,
        _ => return None,
    };

    Some((target, mode))
}

fn send_ok(
    usb_dev: &mut impl Mutex<T = UsbDevice<'static, MyUsbBus>>,
    serial: &mut impl Mutex<T = SerialPort<'static, MyUsbBus>>,
) {
    #[cfg(feature = "echo_usb")]
    (usb_dev, serial).lock(|usb_dev, serial| {
        let _ = serial.write(b"\r\nOK\r\n");
        usb_dev.poll(&mut [serial]);
    });
}

fn send_error(
    usb_dev: &mut impl Mutex<T = UsbDevice<'static, MyUsbBus>>,
    serial: &mut impl Mutex<T = SerialPort<'static, MyUsbBus>>,
    response: &[u8],
) {
    #[cfg(feature = "echo_usb")]
    (usb_dev, serial).lock(|usb_dev, serial| {
        let _ = serial.write(response);
        usb_dev.poll(&mut [serial]);
    });
}
