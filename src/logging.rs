use crate::usb_serial::MyUsbBus;
use core::fmt::Write as _;
use defmt::warn;
use fugit::TimerInstantU64;
use heapless::String;
use rtic::Mutex;
use rtic::mutex_prelude::*;
use usb_device::device::UsbDevice;
use usb_device::device::UsbDeviceState;
use usbd_serial::SerialPort;

#[derive(Clone, Copy)]
pub struct LogData {
    pub timestamp: TimerInstantU64<1_000_000>,
    pub event: LogEvent,
}

#[derive(Clone, Copy)]
pub enum LogEvent {
    Controller {
        steer_value_us: u16,
        throttle_value_us: u16,
        setpoint_value_mps: f32,
        error_value: f32,
        kalman_values: [f32; 4],
    },
    HallDeltaT {
        delta_t_us: u32,
    },
    Ultrasound {
        distance0_cm: Option<u32>,
        distance1_cm: Option<u32>,
        distance2_cm: Option<u32>,
    },
}

pub fn write_log_data(
    data: LogData,
    mut usb_dev: impl Mutex<T = UsbDevice<'static, MyUsbBus>>,
    mut serial: impl Mutex<T = SerialPort<'static, MyUsbBus>>,
) {
    let mut line: String<256> = String::new();

    #[cfg(not(feature = "simple_csv"))]
    {
        format_plotter_line(&mut line, data);
    }

    #[cfg(feature = "simple_csv")]
    {
        format_csv_line(&mut line, data);
    }

    (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
        // Only emit plot data when the device is configured and host has opened the port.
        let device_state = usb_dev.state();
        let dtr_set = serial.dtr();

        defmt::trace!(
            "USB state: {:?}, DTR: {}",
            defmt::Debug2Format(&device_state),
            dtr_set
        );

        if device_state == UsbDeviceState::Configured && dtr_set {
            let _ = usb_dev.poll(&mut [serial]);
            let mut offset = 0;
            let bytes = line.as_bytes();
            while offset < bytes.len() {
                match serial.write(&bytes[offset..]) {
                    Ok(0) => {
                        // FIFO full, stop trying
                        break;
                    }
                    Ok(n) => {
                        offset += n;
                    }
                    Err(e) => {
                        warn!("USB serial write error: {:?}", defmt::Debug2Format(&e));
                        break;
                    }
                }
            }
        } else if device_state != UsbDeviceState::Configured {
            defmt::trace!(
                "Device not configured: {:?}",
                defmt::Debug2Format(&device_state)
            );
        } else if !dtr_set {
            defmt::trace!("DTR not set - host may not have opened the port");
        }
    });
}

#[cfg(not(feature = "simple_csv"))]
fn format_plotter_line(line: &mut String<256>, data: LogData) {
    let _ = write!(line, ">time_us:{}", data.timestamp.ticks());

    match data.event {
        LogEvent::Controller {
            steer_value_us,
            throttle_value_us,
            setpoint_value_mps,
            error_value,
            kalman_values,
        } => {
            push_named_u16(line, "steer_us", steer_value_us);
            push_named_u16(line, "throttle_us", throttle_value_us);
            push_named_f32(line, "setpoint_mps", setpoint_value_mps);
            push_named_f32(line, "error", error_value);
            push_named_f32(line, "kalman0", kalman_values[0]);
            push_named_f32(line, "kalman1", kalman_values[1]);
            push_named_f32(line, "kalman2", kalman_values[2]);
            push_named_f32(line, "kalman3", kalman_values[3]);
        }
        LogEvent::HallDeltaT { delta_t_us } => {
            push_named_u32(line, "delta_t_us", delta_t_us);
        }
        LogEvent::Ultrasound {
            distance0_cm,
            distance1_cm,
            distance2_cm,
        } => {
            push_named_opt_u32(line, "distance0_cm", distance0_cm);
            push_named_opt_u32(line, "distance1_cm", distance1_cm);
            push_named_opt_u32(line, "distance2_cm", distance2_cm);
        }
    }

    let _ = write!(line, "\r\n");
}

#[cfg(feature = "simple_csv")]
fn format_csv_line(line: &mut String<256>, data: LogData) {
    // Stable column order:
    // time_us,event,steer_us,throttle_us,setpoint_mps,error,delta_t_us,
    // kalman0,kalman1,kalman2,kalman3,distance0_cm,distance1_cm,distance2_cm
    match data.event {
        LogEvent::Controller {
            steer_value_us,
            throttle_value_us,
            setpoint_value_mps,
            error_value,
            kalman_values,
        } => {
            let _ = write!(
                line,
                "{},controller,{},{},{:.4},{:.4},",
                data.timestamp.ticks(),
                steer_value_us,
                throttle_value_us,
                setpoint_value_mps,
                error_value,
            );
            let _ = write!(line, "null");
            let _ = write!(
                line,
                ",{:.4},{:.4},{:.4},{:.4},null,null,null\r\n",
                kalman_values[0], kalman_values[1], kalman_values[2], kalman_values[3],
            );
        }
        LogEvent::HallDeltaT { delta_t_us } => {
            let _ = write!(
                line,
                "{},hall_delta_t,null,null,null,null,",
                data.timestamp.ticks()
            );
            let _ = write!(line, "{}", delta_t_us);
            let _ = write!(line, ",null,null,null,null,null,null,null\r\n");
        }
        LogEvent::Ultrasound {
            distance0_cm,
            distance1_cm,
            distance2_cm,
        } => {
            let _ = write!(
                line,
                "{},ultrasound,null,null,null,null,null,",
                data.timestamp.ticks()
            );
            let _ = write!(line, "null,null,null,null,");
            push_csv_opt_u32(line, distance0_cm);
            let _ = write!(line, ",");
            push_csv_opt_u32(line, distance1_cm);
            let _ = write!(line, ",");
            push_csv_opt_u32(line, distance2_cm);
            let _ = write!(line, "\r\n");
        }
    }
}

#[cfg(not(feature = "simple_csv"))]
fn push_named_u16(line: &mut String<256>, name: &str, value: u16) {
    let _ = write!(line, ",{}:{}", name, value);
}

#[cfg(not(feature = "simple_csv"))]
fn push_named_u32(line: &mut String<256>, name: &str, value: u32) {
    let _ = write!(line, ",{}:{}", name, value);
}

#[cfg(not(feature = "simple_csv"))]
fn push_named_f32(line: &mut String<256>, name: &str, value: f32) {
    let _ = write!(line, ",{}:{:.4}", name, value);
}

#[cfg(not(feature = "simple_csv"))]
fn push_named_opt_u32(line: &mut String<256>, name: &str, value: Option<u32>) {
    if let Some(value) = value {
        let _ = write!(line, ",{}:{}", name, value);
    }
}

#[cfg(feature = "simple_csv")]
fn push_csv_opt_u32(line: &mut String<256>, value: Option<u32>) {
    if let Some(value) = value {
        let _ = write!(line, "{}", value);
    } else {
        let _ = write!(line, "null");
    }
}
