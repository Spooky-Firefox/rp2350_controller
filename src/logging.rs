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

pub struct LogData {
    pub timestamp: TimerInstantU64<1_000_000>,
    pub steer_value_ms: u16,
    pub throttle_value_ms: u16,
    pub setpoint_value: f32,
    pub error_value: f32,
    pub speed_value: f32,
    pub kalman_values: [f32; 4],
}

pub fn write_log_data(
    data: LogData,
    mut usb_dev: impl Mutex<T = UsbDevice<'static, MyUsbBus>>,
    mut serial: impl Mutex<T = SerialPort<'static, MyUsbBus>>,
) {
    let mut line: String<256> = String::new();
    
    #[cfg(not(feature = "simple_csv"))]
    {
        // VS Code Serial Plotter line format: ">name:value,name:value\r\n"
        let _ = write!(
            &mut line,
            ">time_us:{}, steer_ms:{}, throttle_ms:{}, speed_mps:{:.4}, setpoint_mps:{:.4}, error:{:.4}, kalman0:{:.4}, kalman1:{:.4}, kalman2:{:.4}, kalman3:{:.4}\r\n",
            data.timestamp,
            data.steer_value_ms,
            data.throttle_value_ms,
            data.speed_value,
            data.setpoint_value,
            data.error_value,
            data.kalman_values[0],
            data.kalman_values[1],
            data.kalman_values[2],
            data.kalman_values[3],
        );
    }
    
    #[cfg(feature = "simple_csv")]
    {
        // Simple CSV format: value,value,value\r\n
        let _ = write!(
            &mut line,
            "{},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}\r\n",
            data.timestamp,
            data.steer_value_ms,
            data.throttle_value_ms,
            data.speed_value,
            data.setpoint_value,
            data.error_value,
            data.kalman_values[0],
            data.kalman_values[1],
            data.kalman_values[2],
            data.kalman_values[3],
        );
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
