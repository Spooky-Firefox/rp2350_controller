//! USB serial initialization and communication.
#![deny(unsafe_code)]

use rp235x_hal::{clocks::UsbClock, pac, usb::UsbBus};
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
