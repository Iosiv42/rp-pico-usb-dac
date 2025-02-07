use rp_pico::{
    hal::{clocks::UsbClock, usb::UsbBus},
    pac::{RESETS, USBCTRL_DPRAM, USBCTRL_REGS}
};
use usb_device::{
    bus::UsbBusAllocator,
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid}
};
use usbd_audio::{AudioClass, AudioClassBuilder, StreamConfig};

/// Setup usbd-audio and usb-dev.
pub fn setup_usb(
    usb_bus: &'static mut Option<UsbBusAllocator<UsbBus>>,
    usbctrl_regs: USBCTRL_REGS,
    usbctrl_dpram: USBCTRL_DPRAM,
    usb_clock: UsbClock,
    resets: &mut RESETS,
) -> (AudioClass<'static, UsbBus>, UsbDevice<'static, UsbBus>) {
    let usb_bus: &'static _ = usb_bus
        .insert(UsbBusAllocator::new(UsbBus::new(
            usbctrl_regs,
            usbctrl_dpram,
            usb_clock,
            true,
            resets,
        )));

    let usbd_audio = AudioClassBuilder::new()
        .output(StreamConfig::new_discrete(
            usbd_audio::Format::S16le,
            2,
            &[48000],
            usbd_audio::TerminalType::OutHeadphones
        ).unwrap())
        .input(StreamConfig::new_discrete(
            usbd_audio::Format::S16le,
            2,
            &[48000],
            usbd_audio::TerminalType::InUndefined
        ).unwrap())
        .build(usb_bus)
        .unwrap();

    let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, 0x0001))
        .strings(&[StringDescriptors::default()
            .manufacturer("StinCross Labs")
            .product("RP2040 USB DAC")
            .serial_number("57")])
        .unwrap()
        .max_packet_size_0(64).unwrap()
        .device_class(0x01)
        .build();

    (usbd_audio, usb_dev)
}
