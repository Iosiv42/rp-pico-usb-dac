#![no_std]
#![no_main]

use panic_halt as _;

use rtic_monotonics::rp2040::prelude::*;

rp2040_timer_monotonic!(Mono);

mod setup_clocks;
mod setup_i2s;
mod setup_usb;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [RTC_IRQ, DMA_IRQ_1])]
mod app {
    use crate::{setup_clocks::setup_clocks, setup_i2s::setup_i2s, setup_usb::setup_usb};

    use super::*;
    use core::slice;

    use cortex_m::singleton;
    use embedded_hal::digital::StatefulOutputPin;
    use rp_pico::{
        hal::{self, dma::{double_buffer::{self, Transfer, ReadNext}, Channel, DMAExt, CH0, CH1}, gpio::{bank0::Gpio25, FunctionSio, Pin, PullDown, SioOutput}, usb::UsbBus, watchdog::Watchdog, Sio}, pac::PIO0
    };
    use hal::pio::{Tx, SM0};
    use rtic_sync::{channel::{Receiver, Sender}, make_channel};
    use usb_device::{bus::UsbBusAllocator, device::UsbDevice};
    use usbd_audio::AudioClass;

    const PACKET_SIZE: usize = 48;
    const BUFFER_SIZE: usize = 8;
    type Packet = [u32; PACKET_SIZE];

    #[shared]
    struct Shared {
        led: Pin<Gpio25, FunctionSio<SioOutput>, PullDown>,
    }

    #[local]
    struct Local {
        tx_transfer: Option<Transfer<
            Channel<CH0>,
            Channel<CH1>,
            &'static mut Packet,
            Tx<(PIO0, SM0)>,
            ReadNext<&'static mut Packet>
        >>,
        usb_dev: UsbDevice<'static, UsbBus>,
        usbd_audio: AudioClass<'static, UsbBus>,
        usb_audio_buf: [u8; 1024],
        sender: Sender<'static, Packet, BUFFER_SIZE>,
        receiver: Receiver<'static, Packet, BUFFER_SIZE>,
    }

    #[init(local = [usb_bus: Option<UsbBusAllocator<UsbBus>> = None])]
    fn init(mut c: init::Context) -> (Shared, Local) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }

        Mono::start(c.device.TIMER, &mut c.device.RESETS);

        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let clocks = setup_clocks(
            &mut resets,
            &mut watchdog,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
        );

        let sio = Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let led = pins.led.into_push_pull_output_in_state(hal::gpio::PinState::Low);

        let (usbd_audio, usb_dev) = setup_usb(
            c.local.usb_bus,
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            &mut resets
        );

        let (_i2s_sm, i2s_tx) = setup_i2s(
            c.device.PIO0,
            &mut resets,
            pins.gpio16,
            pins.gpio17,
            pins.gpio18
        );
        
        let dma = c.device.DMA.split(&mut resets);
        
        // Buffers for DMA tx from mem to SM0.
        let tx_buf0 = singleton!(BUF0: Packet = [0u32; PACKET_SIZE]).unwrap();
        let tx_buf1 = singleton!(BUF1: Packet = [0u32; PACKET_SIZE]).unwrap();

        let tx_transfer = {
            let mut conf = double_buffer::Config::new((dma.ch0, dma.ch1), tx_buf0, i2s_tx);
            conf.start()
        };
        let tx_transfer = tx_transfer.read_next(tx_buf1);

        dma_handler::spawn().ok();

        // Channel for USB and DMA communication.
        let (s, r) = make_channel!(Packet, BUFFER_SIZE);

        (
            Shared { led },
            Local {
                tx_transfer: Some(tx_transfer),
                usb_dev,
                usbd_audio,
                usb_audio_buf: [0u8; 1024],
                sender: s,
                receiver: r,
            },
        )
    }

    #[task(
        binds = USBCTRL_IRQ,
        priority = 4,
        local = [usb_dev, usbd_audio, sender, usb_audio_buf],
        shared = [led],
    )]
    fn usb_handler(mut c: usb_handler::Context) {
        if !c.local.usb_dev.poll(&mut [c.local.usbd_audio]) {
            return;
        }

        let buf = c.local.usb_audio_buf;
        let usbd_audio = c.local.usbd_audio;
        let sender = c.local.sender;

        if let Ok(len) = usbd_audio.read(buf) {
            unsafe { slice::from_raw_parts(
                buf.as_ptr().cast::<Packet>(),
                len / PACKET_SIZE,    // TODO is it good to use division?
            ) }
                .into_iter()
                .for_each( |&packet| {sender.try_send(packet);});

            c.shared.led.lock(|led| led.toggle().ok());
        }
    }

    #[task(priority = 3, local = [tx_transfer, receiver], shared = [led])]
    async fn dma_handler(c: dma_handler::Context) {
        let mut tx_transfer = c.local.tx_transfer.take().unwrap();
        let r = c.local.receiver;

        loop {
            let (tx_buf, next_tx_transfer) = async {
                tx_transfer.wait()
            }.await;

            if let Ok(packet) = r.recv().await {
                *tx_buf = packet;
            }

            tx_transfer = next_tx_transfer.read_next(tx_buf);
        }
    }
}
