#![no_std]
#![no_main]

use panic_halt as _;

use rtic_monotonics::rp2040::prelude::*;

rp2040_timer_monotonic!(Mono);

mod setup_clocks;
mod i2s;
mod setup_usb;
mod clock_configs;

extern crate alloc;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [RTC_IRQ, DMA_IRQ_1])]
mod app {
    use core::slice;

    use crate::{setup_clocks::setup_clocks, i2s::setup_i2s, setup_usb::setup_usb};

    use super::*;

    use alloc::{boxed::Box, collections::linked_list::LinkedList, vec::Vec};
    use cortex_m::singleton;
    use rp_pico::{
        hal::{self, clocks::ValidSrc, dma::{double_buffer::{self, ReadNext, Transfer}, Channel, DMAExt, SingleChannel, CH0, CH1}, gpio::{bank0::Gpio25, FunctionSio, Pin, PullDown, SioOutput}, pio::{Tx, SM0}, usb::UsbBus, watchdog::Watchdog, Clock, Sio}, pac::PIO0
    };
    use rtic_sync::{channel::{Receiver, Sender}, make_channel};
    use usb_device::{bus::UsbBusAllocator, device::UsbDevice};
    use usbd_audio::AudioClass;

    use embedded_alloc::LlffHeap as Heap;
    #[global_allocator]
    static HEAP: Heap = Heap::empty();

    const SAMPLE_RATE: usize = 96000;
    const I2S_BYTE_DEPTH: usize = 2;

    const USB_CHANNELS: usize = 2;    // TODO now only 2 works
    const USB_FRAMES_PER_PACKET: usize = SAMPLE_RATE / 1000;
    const USB_BYTE_DEPTH: usize = 2;
    /// In bytes.
    const USB_PACKET_SIZE: usize = USB_FRAMES_PER_PACKET * USB_CHANNELS * USB_BYTE_DEPTH;

    /// In words (u32)
    const DMA_PACKET_SIZE: usize = USB_FRAMES_PER_PACKET * 2 * I2S_BYTE_DEPTH / 4;

    const BUFFER_SIZE: usize = 4;

    #[shared]
    struct Shared {
        led: Pin<Gpio25, FunctionSio<SioOutput>, PullDown>,
    }

    #[local]
    struct Local {
        tx_transfer: Option<Transfer<
            Channel<CH0>,
            Channel<CH1>,
            &'static mut [u32],
            Tx<(PIO0, SM0)>,
            ReadNext<&'static mut [u32]>
        >>,
        // i2s_tx: Tx<(PIO0, SM0)>,
        usb_dev: UsbDevice<'static, UsbBus>,
        usbd_audio: AudioClass<'static, UsbBus>,
        sender: Sender<'static, Box<[u8]>, BUFFER_SIZE>,
        receiver: Receiver<'static, Box<[u8]>, BUFFER_SIZE>,
    }

    #[init(local = [usb_bus: Option<UsbBusAllocator<UsbBus>> = None])]
    fn init(mut c: init::Context) -> (Shared, Local) {
        {
            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 4096;
            static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
        }

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
            SAMPLE_RATE,
            (I2S_BYTE_DEPTH * 8) as u32,
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
            &mut resets,
            SAMPLE_RATE,
            USB_CHANNELS,
            (USB_BYTE_DEPTH * 8) as u32,
        );

        let (_i2s_sm, i2s_tx) = setup_i2s(
            c.device.PIO0,
            &mut resets,
            pins.gpio16,
            pins.gpio17,
            pins.gpio18,
            clocks.system_clock.freq().to_Hz() as usize,
            SAMPLE_RATE,
            (I2S_BYTE_DEPTH * 8) as u32,
        );
        
        let mut dma = c.device.DMA.split(&mut resets);
        dma.ch0.enable_irq0();
        dma.ch1.enable_irq0();
        
        // Buffers for DMA tx from mem to SM0.
        let tx_buf0 = singleton!(BUF0: [u32; 256] = [0u32; 256]).unwrap();
        let tx_buf1 = singleton!(BUF1: [u32; 256] = [0u32; 256]).unwrap();

        let tx_transfer = double_buffer::Config::new(
            (dma.ch0, dma.ch1),
            &mut tx_buf0[..DMA_PACKET_SIZE],
            i2s_tx,
        ).start();
        let tx_transfer = tx_transfer.read_next(&mut tx_buf1[..DMA_PACKET_SIZE]);
        // Channel for USB and DMA communication.
        let (s, r) = make_channel!(Box<[u8]>, BUFFER_SIZE);

        (
            Shared { led },
            Local {
                tx_transfer: Some(tx_transfer),
                usb_dev,
                usbd_audio,
                sender: s,
                receiver: r,
            },
        )
    }

    #[task(
        binds = USBCTRL_IRQ,
        priority = 2,
        local = [usb_dev, usbd_audio, sender],
        shared = [led],
    )]
    fn usb_handler(c: usb_handler::Context) {
        if !c.local.usb_dev.poll(&mut [c.local.usbd_audio]) {
            return;
        }

        let mut buf = Vec::with_capacity(USB_PACKET_SIZE);
        unsafe { buf.set_len(USB_PACKET_SIZE) };
        if let Ok(_) = c.local.usbd_audio.read(buf.as_mut_slice()) {
            let _ = c.local.sender.try_send(buf.into_boxed_slice());
        }
    }

    #[task(binds = DMA_IRQ_0, priority = 1, local = [tx_transfer, receiver])]
    fn dma_handler(c: dma_handler::Context) {
        let (tx_buf, next_tx_transfer) = c.local.tx_transfer.take().unwrap().wait();

        if let Ok(packet) = c.local.receiver.try_recv() {
            let da = unsafe { slice::from_raw_parts_mut(
                tx_buf.as_mut_ptr().cast::<u8>(),
                DMA_PACKET_SIZE * 4,
            ) };
            let trailing_bytes_count = I2S_BYTE_DEPTH - USB_BYTE_DEPTH;
            let mut i = trailing_bytes_count;
            for &byte in packet.iter() {
                da[i] = byte;
                i += 1;
                if i % I2S_BYTE_DEPTH == 0 {
                    i += trailing_bytes_count;
                }
            }
        }

        c.local.tx_transfer.replace(next_tx_transfer.read_next(&mut tx_buf[..DMA_PACKET_SIZE]));
    }
}
