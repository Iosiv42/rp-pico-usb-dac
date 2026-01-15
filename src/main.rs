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
    use core::fmt::Write;

    use crate::{setup_clocks::setup_clocks, i2s::setup_i2s, setup_usb::setup_usb};

    use super::*;

    use alloc::{boxed::Box, collections::linked_list::LinkedList, vec::Vec};
    use cortex_m::singleton;
    use fugit::RateExtU32;
    use rp_pico::{
        hal::{self, Clock, Sio, clocks::ValidSrc, dma::{CH0, CH1, Channel, DMAExt, SingleChannel, double_buffer::{self, ReadNext, Transfer}, single_buffer}, gpio::{FunctionSio, FunctionUart, Pin, PullDown, SioOutput, bank0::{Gpio4, Gpio5, Gpio25}}, pio::{SM0, Tx}, uart::{self, DataBits, Enabled, StopBits, UartConfig, UartPeripheral}, usb::UsbBus, watchdog::Watchdog}, pac::{PIO0, SYST, UART1}
    };
    use rtic_sync::{channel::{Receiver, Sender}, make_channel};
    use static_assertions::const_assert;
    use usb_device::{bus::UsbBusAllocator, device::UsbDevice};
    use usbd_audio::AudioClass;

    use embedded_alloc::LlffHeap as Heap;
    #[global_allocator]
    static HEAP: Heap = Heap::empty();

    // =======================================================================
    // AUDIO CONFIG START
    // =======================================================================


    const SAMPLE_RATE: usize = 48000;	    // 48k or 96k
    const USB_CHANNELS: usize = 2;	    // TODO now only 2 works
    const USB_BYTE_DEPTH: usize = 2;	    // 2/3/4 byte (16/24/32 bit)



    // =======================================================================
    // AUDIO CONFIG END
    // =======================================================================

    const_assert!(SAMPLE_RATE == 48000 || SAMPLE_RATE == 96000);
    const_assert!(USB_CHANNELS == 2);
    const_assert!(2 <= USB_BYTE_DEPTH && USB_BYTE_DEPTH <= 4);

    // Since UAC 1.0 has 1ms packets.
    const USB_FRAMES_PER_PACKET: usize = SAMPLE_RATE / 1000;
    // In bytes.
    const USB_PACKET_SIZE: usize = USB_FRAMES_PER_PACKET * USB_CHANNELS * USB_BYTE_DEPTH;

    // In words (u32)
    const DMA_TX_BUF_SIZE: usize = USB_FRAMES_PER_PACKET * 2;

    const BUFFER_SIZE: usize = 4;

    #[shared]
    struct Shared {
        led: Pin<Gpio25, FunctionSio<SioOutput>, PullDown>,
        uart: UartPeripheral<Enabled, UART1, (Pin<Gpio4, FunctionUart, PullDown>, Pin<Gpio5, FunctionUart, PullDown>)>,
    }

    #[local]
    struct Local {
        tx_transfer: Option<single_buffer::Transfer<
            Channel<CH0>,
            &'static mut [u32],
            Tx<(PIO0, SM0)>
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
            (USB_BYTE_DEPTH * 8) as u32,
        );

        let sio = Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );


        // ===================================================================
        // PIN CONFIG START
        // ===================================================================


        let DIN_GPIO = pins.gpio18;	// aka SDIN, SD, SDATA, DACDAT
        let LRCK_GPIO = pins.gpio17;	// aka LCK, WS
        let BCLK_GPIO = pins.gpio16;	// aka BCK, SCK


        // ===================================================================
        // PIN CONFIG END
        // ===================================================================


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
            BCLK_GPIO,
            LRCK_GPIO,
            DIN_GPIO,
            clocks.system_clock.freq().to_Hz() as usize,
            SAMPLE_RATE,
            (USB_BYTE_DEPTH * 8) as u32,
        );
        
        let mut dma = c.device.DMA.split(&mut resets);
        dma.ch0.enable_irq0();
        
        // Buffers for DMA tx from mem to SM0.
        let tx_buf0 = singleton!(BUF0: [u32; DMA_TX_BUF_SIZE] = [0; DMA_TX_BUF_SIZE]).unwrap();

        let tx_transfer = single_buffer::Config::new(
            dma.ch0,
            tx_buf0.as_mut_slice(),
            i2s_tx,
        ).start();

        // Channel for USB and DMA communication.
        let (s, r) = make_channel!(Box<[u8]>, BUFFER_SIZE);

        let uart_pins = (
            // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
            pins.gpio4.into_function(),
            // UART RX (characters received by RP2040) on pin 2 (GPIO1)
            pins.gpio5.into_function(),
        );
        let uart = UartPeripheral::new(c.device.UART1, uart_pins, &mut resets)
            .enable(
                UartConfig::new(115200_u32.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        uart.write_full_blocking(b"UART debug mode\r\n");

        (
            Shared {
                led ,
                uart,
            },
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
        shared = [led, uart],
    )]
    fn usb_handler(mut c: usb_handler::Context) {
        if !c.local.usb_dev.poll(&mut [c.local.usbd_audio]) {
            return;
        }

        // Send samples to I2S channel
        let mut buf = Vec::with_capacity(USB_PACKET_SIZE);
        unsafe { buf.set_len(USB_PACKET_SIZE) };
        if let Ok(_) = c.local.usbd_audio.read(buf.as_mut_slice()) {
            let _ = c.local.sender.try_send(buf.into_boxed_slice());
        }
    }

    #[task(
        binds = DMA_IRQ_0,
        priority = 1,
        local = [tx_transfer, receiver],
        shared = [uart],
    )]
    fn dma_handler(mut c: dma_handler::Context) {
        let (ch, tx_buf, i2s_tx) = c.local.tx_transfer.take().unwrap().wait();

        // Try receive USB samples
        if let Ok(packet) = c.local.receiver.try_recv() {
            // Fit samples from USB to words (I2S PIO expects that)
            let mut chunks = packet.chunks_exact(USB_BYTE_DEPTH);
            let mut bytes = [0_u8; 4];
            for word in tx_buf.iter_mut() {
                let chunk = chunks.next().unwrap();
                bytes[(4 - USB_BYTE_DEPTH)..].copy_from_slice(chunk);
                *word = u32::from_le_bytes(bytes);
            }
        }

        c.local.tx_transfer.replace(single_buffer::Config::new(
            ch,
            tx_buf,
            i2s_tx,
        ).start());
    }
}
