#![no_std]
#![no_main]

use panic_halt as _;

use rtic_monotonics::rp2040::prelude::*;

rp2040_timer_monotonic!(Mono);

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [RTC_IRQ, DMA_IRQ_1])]
mod app {
    use super::*;
    use core::{mem::{transmute, transmute_copy}, slice, task};

    use cortex_m::{interrupt, singleton};
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use fugit::{MicrosDurationU32, MillisDuration, MillisDurationU32, RateExtU32};
    use rp_pico::{
        hal::{self, clocks::{init_clocks_and_plls, ClockSource}, dma::{double_buffer::{self, Transfer, ReadNext}, Channel, DMAExt, SingleChannel, CH0, CH1}, gpio::{bank0::Gpio25, FunctionSio, Pin, PullDown, SioOutput}, timer::Alarm, usb::UsbBus, watchdog::Watchdog, Clock, Sio}, pac::{Peripherals, PIO0, RESETS}, XOSC_CRYSTAL_FREQ
    };
    use hal::pio::{PIOBuilder, Running, StateMachine, Tx, ValidStateMachine, SM0, PIOExt};
    use pio::{Instruction, InstructionOperands, OutDestination};
    use pio_proc::pio_file;
    use rtic_sync::{channel::{Receiver, Sender}, make_channel};
    use usb_device::{bus::{self, UsbBusAllocator}, device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid}};
    use usbd_audio::{AudioClass, AudioClassBuilder, StreamConfig};
    use rtic::app;

    const PACKET_SIZE: usize = 48;
    const BUFFER_SIZE: usize = 8;
    const DA: usize = 2;
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
        buf: [u8; 4*PACKET_SIZE*DA],
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

        let mut clocks = {
            let xosc = hal::xosc::setup_xosc_blocking(
                c.device.XOSC,
                rp_pico::XOSC_CRYSTAL_FREQ.Hz()
            )
            .map_err(hal::clocks::InitError::XoscErr)
            .unwrap();

            watchdog.enable_tick_generation((rp_pico::XOSC_CRYSTAL_FREQ / 1_000_000) as u8);

            let mut clocks = hal::clocks::ClocksManager::new(c.device.CLOCKS);

            let pll_sys_132_mhz = hal::pll::PLLConfig {
                vco_freq: 1584.MHz(),
                refdiv: 1,
                post_div1: 6,
                post_div2: 2,
            };

            let pll_sys = hal::pll::setup_pll_blocking(
                c.device.PLL_SYS,
                xosc.operating_frequency().into(),
                pll_sys_132_mhz,
                &mut clocks,
                &mut resets
            )
            .map_err(hal::clocks::InitError::PllError)
            .unwrap();

            let pll_usb = hal::pll::setup_pll_blocking(
                c.device.PLL_USB,
                xosc.operating_frequency().into(),
                hal::pll::common_configs::PLL_USB_48MHZ,
                &mut clocks,
                &mut resets
            )
            .map_err(hal::clocks::InitError::PllError)
            .unwrap();

            clocks.reference_clock.configure_clock(&xosc, xosc.get_freq())
                .map_err(hal::clocks::InitError::ClockError)
                .unwrap();

            clocks.system_clock.configure_clock(&pll_sys, pll_sys.get_freq())
                .map_err(hal::clocks::InitError::ClockError)
                .unwrap();

            clocks.usb_clock.configure_clock(&pll_usb, pll_usb.get_freq())
                .map_err(hal::clocks::InitError::ClockError)
                .unwrap();

            clocks.adc_clock.configure_clock(&pll_usb, pll_usb.get_freq())
                .map_err(hal::clocks::InitError::ClockError)
                .unwrap();

            clocks.rtc_clock.configure_clock(&pll_usb, 46875u32.Hz())
                .map_err(hal::clocks::InitError::ClockError)
                .unwrap();

            clocks.peripheral_clock.configure_clock(
                &clocks.system_clock,
                clocks.system_clock.freq()
            )
            .map_err(hal::clocks::InitError::ClockError)
            .unwrap();

            clocks
        };

        let sio = Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );
        // let mut led = pins.led.reconfigure();
        // led.set_low().unwrap();
        let mut led = pins.led.into_push_pull_output_in_state(hal::gpio::PinState::Low);

        let usb_bus: &'static _ = c.local.usb_bus
            .insert(UsbBusAllocator::new(UsbBus::new(
                c.device.USBCTRL_REGS,
                c.device.USBCTRL_DPRAM,
                clocks.usb_clock,
                true,
                &mut resets,
            )));

        let mut usbd_audio = AudioClassBuilder::new()
            .output(StreamConfig::new_discrete(
                usbd_audio::Format::S16le,
                2,
                &[48000],
                usbd_audio::TerminalType::OutHeadphones
            ).unwrap())
            .build(usb_bus)
            .unwrap();
        
        let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, 0x0001))
            .strings(&[StringDescriptors::default()
                .manufacturer("None")
                .product("None")
                .serial_number("None")])
            .unwrap()
            .max_packet_size_0(64).unwrap()
            .device_class(0x01)
            .build();

        let (mut i2s_sm, mut i2s_tx) = {
            let (mut pio0, sm0, _, _, _) = c.device.PIO0.split(&mut resets);

            let program_with_defines = pio_proc::pio_asm!(
                ".side_set 2",
                ".wrap_target",
                "set y, 14 side 0b01",
                "left:",
                    "out pins, 1 side 0b00",
                    "jmp y--, left side 0b01",
                "out pins, 1 side 0b10",
                "set y, 14 side 0b11",
                "right:",
                    "out pins, 1 side 0b10",
                    "jmp y--, right side 0b11",
                "out pins, 1 side 0b00",
                ".wrap",
            );
            let program = program_with_defines.program;
            let installed = pio0.install(&program).unwrap();

            // Set gpio16 (BCK) to pio
            let _bck: hal::gpio::Pin<_, hal::gpio::FunctionPio0, hal::gpio::PullNone> =
                pins.gpio16.reconfigure();
            let bck_pin_id = 16;

            // Set gpio17 (LRCK) to pio
            let _lrck: hal::gpio::Pin<_, hal::gpio::FunctionPio0, hal::gpio::PullNone> =
                pins.gpio17.reconfigure();
            let lrck_pin_id = 17;

            // Set gpio18 (DIN) to pio
            let _din: hal::gpio::Pin<_, hal::gpio::FunctionPio0, hal::gpio::PullNone> =
                pins.gpio18.reconfigure();
            let din_pin_id = 18;

            // Build the pio program and set pin both for set and side set!
            // We are running with the default divider which is 1 (max speed)
            let (mut sm, _, mut tx) = PIOBuilder::from_installed_program(installed)
                .clock_divisor_fixed_point(42, 248)
                .set_pins(bck_pin_id, 2)
                .side_set_pin_base(bck_pin_id)
                .out_pins(din_pin_id, 1)
                .out_shift_direction(hal::pio::ShiftDirection::Left)
                .autopull(true)
                .pull_threshold(32)
                .build(sm0);

            // Set pio pindir for gpio25
            sm.set_pindirs([
                (bck_pin_id, hal::pio::PinDir::Output),
                (lrck_pin_id, hal::pio::PinDir::Output),
                (din_pin_id, hal::pio::PinDir::Output),
            ]);

            // Start state machine
            (sm.start(), tx)
        };
        
        let mut dma = c.device.DMA.split(&mut resets);
        
        let tx_buf0 = singleton!(BUF0: Packet = [0u32; PACKET_SIZE]).unwrap();
        let tx_buf1 = singleton!(BUF1: Packet = [0u32; PACKET_SIZE]).unwrap();
        
        // heartbeat::spawn().ok();

        let (mut s, r) = make_channel!(Packet, BUFFER_SIZE);
        let mut buf = [0u8; PACKET_SIZE*4*DA];
        let tx_transfer = double_buffer::Config::new((dma.ch0, dma.ch1), tx_buf0, i2s_tx).start();
        let tx_transfer = tx_transfer.read_next(tx_buf1);
        dma_handler::spawn().ok();

        (
            Shared { led },
            Local {
                tx_transfer: Some(tx_transfer),
                usb_dev,
                usbd_audio,
                buf,
                // led,
                sender: s,
                receiver: r,
            },
        )
    }

    #[task(
        binds = USBCTRL_IRQ,
        priority = 4,
        local = [usb_dev, usbd_audio, sender, buf],
        shared = [led],
    )]
    fn usb_handler(mut c: usb_handler::Context) {
        if c.local.usb_dev.poll(&mut [c.local.usbd_audio]) {
            if let Ok(len) = c.local.usbd_audio.read(c.local.buf) {
                let slice = unsafe { slice::from_raw_parts(
                    c.local.buf.as_ptr().cast::<Packet>(),
                    len.div_ceil(PACKET_SIZE)
                ) };
                for packet in slice.into_iter() {
                    // let state = c.local.sender.try_send(*packet).is_err();
                    let _ = c.local.sender.try_send(*packet);
                    // c.shared.led.lock(|led| led.set_state(hal::gpio::PinState::from(state)).ok());
                }
            }
        }
    }

    #[task(priority = 3, local = [tx_transfer, receiver], shared = [led])]
    async fn dma_handler(mut c: dma_handler::Context) {
        let mut tx_transfer = c.local.tx_transfer.take().unwrap();
        let r = c.local.receiver;

        loop {
            let (tx_buf, next_tx_transfer) = tx_transfer.wait();

            if let Ok(packet) = r.recv().await {
                *tx_buf = packet;
            }

            // c.shared.led.lock(|led| 
            //     led.set_state(hal::gpio::PinState::from(next_tx_transfer.is_done()))
            // );

            // // if next_tx_transfer.is_done() {
            // //     c.shared.led.lock(|led| led.set_high().ok());
            // // } else {
            // //     c.shared.led.lock(|led| led.set_low().ok());
            // // }

            tx_transfer = next_tx_transfer.read_next(tx_buf);
        }
    }
}
