use fugit::RateExtU32;
use rp_pico::{
    hal::{
        self,
        clocks::{ClockSource, ClocksManager},
        Clock, Watchdog
    },
    pac::{CLOCKS, PLL_SYS, PLL_USB, RESETS, XOSC}
};

/// Setup RP2040 clocks to 132MHz speed.
pub fn setup_clocks(
    resets: &mut RESETS,
    watchdog: &mut Watchdog,
    xosc_dev: XOSC,
    clocks_block: CLOCKS,
    dev_pll_sys: PLL_SYS,
    dev_pll_usb: PLL_USB,
) -> ClocksManager {
    let xosc = hal::xosc::setup_xosc_blocking(
        xosc_dev,
        rp_pico::XOSC_CRYSTAL_FREQ.Hz()
    )
    .map_err(hal::clocks::InitError::XoscErr)
    .unwrap();

    watchdog.enable_tick_generation((rp_pico::XOSC_CRYSTAL_FREQ / 1_000_000) as u8);

    let mut clocks = hal::clocks::ClocksManager::new(clocks_block);

    let pll_sys_132_mhz = hal::pll::PLLConfig {
        vco_freq: 1584.MHz(),
        refdiv: 1,
        post_div1: 6,
        post_div2: 2,
    };

    let pll_sys = hal::pll::setup_pll_blocking(
        dev_pll_sys,
        xosc.operating_frequency().into(),
        pll_sys_132_mhz,
        &mut clocks,
        resets
    )
    .map_err(hal::clocks::InitError::PllError)
    .unwrap();

    let pll_usb = hal::pll::setup_pll_blocking(
        dev_pll_usb,
        xosc.operating_frequency().into(),
        hal::pll::common_configs::PLL_USB_48MHZ,
        &mut clocks,
        resets
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
}
