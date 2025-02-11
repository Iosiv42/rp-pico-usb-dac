use rp_pico::hal::pll::PLLConfig;
use fugit::HertzU32;

/// 48kHz/16bit
/// 96kHz/16bit
/// 48kHz/32bit
/// 96kHz/32bit (may be highly instable. try using overclocking for better results)
const PLL_SYS_132_MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1584),
    refdiv: 1,
    post_div1: 6,
    post_div2: 2,
};

pub fn from_settings(sample_rate: usize, bit_depth: u32) -> Option<PLLConfig> {
    match (sample_rate, bit_depth) {
        (48000, 16) => Some(PLL_SYS_132_MHZ),
        (96000, 16) => Some(PLL_SYS_132_MHZ),
        (48000, 32) => Some(PLL_SYS_132_MHZ),
        (96000, 32) => Some(PLL_SYS_132_MHZ),
        _ => None
    }
}
