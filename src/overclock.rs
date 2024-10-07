#[cfg(not(feature = "simulator"))]
use fugit::HertzU32;
#[cfg(not(feature = "simulator"))]
use hal::pll::PLLConfig;
#[cfg(not(feature = "simulator"))]
use rp2040_hal as hal;

//                   REF     FBDIV VCO            POSTDIV
// PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHZ / 6 / 1 = 250MHz
#[cfg(not(feature = "simulator"))]
pub const PLL_SYS_250MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1500),
    refdiv: 1,
    post_div1: 6,
    post_div2: 1,
};
