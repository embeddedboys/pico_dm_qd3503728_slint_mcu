#[cfg(not(feature = "simulator"))]
pub mod overclock_configs {
    use fugit::HertzU32;
    use hal::pll::PLLConfig;
    use rp2040_hal as hal;

    //                   REF     FBDIV VCO            POSTDIV
    // PLL SYS: 12 / 1 = 12MHz * 120 = 1440 MHZ / 6 / 1 = 240MHz
    pub const PLL_SYS_240MHZ: PLLConfig = PLLConfig {
        vco_freq: HertzU32::MHz(1440),
        refdiv: 1,
        post_div1: 6,
        post_div2: 1,
    };

        //                   REF     FBDIV VCO            POSTDIV
    // PLL SYS: 12 / 1 = 12MHz * 133 = 1596 MHZ / 6 / 1 = 266MHz
    pub const PLL_SYS_266MHZ: PLLConfig = PLLConfig {
        vco_freq: HertzU32::MHz(1596),
        refdiv: 1,
        post_div1: 6,
        post_div2: 1,
    };

    /*  ./vcocalc.py 400
        Requested: 400.0 MHz
        Achieved:  400.0 MHz
        REFDIV:    1
        FBDIV:     100 (VCO = 1200.0 MHz)
        PD1:       3
        PD2:       1

        FIXME: Should set `PICO_FLASH_SPI_CLKDIV` to 4
     */
    pub const PLL_SYS_400MHZ: PLLConfig = PLLConfig {
        vco_freq: HertzU32::MHz(1200),
        refdiv: 1,
        post_div1: 3,
        post_div2: 1,
    };
}
