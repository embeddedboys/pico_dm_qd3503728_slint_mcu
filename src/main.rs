#![no_std]
#![cfg_attr(not(feature = "simulator"), no_main)]

extern crate alloc;

slint::include_modules!();

fn create_slint_app() -> AppWindow {
    let ui = AppWindow::new().expect("Failed to load UI");

    let ui_handle = ui.as_weak();
    ui.on_request_increase_value(move || {
        let ui = ui_handle.unwrap();
        ui.set_slider_value(ui.get_slider_value() + 1.0);
    });
    ui
}

#[cfg(feature = "simulator")]
fn main() -> Result<(), slint::PlatformError> {
    create_slint_app().run()
}

#[cfg(not(feature = "simulator"))]
#[rp_pico::entry]
fn main() -> ! {
    // Pull in any important traits
    // use bsp::entry;
    use defmt::*;
    use defmt_rtt as _;
    use fugit::RateExtU32;
    // use cortex_m::singleton;
    use hal::{
        clocks::{ClocksManager, InitError},
        // dma::{double_buffer, single_buffer, DMAExt},
        gpio::{FunctionPio0, Pin},
        pac,
        pac::vreg_and_chip_reset::vreg::VSEL_A,
        pio::{Buffers, PIOExt, ShiftDirection},
        pll::{common_configs::PLL_USB_48MHZ, setup_pll_blocking},
        sio::Sio,
        vreg::set_voltage,
        // watchdog::Watchdog,
        xosc::setup_xosc_blocking,
        Clock,
        Timer,
        i2c::I2C,
    };
    // use panic_halt as _;
    use panic_probe as _;
    use rp2040_hal as hal;
    
    const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // Typically found in BSP crates
    use lib::{overclock, Pio16BitBus, ILI9488};
    use overclock::overclock_configs::PLL_SYS_240MHZ;
    use slint::platform::WindowEvent;

    // -------- Setup Allocator --------
    const HEAP_SIZE: usize = 200 * 1024;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    #[global_allocator]
    static ALLOCATOR: embedded_alloc::LlffHeap = embedded_alloc::LlffHeap::empty();
    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }

    // -------- Setup peripherials --------
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    // let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    set_voltage(&mut pac.VREG_AND_CHIP_RESET, VSEL_A::VOLTAGE1_10);

    let xosc = setup_xosc_blocking(pac.XOSC, XOSC_CRYSTAL_FREQ.Hz())
        .map_err(InitError::XoscErr)
        .ok()
        .unwrap();
    let mut clocks = ClocksManager::new(pac.CLOCKS);

    let pll_sys = setup_pll_blocking(
        pac.PLL_SYS,
        xosc.operating_frequency().into(),
        PLL_SYS_240MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .map_err(InitError::PllError)
    .unwrap();
    let pll_usb = setup_pll_blocking(
        pac.PLL_USB,
        xosc.operating_frequency().into(),
        PLL_USB_48MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .map_err(InitError::PllError)
    .unwrap();

    clocks
        .init_default(&xosc, &pll_sys, &pll_usb)
        .map_err(InitError::ClockError)
        .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let program = pio_proc::pio_asm!(
        ".side_set 1"
        ".wrap_target",
        "   out pins, 16    side 0",
        "   nop             side 1",
        ".wrap"
    );

    let wr: Pin<_, FunctionPio0, _> = pins.gpio19.into_function();
    let wr_pin_id = wr.id().num;

    let dc = pins.gpio20.into_push_pull_output();
    let rst = pins.gpio22.into_push_pull_output();
    let bl = pins.gpio28.into_push_pull_output();

    let lcd_d0: Pin<_, FunctionPio0, _> = pins.gpio0.into_function();
    let lcd_d1: Pin<_, FunctionPio0, _> = pins.gpio1.into_function();
    let lcd_d2: Pin<_, FunctionPio0, _> = pins.gpio2.into_function();
    let lcd_d3: Pin<_, FunctionPio0, _> = pins.gpio3.into_function();
    let lcd_d4: Pin<_, FunctionPio0, _> = pins.gpio4.into_function();
    let lcd_d5: Pin<_, FunctionPio0, _> = pins.gpio5.into_function();
    let lcd_d6: Pin<_, FunctionPio0, _> = pins.gpio6.into_function();
    let lcd_d7: Pin<_, FunctionPio0, _> = pins.gpio7.into_function();
    let lcd_d8: Pin<_, FunctionPio0, _> = pins.gpio8.into_function();
    let lcd_d9: Pin<_, FunctionPio0, _> = pins.gpio9.into_function();
    let lcd_d10: Pin<_, FunctionPio0, _> = pins.gpio10.into_function();
    let lcd_d11: Pin<_, FunctionPio0, _> = pins.gpio11.into_function();
    let lcd_d12: Pin<_, FunctionPio0, _> = pins.gpio12.into_function();
    let lcd_d13: Pin<_, FunctionPio0, _> = pins.gpio13.into_function();
    let lcd_d14: Pin<_, FunctionPio0, _> = pins.gpio14.into_function();
    let lcd_d15: Pin<_, FunctionPio0, _> = pins.gpio15.into_function();

    let lcd_d0_pin_id = lcd_d0.id().num;

    let pindirs = [
        (wr_pin_id, hal::pio::PinDir::Output),
        (lcd_d0.id().num, hal::pio::PinDir::Output),
        (lcd_d1.id().num, hal::pio::PinDir::Output),
        (lcd_d2.id().num, hal::pio::PinDir::Output),
        (lcd_d3.id().num, hal::pio::PinDir::Output),
        (lcd_d4.id().num, hal::pio::PinDir::Output),
        (lcd_d5.id().num, hal::pio::PinDir::Output),
        (lcd_d6.id().num, hal::pio::PinDir::Output),
        (lcd_d7.id().num, hal::pio::PinDir::Output),
        (lcd_d8.id().num, hal::pio::PinDir::Output),
        (lcd_d9.id().num, hal::pio::PinDir::Output),
        (lcd_d10.id().num, hal::pio::PinDir::Output),
        (lcd_d11.id().num, hal::pio::PinDir::Output),
        (lcd_d12.id().num, hal::pio::PinDir::Output),
        (lcd_d13.id().num, hal::pio::PinDir::Output),
        (lcd_d14.id().num, hal::pio::PinDir::Output),
        (lcd_d15.id().num, hal::pio::PinDir::Output),
    ];

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program.program).unwrap();
    let (int, frac) = (1, 0); // as slow as possible (0 is interpreted as 65536)
    let (mut sm, _, tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
        .side_set_pin_base(wr_pin_id)
        .out_pins(lcd_d0_pin_id, 16)
        .buffers(Buffers::OnlyTx)
        .clock_divisor_fixed_point(int, frac)
        .out_shift_direction(ShiftDirection::Right)
        .autopull(true)
        .pull_threshold(16)
        .build(sm0);
    sm.set_pindirs(pindirs);
    sm.start();

    info!("PIO block setuped");

    let di = Pio16BitBus::new(tx, dc);
    let mut display = ILI9488::new(di, Some(rst), Some(bl), 480, 320);
    display.init(&mut delay).unwrap();

    let mut i2c = I2C::i2c1(
        pac.I2C1,
        pins.gpio26.reconfigure(),
        pins.gpio27.reconfigure(),
        400.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    let irq_pin = pins.gpio21.into_pull_up_input();
    let rst_pin = pins.gpio18.into_push_pull_output();
    let mut touch = ft6236::FT6236::new(irq_pin, rst_pin, i2c).unwrap();
    let _ = touch.init(&mut delay);

    const HOR_RES: u32 = 480;
    const VER_RES: u32 = 320;

    // -------- Setup the Slint backend --------
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(Default::default());
    window.set_size(slint::PhysicalSize::new(HOR_RES, VER_RES));
    slint::platform::set_platform(alloc::boxed::Box::new(MyPlatform {
        window: window.clone(),
        timer,
    }))
    .unwrap();

    struct MyPlatform {
        window: alloc::rc::Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
        timer: Timer,
    }

    impl slint::platform::Platform for MyPlatform {
        fn create_window_adapter(&self) -> Result<alloc::rc::Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
            Ok(self.window.clone())
        }
        fn duration_since_start(&self) -> core::time::Duration {
            core::time::Duration::from_micros(self.timer.get_counter().ticks())
        }
    }

    // -------- Configure the UI --------
    // (need to be done after the call to slint::platform::set_platform)
    let _ui = create_slint_app();

    // -------- Event loop --------
    let mut line = [slint::platform::software_renderer::Rgb565Pixel(0); HOR_RES as usize];
    let mut last_touch = None;
    loop {
        slint::platform::update_timers_and_animations();
        window.draw_if_needed(|renderer| {
            use embedded_graphics_core::prelude::*;
            struct DisplayWrapper<'a, T>(
                &'a mut T,
                &'a mut [slint::platform::software_renderer::Rgb565Pixel],
            );
            impl<T: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>
                slint::platform::software_renderer::LineBufferProvider for DisplayWrapper<'_, T>
            {
                type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;
                fn process_line(
                    &mut self,
                    line: usize,
                    range: core::ops::Range<usize>,
                    render_fn: impl FnOnce(&mut [Self::TargetPixel]),
                ) {
                    let rect = embedded_graphics_core::primitives::Rectangle::new(
                        Point::new(range.start as _, line as _),
                        Size::new(range.len() as _, 1),
                    );
                    render_fn(&mut self.1[range.clone()]);
                    // NOTE! this is not an efficient way to send pixel to the screen, but it is kept simple on this template.
                    // It would be much faster to use the DMA to send pixel in parallel.
                    // See the example in https://github.com/slint-ui/slint/blob/master/examples/mcu-board-support/pico_st7789.rs 
                    self.0
                        .fill_contiguous(
                            &rect,
                            self.1[range.clone()].iter().map(|p| {
                                embedded_graphics_core::pixelcolor::raw::RawU16::new(p.0).into()
                            }),
                        )
                        .map_err(drop)
                        .unwrap();
                }
            }
            renderer.render_by_line(DisplayWrapper(&mut display, &mut line));
        });

        // handle touch event
        let button = slint::platform::PointerEventButton::Left;
        if let Some(event) = touch
            .read()
            .map_err(|_| ())
            .unwrap()
            .map(|point| {
                let position =
                    slint::PhysicalPosition::new((point.0) as _, (point.1) as _)
                        .to_logical(window.scale_factor());
                match last_touch.replace(position) {
                    Some(_) => WindowEvent::PointerMoved { position },
                    None => WindowEvent::PointerPressed { position, button },
                }
            })
            .or_else(|| {
                last_touch.take().map(|position| WindowEvent::PointerReleased { position, button })
            })
        {
            window.dispatch_event(event);
            // Don't go to sleep after a touch event that forces a redraw
            continue;
        }

        if window.has_active_animations() {
            continue;
        }

        // TODO: we could save battery here by going to sleep up to
        //   slint::platform::duration_until_next_timer_update()
        // or until the next touch interrupt, whatever comes first
        // cortex_m::asm::wfe();
    }
}

#[cfg(not(feature = "simulator"))]
mod ft6236 {
    use cortex_m::delay::Delay;
    use defmt::info;
    use embedded_hal::digital::{InputPin, OutputPin};
    use embedded_hal::i2c::I2c;

    const FT6236_DEF_ADDR: u8 =    0x38;
    const FT6236_REG_TD_STAT: u8 = 0x02;
    const FT6236_REG_TP1_XH: u8 =  0x03;
    const FT6236_REG_TP1_XL: u8 =  0x04;
    const FT6236_REG_TP1_YH: u8 =  0x05;
    const FT6236_REG_TP1_YL: u8 =  0x06;

    pub struct FT6236<IRQ: InputPin, RST: OutputPin, I2C: I2c> {
        irq: IRQ,
        rst: RST,
        i2c: I2C,
        addr: u8,
        pressed: bool,
    }

    impl<PinE, IRQ: InputPin<Error = PinE>, RST: OutputPin<Error = PinE>, I2C: I2c>
        FT6236<IRQ, RST, I2C>
    {
        pub fn new(irq: IRQ, rst: RST, i2c: I2C) -> Result<Self, PinE> {
            Ok(Self {
                irq,
                rst,
                i2c,
                addr: FT6236_DEF_ADDR,
                pressed: false,
            })
        }

        pub fn reset(&mut self, delay_source: &mut Delay) -> Result<(), PinE> {
            self.rst.set_high()?;
            delay_source.delay_ms(10);
            self.rst.set_low()?;
            delay_source.delay_ms(10);
            self.rst.set_high()?;
            delay_source.delay_ms(10);
            Ok(())
        }

        pub fn init(&mut self, delay_source: &mut Delay) -> Result<(), Error<PinE, I2C::Error>> {
            self.reset(delay_source).map_err(|e| Error::Pin(e))?;

            /* This ft6236 variant has read only registers,
             * so we shouldn't do nothing here. */
            Ok(())
        }

        pub fn read_reg(&mut self, reg: u8) -> Result<u8, I2C::Error> {
            let mut readbuf: [u8; 1] = [0];
            self.i2c.write_read(self.addr, &[reg], &mut readbuf)?;
            Ok(readbuf[0])
        }

        // pub fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), I2C::Error> {
        //     Ok(())
        // }

        pub fn is_pressed(&mut self) -> Result<bool, I2C::Error> {
            let val = self.read_reg(FT6236_REG_TD_STAT)?;
            Ok(val == 0x01)
        }

        pub fn read_x(&mut self) -> Result<u16, I2C::Error> {
            let val_h = self.read_reg(FT6236_REG_TP1_YH)? as u16;
            let val_l = self.read_reg(FT6236_REG_TP1_YL)? as u16;
            Ok(val_h << 8 | val_l)
        }

        pub fn read_y(&mut self) -> Result<u16, I2C::Error> {
            let val_h = (self.read_reg(FT6236_REG_TP1_XH)? & 0x1f) as u16;
            let val_l = self.read_reg(FT6236_REG_TP1_XL)? as u16;
            Ok(320 - (val_h << 8 | val_l))
        }

        pub fn read(&mut self) -> Result<Option<(u16, u16)>, Error<PinE, I2C::Error>> {
            match self.is_pressed() {
                Ok(pressed) => {
                    if pressed {
                        Ok(Some((
                            self.read_x().unwrap(),
                            self.read_y().unwrap(),
                        )))
                    } else {
                        Ok(None)
                    }
                }
                Err(e) => {
                    Ok(None)
                }
            }
        }
    }

    pub enum Error<PinE, TransferE> {
        Pin(PinE),
        I2C(TransferE),
    }
}
