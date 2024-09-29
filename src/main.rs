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
    use defmt::*;
    use defmt_rtt as _;
    use panic_halt as _;
    use rp_pico as bsp;
    use bsp::hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio, pac,
        sio::Sio,
        watchdog::Watchdog,
        Timer,
    };
    // use slint::platform::WindowEvent;
    // use embedded_graphics::pixelcolor::Rgb565;
    use display_interface_parallel_gpio::{Generic16BitBus, PGPIO16BitInterface};
    use mipidsi::{models::ILI9486Rgb565, options::Orientation, options::Rotation};
    use mipidsi::options::ColorOrder;
    use mipidsi::Builder;

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
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = DelayCompat(cortex_m::delay::Delay::new(
        core.SYST,
        clocks.system_clock.freq().to_Hz(),
    ));

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let rst = pins.gpio22.into_push_pull_output_in_state(gpio::PinState::High);
    let wr = pins.gpio19.into_push_pull_output_in_state(gpio::PinState::High);
    let dc = pins.gpio20.into_push_pull_output();
    let _blk = pins.gpio28.into_push_pull_output_in_state(gpio::PinState::High);

    let lcd_d0 = pins.gpio0.into_push_pull_output();
    let lcd_d1 = pins.gpio1.into_push_pull_output();
    let lcd_d2 = pins.gpio2.into_push_pull_output();
    let lcd_d3 = pins.gpio3.into_push_pull_output();
    let lcd_d4 = pins.gpio4.into_push_pull_output();
    let lcd_d5 = pins.gpio5.into_push_pull_output();
    let lcd_d6 = pins.gpio6.into_push_pull_output();
    let lcd_d7 = pins.gpio7.into_push_pull_output();
    let lcd_d8 = pins.gpio8.into_push_pull_output();
    let lcd_d9 = pins.gpio9.into_push_pull_output();
    let lcd_d10 = pins.gpio10.into_push_pull_output();
    let lcd_d11 = pins.gpio11.into_push_pull_output();
    let lcd_d12 = pins.gpio12.into_push_pull_output();
    let lcd_d13 = pins.gpio13.into_push_pull_output();
    let lcd_d14 = pins.gpio14.into_push_pull_output();
    let lcd_d15 = pins.gpio15.into_push_pull_output();

    let bus = Generic16BitBus::new((
        lcd_d0,
        lcd_d1,
        lcd_d2,
        lcd_d3,
        lcd_d4,
        lcd_d5,
        lcd_d6,
        lcd_d7,
        lcd_d8,
        lcd_d9,
        lcd_d10,
        lcd_d11,
        lcd_d12,
        lcd_d13,
        lcd_d14,
        lcd_d15,
    ));

    let di = PGPIO16BitInterface::new(bus, dc, wr);
    let rotation = Orientation::new().rotate(Rotation::Deg270).flip_horizontal();
    let mut display = Builder::new(ILI9486Rgb565, di)
        .reset_pin(rst)
        .color_order(ColorOrder::Bgr)
        .orientation(rotation)
        .init(&mut delay)
        .unwrap();

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
    // let mut last_touch = None;
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
        let _button = slint::platform::PointerEventButton::Left;
        // if let Some(event) = touch
        //     .read()
        //     .map_err(|_| ())
        //     .unwrap()
        //     .map(|point| {
        //         let position =
        //             slint::PhysicalPosition::new((point.0 * 480.) as _, (point.1 * 320.) as _)
        //                 .to_logical(window.scale_factor());
        //         match last_touch.replace(position) {
        //             Some(_) => WindowEvent::PointerMoved { position },
        //             None => WindowEvent::PointerPressed { position, button },
        //         }
        //     })
        //     .or_else(|| {
        //         last_touch.take().map(|position| WindowEvent::PointerReleased { position, button })
        //     })
        // {
        //     window.dispatch_event(event);
        //     // Don't go to sleep after a touch event that forces a redraw
        //     continue;
        // }

        if window.has_active_animations() {
            continue;
        }

        // TODO: we could save battery here by going to sleep up to
        //   slint::platform::duration_until_next_timer_update()
        // or until the next touch interrupt, whatever comes first
        // cortex_m::asm::wfe();
    }
}
/// Wrapper around `Delay` to implement the embedded-hal 1.0 delay.
///
/// This can be removed when a new version of the `cortex_m` crate is released.
#[cfg(not(feature = "simulator"))]
struct DelayCompat(cortex_m::delay::Delay);

#[cfg(not(feature = "simulator"))]
impl embedded_hal::delay::DelayNs for DelayCompat {
    fn delay_ns(&mut self, mut ns: u32) {
        while ns > 1000 {
            self.0.delay_us(1);
            ns = ns.saturating_sub(1000);
        }
    }

    fn delay_us(&mut self, us: u32) {
        self.0.delay_us(us);
    }

    fn delay_ms(&mut self, ms: u32) {
        self.delay_us(ms * 1000);
    }
}
// TODO: implement ft6236 touch driver support
#[cfg(not(feature = "simulator"))]
mod ft6236 {

}
