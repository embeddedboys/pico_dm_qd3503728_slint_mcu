#[cfg(not(feature = "simulator"))]
use crate::ILI9488;
#[cfg(not(feature = "simulator"))]
use core::result::Result;
#[cfg(not(feature = "simulator"))]
use defmt::info;
#[cfg(not(feature = "simulator"))]
use display_interface::{DisplayError, WriteOnlyDataCommand};
#[cfg(not(feature = "simulator"))]
use embedded_graphics_core::{
    draw_target::DrawTarget,
    geometry::{Dimensions, OriginDimensions, Size},
    pixelcolor::Rgb565,
    primitives::Rectangle,
    Pixel,
};
#[cfg(not(feature = "simulator"))]
use embedded_hal::digital::OutputPin;

#[cfg(not(feature = "simulator"))]
impl<DI, RST, BL> DrawTarget for ILI9488<DI, RST, BL>
where
    DI: WriteOnlyDataCommand,
    RST: OutputPin,
    BL: OutputPin,
{
    type Color = Rgb565;
    type Error = DisplayError;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for pixel in pixels {
            let x = pixel.0.x as u16;
            let y = pixel.0.y as u16;

            self.set_addr_win(x, y, x, y)?;
            self.write_pixels([pixel.1])?;
        }
        Ok(())
    }

    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        let intersection = area.intersection(&self.bounding_box());
        let Some(bottom_right) = intersection.bottom_right() else {
            // No intersection -> nothing to draw
            return Ok(());
        };
        let xs = area.top_left.x as u16;
        let ys = area.top_left.y as u16;
        let xe = bottom_right.x as u16;
        let ye = bottom_right.y as u16;

        let colors = colors.into_iter();
        if &intersection == area {
            // Draw the original iterator if no edge overlaps the framebuffer
            self.set_addr_win(xs, ys, xe, ye)?;
            self.write_pixels(colors)?;
        } else {
            info!("overlap not supports yet!");
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        let Some(bottom_right) = area.bottom_right() else {
            // No intersection -> nothing to draw
            return Ok(());
        };
        let xs = area.top_left.x as u16;
        let ys = area.top_left.y as u16;
        let xe = bottom_right.x as u16;
        let ye = bottom_right.y as u16;

        let count = area.size.width * area.size.height;
        let colors = core::iter::repeat(color).take(count.try_into().unwrap());

        self.set_addr_win(xs, ys, xe, ye)?;
        self.write_pixels(colors)?;
        Ok(())
    }
}

#[cfg(not(feature = "simulator"))]
impl<DI, RST, BL> OriginDimensions for ILI9488<DI, RST, BL>
where
    DI: WriteOnlyDataCommand,
    RST: OutputPin,
    BL: OutputPin,
{
    fn size(&self) -> Size {
        Size::new(self.size_x as u32, self.size_y as u32)
    }
}
