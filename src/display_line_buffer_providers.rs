// Stole from https://github.com/igiona/rs-watch/blob/main/src/ui_task/display_line_buffer_provider.rs

use embedded_graphics::{
    pixelcolor::{Rgb565, raw::RawU16},
    prelude::{DrawTarget, Point, Size},
    primitives::Rectangle,
};
use slint::platform::software_renderer::{LineBufferProvider, Rgb565Pixel};

pub(super) struct DrawBuffer<'a, T> {
    pub(super) display: &'a mut T,
    pub(super) line_buffer: &'a mut [Rgb565Pixel],
}

impl<T: DrawTarget<Color = Rgb565>> LineBufferProvider for DrawBuffer<'_, T> {
    type TargetPixel = Rgb565Pixel;
    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
    ) {
        // Render into the line
        render_fn(&mut self.line_buffer[range.clone()]);

        // Send the line to the screen using DrawTarget::fill_contiguous
        self.display
            .fill_contiguous(
                &Rectangle::new(
                    Point::new(range.start as _, line as _),
                    Size::new(range.len() as _, 1),
                ),
                self.line_buffer[range.clone()]
                    .iter()
                    .map(|p| RawU16::new(p.0).into()),
            )
            .map_err(drop)
            .unwrap();
    }
}
