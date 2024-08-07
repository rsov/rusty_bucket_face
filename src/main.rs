#![no_std]
#![no_main]

extern crate alloc;

use alloc::{boxed::Box, rc::Rc};
use core::{cell::RefCell, mem::MaybeUninit};
use embassy_executor::Spawner;
use embedded_graphics::{
    pixelcolor::raw::RawU16,
    prelude::{Point, Size},
    primitives::Rectangle,
};
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_hal_bus::spi::RefCellDevice;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{self, Io, Output},
    peripherals::Peripherals,
    prelude::*,
    spi::{self, master::Spi},
    system::SystemControl,
    timer::systimer::SystemTimer,
};
use gc9a01::{mode::BufferedGraphics, prelude::*, Gc9a01, SPIDisplayInterface};
use slint::platform::software_renderer::MinimalSoftwareWindow;

slint::include_modules!();

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

#[main]
async fn main(_spawner: Spawner) {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }

    let window = MinimalSoftwareWindow::new(Default::default());
    window.set_size(slint::PhysicalSize::new(240, 240));

    slint::platform::set_platform(Box::new(EspBackend { window })).unwrap();

    AppWindow::new().unwrap().run().unwrap();
}

struct EspBackend {
    window: Rc<MinimalSoftwareWindow>,
}

impl slint::platform::Platform for EspBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis(
            SystemTimer::now() / (SystemTimer::TICKS_PER_SECOND / 1000),
        )
    }

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);

        let clocks = ClockControl::max(system.clock_control).freeze();
        let mut delay = Delay::new(&clocks);

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        Output::new(io.pins.gpio18, gpio::Level::High);

        let sck = io.pins.gpio48;
        let mosi = io.pins.gpio38;
        let miso = io.pins.gpio47;

        let cs_output = Output::new(io.pins.gpio21, gpio::Level::Low);
        let dc_output = Output::new(io.pins.gpio10, gpio::Level::Low);
        let mut led_reset = Output::new(io.pins.gpio17, gpio::Level::Low);

        let spi = Spi::new(peripherals.SPI2, 40u32.MHz(), spi::SpiMode::Mode0, &clocks).with_pins(
            Some(sck),
            Some(mosi),
            Some(miso),
            gpio::NO_PIN,
        );

        let spi_bus = RefCell::new(spi);
        let spi_device = RefCellDevice::new_no_delay(&spi_bus, cs_output).unwrap();
        let interface = SPIDisplayInterface::new(spi_device, dc_output);

        let driver = Gc9a01::new(
            interface,
            DisplayResolution240x240,
            DisplayRotation::Rotate0,
        );

        let mut display = driver.into_buffered_graphics();
        display.reset(&mut led_reset, &mut delay).unwrap();
        display.init(&mut delay).unwrap();

        let mut draw_buffer = DrawBuffer {
            display,
            buffer: &mut [slint::platform::software_renderer::Rgb565Pixel(0); 240],
        };

        loop {
            slint::platform::update_timers_and_animations();

            self.window.draw_if_needed(|renderer| {
                renderer.render_by_line(&mut draw_buffer);
            });

            if self.window.has_active_animations() {
                continue;
            }
        }
    }
}

struct DrawBuffer<'a, Display> {
    display: Display,
    buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

impl<I: WriteOnlyDataCommand, D: DisplayDefinition>
    slint::platform::software_renderer::LineBufferProvider
    for &mut DrawBuffer<'_, Gc9a01<I, D, BufferedGraphics<D>>>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [slint::platform::software_renderer::Rgb565Pixel]),
    ) {
        let buffer = &mut self.buffer[range.clone()];

        render_fn(buffer);

        self.display
            .fill_contiguous(
                &Rectangle::new(
                    Point::new(range.start as _, line as _),
                    Size::new(range.len() as _, 1),
                ),
                buffer.iter().map(|p| RawU16::new(p.0).into()),
            )
            .unwrap();

        self.display.flush().unwrap();
    }
}
