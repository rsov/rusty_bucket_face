#![no_std]
#![no_main]

extern crate alloc;

pub mod cst816s;

use alloc::{boxed::Box, rc::Rc, vec::Vec};
use core::{cell::RefCell, default::Default, mem::MaybeUninit};
use embedded_hal_bus::spi::RefCellDevice;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{self, Input, Io, Output, Pull},
    i2c::I2C,
    peripherals::Peripherals,
    prelude::*,
    spi::{self, master::Spi},
    system::SystemControl,
    timer::systimer::SystemTimer,
};
use esp_println::println;
use gc9a01::{mode::BufferedGraphics, prelude::*, Gc9a01, SPIDisplayInterface};
use slint::platform::{software_renderer::MinimalSoftwareWindow, PointerEventButton};

slint::include_modules!();

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

#[entry]
fn main() -> ! {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }

    let window = MinimalSoftwareWindow::new(Default::default());
    window.set_size(slint::PhysicalSize::new(240, 240));

    slint::platform::set_platform(Box::new(EspBackend { window })).unwrap();

    AppWindow::new().unwrap().run().unwrap();
    loop {}
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

        // setup logger
        // To change the log_level change the env section in .cargo/config.toml
        // or remove it and set ESP_LOGLEVEL manually before running cargo run
        // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
        esp_println::logger::init_logger_from_env();
        log::info!("Logger is setup");
        println!("Hello world!");

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

        display.clear_fit().unwrap();
        display.reset(&mut led_reset, &mut delay).unwrap();
        display.init(&mut delay).unwrap();
        display.flush().unwrap();

        let mut draw_buffer = DrawBuffer {
            display,
            buffer: &mut [slint::platform::software_renderer::Rgb565Pixel(0); 240],
        };

        // Create a new peripheral object with the described wiring and standard
        // I2C clock speed:
        let i2c = I2C::new(
            peripherals.I2C0,
            io.pins.gpio11, // SDA
            io.pins.gpio12, // SCL
            100u32.kHz(),
            &clocks,
            None,
        );
        let touch_int = Input::new(io.pins.gpio6, Pull::Up);
        let touch_rst = Output::new(io.pins.gpio7, gpio::Level::Low);

        let mut touchpad = cst816s::CST816S::new(i2c, touch_int, touch_rst);
        touchpad.setup(&mut delay).unwrap();

        loop {
            slint::platform::update_timers_and_animations();
            if let Some(evt) = touchpad.read_one_touch_event(true) {
                // TODO: Sync with rotation of the screen, subtraction is a hack
                let position = slint::LogicalPosition::new(evt.x as _, (240 - evt.y) as _);

                if evt.action == 0 {
                    self.window
                        .dispatch_event(slint::platform::WindowEvent::PointerPressed {
                            position,
                            button: PointerEventButton::Left,
                        });
                } else if evt.action == 2 {
                    self.window
                        .dispatch_event(slint::platform::WindowEvent::PointerMoved { position });
                } else if evt.action == 1 {
                    self.window
                        .dispatch_event(slint::platform::WindowEvent::PointerReleased {
                            position,
                            button: PointerEventButton::Left,
                        });
                }
            }

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
        _line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [slint::platform::software_renderer::Rgb565Pixel]),
    ) {
        let buffer = &mut self.buffer[range.clone()];

        render_fn(buffer);

        self.display
            .send_line(&buffer.iter().map(|&x| x.0.to_be()).collect::<Vec<u16>>())
            .unwrap();
    }
}
