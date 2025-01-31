#![no_std]
#![no_main]

extern crate alloc;

pub mod cst816s;

use alloc::{boxed::Box, rc::Rc};
use core::{cell::RefCell, default::Default, mem::MaybeUninit};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_graphics_core::{
    draw_target::DrawTarget,
    pixelcolor::raw::RawU16,
    prelude::{Point, Size},
    primitives::Rectangle,
};
use embedded_hal_bus::spi::RefCellDevice;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{self, Input, Io, Output, Pull},
    i2c::I2c,
    interrupt,
    peripherals::{self, TWAI0},
    prelude::*,
    spi::{self, master::Spi},
    timer::{systimer::SystemTimer, timg::TimerGroup},
    twai::{self, TwaiMode, TwaiRx},
};
use esp_println::println;
use gc9a01::{mode::BasicMode, prelude::*, Gc9a01, SPIDisplayInterface};
use slint::platform::{software_renderer::MinimalSoftwareWindow, PointerEventButton};

slint::include_modules!();

const WINDOW_WIDTH: i32 = 240;
const WINDOW_HEIGHT: i32 = 240;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // ------------------------ MCU SET UP ------------------------
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP.as_mut_ptr() as *mut u8,
            HEAP_SIZE,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
    log::info!("Logger is setup");
    println!("Hello world!");

    let peripherals = esp_hal::init(esp_hal::Config::default());
    // let system = SystemControl::new(peripherals.SYSTEM);

    // let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // ------------------------ DISPLAY SET UP ------------------------
    Output::new(io.pins.gpio18, gpio::Level::High);

    let sck = io.pins.gpio48;
    let mosi = io.pins.gpio38;
    let miso = io.pins.gpio47;

    let cs_output = Output::new(io.pins.gpio21, gpio::Level::Low);
    let dc_output = Output::new(io.pins.gpio10, gpio::Level::Low);
    let mut led_reset = Output::new(io.pins.gpio17, gpio::Level::Low);

    let spi = Spi::new(peripherals.SPI2, 40u32.MHz(), spi::SpiMode::Mode0).with_pins(
        sck,
        mosi,
        miso,
        gpio::NoPin,
    );

    let spi_bus = RefCell::new(spi);
    let spi_device = RefCellDevice::new_no_delay(&spi_bus, cs_output).unwrap();
    let interface = SPIDisplayInterface::new(spi_device, dc_output);

    let mut display = Gc9a01::new(
        interface,
        DisplayResolution240x240,
        DisplayRotation::Rotate0,
    );

    display.clear_fit().unwrap();
    display.reset(&mut led_reset, &mut delay).unwrap();
    display.init(&mut delay).unwrap();

    // ------------------------ TOUCH PAD SET UP ------------------------

    // Create a new peripheral object with the described wiring and standard
    // I2C clock speed:
    let i2c = I2c::new(
        peripherals.I2C0,
        io.pins.gpio11, // SDA
        io.pins.gpio12, // SCL
        100u32.kHz(),
    );
    let touch_int = Input::new(io.pins.gpio6, Pull::Up);
    let touch_rst = Output::new(io.pins.gpio7, gpio::Level::Low);

    let mut touchpad = cst816s::CST816S::new(i2c, touch_int, touch_rst);
    touchpad.setup(&mut delay).unwrap();

    // ------------------------ SLINT SET UP ------------------------

    let window = MinimalSoftwareWindow::new(Default::default());
    window.set_size(slint::PhysicalSize::new(
        WINDOW_WIDTH as _,
        WINDOW_WIDTH as _,
    ));

    slint::platform::set_platform(Box::new(EspBackend {
        window: window.clone(),
    }))
    .unwrap();

    let mut draw_buffer = DrawBuffer {
        display,
        buffer: &mut [slint::platform::software_renderer::Rgb565Pixel(0); WINDOW_WIDTH as usize],
    };

    let app_window = AppWindow::new().unwrap();

    // ------------------------ CAN Bus (TWAI) ------------------------

    let can_tx_pin = io.pins.gpio43; // Needs it?
    let can_rx_pin = io.pins.gpio44;

    // The speed of the CAN bus.
    const CAN_BAUDRATE: twai::BaudRate = twai::BaudRate::B1000K;

    // Begin configuring the TWAI peripheral. The peripheral is in a reset like
    // state that prevents transmission but allows configuration.
    let mut can_config = twai::TwaiConfiguration::new_async_no_transceiver(
        peripherals.TWAI0,
        can_tx_pin,
        can_rx_pin,
        CAN_BAUDRATE,
        TwaiMode::Normal,
    );

    // Partially filter the incoming messages to reduce overhead of receiving
    // undesired messages. Note that due to how the hardware filters messages,
    // standard ids and extended ids may both match a filter. Frame ids should
    // be explicitly checked in the application instead of fully relying on
    // these partial acceptance filters to exactly match. A filter that matches
    // standard ids of an even value.
    const FILTER: twai::filter::SingleStandardFilter =
        twai::filter::SingleStandardFilter::new(b"xxxxxxxxxx0", b"x", [b"xxxxxxxx", b"xxxxxxxx"]);
    can_config.set_filter(FILTER);

    // Start the peripheral. This locks the configuration settings of the peripheral
    // and puts it into operation mode, allowing packets to be sent and
    // received.
    let can = can_config.start();

    // Get separate transmit and receive halves of the peripheral.
    let (rx, _tx) = can.split();

    interrupt::enable(
        peripherals::Interrupt::TWAI0,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    spawner.spawn(receiver(rx, app_window)).ok();

    // ------------------------ APP ------------------------
    loop {
        slint::platform::update_timers_and_animations();

        if let Some(evt) = touchpad.read_one_touch_event(true) {
            // TODO: Sync with rotation of the screen, subtraction is a hack
            // dunno if height of width should be used here
            let position = slint::LogicalPosition::new(
                (WINDOW_WIDTH - evt.x) as _,
                (WINDOW_HEIGHT - evt.y) as _,
            );

            if evt.action == 0 {
                window.dispatch_event(slint::platform::WindowEvent::PointerPressed {
                    position,
                    button: PointerEventButton::Left,
                });
            } else if evt.action == 2 {
                window.dispatch_event(slint::platform::WindowEvent::PointerMoved { position });
            } else if evt.action == 1 {
                window.dispatch_event(slint::platform::WindowEvent::PointerReleased {
                    position,
                    button: PointerEventButton::Left,
                });
            }
        }

        window.draw_if_needed(|renderer| {
            renderer.render_by_line(&mut draw_buffer);
        });

        if window.has_active_animations() {
            continue;
        } else {
            // Needs to have await otherwise won't spawn tasks??
            Timer::after(Duration::from_millis(1)).await;
        }
    }
}

#[embassy_executor::task]
async fn receiver(mut _rx: TwaiRx<'static, TWAI0, esp_hal::Async>, app_window: AppWindow) -> ! {
    println!("CAN Listening..");
    let mut afr: f32 = 0.500;

    loop {
        esp_println::println!("Bing!");

        if afr > 2.0 {
            afr = 0.5;
        }
        afr += 0.01;

        // Needs to have await otherwise won't spawn the task??
        Timer::after(Duration::from_millis(250)).await;

        app_window.set_o2_lambda_reading(afr);

        // let frame = rx.receive_async().await;
        // match frame {
        //     Ok(frame) => {
        //         println!("Received a frame: {:?}", frame);
        //     }
        //     Err(e) => {
        //         println!("Receive error: {:?}", e);
        //     }
        // }
    }
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
            SystemTimer::now() / (SystemTimer::ticks_per_second() / 1000),
        )
    }
}

struct DrawBuffer<'a, Display> {
    display: Display,
    buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

impl<I: WriteOnlyDataCommand, D: DisplayDefinition>
    slint::platform::software_renderer::LineBufferProvider
    for &mut DrawBuffer<'_, Gc9a01<I, D, BasicMode>>
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
            .map_err(drop)
            .unwrap();
    }
}
