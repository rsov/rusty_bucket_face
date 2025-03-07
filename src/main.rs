#![no_std]
#![no_main]

extern crate alloc;

pub mod cst816s;

use alloc::{boxed::Box, rc::Rc};
use core::{cell::RefCell, default::Default};
use cst816s::CST816S;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};
use embedded_graphics_core::{
    draw_target::DrawTarget,
    pixelcolor::raw::RawU16,
    prelude::{Point, Size},
    primitives::Rectangle,
};
use embedded_hal_bus::spi::RefCellDevice;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{self, Input, Output, Pull},
    i2c::master::I2c,
    interrupt, peripherals,
    spi::{master::Spi, Mode},
    time::RateExtU32,
    twai::{self, TwaiMode, TwaiRx},
    Async,
};
use esp_println::println;
use gc9a01::{mode::BasicMode, prelude::*, Gc9a01, SPIDisplayInterface};
use log::info;
use slint::platform::{software_renderer::MinimalSoftwareWindow, PointerEventButton};

slint::include_modules!();

const WINDOW_WIDTH: i32 = 240;
const WINDOW_HEIGHT: i32 = 240;

static CHANNEL: Channel<CriticalSectionRawMutex, slint::platform::WindowEvent, 3> = Channel::new();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    info!("Logger is setup");
    // ------------------------ MCU SET UP ------------------------
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::default());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358

    println!("Hello world!");

    let mut delay = Delay::new();

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    // ------------------------ DISPLAY SET UP ------------------------
    Output::new(peripherals.GPIO18, gpio::Level::High);

    let sck = peripherals.GPIO48;
    let mosi = peripherals.GPIO38;
    let miso = peripherals.GPIO47;

    let cs_output = Output::new(peripherals.GPIO21, gpio::Level::Low);
    let dc_output = Output::new(peripherals.GPIO10, gpio::Level::Low);
    let mut led_reset = Output::new(peripherals.GPIO17, gpio::Level::Low);

    let spi = Spi::new(
        peripherals.SPI2,
        esp_hal::spi::master::Config::default()
            .with_frequency(100.kHz())
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_miso(miso)
    .with_mosi(mosi)
    .with_sck(sck);

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
    let i2c = I2c::new(peripherals.I2C0, {
        esp_hal::i2c::master::Config::default().with_frequency(100u32.kHz())
    })
    .unwrap()
    .with_sda(peripherals.GPIO11)
    .with_scl(peripherals.GPIO12)
    .into_async();

    let touch_int = Input::new(peripherals.GPIO6, Pull::Up);
    let touch_rst = Output::new(peripherals.GPIO7, gpio::Level::Low);

    let mut touchpad = cst816s::CST816S::new(i2c, touch_int, touch_rst);
    touchpad.setup(&mut delay).unwrap();

    // let mut touchpad = Cst816s::new(i2c, delay);
    // touchpad.reset(&mut touch_rst, &mut delay).unwrap();

    // ------------------------ SLINT SET UP ------------------------

    let window = MinimalSoftwareWindow::new(Default::default());
    window.set_size(slint::PhysicalSize::new(
        WINDOW_WIDTH as _,
        WINDOW_WIDTH as _,
    ));

    slint::platform::set_platform(Box::new(EspBackend {
        window: window.clone(),
        instant: Instant::now(),
    }))
    .unwrap();

    let mut draw_buffer = DrawBuffer {
        display,
        buffer: &mut [slint::platform::software_renderer::Rgb565Pixel(0); WINDOW_WIDTH as usize],
    };

    let app_window = AppWindow::new().unwrap();

    // ------------------------ CAN Bus (TWAI) ------------------------

    let can_tx_pin = peripherals.GPIO43; // Needs it?
    let can_rx_pin = peripherals.GPIO44;

    // The speed of the CAN bus.
    const CAN_BAUDRATE: twai::BaudRate = twai::BaudRate::B1000K;

    // Begin configuring the TWAI peripheral. The peripheral is in a reset like
    // state that prevents transmission but allows configuration.
    let mut can_config = twai::TwaiConfiguration::new_no_transceiver(
        peripherals.TWAI0,
        can_tx_pin,
        can_rx_pin,
        CAN_BAUDRATE,
        TwaiMode::Normal,
    )
    .into_async();

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
    spawner.spawn(touch(touchpad)).ok();
    println!("spawned??");

    // ------------------------ APP ------------------------

    loop {
        slint::platform::update_timers_and_animations();
        println!("main loop");

        println!("EMPTY? {:}", CHANNEL.is_empty());
        if !CHANNEL.is_empty() {
            let evt = CHANNEL.len();
            println!("YIPPY {:?}", evt);
        }

        window.draw_if_needed(|renderer| {
            renderer.render_by_line(&mut draw_buffer);
        });

        if window.has_active_animations() {
            continue;
        }
        Timer::after(Duration::from_millis(250)).await;
    }
}

#[embassy_executor::task]
async fn touch(mut touchpad: CST816S<I2c<'static, Async>, Input<'static>, Output<'static>>) -> ! {
    println!("touch loop");

    loop {
        if let Some(evt) = touchpad.read_one_touch_event(true) {
            // println!("GOT EVENT {:?}", evt);
            // TODO: Sync with rotation of the screen, subtraction is a hack
            // dunno if height of width should be used here
            let position = slint::LogicalPosition::new(
                (WINDOW_WIDTH - evt.x) as _,
                (WINDOW_HEIGHT - evt.y) as _,
            );
            println!("got touched");
            if evt.action == 0 {
                CHANNEL
                    .send(slint::platform::WindowEvent::PointerPressed {
                        position,
                        button: PointerEventButton::Left,
                    })
                    .await;
            } else if evt.action == 2 {
                CHANNEL
                    .send(slint::platform::WindowEvent::PointerMoved { position })
                    .await;
            } else if evt.action == 1 {
                CHANNEL
                    .send(slint::platform::WindowEvent::PointerReleased {
                        position,
                        button: PointerEventButton::Left,
                    })
                    .await;
            }
        }
    }
}

#[embassy_executor::task]
async fn receiver(mut _rx: TwaiRx<'static, Async>, app_window: AppWindow) -> ! {
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
    instant: Instant,
}

impl slint::platform::Platform for EspBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        self.instant.elapsed().into()
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
