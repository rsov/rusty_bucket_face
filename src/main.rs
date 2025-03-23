#![no_std]
#![no_main]

extern crate alloc;

mod cst816s;
mod display_line_buffer_providers;

use alloc::{boxed::Box, rc::Rc};
use core::default::Default;
use cst816s::CST816S;
use defmt::info;
use display_line_buffer_providers::DrawBuffer;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use embedded_can::Frame;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{self, Input, Output, Pull},
    i2c::master::I2c,
    spi::master::Spi,
    time::RateExtU32,
    timer::systimer::SystemTimer,
    twai::{self, TwaiMode, TwaiRx},
    Async, Blocking,
};
use gc9a01::{prelude::*, Gc9a01, SPIDisplayInterface};
use panic_rtt_target as _;
use slint::platform::{software_renderer::MinimalSoftwareWindow, PointerEventButton, WindowEvent};

slint::include_modules!();

const WINDOW_WIDTH: i32 = 240;
const WINDOW_HEIGHT: i32 = 240;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // ------------------------ MCU SET UP ------------------------
    rtt_target::rtt_init_defmt!();

    info!("✓ Logger");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::default());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    let mut delay = Delay::new();

    info!("✓ MCU");

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
            .with_mode(esp_hal::spi::Mode::_0)
            .with_frequency(20_u32.MHz()),
    )
    .unwrap()
    .with_miso(miso)
    .with_mosi(mosi)
    .with_sck(sck);

    info!("✓ SPI");

    let spi_dev = ExclusiveDevice::new_no_delay(spi, cs_output).unwrap();
    let interface = SPIDisplayInterface::new(spi_dev, dc_output);

    let mut display = Gc9a01::new(
        interface,
        DisplayResolution240x240,
        DisplayRotation::Rotate0,
    );

    display.clear_fit().unwrap();
    display.reset(&mut led_reset, &mut delay).unwrap();
    display.init(&mut delay).unwrap();

    info!("✓ Display");

    // ------------------------ TOUCH PAD SET UP ------------------------

    // Create a new peripheral object with the described wiring and standard
    // I2C clock speed:
    let i2c = I2c::new(peripherals.I2C0, {
        esp_hal::i2c::master::Config::default().with_frequency(100u32.kHz())
    })
    .unwrap()
    .with_sda(peripherals.GPIO11)
    .with_scl(peripherals.GPIO12);

    info!("✓ I2C");

    let touch_int = Input::new(peripherals.GPIO6, Pull::Up);
    let touch_rst = Output::new(peripherals.GPIO7, gpio::Level::Low);

    let mut touchpad = CST816S::new(i2c, touch_int, touch_rst);
    touchpad.setup(&mut delay).unwrap();

    info!("✓ Touchpad");

    // ------------------------ SLINT SET UP ------------------------

    let window = MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
    );

    window.set_size(slint::PhysicalSize::new(
        WINDOW_WIDTH as u32,
        WINDOW_WIDTH as u32,
    ));

    slint::platform::set_platform(Box::new(EspBackend {
        window: window.clone(),
    }))
    .unwrap();

    let mut line_buffer =
        [slint::platform::software_renderer::Rgb565Pixel(0); WINDOW_WIDTH as usize];

    // https://github.com/KortanZ/mcu-slint-demo/blob/main/src/ui.rs

    let app_window = AppWindow::new().unwrap();

    info!("✓ Slint");

    // ------------------------ CAN Bus (TWAI) ------------------------

    let can_tx_pin = peripherals.GPIO8; // Needs it?
    let can_rx_pin = peripherals.GPIO9;

    // Begin configuring the TWAI peripheral. The peripheral is in a reset like
    // state that prevents transmission but allows configuration.
    let mut can_config = twai::TwaiConfiguration::new(
        peripherals.TWAI0,
        can_rx_pin,
        can_tx_pin,
        twai::BaudRate::B1000K,
        TwaiMode::SelfTest,
    )
    .into_async();

    // TODO: Get this working?? only allows lambda
    const FILTER: twai::filter::SingleStandardFilter =
        twai::filter::SingleStandardFilter::new(b"10001110000", b"x", [b"xxxxxxxx", b"xxxxxxxx"]);
    can_config.set_filter(FILTER);

    // Start the peripheral. This locks the configuration settings of the peripheral
    // and puts it into operation mode, allowing packets to be sent and
    // received.
    let can = can_config.start();

    // Get separate transmit and receive halves of the peripheral.
    let (rx, _tx) = can.split();

    // ------------------------ APP ------------------------

    info!("✓ Main Loop");

    spawner.spawn(touch(touchpad, window.clone())).ok();
    spawner.spawn(receiver(rx, app_window)).ok();

    loop {
        slint::platform::update_timers_and_animations();

        window.draw_if_needed(|renderer| {
            renderer.render_by_line(DrawBuffer {
                display: &mut display,
                line_buffer: &mut line_buffer,
            });
        });

        if window.has_active_animations() {
            continue;
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn touch(
    mut touchpad: CST816S<I2c<'static, Blocking>, Input<'static>, Output<'static>>,
    window: Rc<MinimalSoftwareWindow>,
) -> ! {
    info!("✓ Touch task");

    loop {
        if let Some(evt) = touchpad.read_one_touch_event(true).await {
            info!("Touched {:?}", evt);
            let position = slint::LogicalPosition::new(
                (WINDOW_WIDTH - evt.x) as _,
                (WINDOW_HEIGHT - evt.y) as _,
            );

            if evt.action == 0 {
                window.dispatch_event(WindowEvent::PointerPressed {
                    position,
                    button: PointerEventButton::Left,
                });
            } else if evt.action == 2 {
                window.dispatch_event(WindowEvent::PointerMoved { position });
            } else if evt.action == 1 {
                window.dispatch_event(WindowEvent::PointerReleased {
                    position,
                    button: PointerEventButton::Left,
                })
            }
        }
    }
}

#[embassy_executor::task]
async fn receiver(mut rx: TwaiRx<'static, Async>, app_window: AppWindow) -> ! {
    info!("✓ CAN task");

    app_window.set_o2_lambda_reading(0.5);

    loop {
        let frame = rx.receive_async().await;

        match frame {
            Ok(frame) => {
                info!("[CAN] {:?}", frame);

                // TODO: This is a bit cumbersome, implement match or something
                let id: embedded_can::Id = frame.id();
                let lambda_id = twai::Id::from(twai::StandardId::new(0x470).unwrap());

                if lambda_id == id.into() {
                    // TODO: Skill issue, there's a better way to do this
                    let raw_value: u16 = ((frame.data()[0] as u16) << 8) | (frame.data()[1] as u16);
                    let lambda_1 = raw_value as f32 / 1000.0;
                    app_window.set_o2_lambda_reading(lambda_1);
                }
            }
            Err(e) => {
                info!("[CAN] Error: {:?}", e);
            }
        }
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
        Instant::now()
            .duration_since(Instant::from_micros(0))
            .into()
    }
}

// See https://github.com/igiona/rs-watch/blob/main/src/ui_task/ui_task.rs
