#![no_std]
#![no_main]

extern crate alloc;

use alloc::format;
use core::{borrow::Borrow, mem::MaybeUninit};
use embassy_executor::Spawner;
use embedded_can::nb::Can;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::IO,
    i2c::I2C,
    peripherals::{Peripherals, TWAI0},
    prelude::*,
    spi::{self, master::Spi},
    timer::TimerGroup,
    twai, Delay, Rng,
};

use esp_println::println;
use esp_wifi::{initialize, EspWifiInitFor};
use gc9a01::{mode::BufferedGraphics, prelude::*, Gc9a01, SPIDisplayInterface};
use nb::block;
use u8g2_fonts::{
    fonts,
    types::{FontColor, HorizontalAlignment, VerticalPosition},
    FontRenderer,
};

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

/// Test Function : will be removed later
fn draw<I: WriteOnlyDataCommand, D: DisplayDefinition>(
    display: &mut Gc9a01<I, D, BufferedGraphics<D>>,
    lambda: u32,
) {
    let font = FontRenderer::new::<fonts::u8g2_font_logisoso78_tn>();

    font.render_aligned(
        format!(
            "{number:.precision$}",
            precision = 3,
            number = lambda as f32 * 0.001
        )
        .borrow(),
        Point::new(4, 155), // eyeballed it
        VerticalPosition::Baseline,
        HorizontalAlignment::Left,
        FontColor::Transparent(Rgb565::WHITE),
        display,
    )
    .unwrap();
}

#[embassy_executor::task]
async fn receiver(can_config: esp_hal::twai::TwaiConfiguration<'static, TWAI0>) {
    let mut can = can_config.start();

    loop {
        // I think the frame I will be looking is 0X470 or something like that
        let frame = block!(can.receive()).unwrap();
        println!("Received a frame: {frame:?}");
    }
}

#[main]
async fn main(spawner: Spawner) {
    init_heap();
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
    log::info!("Logger is setup");
    println!("Hello world!");
    let timer = TimerGroup::new(peripherals.TIMG1, &clocks).timer0;
    let _init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    io.pins.gpio18.into_push_pull_output().set_high().unwrap();

    let sck = io.pins.gpio48;
    let mosi = io.pins.gpio38;

    let cs_output = io.pins.gpio21.into_push_pull_output();
    let dc_output = io.pins.gpio10.into_push_pull_output();

    let spi = Spi::new(peripherals.SPI2, 40u32.MHz(), spi::SpiMode::Mode0, &clocks)
        .with_sck(sck)
        .with_mosi(mosi);

    let interface = SPIDisplayInterface::new(spi, dc_output, cs_output);

    let driver = Gc9a01::new(
        interface,
        DisplayResolution240x240,
        DisplayRotation::Rotate180,
    );

    let mut display = driver.into_buffered_graphics();
    display.init(&mut delay).ok();
    let mut lambda: u32 = 850;

    // Set the tx pin as open drain. Skip this if using transceivers.
    let can_tx_pin = io.pins.gpio0.into_open_drain_output();
    let can_rx_pin = io.pins.gpio2;

    // The speed of the CAN bus.
    const CAN_BAUDRATE: twai::BaudRate = twai::BaudRate::B1000K;

    // Begin configuring the TWAI peripheral. The peripheral is in a reset like
    // state that prevents transmission but allows configuration.
    let can_config = twai::TwaiConfiguration::new(
        peripherals.TWAI0,
        can_tx_pin,
        can_rx_pin,
        &clocks,
        CAN_BAUDRATE,
    );

    // Partially filter the incoming messages to reduce overhead of receiving
    // undesired messages. Note that due to how the hardware filters messages,
    // standard ids and extended ids may both match a filter. Frame ids should
    // be explicitly checked in the application instead of fully relying on
    // these partial acceptance filters to exactly match.
    // A filter that matches StandardId::ZERO.

    // TODO: Uncomment this once we know the frames we're looking for
    // const FILTER: SingleStandardFilter =
    //     SingleStandardFilter::new(b"00000000000", b"x", [b"xxxxxxxx", b"xxxxxxxx"]);
    // can_config.set_filter(FILTER);

    // Start the peripheral. This locks the configuration settings of the peripheral
    // and puts it into operation mode, allowing packets to be sent and
    // received.
    spawner.spawn(receiver(can_config)).ok();

    // Create a new peripheral object with the described wiring and standard
    // I2C clock speed:
    let mut i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio11, // SDA
        io.pins.gpio12, // SCL
        100u32.kHz(),
        &clocks,
    );

    let _touch_int = io.pins.gpio6.into_pull_up_input().degrade();
    let mut touch_rst = io.pins.gpio7.into_push_pull_output().degrade();

    // I think this is needed to reset it?
    touch_rst.set_low().unwrap();
    delay.delay_us(100u32);
    touch_rst.set_high().unwrap();
    delay.delay_us(100u32);

    delay.delay_us(100u32);

    loop {
        display.clear();

        let mut data = [0u8; 6];
        i2c.write_read(0x15, &[0x01], &mut data).ok();
        println!("{:?}", data);

        let style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);

        Text::new(format!("{:?}", data).borrow(), Point::new(50, 65), style)
            .draw(&mut display)
            .unwrap();

        if lambda >= 1200 {
            lambda = 850;
        } else {
            lambda += 1;
        }

        draw(&mut display, lambda);
        display.flush().ok();

        delay.delay_ms(50u32);
    }
}
