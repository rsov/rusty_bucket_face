#![no_std]
#![no_main]

extern crate alloc;

use core::mem::MaybeUninit;
use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, Primitive, PrimitiveStyleBuilder, Rectangle},
};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    spi::{self, master::Spi},
    Delay,
};
use esp_hal::{timer::TimerGroup, Rng};
use esp_println::println;
use esp_wifi::{initialize, EspWifiInitFor};
use gc9a01::{mode::BufferedGraphics, prelude::*, Gc9a01, SPIDisplayInterface};
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
    tick: u32,
) {
    let (w, h) = display.dimensions();
    let w = w as u32;
    let h = h as u32;
    let x = tick % w;
    let y = tick % h;

    let style = PrimitiveStyleBuilder::new()
        .stroke_width(4)
        .stroke_color(Rgb565::new(tick as u8, x as u8, y as u8))
        .fill_color(Rgb565::RED)
        .build();

    let cdiameter = 20;

    // circle
    Circle::new(
        Point::new(119 - cdiameter / 2 + 40, 119 - cdiameter / 2 + 40),
        cdiameter as u32,
    )
    .into_styled(style)
    .draw(display)
    .unwrap();

    // circle
    Circle::new(
        Point::new(119 - cdiameter / 2 - 40, 119 - cdiameter / 2 + 40),
        cdiameter as u32,
    )
    .into_styled(style)
    .draw(display)
    .unwrap();

    // rectangle
    let rw = 80;
    let rh = 20;
    Rectangle::new(
        Point::new(119 - rw / 2, 119 - rh / 2 - 40),
        Size::new(rw as u32, rh as u32),
    )
    .into_styled(style)
    .draw(display)
    .unwrap();
}

#[entry]
fn main() -> ! {
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
    let mut led_b = io.pins.gpio0.into_push_pull_output();
    let mut led_r = io.pins.gpio45.into_push_pull_output();
    let mut led_g = io.pins.gpio46.into_push_pull_output();

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
        DisplayRotation::Rotate0,
    );

    let mut display = driver.into_buffered_graphics();
    display.init(&mut delay).ok();
    let mut tick: u32 = 0;

    loop {
        println!("Loop... {tick}");

        display.clear();
        draw(&mut display, tick);
        display.flush().ok();
        tick += 1;

        led_r.set_low().unwrap();
        led_g.set_high().unwrap();
        led_b.set_high().unwrap();
        delay.delay_ms(1000u32);

        led_r.set_high().unwrap();
        led_g.set_low().unwrap();
        led_b.set_high().unwrap();
        delay.delay_ms(1000u32);

        led_r.set_high().unwrap();
        led_g.set_high().unwrap();
        led_b.set_low().unwrap();
        delay.delay_ms(1000u32);
    }
}
