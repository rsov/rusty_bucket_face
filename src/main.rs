#![no_std]
#![no_main]

extern crate alloc;
use core::mem::MaybeUninit;
use esp_backtrace as _;
use esp_hal::{clock::ClockControl, gpio::IO, peripherals::Peripherals, prelude::*, Delay};
use esp_println::println;

use esp_wifi::{initialize, EspWifiInitFor};

use esp_hal::{timer::TimerGroup, Rng};
#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
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
    let mut led_builtin = io.pins.gpio48.into_push_pull_output();
    let mut led_b = io.pins.gpio0.into_push_pull_output();
    let mut led_r = io.pins.gpio45.into_push_pull_output();
    let mut led_g = io.pins.gpio46.into_push_pull_output();

    loop {
        println!("Loop...");
        led_builtin.set_high().unwrap();
        led_r.set_low().unwrap();
        led_g.set_high().unwrap();
        led_b.set_high().unwrap();
        delay.delay_ms(1000u32);

        led_builtin.set_low().unwrap();
        led_r.set_high().unwrap();
        led_g.set_low().unwrap();
        led_b.set_high().unwrap();

        delay.delay_ms(1000u32);

        led_builtin.set_high().unwrap();
        led_r.set_high().unwrap();
        led_g.set_high().unwrap();
        led_b.set_low().unwrap();

        delay.delay_ms(1000u32);
        led_builtin.set_low().unwrap();
    }
}
