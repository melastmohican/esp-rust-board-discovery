#![no_std]
#![no_main]
use defmt::info;
use esp_hal::{
    delay::Delay,
    gpio::OutputConfig,
    gpio::{Level, Output},
    main,
};
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("Blinky example started!");

    // Set GPIO7 as an output, and set its state high initially.
    let mut led = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());

    led.set_high();

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let delay = Delay::new();

    loop {
        led.toggle();
        delay.delay_millis(500);
    }
}
