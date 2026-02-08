#![no_std]
#![no_main]

use defmt::info;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig};
use esp_hal::main;
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("Button test");

    // Set GPIO7 as an output, and set its state high initially.
    let mut led = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());
    let button = Input::new(peripherals.GPIO9, InputConfig::default());

    // Check the button state and set the LED state accordingly.
    loop {
        if button.is_high() {
            led.set_high();
        } else {
            led.set_low();
        }
    }
}
