//! # WS2812 RGB LED Example for Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//!
//! This example drives a WS2812 addressable RGB LED connected to GPIO2.
//! It cycles through rainbow colors using HSV color space with gamma correction.

#![no_std]
#![no_main]

use defmt::info;
use esp_hal::main;
use esp_hal::{delay::Delay, rmt::Rmt, time::Rate};
use esp_hal_smartled::{SmartLedsAdapter, smart_led_buffer};
use panic_rtt_target as _;
use smart_leds::{
    RGB8, SmartLedsWrite, brightness, gamma,
    hsv::{Hsv, hsv2rgb},
};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    // Initialize the HAL Peripherals
    let p = esp_hal::init(esp_hal::Config::default());

    info!("Initializing WS2812 LED on GPIO2...");

    // Configure RMT (Remote Control Transceiver) peripheral
    // The RMT peripheral is used to generate the precise timing required for WS2812 LEDs
    let rmt = Rmt::new(p.RMT, Rate::from_mhz(80)).expect("Failed to initialize RMT");

    // Use RMT channel 0 for the LED control
    let rmt_channel = rmt.channel0;

    // Create a buffer for one LED (adjust the number if you have more LEDs)
    let mut rmt_buffer = smart_led_buffer!(1);

    // Create SmartLED adapter using GPIO2
    let mut led = SmartLedsAdapter::new(rmt_channel, p.GPIO2, &mut rmt_buffer);

    let delay = Delay::new();

    // Initial HSV color (hue, saturation, value/brightness)
    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };

    // Brightness level (0-255, setting to 10 to avoid too bright output)
    let brightness_level = 10;

    info!("Starting rainbow animation...");

    loop {
        // Iterate through all hues to create rainbow effect
        for hue in 0..=255 {
            color.hue = hue;

            // Convert from HSV to RGB color space
            let data: RGB8 = hsv2rgb(color);

            // Apply gamma correction and brightness limiting
            led.write(brightness(gamma([data].into_iter()), brightness_level))
                .unwrap();

            delay.delay_millis(20);
        }
    }
}
