//! # Arduino Modulino Pixels Example for Rust ESP Board
//!
//! This example uses the **modulino** library: https://github.com/melastmohican/modulino-rs
//!
//! Controls 8 RGB LEDs on the Arduino Modulino Pixels module over I2C using the `modulino` library.
//!
//! ## Hardware
//!
//! - **Module:** Arduino Modulino Pixels
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x36 (7-bit)
//!
//! ## Wiring with Qwiic/STEMMA QT on Rust ESP Board
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Pixels.
//! The cable provides:
//! ```
//!      Modulino Pixels -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO8  (I2C Clock)
//! (blue)   SDA -> GPIO10 (I2C Data)
//! ```
//!
//! Run with `cargo run --example modulino_pixels_i2c`.

#![no_std]
#![no_main]

use defmt::{Debug2Format, error, info};
use esp_hal::{
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
    main,
};
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

// Import from modulino library
use modulino::{Color, Pixels};

/// Number of LEDs on the Modulino Pixels
const NUM_LEDS: usize = 8;

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();

    info!("Initializing Arduino Modulino Pixels...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral with default configuration (100kHz)
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    // Create Modulino Pixels driver
    // Note: Pixels::new() automatically uses the default address (0x36)
    let mut pixels = match Pixels::new(i2c) {
        Ok(p) => p,
        Err(e) => {
            error!(
                "Failed to initialize Modulino Pixels: {:?}",
                Debug2Format(&e)
            );
            loop {
                delay.delay_millis(1000);
            }
        }
    };

    info!(
        "Modulino Pixels initialized at address 0x{:02X}!",
        pixels.address()
    );
    info!("Starting LED animations...");

    // Test connection by turning on first LED
    if let Err(e) = pixels.set_color_show(0, Color::RED, 50) {
        error!("Failed to set pixel: {:?}", Debug2Format(&e));
    }

    delay.delay_millis(1000);

    // Animation 1: Rainbow colors
    info!("Animation 1: Rainbow colors");
    let rainbow_colors = [
        Color::RED,
        Color::new(255, 127, 0), // Orange
        Color::YELLOW,
        Color::GREEN,
        Color::CYAN,
        Color::BLUE,
        Color::new(75, 0, 130), // Indigo
        Color::MAGENTA,
    ];

    for _ in 0..3 {
        for (i, color) in rainbow_colors.iter().enumerate() {
            // We only have 8 LEDs, but array might have more or fewer colors
            if i < NUM_LEDS {
                pixels.set_color(i, *color, 50).ok();
            }
        }
        pixels.show().ok();
        delay.delay_millis(500);

        pixels.clear_all();
        pixels.show().ok();
        delay.delay_millis(200);
    }

    // Animation 2: Knight Rider / Larson Scanner effect
    info!("Animation 2: Knight Rider effect");
    for _ in 0..3 {
        // Forward
        for i in 0..NUM_LEDS {
            pixels.clear_all();

            // Main bright LED
            pixels.set_color(i, Color::RED, 100).ok();

            // Trailing glow effect
            if i > 0 {
                pixels.set_color(i - 1, Color::RED, 12).ok();
            }
            if i > 1 {
                pixels.set_color(i - 2, Color::RED, 6).ok();
            }

            pixels.show().ok();
            delay.delay_millis(100);
        }

        // Backward
        for i in (0..NUM_LEDS).rev() {
            pixels.clear_all();

            // Main bright LED
            pixels.set_color(i, Color::RED, 100).ok();

            // Trailing glow effect
            if i < NUM_LEDS - 1 {
                pixels.set_color(i + 1, Color::RED, 12).ok();
            }
            if i < NUM_LEDS - 2 {
                pixels.set_color(i + 2, Color::RED, 6).ok();
            }

            pixels.show().ok();
            delay.delay_millis(100);
        }
    }

    // Animation 3: Color fade
    info!("Animation 3: Color fade cycle");
    loop {
        // Fade through different colors
        let colors = [
            Color::RED,
            Color::GREEN,
            Color::BLUE,
            Color::YELLOW,
            Color::CYAN,
            Color::MAGENTA,
        ];

        for color in colors.iter() {
            // Fade in
            for brightness in (0..=100).step_by(5) {
                // Use set_all_color helper
                pixels.set_all_color(*color, brightness as u8);
                pixels.show().ok();
                delay.delay_millis(20);
            }

            delay.delay_millis(300);

            // Fade out
            for brightness in (0..=100).rev().step_by(5) {
                pixels.set_all_color(*color, brightness as u8);
                pixels.show().ok();
                delay.delay_millis(20);
            }
        }
    }
}
