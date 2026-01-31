//! # Arduino Modulino Knob Example for Rust ESP Board
//!
//! This example uses the **modulino** library: https://github.com/melastmohican/modulino-rs
//!
//! Reads rotary encoder value and button state from the Arduino Modulino Knob module.
//!
//! ## Hardware
//!
//! - **Module:** Arduino Modulino Knob (Rotary Encoder)
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x3A (default, or 0x3B)
//!
//! ## Wiring with Qwiic/STEMMA QT on Rust ESP Board
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Knob.
//! The cable provides:
//! ```
//!      Modulino Knob -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO8  (I2C Clock)
//! (blue)   SDA -> GPIO10 (I2C Data)
//! ```
//!
//! Run with `cargo run --example modulino_knob_i2c`.

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
use modulino::Knob;

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();

    info!("Initializing Arduino Modulino Knob...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral with default configuration (100kHz)
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    // Create Modulino Knob driver
    let mut knob = match Knob::new(i2c) {
        Ok(k) => k,
        Err(e) => {
            error!("Failed to initialize Modulino Knob: {:?}", Debug2Format(&e));
            loop {
                delay.delay_millis(1000);
            }
        }
    };

    info!(
        "Modulino Knob initialized at address 0x{:02X}!",
        knob.address()
    );

    // Set range to 0-100 (e.g. for volume)
    knob.set_range(0, 100);
    // Reset starting value to 50
    if let Err(e) = knob.set_value(50) {
        error!("Failed to set initial value: {:?}", Debug2Format(&e));
    }

    info!("Turn the knob! (Range 0-100)");

    let mut prev_value = knob.value();
    let mut prev_pressed = knob.pressed();

    loop {
        // Poll for updates
        match knob.update() {
            Ok(_changed) => {
                let current_value = knob.value();
                let current_pressed = knob.pressed();

                if current_value != prev_value {
                    info!("Value: {}", current_value);
                    prev_value = current_value;
                }

                if current_pressed != prev_pressed {
                    if current_pressed {
                        info!("Button Pressed!");
                        // Optional: Reset on press
                        // knob.set_value(50).ok();
                    } else {
                        info!("Button Released");
                    }
                    prev_pressed = current_pressed;
                }
            }
            Err(e) => {
                error!("Failed to update knob: {:?}", Debug2Format(&e));
            }
        }

        // Poll interval
        delay.delay_millis(20);
    }
}
