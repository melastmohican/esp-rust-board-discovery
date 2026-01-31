//! # Arduino Modulino Distance Example for Rust ESP Board
//!
//! This example uses the **modulino** library: https://github.com/melastmohican/modulino-rs
//!
//! Reads distance from the Arduino Modulino Distance module (VL53L4CD) over I2C.
//!
//! ## Hardware
//!
//! - **Module:** Arduino Modulino Distance (VL53L4CD)
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x29 (default)
//!
//! ## Wiring with Qwiic/STEMMA QT on Rust ESP Board
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Distance.
//! The cable provides:
//! ```
//!      Modulino Distance -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO8  (I2C Clock)
//! (blue)   SDA -> GPIO10 (I2C Data)
//! ```
//!
//! Run with `cargo run --example modulino_distance_i2c`.

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
use modulino::Distance;

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut delay = Delay::new();

    info!("Initializing Arduino Modulino Distance...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral with default configuration (100kHz)
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    // Create Modulino Distance driver
    let mut distance = Distance::new(i2c);

    // Initialize the sensor (loads firmware, sets tuning)
    info!("Waiting for sensor boot and loading firmware...");
    // We pass `delay` for the initialization delays
    if let Err(e) = distance.init(&mut delay) {
        error!(
            "Failed to initialize Modulino Distance: {:?}",
            Debug2Format(&e)
        );
        loop {
            delay.delay_millis(1000);
        }
    }

    info!(
        "Modulino Distance initialized at address 0x{:02X}!",
        distance.address()
    );

    // Start continuous ranging
    if let Err(e) = distance.start_ranging() {
        error!("Failed to start ranging: {:?}", Debug2Format(&e));
    }
    info!("Ranging started...");

    loop {
        // Check if data is ready
        match distance.data_ready() {
            Ok(ready) => {
                if ready {
                    // Read distance
                    match distance.read_distance() {
                        Ok(Some(mm)) => {
                            info!("Distance: {} mm", mm);
                        }
                        Ok(None) => {
                            // If None is returned, check raw status if possible
                            if let Ok(status) = distance.read_range_status() {
                                info!("Invalid measurement (Status: {})", status);
                            } else {
                                info!("Invalid measurement");
                            }
                        }
                        Err(e) => {
                            error!("Failed to read distance: {:?}", Debug2Format(&e));
                        }
                    }
                }
            }
            Err(e) => {
                error!("Failed to check data ready: {:?}", Debug2Format(&e));
            }
        }

        // Poll interval - increase to 500ms to reduce output frequency
        delay.delay_millis(500);
    }
}
