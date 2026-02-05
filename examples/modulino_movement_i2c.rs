//! # Arduino Modulino Movement Example for Rust ESP Board
//!
//! This example uses the **modulino** library: https://crates.io/crates/modulino
//!
//! Reads accelerometer and gyroscope data from the Arduino Modulino Movement module (LSM6DSOX) over I2C.
//!
//! ## Hardware
//!
//! - **Module:** Arduino Modulino Movement
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x6A or 0x6B (default)
//!
//! ## Wiring with Qwiic/STEMMA QT on Rust ESP Board
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Movement.
//! The cable provides:
//! ```
//!      Modulino Movement -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO8  (I2C Clock)
//! (blue)   SDA -> GPIO10 (I2C Data)
//! ```
//!
//! Run with `cargo run --example modulino_movement_i2c`.

#![no_std]
#![no_main]

use defmt::{Debug2Format, error, info};
use esp_hal::{
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
    main,
    time::Rate,
};
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

// Import from modulino library
use modulino::Movement;

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();

    info!("Initializing Arduino Modulino Movement...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral with default configuration (400kHz)
    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(sda)
    .with_scl(scl);

    // Create Modulino Movement driver
    let mut movement = match Movement::new(i2c) {
        Ok(m) => m,
        Err(e) => {
            error!(
                "Failed to initialize Modulino Movement: {:?}",
                Debug2Format(&e)
            );
            loop {
                delay.delay_millis(1000);
            }
        }
    };

    info!(
        "Modulino Movement initialized at address 0x{:02X}!",
        movement.address()
    );
    info!("Starting measurements...");

    loop {
        // Read accelerometer and gyroscope values
        match movement.acceleration() {
            Ok(values) => {
                info!(
                    "Accel: x={} g, y={} g, z={} g",
                    values.x, values.y, values.z
                );
            }
            Err(e) => {
                error!("Failed to read acceleration: {:?}", Debug2Format(&e));
            }
        }

        match movement.gyro() {
            Ok(values) => {
                info!(
                    "Gyro:  x={} dps, y={} dps, z={} dps",
                    values.x, values.y, values.z
                );
            }
            Err(e) => {
                error!("Failed to read gyro: {:?}", Debug2Format(&e));
            }
        }

        // Wait 500ms before next measurement
        delay.delay_millis(500);
    }
}
