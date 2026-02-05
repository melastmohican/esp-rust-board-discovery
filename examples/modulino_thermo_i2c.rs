//! # Arduino Modulino Thermo Example for Rust ESP Board
//!
//! This example uses the **modulino** library: https://crates.io/crates/modulino
//!
//! Reads temperature and humidity from the Arduino Modulino Thermo module (HS3003) over I2C.
//!
//! ## Hardware
//!
//! - **Module:** Arduino Modulino Thermo (HS3003)
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x44 (fixed)
//!
//! ## Wiring with Qwiic/STEMMA QT on Rust ESP Board
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Thermo.
//! The cable provides:
//! ```
//!      Modulino Thermo -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO8  (I2C Clock)
//! (blue)   SDA -> GPIO10 (I2C Data)
//! ```
//!
//! Run with `cargo run --example modulino_thermo_i2c`.

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
use modulino::Thermo;

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut delay = Delay::new();

    info!("Initializing Arduino Modulino Thermo...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral with default configuration (100kHz)
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    // Create Modulino Thermo driver
    // The HS3003 has a fixed address of 0x44
    let mut thermo = Thermo::new(i2c);

    info!(
        "Modulino Thermo initialized at address 0x{:02X}!",
        thermo.address()
    );
    info!("Starting measurements...");

    loop {
        // Read temperature and humidity
        // The read method requires a delay provider to wait for the measurement to complete
        match thermo.read(&mut delay) {
            Ok(measurement) => {
                info!(
                    "Temperature: {} °C, Humidity: {} %",
                    measurement.temperature, measurement.humidity
                );
            }
            Err(e) => {
                error!("Failed to read sensor: {:?}", Debug2Format(&e));
            }
        }

        // Wait 1 second before next measurement
        delay.delay_millis(1000);
    }
}
