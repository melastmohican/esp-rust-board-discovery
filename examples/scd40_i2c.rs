//! # SCD40 CO2/Temperature/Humidity Sensor Example for Rust ESP Board
//!
//! Reads CO2 concentration, temperature, and humidity from an SCD40 sensor over I2C.
//!
//! ## Hardware
//!
//! - **Sensor:** Apollo Automation SCD40 Breakout (or compatible Sensirion SCD4x)
//! - **Connection:** I2C (Qwiic/STEMMA QT recommended)
//! - **I2C Address:** 0x62 (Fixed for SCD4x)
//!
//! ## Wiring
//!
//! ```
//!      SCD40 -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO8  (I2C Clock)
//! (blue)   SDA -> GPIO10 (I2C Data)
//! ```
//!
//! Run with `cargo run --example scd40_i2c`.

#![no_std]
#![no_main]

use defmt::{Debug2Format, error, info, warn};
use esp_hal::{
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
    main,
};
use panic_rtt_target as _;

// The SCD4x driver
use scd4x::Scd4x;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("Initializing SCD40 sensor...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    let delay = Delay::new();

    // Create a SCD4x driver instance
    let mut scd40 = Scd4x::new(i2c, delay);

    info!("SCD40: Stopping periodic measurement (in case it was running)...");
    // Stop periodic measurement to ensure we can send commands
    if let Err(e) = scd40.stop_periodic_measurement() {
        warn!(
            "Failed to stop periodic measurement (might already be stopped): {:?}",
            Debug2Format(&e)
        );
    }

    // Wait a bit after stopping
    delay.delay_millis(500);

    info!("SCD40: Initializing...");

    // Start periodic measurement
    if let Err(e) = scd40.start_periodic_measurement() {
        error!(
            "Failed to start periodic measurement: {:?}",
            Debug2Format(&e)
        );
        panic!("Failed to start periodic measurement");
    }

    info!("SCD40 initialized successfully!");
    info!("Waiting for first measurement (approx 5 seconds)...");

    loop {
        // Check if data is ready
        match scd40.data_ready_status() {
            Ok(true) => match scd40.measurement() {
                Ok(m) => {
                    info!(
                        "CO2: {} ppm, Temperature: {} C, Humidity: {} %",
                        m.co2, m.temperature, m.humidity
                    );
                }
                Err(e) => {
                    error!("Error reading SCD40 measurement: {:?}", Debug2Format(&e));
                }
            },
            Ok(false) => {
                // Data not ready yet
            }
            Err(e) => {
                error!(
                    "Error checking SCD40 data ready status: {:?}",
                    Debug2Format(&e)
                );
            }
        }

        // The sensor updates every 5 seconds in normal mode
        delay.delay_millis(1000);
    }
}
