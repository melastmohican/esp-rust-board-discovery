//! # ADXL345 Accelerometer Example for Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//!
//! Reads accelerometer data (X, Y, Z) from an ADXL345 sensor over I2C.
//!
//! ## Hardware
//!
//! - **Sensor:** ADXL345 Accelerometer
//! - **Connection:** I2C
//! - **I2C Address:** 0x53 (default) or 0x1D
//!
//! ## Wiring
//!
//! | Sensor Pin | ESP32-C3 | Notes |
//! |------------|----------|-------|
//! | VCC        | 3.3V     |       |
//! | GND        | GND      |       |
//! | SDA        | GPIO10   |       |
//! | SCL        | GPIO8    |       |
//! | CS         | VCC      | I2C mode (high) |
//! | SDO/ALT    | GND      | Address 0x53 (low) |
//!
//! ## Note
//! This example uses `adxl345_driver2` crate.

#![no_std]
#![no_main]

use adxl345_driver2::{
    Adxl345Reader,
    Adxl345Writer, // Traits for methods
    i2c::Device as Adxl345I2c,
};
use defmt::info;
use esp_hal::{
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
    main,
};
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("Initializing ADXL345 accelerometer...");

    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default());
    let i2c = i2c.expect("I2C init failed").with_sda(sda).with_scl(scl);

    let delay = Delay::new();

    // Initialize ADXL345
    // Device::new takes (bus)
    let mut adxl = Adxl345I2c::new(i2c).expect("Failed to initialize ADXL345");

    // Configure sensor
    // Set data rate to 100 Hz (0x0A)
    if let Err(e) = adxl.set_bandwidth_rate(0x0A) {
        info!("Failed to set data rate: {:?}", defmt::Debug2Format(&e));
    }

    // Set measure bit in Power Control register to start measurement (0x08)
    if let Err(e) = adxl.set_power_control(0x08) {
        info!("Failed to set power control: {:?}", defmt::Debug2Format(&e));
    }

    info!("ADXL345 initialized and started!");

    loop {
        match adxl.acceleration() {
            Ok((x, y, z)) => {
                info!("X: {}, Y: {}, Z: {}", x, y, z);
            }
            Err(e) => {
                info!("Error reading ADXL345: {:?}", defmt::Debug2Format(&e));
            }
        }

        delay.delay_millis(100);
    }
}
