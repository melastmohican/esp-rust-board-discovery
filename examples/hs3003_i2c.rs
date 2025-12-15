//! # HS3003 Temperature/Humidity Sensor Example for Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//!
//! Reads temperature and humidity from an HS3003 sensor over I2C.
//!
//! This example is configured for the **Arduino Modulino Thermo** connected via **Qwiic/STEMMA QT** cable.
//!
//! ## Hardware
//!
//! - **Sensor:** Arduino Modulino Thermo (Renesas HS3003)
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x44 (fixed for HS3003)
//!
//! ## Wiring with Qwiic/STEMMA QT
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Thermo.
//! The cable provides:
//! ```
//!      Modulino -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND  -> GND
//! (red)    VCC  -> 3.3V
//! (yellow) SCL  -> GPIO8  (I2C Clock)
//! (blue)   SDA  -> GPIO10 (I2C Data)
//! ```
//!
//! ## About HS3003
//!
//! The Renesas HS3003 is a high-performance temperature and humidity sensor:
//! - Temperature range: -40°C to +125°C (±0.2°C accuracy)
//! - Humidity range: 0% to 100% RH (±1.5% accuracy)
//! - 14-bit resolution for both measurements
//! - Ultra-low power consumption
//!
//! Run with `cargo run --example hs3003_i2c`.

#![no_std]
#![no_main]

use defmt::info;
use esp_hal::esp_riscv_rt::entry;
use esp_hal::{
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
};
use hs3003::Hs3003;
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[entry]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("Initializing HS3003 sensor...");

    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    let mut delay = Delay::new();

    let mut sensor = Hs3003::new(i2c);

    info!("HS3003 Sensor Example for Rust ESP Board (ESP32-C3-DevKit-RUST-1)");

    loop {
        match sensor.read(&mut delay) {
            Ok(measurement) => {
                info!(
                    "Temperature: {}°C, Humidity: {}%",
                    measurement.temperature, measurement.humidity
                );
            }
            Err(_) => {
                info!("Failed to read sensor");
            }
        }

        delay.delay_millis(2000);
    }
}
