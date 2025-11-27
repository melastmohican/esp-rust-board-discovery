//! # BME280 Temperature/Humidity/Pressure Sensor Example for Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//!
//! Reads temperature, humidity, and atmospheric pressure from a BME280 sensor over I2C.
//!
//! This example is configured for the Adafruit BME280 breakout board connected via Qwiic/STEMMA QT connector.
//!
//! ## Hardware
//!
//! - **Sensor:** Adafruit BME280 Temperature Humidity Pressure Sensor
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x77 (default for Adafruit BME280)
//!
//! ## Wiring with Qwiic/STEMMA QT
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the BME280 sensor.
//! The cable provides:
//! ```
//!      BME280 -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO8  (I2C Clock)
//! (blue)   SDA -> GPIO10 (I2C Data)
//! ```
//!
//! ## I2C Address
//!
//! The BME280 can have two I2C addresses:
//! - 0x76 (SDO pin to GND) - use `BME280::new_primary()`
//! - 0x77 (SDO pin to VCC) - use `BME280::new_secondary()` **← Adafruit default**
//!
//! The Adafruit BME280 uses address 0x77 by default.
//!
//! Run with `cargo run --example bme280_i2c`.

#![no_std]
#![no_main]

use bme280::i2c::BME280;
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

    info!("Initializing BME280 sensor...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral with default configuration (100kHz)
    // BME280 supports up to 400kHz (Fast mode)
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    // Create delay provider
    let mut delay = Delay::new();

    // Initialize BME280 sensor
    // Adafruit BME280 uses 0x77 address (new_secondary)
    // Use new_primary() for 0x76 address if needed
    let mut bme280 = BME280::new_secondary(i2c);

    // Initialize the sensor
    match bme280.init(&mut delay) {
        Ok(_) => info!("BME280 initialized successfully!"),
        Err(_) => {
            info!("Failed to initialize BME280!");
            info!("Check wiring and I2C address");
            info!("Adafruit BME280 uses 0x77 (new_secondary), others may use 0x76 (new_primary)");
            loop {
                delay.delay_millis(1000);
            }
        }
    }

    info!("Starting measurements...");

    loop {
        // Take a measurement
        match bme280.measure(&mut delay) {
            Ok(measurements) => {
                info!(
                    "Temperature: {}.{:02} °C | Humidity: {}.{:02} % | Pressure: {}.{:02} hPa",
                    measurements.temperature as i32,
                    ((measurements.temperature.abs() % 1.0) * 100.0) as u32,
                    measurements.humidity as u32,
                    ((measurements.humidity % 1.0) * 100.0) as u32,
                    (measurements.pressure / 100.0) as u32,
                    ((measurements.pressure / 100.0 % 1.0) * 100.0) as u32,
                );
            }
            Err(_) => {
                info!("Error reading BME280 sensor!");
            }
        }

        // Wait 1 second between measurements
        delay.delay_millis(1000);
    }
}
