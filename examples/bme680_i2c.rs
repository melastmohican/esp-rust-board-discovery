//! # BME680/BME688 Temperature/Humidity/Pressure/Gas Sensor Example
//!
//! Reads temperature, humidity, atmospheric pressure, and gas resistance (VOCs)
//! from a BME680 or BME688 sensor over I2C.
//!
//! This example is configured for the Adafruit BME688 breakout board connected via Qwiic/STEMMA QT connector.
//!
//! ## Hardware
//!
//! - **Sensor:** Adafruit BME688 Temperature Humidity Pressure Gas Sensor
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x77 (default for Adafruit BME680/BME688)
//!
//! ## Wiring with Qwiic/STEMMA QT
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the sensor.
//! ```text
//!      BME688 -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO8  (I2C Clock)
//! (blue)   SDA -> GPIO10 (I2C Data)
//! ```
//!
//! Run with `cargo run --example bme680_i2c`.

#![no_std]
#![no_main]

use bosch_bme680::{Bme680, Configuration, DeviceAddress};
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

    info!("Initializing BME680/BME688 sensor...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral with default configuration (100kHz)
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    // Create delay provider for sensor initialization
    let mut sensor_delay = Delay::new();
    // Create delay provider for main loop
    let delay = Delay::new();

    // Initialize configuration (default values for oversampling and filter)
    let config = Configuration::default();

    // Initialize the sensor
    // Address: 0x77 (Secondary) for Adafruit BME688
    // ambient_temp: 25°C (used for initial calibration)
    let mut bme680 = match Bme680::new(
        i2c,
        DeviceAddress::Secondary,
        &mut sensor_delay,
        &config,
        25,
    ) {
        Ok(s) => {
            info!("BME680/BME688 initialized successfully!");
            s
        }
        Err(_e) => {
            info!("Failed to initialize BME680/BME688!");
            info!("Check wiring and I2C address (default 0x77)");
            loop {
                delay.delay_millis(1000);
            }
        }
    };

    info!("Starting measurements...");

    loop {
        // Take a measurement
        match bme680.measure() {
            Ok(values) => {
                info!("--- Measurements ---");
                info!(
                    "Temperature: {}.{:02} °C",
                    values.temperature as i32,
                    ((values.temperature % 1.0) * 100.0) as u32
                );
                info!(
                    "Pressure:    {}.{:02} hPa",
                    values.pressure as i32,
                    ((values.pressure % 1.0) * 100.0) as u32
                );
                info!(
                    "Humidity:    {}.{:02} %",
                    values.humidity as i32,
                    ((values.humidity % 1.0) * 100.0) as u32
                );

                if let Some(gas) = values.gas_resistance {
                    info!("Gas Res.:    {} Ohms", gas as u32);
                } else {
                    info!("Gas Res.:    (Not ready or heater disabled)");
                }
            }
            Err(_e) => {
                info!("Error reading sensor data!");
            }
        }

        // Wait 2 seconds between measurements (gas sensing takes time)
        delay.delay_millis(2000);
    }
}
