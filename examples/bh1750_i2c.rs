//! # BH1750 Light Sensor Example for Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//!
//! Reads ambient light levels in lux from a BH1750 sensor over I2C.
//!
//! This example is configured for the Adafruit BH1750 breakout board connected via Qwiic/STEMMA QT connector.
//!
//! ## Hardware
//!
//! - **Sensor:** Adafruit BH1750 Light Sensor
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x23 (default, ADDR pin to GND) or 0x5C (ADDR pin to VCC)
//!
//! ## Wiring with Qwiic/STEMMA QT
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the BH1750 sensor.
//! The cable provides:
//! ```
//!      BH1750 -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO8  (I2C Clock)
//! (blue)   SDA -> GPIO10 (I2C Data)
//! ```
//!
//! ## I2C Address
//!
//! The BH1750 can have two I2C addresses:
//! - 0x23 (ADDR pin to GND) - use `BH1750::new(i2c, delay, false)` **← default**
//! - 0x5C (ADDR pin to VCC) - use `BH1750::new(i2c, delay, true)`
//!
//! The Adafruit BH1750 uses address 0x23 by default.
//!
//! ## About the BH1750
//!
//! The BH1750 is a digital ambient light sensor that measures light intensity in lux.
//! It features:
//! - Wide range and high resolution (1-65535 lx)
//! - Spectral response close to human eye
//! - Three resolution modes: Low (4 lx), High (1 lx), High2 (0.5 lx)
//! - Low power consumption
//!
//! Run with `cargo run --example bh1750_i2c`.

#![no_std]
#![no_main]

use bh1750::{BH1750, Resolution};
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

    info!("Initializing BH1750 light sensor...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral with default configuration (100kHz)
    // BH1750 supports up to 400kHz (Fast mode)
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    // Create delay provider
    let delay = Delay::new();

    // Initialize BH1750 sensor
    // false = address 0x23 (ADDR pin to GND) - Adafruit default
    // true = address 0x5C (ADDR pin to VCC)
    let mut bh1750 = BH1750::new(i2c, delay, false);

    info!("BH1750 initialized successfully!");
    info!("Starting light measurements...");
    info!("Resolution modes will cycle: Low -> High -> High2");

    let mut resolution_mode = 0;

    loop {
        // Cycle through different resolution modes
        let (resolution, mode_name) = match resolution_mode {
            0 => (Resolution::Low, "Low (4 lx)"),
            1 => (Resolution::High, "High (1 lx)"),
            _ => (Resolution::High2, "High2 (0.5 lx)"),
        };

        // Get a one-time measurement
        match bh1750.get_one_time_measurement(resolution) {
            Ok(lux) => {
                info!(
                    "[{}] Light level: {}.{:02} lx",
                    mode_name,
                    lux as u32,
                    ((lux % 1.0) * 100.0) as u32
                );
            }
            Err(_) => {
                info!("Error reading BH1750 sensor!");
                info!("Check wiring and I2C address");
                info!("Default address is 0x23 (ADDR pin to GND)");
            }
        }

        // Cycle resolution mode every 3 measurements
        delay.delay_millis(1000);

        resolution_mode = (resolution_mode + 1) % 3;
    }
}
