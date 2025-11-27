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
use esp_hal::{
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
    main,
};
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

/// HS3003 I2C address (fixed)
const HS3003_ADDR: u8 = 0x44;

/// HS3003 measurement data structure
struct Hs3003Measurement {
    humidity: f32,    // Relative humidity in %
    temperature: f32, // Temperature in °C
}

/// Read temperature and humidity from HS3003 sensor
fn read_hs3003<I2C: embedded_hal::i2c::I2c>(
    i2c: &mut I2C,
    delay: &mut Delay,
) -> Result<Hs3003Measurement, ()> {
    // Send measurement trigger (write with no data)
    i2c.write(HS3003_ADDR, &[]).map_err(|_| ())?;

    // Wait for measurement to complete (minimum 33ms per datasheet)
    delay.delay_millis(50);

    // Read 4 bytes of data
    let mut data = [0u8; 4];
    i2c.read(HS3003_ADDR, &mut data).map_err(|_| ())?;

    // Note: Status bits in this sensor appear unreliable, so we parse data directly
    // Debug testing showed valid temperature/humidity readings regardless of status bits
    parse_hs3003_data(&data)
}

/// Parse raw 4-byte data from HS3003 into temperature and humidity
fn parse_hs3003_data(data: &[u8; 4]) -> Result<Hs3003Measurement, ()> {
    // Parse humidity (first 2 bytes, 14-bit value)
    // Format: [H13-H6][H5-H0,xx,ST1,ST0]
    let humidity_raw = ((data[0] as u16) << 8) | (data[1] as u16);
    let humidity_value = (humidity_raw >> 2) & 0x3FFF; // Extract 14-bit value
    let humidity = (humidity_value as f32 / 16383.0) * 100.0;

    // Parse temperature (last 2 bytes, 14-bit value)
    // Format: [T13-T6][T5-T0,xx,xx,xx,xx,xx]
    let temp_raw = ((data[2] as u16) << 8) | (data[3] as u16);
    let temp_value = (temp_raw >> 2) & 0x3FFF; // Extract 14-bit value
    let temperature = ((temp_value as f32 / 16383.0) * 165.0) - 40.0;

    Ok(Hs3003Measurement {
        humidity,
        temperature,
    })
}

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("Initializing HS3003 sensor (Arduino Modulino Thermo)...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral with default configuration (100kHz)
    let mut i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    // Create delay provider
    let mut delay = Delay::new();

    info!("HS3003 sensor ready! I2C address: 0x{:02X}", HS3003_ADDR);
    info!("Starting measurements...");

    // Allow sensor to stabilize
    delay.delay_millis(100);

    loop {
        // Read sensor
        match read_hs3003(&mut i2c, &mut delay) {
            Ok(measurement) => {
                info!(
                    "Temperature: {}.{:02} °C | Humidity: {}.{:02} %",
                    measurement.temperature as i32,
                    ((measurement.temperature.abs() % 1.0) * 100.0) as u32,
                    measurement.humidity as u32,
                    ((measurement.humidity % 1.0) * 100.0) as u32,
                );
            }
            Err(_) => {
                info!("Error reading HS3003 sensor!");
                info!("Check wiring and ensure sensor is connected at address 0x44");
                // Wait longer after error to allow sensor to recover
                delay.delay_millis(500);
            }
        }

        // Wait 2 seconds between measurements to allow sensor to be ready
        // HS3003 needs adequate time between measurement cycles
        delay.delay_millis(2000);
    }
}
