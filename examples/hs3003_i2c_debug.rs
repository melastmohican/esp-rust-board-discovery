//! # HS3003 Debug Version - Detailed Logging
//!
//! This is a debug version with verbose logging to diagnose the communication issue.

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

const HS3003_ADDR: u8 = 0x44;

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("HS3003 Debug Mode");
    info!("=================");

    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    let mut i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    let mut delay = Delay::new();

    info!(
        "I2C initialized. Testing communication with HS3003 at 0x{:02X}",
        HS3003_ADDR
    );
    delay.delay_millis(100);

    loop {
        info!("");
        info!("--- New Measurement Cycle ---");

        // Test 1: Try writing (wake/trigger)
        info!("Test 1: Sending wake/measurement trigger (empty write)");
        match i2c.write(HS3003_ADDR, &[]) {
            Ok(_) => info!("  Write OK"),
            Err(_) => info!("  Write FAILED"),
        }

        delay.delay_millis(50);

        // Test 2: Try reading immediately
        info!("Test 2: Reading 4 bytes after 50ms");
        let mut data = [0u8; 4];
        match i2c.read(HS3003_ADDR, &mut data) {
            Ok(_) => {
                info!(
                    "  Read OK: [{:02X} {:02X} {:02X} {:02X}]",
                    data[0], data[1], data[2], data[3]
                );
                let status = data[1] & 0x03;
                info!(
                    "  Status bits (data[1] & 0x03): {:02b} ({})",
                    status, status
                );

                // Try to parse
                let humidity_raw = ((data[0] as u16) << 8) | (data[1] as u16);
                let humidity_value = (humidity_raw >> 2) & 0x3FFF;
                let humidity = (humidity_value as f32 / 16383.0) * 100.0;

                let temp_raw = ((data[2] as u16) << 8) | (data[3] as u16);
                let temp_value = (temp_raw >> 2) & 0x3FFF;
                let temperature = ((temp_value as f32 / 16383.0) * 165.0) - 40.0;

                info!(
                    "  Parsed: Temp={}.{:02}°C, Hum={}.{:02}%",
                    temperature as i32,
                    ((temperature.abs() % 1.0) * 100.0) as u32,
                    humidity as u32,
                    ((humidity % 1.0) * 100.0) as u32
                );
            }
            Err(_) => info!("  Read FAILED"),
        }

        delay.delay_millis(100);

        // Test 3: Try reading again after longer delay
        info!("Test 3: Reading again after 100ms more");
        match i2c.read(HS3003_ADDR, &mut data) {
            Ok(_) => {
                info!(
                    "  Read OK: [{:02X} {:02X} {:02X} {:02X}]",
                    data[0], data[1], data[2], data[3]
                );
                let status = data[1] & 0x03;
                info!("  Status bits: {:02b} ({})", status, status);
            }
            Err(_) => info!("  Read FAILED"),
        }

        info!("Waiting 2 seconds before next cycle...");
        delay.delay_millis(2000);
    }
}
