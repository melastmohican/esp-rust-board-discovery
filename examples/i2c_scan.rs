//! # I2C Bus Scanner Example for Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//!
//! This example scans the I2C bus for connected devices and prints their addresses.
//!
//! The Rust ESP Board (ESP32-C3-DevKit-RUST-1) uses:
//! - SDA: GPIO10
//! - SCL: GPIO8

#![no_std]
#![no_main]

use defmt::info;
use esp_hal::{
    i2c::master::{Config as I2cConfig, I2c},
    main,
};
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("Initializing I2C bus scanner...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral with default configuration (100kHz)
    let mut i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    info!("Scanning I2C bus (addresses 0x00 to 0x7F)...");
    info!("-------------------------------------------");

    let mut devices_found = 0;

    // Scan all possible 7-bit I2C addresses (0x00 to 0x7F)
    for address in 0x00..=0x7F {
        // Try to write a zero-length message to the address
        // If a device is present, it will ACK
        match i2c.write(address, &[]) {
            Ok(_) => {
                info!("Device found at address 0x{:02X}", address);
                devices_found += 1;
            }
            Err(_) => {
                // No device at this address, continue silently
            }
        }
    }

    info!("-------------------------------------------");
    if devices_found == 0 {
        info!("No I2C devices found!");
    } else {
        info!("Scan complete! Found {} device(s)", devices_found);
    }

    // Halt after scanning
    loop {
        // Wait for interrupts (low power mode)
        unsafe { core::arch::asm!("wfi") };
    }
}
