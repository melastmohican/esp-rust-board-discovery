//! # Arduino Modulino Vibro Example for Rust ESP Board
//!
//! This example uses the **modulino** library: https://github.com/melastmohican/modulino-rs
//!
//! Demonstrates various vibration patterns on the Arduino Modulino Vibro module over I2C.
//!
//! ## Hardware
//!
//! - **Module:** Arduino Modulino Vibro
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x24 (7-bit)
//!
//! ## Wiring with Qwiic/STEMMA QT on Rust ESP Board
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Vibro.
//! The cable provides:
//! ```
//!      Modulino Vibro -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO8  (I2C Clock)
//! (blue)   SDA -> GPIO10 (I2C Data)
//! ```
//!
//! Run with `cargo run --example modulino_vibro_i2c`.

#![no_std]
#![no_main]

use defmt::{Debug2Format, error, info};
use esp_hal::{
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
    main,
    time::Rate,
};
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

// Import from modulino library
use modulino::{PowerLevel, Vibro};

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();

    info!("Initializing Arduino Modulino Vibro...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral with default configuration (100kHz)
    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_sda(sda)
    .with_scl(scl);

    // Create Modulino Vibro driver
    // Automatically uses default address 0x24
    let mut vibro = match Vibro::new(i2c) {
        Ok(v) => v,
        Err(e) => {
            error!(
                "Failed to initialize Modulino Vibro: {:?}",
                Debug2Format(&e)
            );
            loop {
                delay.delay_millis(1000);
            }
        }
    };

    info!(
        "Modulino Vibro initialized at address 0x{:02X}!",
        vibro.address()
    );

    loop {
        // 1. Gentle Pulses
        info!("Pattern 1: Gentle Pulses");
        for _ in 0..3 {
            vibro.pulse(100, PowerLevel::Gentle).ok();
            delay.delay_millis(500);
        }
        delay.delay_millis(1000);

        // 2. Medium Vibration
        info!("Pattern 2: Medium Vibration");
        vibro.on(1000, PowerLevel::Medium).ok();
        delay.delay_millis(2000);

        // 3. Intense Double Pulse
        info!("Pattern 3: Intense Double Pulse");
        vibro.pulse(200, PowerLevel::Intense).ok();
        delay.delay_millis(300);
        vibro.pulse(200, PowerLevel::Intense).ok();
        delay.delay_millis(2000);

        // 4. Power Sweep
        info!("Pattern 4: Power Sweep");
        let power_levels = [
            PowerLevel::Gentle,
            PowerLevel::Moderate,
            PowerLevel::Medium,
            PowerLevel::Intense,
            PowerLevel::Powerful,
            PowerLevel::Maximum,
        ];

        for &level in power_levels.iter() {
            info!("Power Level: {:?}", Debug2Format(&level));
            vibro.on(500, level).ok();
            delay.delay_millis(1000);
        }

        info!("Pattern complete. Waiting 3 seconds...");
        delay.delay_millis(3000);
    }
}
