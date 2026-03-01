//! # Arduino Modulino Joystick Example for Rust ESP Board
//!
//! This example uses the **modulino** library: https://crates.io/crates/modulino
//!
//! It reads joystick position and button state over I2C and prints values using `defmt`.
//!
//! ## Hardware
//!
//! - **Module:** Arduino Modulino Joystick
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x3F (7-bit)
//!
//! ## Wiring with Qwiic/STEMMA QT on Rust ESP Board
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Joystick.
//! The cable provides:
//! ```
//!      Modulino Joystick -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO8  (I2C Clock)
//! (blue)   SDA -> GPIO10 (I2C Data)
//! ```
//!
//! Run with `cargo run --example modulino_joystick_i2c`.

#![no_std]
#![no_main]

use defmt::{Debug2Format, error, info};
use esp_hal::{
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
    main,
};
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

// Import from modulino library
use modulino::Joystick;

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();

    info!("Initializing Arduino Modulino Joystick...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral with default configuration (100kHz)
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    // Create Modulino Joystick driver
    let mut joystick = match Joystick::new(i2c) {
        Ok(j) => j,
        Err(e) => {
            error!(
                "Failed to initialize Modulino Joystick: {:?}",
                Debug2Format(&e)
            );
            loop {
                delay.delay_millis(1000);
            }
        }
    };

    info!(
        "Modulino Joystick initialized at address 0x{:02X}!",
        joystick.address()
    );

    loop {
        if let Err(e) = joystick.update() {
            error!("Joystick update error: {:?}", Debug2Format(&e));
        } else {
            let (x, y) = joystick.position();
            let btn = joystick.button_pressed();
            let angle = joystick.angle();
            let mag = joystick.magnitude();
            // defmt doesn't support float display formatting (.2), so formatting as standard floats
            info!(
                "Pos: ({}, {}), Button: {}, Angle: {}, Mag: {}",
                x, y, btn, angle, mag
            );
        }
        delay.delay_millis(200);
    }
}
