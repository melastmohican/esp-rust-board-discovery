//! # Arduino Modulino Buzzer Example for Rust ESP Board
//!
//! This example uses the **modulino** library: https://crates.io/crates/modulino
//!
//! Plays a simple melody on the Arduino Modulino Buzzer module over I2C.
//!
//! ## Hardware
//!
//! - **Module:** Arduino Modulino Buzzer
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x1E (7-bit)
//!
//! ## Wiring with Qwiic/STEMMA QT on Rust ESP Board
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Buzzer.
//! The cable provides:
//! ```
//!      Modulino Buzzer -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO8  (I2C Clock)
//! (blue)   SDA -> GPIO10 (I2C Data)
//! ```
//!
//! Run with `cargo run --example modulino_buzzer_i2c`.

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
use modulino::{Buzzer, Note};

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();

    info!("Initializing Arduino Modulino Buzzer...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral with default configuration (100kHz)
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    // Create Modulino Buzzer driver
    // Automatically uses default address 0x1E
    let mut buzzer = match Buzzer::new(i2c) {
        Ok(b) => b,
        Err(e) => {
            error!(
                "Failed to initialize Modulino Buzzer: {:?}",
                Debug2Format(&e)
            );
            loop {
                delay.delay_millis(1000);
            }
        }
    };

    info!(
        "Modulino Buzzer initialized at address 0x{:02X}!",
        buzzer.address()
    );
    info!("Playing melody...");

    // Simple Melody: Super Mario Theme (Intro)
    let melody = [
        (Note::E5, 100),
        (Note::E5, 100),
        (Note::Rest, 100),
        (Note::E5, 100),
        (Note::Rest, 100),
        (Note::C5, 100),
        (Note::E5, 100),
        (Note::Rest, 100),
        (Note::G5, 100),
        (Note::Rest, 300),
        (Note::G4, 100),
        (Note::Rest, 300),
    ];

    loop {
        for (note, duration) in melody.iter() {
            if *note == Note::Rest {
                // For rest, ensure no tone is playing and wait
                buzzer.no_tone().ok();
            } else {
                // Play the note
                // Note: The buzzer module handles the duration internally for the tone generation,
                // but we also need to wait here so we don't immediately send the next command.
                // We add a small gap between notes for articulation.
                if let Err(e) = buzzer.play_note(*note, *duration) {
                    error!("Failed to play note: {:?}", Debug2Format(&e));
                }
            }

            // Wait for the note duration plus a little gap
            delay.delay_millis(*duration as u32);

            // Small gap between notes (articulation)
            delay.delay_millis(50);
        }

        // Wait before repeating
        delay.delay_millis(2000);
    }
}
