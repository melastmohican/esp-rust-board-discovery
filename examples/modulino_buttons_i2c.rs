//! # Arduino Modulino Buttons Example for Rust ESP Board
//!
//! This example uses the **modulino** library: https://crates.io/crates/modulino
//!
//! Reads button states from the Arduino Modulino Buttons module and controls the LEDs.
//!
//! ## Hardware
//!
//! - **Module:** Arduino Modulino Buttons
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x3E (7-bit)
//!
//! ## Wiring with Qwiic/STEMMA QT on Rust ESP Board
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Buttons.
//! The cable provides:
//! ```
//!      Modulino Buttons -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO8  (I2C Clock)
//! (blue)   SDA -> GPIO10 (I2C Data)
//! ```
//!
//! Run with `cargo run --example modulino_buttons_i2c`.

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

// Import the Modulino library
use modulino::Buttons;

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();

    info!("Initializing Arduino Modulino Buttons...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral with default configuration (100kHz)
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    // Create Modulino Buttons driver
    let mut buttons = match Buttons::new(i2c) {
        Ok(b) => b,
        Err(e) => {
            error!(
                "Failed to initialize Modulino Buttons: {:?}",
                Debug2Format(&e)
            );
            loop {
                delay.delay_millis(1000);
            }
        }
    };

    info!(
        "Modulino Buttons initialized at address 0x{:02X}",
        buttons.address()
    );

    // Startup Test: Blink all LEDs to confirm connection
    info!("Testing LEDs...");
    if let Err(e) = buttons.all_leds_on() {
        error!("Failed to turn on LEDs: {:?}", Debug2Format(&e));
    }
    delay.delay_millis(500);
    if let Err(e) = buttons.all_leds_off() {
        error!("Failed to turn off LEDs: {:?}", Debug2Format(&e));
    }
    info!("LED Test Complete.");

    info!("Press buttons to toggle LEDs!");

    let mut prev_state = modulino::ButtonState::default();

    loop {
        // Read button states
        match buttons.read() {
            Ok(state) => {
                let mut need_update = false;

                // Button A: Rising Edge Detection
                if state.a && !prev_state.a {
                    info!("Button A pressed - Toggling LED A");
                    buttons.led_a.toggle();
                    need_update = true;
                }

                // Button B: Rising Edge Detection
                if state.b && !prev_state.b {
                    info!("Button B pressed - Toggling LED B");
                    buttons.led_b.toggle();
                    need_update = true;
                }

                // Button C: Rising Edge Detection
                if state.c && !prev_state.c {
                    info!("Button C pressed - Toggling LED C");
                    buttons.led_c.toggle();
                    need_update = true;
                }

                // Update LEDs only if state changed
                if need_update {
                    let update_res = buttons.update_leds();
                    if let Err(e) = update_res {
                        error!("Failed to update LEDs: {:?}", Debug2Format(&e));
                    }
                }

                // Save state for next iteration
                prev_state = state;
            }
            Err(e) => {
                error!("Failed to read buttons: {:?}", Debug2Format(&e));
            }
        }

        // Poll every 20ms for better responsiveness
        delay.delay_millis(20);
    }
}
