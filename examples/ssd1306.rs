//! # SSD1306 OLED Display Example for Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//!
//! Draw a 1 bit per pixel black and white image on a 128x64 SSD1306 display over I2C.
//!
//! This example is for the Rust ESP Board (ESP32-C3-DevKit-RUST-1) using I2C0.
//!
//! Wiring connections are as follows for the display:
//!
//! ```
//!      Display -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO8
//! (green)  SDA -> GPIO10
//! ```
//!
//! Run with `cargo run --example ssd1306`.

#![no_std]
#![no_main]

use defmt::info;
use embedded_graphics::{
    image::{Image, ImageRaw},
    pixelcolor::BinaryColor,
    prelude::*,
};
use esp_hal::{
    i2c::master::{Config as I2cConfig, I2c},
    main,
};
use panic_rtt_target as _;
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("Initializing SSD1306 OLED display...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral with default configuration (100kHz)
    // You can increase speed to 400kHz with: I2cConfig::default().with_frequency(400.kHz())
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    // Create display interface
    let interface = I2CDisplayInterface::new(i2c);

    // Create display driver
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    // Initialize the display
    display.init().unwrap();
    info!("Display initialized!");

    // Load the raw image data (64x64 pixels, 1 bit per pixel)
    let raw: ImageRaw<BinaryColor> = ImageRaw::new(include_bytes!("./rust.raw"), 64);

    // Create an image positioned at x=32, y=0 to center it horizontally
    let image = Image::new(&raw, Point::new(32, 0));

    // Draw the image to the display buffer
    image.draw(&mut display).unwrap();

    // Flush the buffer to the display
    display.flush().unwrap();
    info!("Image displayed!");

    // Halt - image is now displayed
    loop {
        unsafe { core::arch::asm!("wfi") };
    }
}
