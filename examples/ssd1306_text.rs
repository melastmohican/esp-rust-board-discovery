//! # SSD1306 OLED Display with Text Example for Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//!
//! This example demonstrates drawing text and shapes on a 128x64 SSD1306 display over I2C.
//!
//! Wiring connections:
//!
//! ```
//!      Display -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO8
//! (green)  SDA -> GPIO10
//! ```
//!
//! Run with `cargo run --example ssd1306_text`.

#![no_std]
#![no_main]

use defmt::info;
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use esp_hal::{
    delay::Delay,
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

    // Create I2C peripheral
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

    let delay = Delay::new();

    // Create text style
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    // Clear the display buffer
    display.clear(BinaryColor::Off).unwrap();

    // Draw title text
    Text::with_baseline("Rust ESP", Point::new(30, 0), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    Text::with_baseline(
        "Rust ESP Board Demo",
        Point::new(25, 12),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    // Draw a separator line
    Line::new(Point::new(0, 24), Point::new(127, 24))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(&mut display)
        .unwrap();

    // Draw a rectangle
    Rectangle::new(Point::new(10, 30), Size::new(30, 20))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(&mut display)
        .unwrap();

    // Draw a filled circle
    Circle::new(Point::new(60, 35), 10)
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
        .draw(&mut display)
        .unwrap();

    // Draw some text at bottom
    Text::with_baseline("Hello Rust!", Point::new(10, 54), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    // Flush to display
    display.flush().unwrap();

    info!("Display content rendered!");

    // Keep display showing
    loop {
        delay.delay_millis(1000);
    }
}
