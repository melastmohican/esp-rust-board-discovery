//! # Arduino Modulino Pixels Example for Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//!
//! Controls 8 RGB LEDs on the Arduino Modulino Pixels module over I2C.
//!
//! This example is configured for the Arduino Modulino Pixels breakout board connected via Qwiic connector.
//!
//! ## Hardware
//!
//! - **Module:** Arduino Modulino Pixels (ABX00109)
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x36 (7-bit), or 0x6C (8-bit write address), configurable via software
//! - **LEDs:** 8x LC8822-2020 addressable RGB LEDs
//! - **MCU:** STM32C011F4 (handles LED control over I2C)
//!
//! ## Wiring with Qwiic/STEMMA QT
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Pixels.
//! The cable provides:
//! ```
//!      Modulino Pixels -> Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO8  (I2C Clock)
//! (blue)   SDA -> GPIO10 (I2C Data)
//! ```
//!
//! ## I2C Address
//!
//! The Modulino Pixels responds at I2C address 0x36 (7-bit addressing).
//! Note: Some documentation mentions 0x6C, which is the 8-bit write address (0x36 << 1).
//! The address can be changed via software to allow multiple modules on the same bus.
//!
//! ## About the Modulino Pixels
//!
//! The Arduino Modulino Pixels features 8 individually addressable RGB LEDs controlled via I2C.
//! Each LED can display full-color with adjustable brightness. The onboard STM32C011F4 handles
//! the LED control, reducing the processing load on the main board.
//!
//! ## Protocol
//!
//! The module uses a simple I2C protocol:
//! - Each LED uses 4 bytes: [red, green, blue, 0xE0|brightness]
//! - RGB values: 0-255 each
//! - Brightness: 0-31 (5 bits) ORed with 0xE0 control bits
//! - Control bits (0xE0): Always set to 0b111xxxxx
//! - Total buffer: 32 bytes for 8 LEDs
//! - Write the buffer to I2C address 0x36 to update all LEDs
//!
//! Run with `cargo run --example modulino_pixels_i2c`.

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

/// Modulino Pixels I2C address
/// Note: The documented address is 0x6C, but the actual 7-bit address is 0x36
const MODULINO_PIXELS_ADDR: u8 = 0x36;

/// Number of LEDs on the Modulino Pixels
const NUM_LEDS: usize = 8;

/// RGB Color structure
#[derive(Copy, Clone)]
struct Color {
    r: u8,
    g: u8,
    b: u8,
}

impl Color {
    const fn new(r: u8, g: u8, b: u8) -> Self {
        Color { r, g, b }
    }

    // Predefined colors
    const RED: Color = Color::new(255, 0, 0);
    const GREEN: Color = Color::new(0, 255, 0);
    const BLUE: Color = Color::new(0, 0, 255);
    const YELLOW: Color = Color::new(255, 255, 0);
    const CYAN: Color = Color::new(0, 255, 255);
    const MAGENTA: Color = Color::new(255, 0, 255);
}

/// Modulino Pixels driver
struct ModulinoPixels {
    buffer: [u8; NUM_LEDS * 4],
}

impl ModulinoPixels {
    fn new() -> Self {
        let mut pixels = ModulinoPixels {
            buffer: [0; NUM_LEDS * 4],
        };
        // Initialize all brightness bytes with 0xE0 (control bits set, brightness 0)
        for i in 0..NUM_LEDS {
            pixels.buffer[i * 4 + 3] = 0xE0;
        }
        pixels
    }

    /// Set a pixel color with brightness
    /// idx: LED index (0-7)
    /// color: RGB color
    /// brightness: 0-100 (mapped to 0-31 internally)
    fn set_pixel(&mut self, idx: usize, color: Color, brightness: u8) {
        if idx >= NUM_LEDS {
            return;
        }

        // Map brightness from 0-100 to 0-31 (5 bits)
        let brightness_mapped = (brightness.min(100) as u32 * 31 / 100) as u8;

        // Add the 0xE0 flag bits to brightness
        let brightness_byte = brightness_mapped | 0xE0;

        // Each pixel uses 4 bytes: [red, green, blue, 0xE0|brightness]
        let offset = idx * 4;
        self.buffer[offset] = color.r;
        self.buffer[offset + 1] = color.g;
        self.buffer[offset + 2] = color.b;
        self.buffer[offset + 3] = brightness_byte;
    }

    /// Clear all pixels (turn them all off)
    fn clear_all(&mut self) {
        self.buffer.fill(0);
        // Restore brightness control bits
        for i in 0..NUM_LEDS {
            self.buffer[i * 4 + 3] = 0xE0;
        }
    }

    /// Get the buffer to write to I2C
    fn get_buffer(&self) -> &[u8] {
        &self.buffer
    }
}

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("Initializing Arduino Modulino Pixels...");

    // Configure I2C pins
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // Create I2C peripheral with default configuration (100kHz)
    let mut i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    // Create delay provider
    let delay = Delay::new();

    // Create Modulino Pixels driver
    let mut pixels = ModulinoPixels::new();

    info!("Modulino Pixels initialized!");
    info!("Starting LED animations...");

    // Test connection by turning on first LED
    pixels.set_pixel(0, Color::RED, 50);
    match i2c.write(MODULINO_PIXELS_ADDR, pixels.get_buffer()) {
        Ok(_) => info!(
            "Connected to Modulino Pixels at 0x{:02X}",
            MODULINO_PIXELS_ADDR
        ),
        Err(_) => {
            info!("Failed to connect to Modulino Pixels!");
            info!("Check wiring and I2C address (default: 0x36)");
            loop {
                delay.delay_millis(1000);
            }
        }
    }

    delay.delay_millis(1000);

    // Animation 1: Rainbow colors
    info!("Animation 1: Rainbow colors");
    let rainbow_colors = [
        Color::RED,
        Color::new(255, 127, 0), // Orange
        Color::YELLOW,
        Color::GREEN,
        Color::CYAN,
        Color::BLUE,
        Color::new(75, 0, 130), // Indigo
        Color::MAGENTA,
    ];

    for _ in 0..3 {
        for i in 0..NUM_LEDS {
            pixels.set_pixel(i, rainbow_colors[i], 50);
        }
        i2c.write(MODULINO_PIXELS_ADDR, pixels.get_buffer()).ok();
        delay.delay_millis(500);

        pixels.clear_all();
        i2c.write(MODULINO_PIXELS_ADDR, pixels.get_buffer()).ok();
        delay.delay_millis(200);
    }

    // Animation 2: Knight Rider / Larson Scanner effect
    info!("Animation 2: Knight Rider effect");
    for _ in 0..3 {
        // Forward
        for i in 0..NUM_LEDS {
            pixels.clear_all();

            // Main bright LED
            pixels.set_pixel(i, Color::RED, 100);

            // Trailing glow effect
            if i > 0 {
                pixels.set_pixel(i - 1, Color::RED, 12);
            }
            if i > 1 {
                pixels.set_pixel(i - 2, Color::RED, 6);
            }

            i2c.write(MODULINO_PIXELS_ADDR, pixels.get_buffer()).ok();
            delay.delay_millis(100);
        }

        // Backward
        for i in (0..NUM_LEDS).rev() {
            pixels.clear_all();

            // Main bright LED
            pixels.set_pixel(i, Color::RED, 100);

            // Trailing glow effect
            if i < NUM_LEDS - 1 {
                pixels.set_pixel(i + 1, Color::RED, 12);
            }
            if i < NUM_LEDS - 2 {
                pixels.set_pixel(i + 2, Color::RED, 6);
            }

            i2c.write(MODULINO_PIXELS_ADDR, pixels.get_buffer()).ok();
            delay.delay_millis(100);
        }
    }

    // Animation 3: Color fade
    info!("Animation 3: Color fade cycle");
    loop {
        // Fade through different colors
        let colors = [
            Color::RED,
            Color::GREEN,
            Color::BLUE,
            Color::YELLOW,
            Color::CYAN,
            Color::MAGENTA,
        ];

        for color in colors.iter() {
            // Fade in
            for brightness in (0..=100).step_by(5) {
                for i in 0..NUM_LEDS {
                    pixels.set_pixel(i, *color, brightness);
                }
                i2c.write(MODULINO_PIXELS_ADDR, pixels.get_buffer()).ok();
                delay.delay_millis(20);
            }

            delay.delay_millis(300);

            // Fade out
            for brightness in (0..=100).rev().step_by(5) {
                for i in 0..NUM_LEDS {
                    pixels.set_pixel(i, *color, brightness);
                }
                i2c.write(MODULINO_PIXELS_ADDR, pixels.get_buffer()).ok();
                delay.delay_millis(20);
            }
        }
    }
}
