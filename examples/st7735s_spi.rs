//! # ST7735S LCD Display SPI Example for Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//!
//! Draw images on a 80x160 ST7735S display (Waveshare 0.96 inch LCD module) over SPI.
//!
//! This example is for the Rust ESP Board (ESP32-C3-DevKit-RUST-1) using SPI2.
//!
//! ## Wiring for Waveshare 0.96 inch LCD Module
//!
//! The Waveshare 0.96" LCD module uses an ST7735S controller and has a resolution of 80x160 pixels.
//!
//! ```
//!      LCD Pin  ->  ESP32-C3-DevKit-RUST-1
//! ----------------------------------------
//!        VCC    ->  3.3V
//!        GND    ->  GND
//!        DIN    ->  GPIO7  (MOSI)
//!        CLK    ->  GPIO6  (SCK)
//!        CS     ->  GPIO5  (Chip Select)
//!        DC     ->  GPIO4  (Data/Command)
//!        RST    ->  GPIO3  (Reset)
//!        BL     ->  GPIO2  (Backlight, optional - can connect to 3.3V directly)
//! ```
//!
//! ### Pin Details:
//! - **VCC**: Power supply (3.3V)
//! - **GND**: Ground
//! - **DIN (MOSI)**: SPI data input to LCD
//! - **CLK (SCK)**: SPI clock
//! - **CS**: Chip select (active low)
//! - **DC**: Data/Command select (Low=Command, High=Data)
//! - **RST**: Reset (active low)
//! - **BL**: Backlight control (HIGH=on, can be connected directly to 3.3V if software control not needed)
//!
//! ### SPI Configuration:
//! - SPI Mode: 0 (CPOL=0, CPHA=0)
//! - Clock Speed: 26 MHz (ESP32-C3 can handle higher speeds than RP2040)
//! - Display Resolution: 80x160 pixels
//! - Color Format: RGB565 (16-bit color)
//!
//! Run with `cargo run --example st7735s_spi`.

#![no_std]
#![no_main]

use defmt::info;
use embedded_graphics::{
    image::{Image, ImageRaw},
    pixelcolor::Rgb565,
    prelude::*,
};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    main,
    spi::{
        Mode,
        master::{Config as SpiConfig, Spi},
    },
    time::Rate,
};
use panic_rtt_target as _;
use st7735_lcd::Orientation;
use tinybmp::Bmp;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("Initializing ST7735S LCD display...");

    // Configure SPI pins for Waveshare 0.96" LCD
    let sck = peripherals.GPIO6; // Clock
    let mosi = peripherals.GPIO7; // MOSI (DIN on LCD)
    let miso = peripherals.GPIO1; // MISO (not used by display, but needed for SPI config)

    // Control pins
    let cs = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default()); // Chip Select
    let dc = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default()); // Data/Command
    let rst = Output::new(peripherals.GPIO3, Level::High, OutputConfig::default()); // Reset
    let mut bl = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default()); // Backlight

    // Create SPI bus with 26 MHz clock speed
    // ST7735S supports up to 15 MHz typical, 33 MHz max
    // ESP32-C3 can easily handle this speed
    let spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(26))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sck)
    .with_mosi(mosi)
    .with_miso(miso);

    info!("SPI configured at 26 MHz");

    // Create exclusive SPI device with CS pin
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    // Create display driver
    // Parameters: (spi_device, dc, rst, rgb, inverted, width, height)
    // - rgb: false (BGR order for Waveshare module)
    // - inverted: true (Waveshare module typically needs inversion)
    let mut display = st7735_lcd::ST7735::new(spi_device, dc, rst, false, true, 160, 80);

    let mut delay = Delay::new();

    info!("Initializing display...");
    display.init(&mut delay).unwrap();

    // Set display offset for Waveshare 0.96" module
    // These offsets align the visible area correctly
    display.set_offset(1, 26);

    // Set orientation to landscape (swapped)
    display
        .set_orientation(&Orientation::LandscapeSwapped)
        .unwrap();

    // Clear screen to black
    display.clear(Rgb565::BLACK).unwrap();

    // Adjust offset for drawing
    display.set_offset(0, 25);

    info!("Drawing images...");

    // Draw ferris (raw RGB565 image)
    let image_raw: ImageRaw<Rgb565> = ImageRaw::new(include_bytes!("ferris.raw"), 86);
    let image: Image<_> = Image::new(&image_raw, Point::new(80, 8));
    image.draw(&mut display).unwrap();

    info!("Ferris drawn!");

    // Draw Rust logo (BMP format)
    let logo = Bmp::from_slice(include_bytes!("rust.bmp")).unwrap();
    let logo = Image::new(&logo, Point::new(0, 0));
    logo.draw(&mut display).unwrap();

    info!("Rust logo drawn!");

    // Turn on backlight
    bl.set_high();
    info!("Backlight enabled - display complete!");

    // Main loop - display is now showing the images
    loop {
        // Use WFI (Wait For Interrupt) to save power
        unsafe { core::arch::asm!("wfi") };
    }
}
