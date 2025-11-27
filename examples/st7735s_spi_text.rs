//! # ST7735S LCD Display SPI Text Example for Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//!
//! This example demonstrates drawing text and shapes on a 80x160 ST7735S display (Waveshare 0.96 inch LCD module) over SPI.
//!
//! ## Wiring for Waveshare 0.96 inch LCD Module
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
//!        BL     ->  GPIO2  (Backlight)
//! ```
//!
//! Run with `cargo run --example st7735s_spi_text`.

#![no_std]
#![no_main]

use defmt::info;
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10, ascii::FONT_9X15_BOLD},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
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
    let mut display = st7735_lcd::ST7735::new(spi_device, dc, rst, false, true, 160, 80);

    let mut delay = Delay::new();

    info!("Initializing display...");
    display.init(&mut delay).unwrap();

    // Set display offset for Waveshare 0.96" module
    display.set_offset(1, 26);

    // Set orientation to landscape (swapped)
    display
        .set_orientation(&Orientation::LandscapeSwapped)
        .unwrap();

    // Clear screen to black
    display.clear(Rgb565::BLACK).unwrap();

    // Adjust offset for drawing
    display.set_offset(0, 25);

    info!("Drawing text and shapes...");

    // Create text styles
    let title_style = MonoTextStyleBuilder::new()
        .font(&FONT_9X15_BOLD)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::BLUE)
        .build();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::YELLOW)
        .build();

    // Draw title background
    Rectangle::new(Point::new(0, 0), Size::new(160, 16))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLUE))
        .draw(&mut display)
        .unwrap();

    // Draw title text
    Text::with_baseline(
        "Rust ESP Board",
        Point::new(5, 2),
        title_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    // Draw a separator line
    Line::new(Point::new(0, 18), Point::new(159, 18))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 2))
        .draw(&mut display)
        .unwrap();

    // Draw a red rectangle
    Rectangle::new(Point::new(10, 25), Size::new(40, 30))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 2))
        .draw(&mut display)
        .unwrap();

    // Draw a filled green circle
    Circle::new(Point::new(70, 30), 20)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::GREEN))
        .draw(&mut display)
        .unwrap();

    // Draw a filled orange rectangle
    Rectangle::new(Point::new(110, 28), Size::new(40, 24))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::CSS_ORANGE))
        .draw(&mut display)
        .unwrap();

    // Draw some text at bottom
    Text::with_baseline(
        "ST7735S Display",
        Point::new(15, 62),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    Text::with_baseline("Hello Rust!", Point::new(90, 62), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    // Turn on backlight
    bl.set_high();
    info!("Backlight enabled - display complete!");

    // Keep display showing
    loop {
        unsafe { core::arch::asm!("wfi") };
    }
}
