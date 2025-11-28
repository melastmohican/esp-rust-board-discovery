//! # Zermatt Image Display Example for ILI9341 TFT LCD
//!
//! Display a 320x240 image of Zermatt on the ILI9341 2.8" TFT LCD display.
//!
//! This example demonstrates displaying a full-screen image in landscape mode.
//!
//! ## Hardware: 2.8" TFT SPI 240x320 V1.2 Display Module
//!
//! ## Wiring for 2.8" TFT SPI 240x320 V1.2
//!
//! ```
//!      LCD Pin     ->  ESP32-C3-DevKit-RUST-1
//! -----------------------------------------------
//!        VCC       ->  3.3V
//!        GND       ->  GND
//!        CS        ->  GPIO5  (Chip Select)
//!        RESET     ->  GPIO3  (Reset)
//!        DC        ->  GPIO4  (Data/Command)
//!        SDI(MOSI) ->  GPIO7  (SPI MOSI/Data)
//!        SCK       ->  GPIO6  (SPI Clock)
//!        LED       ->  3.3V (Backlight)
//!        SDO(MISO) ->  GPIO1  (optional)
//! ```
//!
//! ## Image Conversion
//!
//! The JPEG image was converted to BMP format using:
//! ```bash
//! python3 examples/convert_jpg_to_bmp.py examples/zermatt_320x240.jpg examples/zermatt_320x240.bmp
//! ```
//!
//! Run with `cargo run --example zermatt`.

#![no_std]
#![no_main]

use defmt::info;
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    Drawable,
    draw_target::DrawTarget,
    geometry::Point,
    image::Image,
    pixelcolor::{Rgb565, RgbColor},
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
use mipidsi::{
    Builder,
    models::ILI9341Rgb565,
    options::{ColorOrder, Orientation, Rotation},
};
use panic_rtt_target as _;
use tinybmp::Bmp;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("Initializing ILI9341 TFT LCD display for Zermatt image...");

    // Configure SPI pins
    let sck = peripherals.GPIO6;
    let mosi = peripherals.GPIO7;
    let miso = peripherals.GPIO1;

    // Control pins
    let cs = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    let dc = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
    let rst = Output::new(peripherals.GPIO3, Level::High, OutputConfig::default());

    // Create SPI bus with 40 MHz clock speed
    let spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sck)
    .with_mosi(mosi)
    .with_miso(miso);

    info!("SPI configured at 40 MHz");

    // Create exclusive SPI device with CS pin
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    // Create display interface
    let di = SPIInterface::new(spi_device, dc);

    let mut delay = Delay::new();

    info!("Initializing display in landscape mode...");

    // Create and initialize display using mipidsi
    // Physical display is 240x320 (portrait), we rotate to landscape (320x240)
    // Keep display_size at physical dimensions, rotation changes logical dimensions
    // Remove color inversion, try BGR color order instead
    let mut display = Builder::new(ILI9341Rgb565, di)
        .reset_pin(rst)
        .display_size(240, 320) // Physical dimensions
        .orientation(Orientation::new().rotate(Rotation::Deg90).flip_horizontal())
        .color_order(ColorOrder::Bgr)
        .init(&mut delay)
        .unwrap();

    info!("Display initialized in landscape mode (320x240)!");

    // Clear screen to black
    display.clear(Rgb565::BLACK).unwrap();

    info!("Loading Zermatt image (320x240 BMP)...");

    // Load the BMP image data using tinybmp
    let bmp = Bmp::<Rgb565>::from_slice(include_bytes!("zermatt_320x240.bmp"))
        .expect("Failed to load BMP image");

    info!("Drawing Zermatt image...");

    // Draw the image at origin (0, 0) to fill the entire screen
    let image = Image::new(&bmp, Point::new(0, 0));
    image.draw(&mut display).unwrap();

    info!("Zermatt image displayed! Enjoy the view!");

    // Main loop - image is now showing
    loop {
        // Use WFI (Wait For Interrupt) to save power
        unsafe { core::arch::asm!("wfi") };
    }
}
