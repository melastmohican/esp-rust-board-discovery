//! # ILI9341 TFT LCD Display SPI Example for Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//!
//! Draw images and text on a 240x320 ILI9341 display over SPI.
//!
//! This example is for the Rust ESP Board (ESP32-C3-DevKit-RUST-1) using SPI2.
//!
//! ## Hardware: 2.8" TFT SPI 240x320 V1.2 Display Module
//!
//! This is a 240x320 rectangular TFT LCD with ILI9341 controller.
//! The module includes a resistive touchscreen (not used in this example).
//!
//! ## Wiring for 2.8" TFT SPI 240x320 V1.2 (Display pins only)
//!
//! ```
//!      LCD Pin     ->  ESP32-C3-DevKit-RUST-1
//! -----------------------------------------------
//!        VCC       ->  3.3V (or 5V if module has regulator)
//!        GND       ->  GND
//!        CS        ->  GPIO5  (Chip Select)
//!        RESET     ->  GPIO3  (Reset)
//!        DC        ->  GPIO4  (Data/Command)
//!        SDI(MOSI) ->  GPIO7  (SPI MOSI/Data)
//!        SCK       ->  GPIO6  (SPI Clock)
//!        LED       ->  3.3V (Backlight - can connect to GPIO for control)
//!        SDO(MISO) ->  GPIO1  (SPI MISO - optional for display)
//!
//! Touchscreen pins (not used in this example):
//!        T_CLK     ->  (not connected)
//!        T_CS      ->  (not connected)
//!        T_DIN     ->  (not connected)
//!        T_DO      ->  (not connected)
//!        T_IRQ     ->  (not connected)
//! ```
//!
//! ### Pin Details:
//! - **VCC**: Power supply (check if your module needs 3.3V or 5V)
//! - **GND**: Ground
//! - **CS**: Chip select for LCD (active low)
//! - **RESET**: Reset (active low)
//! - **DC**: Data/Command select (Low=Command, High=Data)
//! - **SDI (MOSI)**: SPI data input to display
//! - **SCK**: SPI clock
//! - **LED**: Backlight power (connect to 3.3V or GPIO for PWM control)
//! - **SDO (MISO)**: SPI data output from display (optional, rarely used)
//!
//! ### SPI Configuration:
//! - SPI Mode: 0 (CPOL=0, CPHA=0)
//! - Clock Speed: 40 MHz (ILI9341 supports up to 60 MHz)
//! - Display Resolution: 240x320 pixels
//! - Color Format: RGB565 (16-bit color)
//!
//! Run with `cargo run --example ili9341_spi`.

#![no_std]
#![no_main]

use defmt::info;
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    Drawable,
    draw_target::DrawTarget,
    geometry::Point,
    image::Image,
    mono_font::{MonoTextStyle, ascii::FONT_10X20},
    pixelcolor::{Rgb565, RgbColor},
    text::Text,
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
    options::{ColorOrder, Orientation},
};
use panic_rtt_target as _;
use tinybmp::Bmp;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("Initializing ILI9341 TFT LCD display (2.8 inch 240x320)...");

    // Configure SPI pins for 2.8" TFT SPI 240x320 V1.2 display
    let sck = peripherals.GPIO6; // SCK
    let mosi = peripherals.GPIO7; // SDI (MOSI)
    let miso = peripherals.GPIO1; // SDO (MISO) - optional for display

    // Control pins
    let cs = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default()); // CS - Chip Select
    let dc = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default()); // DC - Data/Command
    let rst = Output::new(peripherals.GPIO3, Level::High, OutputConfig::default()); // RESET

    // Note: LED (backlight) is connected directly to 3.3V in this example
    // For PWM backlight control, connect LED to a GPIO pin with PWM capability

    // Create SPI bus with 40 MHz clock speed
    // ILI9341 supports up to 60 MHz, 40 MHz is a safe and fast speed
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

    info!("Initializing display with mipidsi...");

    // Create and initialize display using mipidsi
    // ILI9341 typically uses RGB565 color format
    // Flip horizontal to fix left-to-right mirroring
    // Use BGR color order for correct colors on this 2.8" TFT module
    let mut display = Builder::new(ILI9341Rgb565, di)
        .reset_pin(rst)
        .display_size(240, 320)
        .orientation(Orientation::new().flip_horizontal())
        .color_order(ColorOrder::Bgr)
        .init(&mut delay)
        .unwrap();

    info!("Display initialized!");

    // Clear screen to black
    display.clear(Rgb565::BLACK).unwrap();

    info!("Drawing text and images...");

    // Create text style
    let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);

    // Draw "Hello" in white
    Text::new("Hello", Point::new(10, 20), text_style)
        .draw(&mut display)
        .unwrap();

    // Draw "World" in blue
    let blue_style = MonoTextStyle::new(&FONT_10X20, Rgb565::BLUE);
    Text::new("World", Point::new(10, 50), blue_style)
        .draw(&mut display)
        .unwrap();

    // Draw "ESP32-C3" in green
    let green_style = MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN);
    Text::new("ESP32-C3", Point::new(10, 80), green_style)
        .draw(&mut display)
        .unwrap();

    // Draw "ILI9341 Display" in yellow
    let yellow_style = MonoTextStyle::new(&FONT_10X20, Rgb565::YELLOW);
    Text::new("ILI9341 Display", Point::new(10, 110), yellow_style)
        .draw(&mut display)
        .unwrap();

    // Draw "240x320" in cyan
    let cyan_style = MonoTextStyle::new(&FONT_10X20, Rgb565::CYAN);
    Text::new("240x320", Point::new(10, 140), cyan_style)
        .draw(&mut display)
        .unwrap();

    info!("Text drawn!");

    // Draw Rust logo (BMP format)
    let logo = Bmp::from_slice(include_bytes!("rust.bmp")).unwrap();
    let logo = Image::new(&logo, Point::new(80, 180));
    logo.draw(&mut display).unwrap();

    info!("Rust logo drawn!");
    info!("Display complete!");

    // Main loop - display is now showing the content
    loop {
        // Use WFI (Wait For Interrupt) to save power
        unsafe { core::arch::asm!("wfi") };
    }
}
