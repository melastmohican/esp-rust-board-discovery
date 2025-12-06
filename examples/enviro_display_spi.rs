//! # [EXPERIMENTAL/PROBLEMATIC] Enviro+ FeatherWing LCD Display Example
//!
//! **STATUS: NOT RECOMMENDED FOR DAILY USE**
//!
//! This example demonstrates how to drive the ST7735s display on the Enviro+ FeatherWing,
//! BUT it faces severe hardware limitations when used with the **ESP32-C3-DevKit-RUST-1**.
//!
//! ## CRITICAL HARDWARE CONFLICTS
//!
//! 1.  **USB LOSS (The "JTAG Crash")**:
//!     - The Enviro+ uses standard Feather pins D5 and D6 for **DC** and **CS**.
//!     - On the Rust Board, D5/D6 are mapped to **GPIO 18 and 19**.
//!     - **GPIO 18/19 are the USB D-/D+ pins.**
//!     - **Result**: As soon as this code runs, **USB communication is severed**. You lose JTAG debugging, serial logs, and must manually reset the board (BOOT+RST) to re-flash.
//!
//! 2.  **NO BACKLIGHT**:
//!     - The Enviro+ expects Backlight/Reset control on Pin D9.
//!     - On the Rust Board, the corresponding pin on the right header is **NC (Not Connected)**.
//!     - **Result**: The screen will remain **BLACK** unless you manually solder a jumper wire from Pin 8 (Right) to 3.3V.
//!
//! ## Why keep this file?
//!
//! It serves as a reference for:
//! - Setting up `mipidsi` 0.8.0 with `display-interface-spi`.
//! - Handling the specific `display_offset(26, 1)` and `color_order(BGR)` required for this panel.
//! - Using the onboard RGB LED (GPIO 2) as a "Heartbeat" status indicator when USB logs are unavailable.
//!
//! **For a working display experience, use an I2C OLED (SSD1306) which does not conflict with USB.**
//!
//! ## Pin Mapping (Feather Form Factor)
//!
//! The Enviro+ FeatherWing expects standard Feather pins. Mapping for Rust Board:
//!
//! - **Board:** ESP32-C3-DevKit-RUST-1 (Rust ESP Board)
//! - **Module:** Pimoroni Enviro+ FeatherWing
//! - **Display:** 0.96" IPS LCD (160x80), ST7735s controller, SPI interface
//!
//! ## Pin Mapping (Feather Form Factor)
//!
//! The Enviro+ FeatherWing expects standard Feather pins. Mapping for Rust Board:
//!
//! - **SCK**: GPIO 6 (Left Pin 11)
//! - **MOSI**: GPIO 7 (Left Pin 12)
//! - **LCD_CS (D6)**: GPIO 18 (Right Pin 9) -> **WARNING: BREAKS USB/JTAG**
//! - **LCD_DC (D5)**: GPIO 19 (Right Pin 10) -> **WARNING: BREAKS USB/JTAG**
//! - **LCD_RST (D9)**: Pin 8 (Right Header) -> **NC (Not Connected)** on Rust Board.
//! - **BL_EN**: Connected to D9/RST on this shield.
//!
//! ## CRITICAL WARNINGS
//!
//! 1.  **USB LOSS**: Using GPIO 18 and 19 for the display (CS/DC) **WILL DISCONNECT THE USB-JTAG/SERIAL CONNECTION**.
//!     - You will lose logs and debugging ability once the program runs.
//!     - To recover and re-flash: Hold BOOT (GPIO9), Press RST, Release BOOT.
//!
//! 2.  **BACKLIGHT**: The Backlight Enable pin (D9) is **NOT CONNECTED** on the Rust Board (Pin 8 right is NC).
//!     - The display will likely be **BLACK/DARK** unless you fix this.
//!     - **FIX**: Connect a jumper wire from Pin 8 (Right Header) to 3.3V (Pin 2 Left or Pin 3 Right) to force backlight ON.
//!
//! Run with `cargo run --example enviro_display_spi`.

#![no_std]
#![no_main]

use defmt::info;
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, PrimitiveStyle, Rectangle},
    text::{Alignment, Text},
};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    main,
    rmt::Rmt,
    spi::{
        Mode,
        master::{Config as SpiConfig, Spi},
    },
    time::Rate,
};
use esp_hal_smartled::{SmartLedsAdapter, smart_led_buffer};
use mipidsi::{
    Builder,
    models::ST7735s,
    options::{ColorInversion, ColorOrder, Orientation, Rotation},
};
use panic_rtt_target as _;
use smart_leds::{RGB8, SmartLedsWrite, brightness, gamma, hsv::Hsv, hsv::hsv2rgb};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut delay = Delay::new();

    info!("Initializing...");

    // --- 1. SETUP RGB LED ---
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).unwrap();
    let rmt_channel = rmt.channel0;
    let mut rmt_buffer = smart_led_buffer!(1);
    let mut smart_led = SmartLedsAdapter::new(rmt_channel, peripherals.GPIO2, &mut rmt_buffer);

    // DEBUG: Set LED to BLUE (Stage 1: Started)
    // We use a helper closure/macro to write color to allow easier swapping
    let mut set_color = |r, g, b| {
        let color = RGB8::new(r, g, b);
        let data = [color];
        smart_led
            .write(brightness(gamma(data.iter().cloned()), 20))
            .ok();
    };

    set_color(0, 0, 255); // BLUE
    info!("LED: BLUE (Started)");
    delay.delay_millis(1000);

    // --- 2. CONFIG SPI & GPIO (USB BREAK AHEAD) ---
    info!("Configuring SPI/GPIO (USB Disconnect Imminent)...");

    let sck = peripherals.GPIO6;
    let mosi = peripherals.GPIO7;
    let miso = peripherals.GPIO1;

    // WARNING: These lines disconnect USB-JTAG
    let cs = Output::new(peripherals.GPIO18, Level::High, OutputConfig::default());
    let dc = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());

    // DEBUG: Set LED to ORANGE (Stage 2: GPIO Configured / USB Lost)
    set_color(255, 165, 0); // ORANGE
    delay.delay_millis(500); // Short delay to ensure color is seen before next step

    // SPI Init
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

    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let di = SPIInterface::new(spi_device, dc);

    // --- 3. DISPLAY INIT ---
    // If it crashes here, LED stays ORANGE
    let mut display = Builder::new(ST7735s, di)
        .display_size(160, 80)
        .invert_colors(ColorInversion::Inverted)
        .color_order(ColorOrder::Bgr)
        .orientation(Orientation::default().rotate(Rotation::Deg270))
        .init(&mut delay)
        .unwrap();

    // DEBUG: Set LED to GREEN (Stage 3: Success)
    set_color(0, 255, 0); // GREEN
    info!("Display Init Success. LED: GREEN");

    // Clear screen
    display.clear(Rgb565::BLACK).unwrap();

    // Draw something
    let style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    Rectangle::new(Point::new(0, 0), Size::new(160, 80))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::CSS_DARK_SLATE_BLUE))
        .draw(&mut display)
        .unwrap();
    Text::with_alignment("Enviro+ Rust", Point::new(80, 20), style, Alignment::Center)
        .draw(&mut display)
        .unwrap();
    Circle::new(Point::new(15, 45), 20)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::YELLOW))
        .draw(&mut display)
        .unwrap();
    Text::new("Sensor: Active", Point::new(45, 60), style)
        .draw(&mut display)
        .unwrap();

    // --- 4. LOOP ---
    info!("Looping...");
    let mut hue = 0;
    loop {
        // Rainbow Cycle
        let color = hsv2rgb(Hsv {
            hue,
            sat: 255,
            val: 20,
        });
        let data = [color];
        smart_led
            .write(brightness(gamma(data.iter().cloned()), 10))
            .ok();

        hue = hue.wrapping_add(10);
        delay.delay_millis(100);
    }
}
