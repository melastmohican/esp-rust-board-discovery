//! # Sharp GP2Y1010AU0F Dust Sensor Example (Waveshare Dust Sensor)
//!
//! Reads dust density (PM2.5/PM10) from the Sharp GP2Y1010AU0F optical dust sensor.
//!
//! This example is configured for the Waveshare Dust Sensor module.
//!
//! ## Hardware
//!
//! - **Sensor:** Waveshare Dust Sensor (Sharp GP2Y1010AU0F)
//! - **Connection:** 4-wire interface (VCC, GND, AOUT, ILED)
//! - **Voltage:** 5V (sensor requires 4.5V-5.5V)
//!
//! ## Wiring
//!
//! **Important:** This sensor requires 5V power and a 150Ω current-limiting resistor
//! for the LED, plus a 220µF capacitor for power stabilization.
//!
//! ```text
//! Waveshare Dust Sensor -> ESP32-C3 Rust Board
//! ----------------------   ---------------------
//! VCC (red)             -> 5V/VUSB (via 220µF capacitor to GND)
//! GND (black)           -> GND
//! AOUT (yellow)         -> GPIO0 (ADC input)
//! ILED (blue)           -> GPIO1 (digital output to 150Ω resistor to 5V)
//!
//! Additional components required:
//! - 150Ω resistor between ILED and 5V (current limiting for LED)
//! - 220µF capacitor between VCC and GND (power stabilization)
//! ```
//!
//! **Circuit Diagram:**
//! ```text
//!     5V/VUSB ----+---- VCC (Sensor)
//!                 |
//!              [220µF]
//!                 |
//!     GND --------+---- GND (Sensor)
//!
//!     5V/VUSB ----[150Ω]---- ILED (Sensor)
//!                             ^
//!                             |
//!     GPIO1 ------------------+ (controls LED via transistor inside sensor)
//!
//!     GPIO0 (ADC) ----------- AOUT (Sensor)
//! ```
//!
//! ## How it Works
//!
//! The GP2Y1010AU0F uses an infrared LED and photodetector to measure dust particles:
//! 1. LED is pulsed ON for 0.32ms every 10ms (3.2% duty cycle)
//! 2. After 0.28ms delay, the analog output is sampled
//! 3. Dust particles reflect LED light, increasing the output voltage
//! 4. Output voltage is proportional to dust density (0.5V per 0.1mg/m³)
//!
//! **Timing Diagram:**
//! ```text
//! LED:  ___┌────┐____________________┌────┐___
//!          |0.32|                    |0.32|
//!          |ms  |                    |ms  |
//!          ↑    ↑                    ↑
//!        ON  OFF                   ON
//!
//! Read:    ↑ 0.28ms delay
//! ```
//!
//! ## Output
//!
//! The sensor outputs:
//! - **Clean air:** ~0V to 0.6V
//! - **Dusty air:** 0.6V to 3.5V
//! - **Sensitivity:** 0.5V per 0.1mg/m³
//!
//! Typical indoor air quality:
//! - Good: < 0.035 mg/m³ (< 0.175V)
//! - Moderate: 0.035 - 0.075 mg/m³ (0.175V - 0.375V)
//! - Poor: > 0.075 mg/m³ (> 0.375V)
//!
//! Run with `cargo run --example gp2y1010au0f_dust`.

#![no_std]
#![no_main]

use defmt::info;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    main,
};
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

// Timing constants (in microseconds)
const LED_PULSE_WIDTH_US: u32 = 320; // LED on time: 0.32ms
const SAMPLING_DELAY_US: u32 = 280; // Wait before reading: 0.28ms
const CYCLE_TIME_MS: u32 = 10; // Total cycle: 10ms

// ADC constants
const ADC_MAX: u32 = 4095; // 12-bit ADC
const ADC_VOLTAGE: f32 = 3.3; // Reference voltage

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("Initializing Sharp GP2Y1010AU0F Dust Sensor...");

    let delay = Delay::new();

    // Configure LED control pin (ILED)
    // The sensor has an internal transistor, so LOW turns LED ON, HIGH turns LED OFF
    let mut led_pin = Output::new(peripherals.GPIO1, Level::High, OutputConfig::default());
    info!("LED control pin configured (GPIO1)");

    // Configure ADC for analog output (AOUT)
    let mut adc_config = AdcConfig::new();
    let mut aout_pin = adc_config.enable_pin(peripherals.GPIO0, Attenuation::_11dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config);
    info!("ADC configured (GPIO0)");

    info!("");
    info!("=== Dust Sensor Information ===");
    info!("Sensor: Sharp GP2Y1010AU0F");
    info!("Measurement: PM2.5 and PM10 dust particles");
    info!("Timing: LED pulse 0.32ms, sample at 0.28ms, 10ms cycle");
    info!("");
    info!("Required components:");
    info!("- 150 Ohm resistor between ILED and 5V");
    info!("- 220 uF capacitor between VCC and GND");
    info!("");

    delay.delay_millis(1000);

    info!("Starting dust measurements...");

    let mut sample_count: u32 = 0;
    let mut running_avg: f32 = 0.0;

    loop {
        // 1. Turn LED ON (LOW signal due to internal transistor)
        led_pin.set_low();

        // 2. Wait for sensor to stabilize (0.28ms)
        delay.delay_micros(SAMPLING_DELAY_US);

        // 3. Read analog value
        let raw_value: u16 = match nb::block!(adc.read_oneshot(&mut aout_pin)) {
            Ok(v) => v,
            Err(_) => 0,
        };

        // 4. Turn LED OFF (HIGH signal)
        led_pin.set_high();

        // Convert ADC reading to voltage
        let voltage = (raw_value as f32 / ADC_MAX as f32) * ADC_VOLTAGE;

        // Calculate dust density in mg/m³
        // Formula: dust_density = voltage / (0.5V per 0.1mg/m³) * 0.1mg/m³
        // Simplified: dust_density = voltage * 0.2
        let dust_density = voltage * 0.2;

        // Update running average
        sample_count += 1;
        running_avg =
            (running_avg * (sample_count - 1) as f32 + dust_density) / sample_count as f32;

        // Determine air quality level
        let air_quality = if dust_density < 0.035 {
            "Good"
        } else if dust_density < 0.075 {
            "Moderate"
        } else {
            "Poor"
        };

        // Format voltage for display
        let v_int = voltage as u32;
        let v_frac = ((voltage % 1.0) * 1000.0) as u32;

        // Format dust density for display
        let d_int = dust_density as u32;
        let d_frac = ((dust_density % 1.0) * 1000.0) as u32;

        // Format running average for display
        let avg_int = running_avg as u32;
        let avg_frac = ((running_avg % 1.0) * 1000.0) as u32;

        info!("--- Sample {} ---", sample_count);
        info!("Raw ADC:      {}", raw_value);
        info!("Voltage:      {}.{:03} V", v_int, v_frac);
        info!("Dust Density: {}.{:03} mg/m³", d_int, d_frac);
        info!("Avg Density:  {}.{:03} mg/m³", avg_int, avg_frac);
        info!("Air Quality:  {}", air_quality);

        // Reset running average every 100 samples
        if sample_count % 100 == 0 {
            running_avg = 0.0;
            sample_count = 0;
            info!("(Reset running average)");
        }

        // Wait for remainder of 10ms cycle
        let remaining_time_us = (CYCLE_TIME_MS * 1000) - LED_PULSE_WIDTH_US;
        delay.delay_micros(remaining_time_us);
    }
}
