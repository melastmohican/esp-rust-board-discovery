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
//! **Good News:** The Waveshare Dust Sensor breakout board includes all required components
//! onboard (150Ω resistor, 220µF capacitor, voltage regulator, signal conditioning).
//! Simply connect the 4-wire cable directly - no additional components needed!
//!
//! ```text
//! Waveshare Dust Sensor Breakout -> ESP32-C3 Rust Board
//! --------------------------------  ---------------------
//! VCC (red)                      -> 5V/VUSB or 3.3V*
//! GND (black)                    -> GND
//! AOUT (yellow)                  -> GPIO0 (ADC input)
//! ILED (blue)                    -> GPIO1 (digital output)
//! ```
//!
//! **Note:** The Waveshare board has an onboard PT1301 DC/DC converter that provides
//! stable 5V to the Sharp sensor from input voltages as low as 2.5V. You can power it
//! from either 5V/VUSB or 3.3V.
//!
//! **Onboard Components (already included on Waveshare breakout):**
//! - 150Ω resistor for LED current limiting
//! - 220µF capacitor for power stabilization
//! - PT1301 DC/DC converter (2.5V-5.5V input → 5V output)
//! - Transistor Q1 for LED pulse control
//! - Resistor divider R10(10kΩ) + R6(1kΩ) for output voltage scaling
//!
//! **If using the raw Sharp GP2Y1010AU0F sensor (not Waveshare breakout):**
//! You will need to add external 150Ω resistor and 220µF capacitor yourself.
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

// Calibration constants
// The Waveshare board has voltage offset - measure in clean air to calibrate
// Based on typical readings: ~1.35V in clean air
const VOLTAGE_OFFSET: f32 = 1.35; // Clean air baseline voltage (adjust if needed)

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
    info!("Sensor: Sharp GP2Y1010AU0F (Waveshare breakout)");
    info!("Measurement: PM2.5 and PM10 dust particles");
    info!("Timing: LED pulse 0.32ms, sample at 0.28ms, 10ms cycle");
    info!("");
    info!("Waveshare board includes all components onboard:");
    info!("- 150 Ohm resistor, 220 uF capacitor");
    info!("- PT1301 DC/DC converter (2.5V-5.5V input)");
    info!("- Signal conditioning circuitry");
    info!("");
    info!(
        "Calibration: Voltage offset = {} V (clean air baseline)",
        VOLTAGE_OFFSET as u32
    );
    info!("Note: If readings seem off, adjust VOLTAGE_OFFSET in code");
    info!("      to match your sensor's clean air voltage reading");
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
        let raw_value: u16 = nb::block!(adc.read_oneshot(&mut aout_pin)).unwrap_or_default();

        // 4. Turn LED OFF (HIGH signal)
        led_pin.set_high();

        // Convert ADC reading to voltage
        let voltage = (raw_value as f32 / ADC_MAX as f32) * ADC_VOLTAGE;

        // Subtract baseline voltage to get dust signal
        let voltage_dust = if voltage > VOLTAGE_OFFSET {
            voltage - VOLTAGE_OFFSET
        } else {
            0.0
        };

        // Calculate dust density in mg/m³
        // Formula from datasheet: sensitivity = 0.5V per 0.1mg/m³
        // Therefore: dust_density = (voltage_dust / 0.5V) * 0.1mg/m³
        // Simplified: dust_density = voltage_dust * 0.2
        let dust_density = voltage_dust * 0.2;

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
        if sample_count.is_multiple_of(100) {
            running_avg = 0.0;
            sample_count = 0;
            info!("(Reset running average)");
        }

        // Wait for remainder of 10ms cycle
        let remaining_time_us = (CYCLE_TIME_MS * 1000) - LED_PULSE_WIDTH_US;
        delay.delay_micros(remaining_time_us);
    }
}
