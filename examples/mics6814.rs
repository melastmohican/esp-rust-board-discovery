//! # MICS6814 Analog Gas Sensor Example (Enviro+ FeatherWing)
//!
//! Reads three analog gas channels (oxidising, reducing, NH3) from the
//! MICS6814 on the Pimoroni Enviro+ FeatherWing and prints resistance
//! estimates via RTT/defmt.
//!
//! This example targets the Rust ESP Board (ESP32-C3-DevKit-RUST-1).
//!
//! Wiring / pin mapping (editable constants below):
//! - `ENABLE_PIN` : digital output that enables the MICS6814 heater (if wired)
//! - `OX_PIN` : analog channel for oxidising sensor
//! - `RED_PIN` : analog channel for reducing sensor
//! - `NH3_PIN` : analog channel for NH3 sensor
//!
//! Pimoroni Enviro+ FeatherWing pin mapping (Feather pin numbers => MCU nets):
//! - `Pimoroni pin5`  -> `NH3_SENSE` -> MCU label `GPIOA0`
//! - `Pimoroni pin6`  -> `RED_SENSE` -> MCU label `GPIOA1`
//! - `Pimoroni pin7`  -> `OX_SENSE`  -> MCU label `GPIOA2`
//!
//! Note: the `GPIOA*` labels are Feather/SAMD board net names. On ESP32 boards
//! use the `peripherals.GPIOx` pins (the example defaults are set for an
//! ESP32-C3 dev board).
//!
//! The conversion formula mirrors Pimoroni's Python implementation:
//! R = 56000 / ((ADC_MAX / raw) - 1)
//!
//! Run with:
//! ```bash
//! cargo run --example mics6814
//! ```

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

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("Initializing MICS6814 example");

    let delay = Delay::new();

    let mut en = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
    en.set_high();
    info!("Enable pin set high (GPIO4)");

    let mut adc_config = AdcConfig::new();

    let mut pin_ox = adc_config.enable_pin(peripherals.GPIO2, Attenuation::_11dB);
    let mut pin_red = adc_config.enable_pin(peripherals.GPIO1, Attenuation::_11dB);
    let mut pin_nh3 = adc_config.enable_pin(peripherals.GPIO0, Attenuation::_11dB);

    let mut adc = Adc::new(peripherals.ADC1, adc_config);
    fn adc_to_resistance(raw: u32, adc_max: u32) -> Option<f32> {
        if raw == 0 || raw >= adc_max {
            // Match Pimoroni Python: on out-of-range or division-by-zero,
            // return numeric zero instead of None.
            return Some(0.0);
        }
        let denom = (adc_max as f32) / (raw as f32) - 1.0;
        if denom <= 0.0 {
            return Some(0.0);
        }
        Some(56000.0 / denom)
    }

    info!("Starting readings...");

    // Use 12-bit ADC scale by default for ESP targets.
    // If you're using a platform or HAL that provides 16-bit ADC values
    // (Pimoroni Python uses 65535), change `adc_max` to `65535u32` below.
    let adc_max: u32 = 4095u32;

    loop {
        let raw_ox_u16: u16 = match nb::block!(adc.read_oneshot(&mut pin_ox)) {
            Ok(v) => v,
            Err(_) => 0,
        };
        let raw_red_u16: u16 = match nb::block!(adc.read_oneshot(&mut pin_red)) {
            Ok(v) => v,
            Err(_) => 0,
        };
        let raw_nh3_u16: u16 = match nb::block!(adc.read_oneshot(&mut pin_nh3)) {
            Ok(v) => v,
            Err(_) => 0,
        };

        let raw_ox: u32 = raw_ox_u16 as u32;
        let raw_red: u32 = raw_red_u16 as u32;
        let raw_nh3: u32 = raw_nh3_u16 as u32;

        let r_ox = adc_to_resistance(raw_ox, adc_max);
        let r_red = adc_to_resistance(raw_red, adc_max);
        let r_nh3 = adc_to_resistance(raw_nh3, adc_max);

        let v_ox = (raw_ox as f32) / (adc_max as f32) * 3.3;
        let v_red = (raw_red as f32) / (adc_max as f32) * 3.3;
        let v_nh3 = (raw_nh3 as f32) / (adc_max as f32) * 3.3;

        let r = r_ox.unwrap_or(0.0);
        let ox_int = r as i32;
        let ox_frac = (((r.abs() % 1.0) * 1000.0) as u32).min(999);
        let v_ox_int = v_ox as u32;
        let v_ox_frac = (((v_ox % 1.0) * 100.0) as u32).min(99);
        info!(
            "OX raw={} V={}.{:02}V Oxidising: {}.{:03} Ohms",
            raw_ox, v_ox_int, v_ox_frac, ox_int, ox_frac
        );

        let r = r_red.unwrap_or(0.0);
        let red_int = r as i32;
        let red_frac = (((r.abs() % 1.0) * 1000.0) as u32).min(999);
        let v_red_int = v_red as u32;
        let v_red_frac = (((v_red % 1.0) * 100.0) as u32).min(99);
        info!(
            "RED raw={} V={}.{:02}V Reducing:  {}.{:03} Ohms",
            raw_red, v_red_int, v_red_frac, red_int, red_frac
        );

        let r = r_nh3.unwrap_or(0.0);
        let nh3_int = r as i32;
        let nh3_frac = (((r.abs() % 1.0) * 1000.0) as u32).min(999);
        let v_nh3_int = v_nh3 as u32;
        let v_nh3_frac = (((v_nh3 % 1.0) * 100.0) as u32).min(99);
        info!(
            "NH3 raw={} V={}.{:02}V NH3:       {}.{:03} Ohms",
            raw_nh3, v_nh3_int, v_nh3_frac, nh3_int, nh3_frac
        );

        delay.delay_millis(1000u32);
    }
}
