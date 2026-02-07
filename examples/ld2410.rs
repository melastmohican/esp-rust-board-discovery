//! # HLK-LD2410C Human Presence Radar Sensor Example
//!
//! This example demonstrates how to interface with the HLK-LD2410C 24GHz mmWave radar sensor
//! using the `ld2410` crate. It initializes UART at 256000 baud and parses the sensor's
//! data stream to display target status, distance, and energy levels.
//!
//! ## Hardware
//!
//! - **Board:** Rust ESP Board (ESP32-C3-DevKit-RUST-1)
//! - **Sensor:** HLK-LD2410C (24GHz Human Presence Sensing Module)
//!
//! ## Wiring
//!
//! | HLK-LD2410C Pin | Rust ESP Board | Notes |
//! |-----------------|----------------|-------|
//! | VCC (5V)        | 5V / VBUS      | Sensor requires 5V supply |
//! | GND             | GND            | |
//! | TX              | GPIO21 (RX)    | MCU RX connects to Sensor TX |
//! | RX              | GPIO20 (TX)    | MCU TX connects to Sensor RX |
//!
//! ## Protocol Notes
//!
//! - **Baud Rate:** 256000 (standard for LD2410)
//!
//! ## Run
//!
//! ```bash
//! cargo run --example ld2410
//! ```

#![no_std]
#![no_main]

use defmt::{info, println};
use esp_hal::{
    Config as HalConfig, main,
    uart::{Config as UartConfig, Uart},
};
use ld2410::LD2410;
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(HalConfig::default());

    // LD2410 default baud rate is 256000
    let config = UartConfig::default().with_baudrate(256_000);

    let tx = peripherals.GPIO20;
    let rx = peripherals.GPIO21;

    info!("Initializing LD2410 UART at 256000 baud");
    info!("Connect Sensor TX -> GPIO21 and Sensor RX -> GPIO20");

    let uart = Uart::new(peripherals.UART1, config)
        .unwrap()
        .with_tx(tx)
        .with_rx(rx);

    let mut radar = LD2410::new(uart);
    let mut last_dist: Option<u16> = None;

    loop {
        match radar.read_presence() {
            Ok(data) => {
                println!("--- HLK-LD2410C Report ---");

                let state_str = match data.target_state {
                    ld2410::TargetState::NoTarget => "NoTarget",
                    ld2410::TargetState::Moving => "Moving",
                    ld2410::TargetState::Stationary => "Stationary",
                    ld2410::TargetState::Both => "Both",
                };
                println!("Target State: {}", state_str);

                println!(
                    "Moving Target: {} cm (Energy: {})",
                    data.moving_distance_cm, data.moving_energy
                );

                // Calculate approximate speed based on distance change
                if let Some(prev) = last_dist {
                    let diff = if data.moving_distance_cm > prev {
                        data.moving_distance_cm - prev
                    } else {
                        prev - data.moving_distance_cm
                    };

                    if diff > 0 {
                        let direction = if data.moving_distance_cm < prev {
                            "Approaching"
                        } else {
                            "Receding"
                        };
                        println!("Motion: {} (delta: {} cm)", direction, diff);
                    }
                }
                last_dist = Some(data.moving_distance_cm);
                println!(
                    "Static Target: {} cm (Energy: {})",
                    data.still_distance_cm, data.still_energy
                );
                println!("Detection Distance: {} cm", data.detection_distance_cm);
                println!("--------------------------");
            }
            Err(_) => {
                // Ignore errors (timeouts, partial frames)
            }
        }
    }
}
