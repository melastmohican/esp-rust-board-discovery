//! HC-SR501 PIR Motion Sensor Example
//!
//! This example demonstrates how to interface with an HC-SR501 PIR motion sensor.
//! It tracks the motion state and logs "Motion detected!" and "Motion ended!"
//! on transitions. It also controls the onboard RGB LED (GPIO2) based on the sensor state.
//!
//! Wiring:
//! HC-SR501 Pin -> Rust ESP Board (ESP32-C3)
//! ----------    --------------
//! VCC          -> 5V / VBUS
//! GND          -> GND
//! OUT          -> GPIO1

#![no_std]
#![no_main]

use defmt::info;
use esp_hal::{
    delay::Delay,
    gpio::{Input, InputConfig, Pull},
    main,
    rmt::Rmt,
    time::Rate,
};
use esp_hal_smartled::{SmartLedsAdapter, smart_led_buffer};
use panic_rtt_target as _;
use smart_leds::{RGB8, SmartLedsWrite, brightness};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("HC-SR501 test started");

    // Configure RMT peripheral for onboard RGB LED (GPIO2)
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).expect("Failed to initialize RMT");
    let rmt_channel = rmt.channel0;
    let mut rmt_buffer = smart_led_buffer!(1);
    let mut led = SmartLedsAdapter::new(rmt_channel, peripherals.GPIO2, &mut rmt_buffer);

    // Configure the HC-SR501 OUT pin as input on GPIO1
    let pir_input = Input::new(
        peripherals.GPIO1,
        InputConfig::default().with_pull(Pull::Down),
    );

    let delay = Delay::new();
    let mut pir_state = false;
    let brightness_level = 10; // Low brightness

    loop {
        // Read the sensor state
        let val = pir_input.is_high();

        if val {
            // Motion detected - Set LED to Red
            let color = RGB8 { r: 255, g: 0, b: 0 };
            led.write(brightness([color].into_iter(), brightness_level))
                .unwrap();

            if !pir_state {
                info!("Motion detected!");
                pir_state = true;
            }
        } else {
            // No motion - Turn LED off
            let color = RGB8 { r: 0, g: 0, b: 0 };
            led.write(brightness([color].into_iter(), brightness_level))
                .unwrap();

            if pir_state {
                info!("Motion ended!");
                pir_state = false;
            }
        }

        // Small delay to avoid tight loop
        delay.delay_millis(100);
    }
}
