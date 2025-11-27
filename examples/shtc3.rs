#![no_std]
#![no_main]
use defmt::info;
use esp_hal::{
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
    main,
};
use panic_rtt_target as _;
use shtcx::{self, PowerMode};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // 1. Instanciate the SDA and SCL pins, correct pins are in the training material.
    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    // 2. Instanciate the i2c peripheral
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    // 3. Create an instance of the SHTC3 sensor.
    let mut sht = shtcx::shtc3(i2c);

    // 4. Read and print the sensor's device ID.
    let device_id = sht.device_identifier().unwrap();
    info!("Device ID SHTC3: {:#02x}", device_id);

    // Initialize delay
    let delay = Delay::new();

    loop {
        // 5. This loop initiates measurements, reads values and prints humidity in % and Temperature in °C.
        sht.start_measurement(PowerMode::NormalMode).unwrap();
        delay.delay_millis(500);
        let measurement = sht.get_measurement_result().unwrap();

        info!(
            "TEMP: {} °C | HUM: {} %",
            measurement.temperature.as_degrees_celsius(),
            measurement.humidity.as_percent(),
        );

        delay.delay_millis(500);
    }
}
