#![no_std]
#![no_main]
use defmt::info;
use esp_hal::{
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
    main,
};
use panic_rtt_target as _;

use embedded_hal_bus::i2c::RefCellDevice;
use icm42670::{Address, Icm42670, PowerMode as imuPowerMode};
use shtcx::{self, PowerMode as shtPowerMode};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let sda = peripherals.GPIO10;
    let scl = peripherals.GPIO8;

    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    let i2c_ref_cell = core::cell::RefCell::new(i2c);

    let i2c_dev1 = RefCellDevice::new(&i2c_ref_cell);
    let i2c_dev2 = RefCellDevice::new(&i2c_ref_cell);

    let mut sht = shtcx::shtc3(i2c_dev1);

    let device_id = sht.device_identifier().unwrap();
    info!("Device ID SHTC3: {:#02x}", device_id);

    let mut imu = Icm42670::new(i2c_dev2, Address::Primary).unwrap();

    let device_id = imu.device_id().unwrap();
    info!("Device ID ICM42670p: {:#02x}", device_id);

    imu.set_power_mode(imuPowerMode::GyroLowNoise).unwrap();

    let delay = Delay::new();

    loop {
        let gyro_data = imu.gyro_norm().unwrap();
        sht.start_measurement(shtPowerMode::NormalMode).unwrap();
        delay.delay_millis(100);
        let measurement = sht.get_measurement_result().unwrap();

        info!(
            "TEMP: {} °C | HUM: {} % | GYRO: X= {}  Y= {}  Z= {}",
            measurement.temperature.as_degrees_celsius(),
            measurement.humidity.as_percent(),
            gyro_data.x,
            gyro_data.y,
            gyro_data.z,
        );

        delay.delay_millis(500);
    }
}
