#![no_std]

use esp_hal::gpio::OutputPin;
use esp_hal::i2c::master::{Config, I2c, Instance};
use esp_hal::peripheral::Peripheral;
use esp_hal::DriverMode;
use esp_hal::{Async, Blocking};

// implementation of the I2C based MPU6500
// Addr: 0xD1
//  SDA: IO4 / pin 8
//  SCL: IO5 / pin 9
struct IMU<'d, Dm: DriverMode> {
    DEVICE_ADDR: u8,
    i2c: I2c<'d, Dm>,
}

impl<'d> IMU<'d, Async> {
    pub fn new(
        SDA: impl Peripheral<P = impl OutputPin> + 'd,
        SCL: impl Peripheral<P = impl OutputPin> + 'd,
        i2c_peripheral: impl Peripheral<P = impl Instance> + 'd,
    ) -> Self {
        let mut i2c_master = I2c::new(i2c_peripheral, Config::default())
            .expect("Could not create i2c")
            .with_sda(SDA)
            .with_scl(SCL)
            .into_async();

        let imu = Self {
            DEVICE_ADDR: 0xD1,
            i2c: i2c_master,
        };

        imu.init();

        imu
    }

    pub fn init(&self) {
        todo!();
    }
}

impl<'d, Dm: DriverMode> IMU<'d, Dm> {}
