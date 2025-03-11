#![no_std]

use esp_backtrace as _;
use esp_hal::mcpwm::*;

pub struct Motor<'d, PWM> {
    mot_p1: operator::PwmPin<'d, PWM, 0, true>,
    mot_p2: operator::PwmPin<'d, PWM, 1, true>,
}

impl<'d, PWM: PwmPeripheral> Motor<'d, PWM> {
    pub fn new(
        mut mot_pin_1: operator::PwmPin<'d, PWM, 0, true>,
        mut mot_pin_2: operator::PwmPin<'d, PWM, 1, true>,
    ) -> Self {
        mot_pin_1.set_timestamp(0);
        mot_pin_2.set_timestamp(0);
        Self {
            mot_p1: mot_pin_1,
            mot_p2: mot_pin_2,
        }
    }

    pub fn forward(&mut self) {
        self.mot_p1.set_timestamp(0);
        self.mot_p2.set_timestamp(75);
    }

    pub fn backwards(&mut self) {
        self.mot_p1.set_timestamp(75);
        self.mot_p2.set_timestamp(0);
    }

    pub fn brake(&mut self) {
        self.mot_p1.set_timestamp(75);
        self.mot_p2.set_timestamp(75);
    }

    pub fn coast(&mut self) {
        self.mot_p1.set_timestamp(0);
        self.mot_p2.set_timestamp(0);
    }
}
