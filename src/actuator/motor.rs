#![no_std]

use esp_backtrace as _;
use esp_hal::gpio::{Input, InputPin, Pull};
use esp_hal::mcpwm::*;
use esp_hal::peripheral::Peripheral;

pub struct Motor<'d, PWM> {
    // pin 1 & 2 for the motor
    mot_p1: operator::PwmPin<'d, PWM, 0, true>,
    mot_p2: operator::PwmPin<'d, PWM, 1, true>,
    // two hall effect sensors to identify turning direction
    hal1_p1: Input<'d>,
    hal2_p1: Input<'d>,
}

impl<'d, PWM: PwmPeripheral> Motor<'d, PWM> {
    pub fn new(
        mut mot_pin_1: operator::PwmPin<'d, PWM, 0, true>,
        mut mot_pin_2: operator::PwmPin<'d, PWM, 1, true>,
        mut hal1_mot1_pin: impl Peripheral<P = impl InputPin> + 'd,
        mut hal2_mot1_pin: impl Peripheral<P = impl InputPin> + 'd,
    ) -> Self {
        mot_pin_1.set_timestamp(0);
        mot_pin_2.set_timestamp(0);
        let hal1_mot1 = Input::new(hal1_mot1_pin, Pull::None);
        let hal2_mot1 = Input::new(hal2_mot1_pin, Pull::None);

        Self {
            mot_p1: mot_pin_1,
            mot_p2: mot_pin_2,
            hal1_p1: hal1_mot1,
            hal2_p1: hal2_mot1,
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
