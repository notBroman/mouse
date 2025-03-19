#![no_std]

use esp_backtrace as _;
use esp_hal::gpio::{Input, InputPin, Level, Output, OutputPin, Pull};
use esp_hal::mcpwm::*;
use esp_hal::peripheral::Peripheral;

pub struct IRSensor<'d> {
    trigger: Output<'d>,
    left_side: Input<'d>,
    left_front: Input<'d>,
    right_side: Input<'d>,
    right_front: Input<'d>,
}

impl<'d> IRSensor<'d> {
    pub fn new(
        mut trig_pin: impl Peripheral<P = impl OutputPin> + 'd,
        mut lf_pin: impl Peripheral<P = impl InputPin> + 'd,
        mut ls_pin: impl Peripheral<P = impl InputPin> + 'd,
        mut rf_pin: impl Peripheral<P = impl InputPin> + 'd,
        mut rs_pin: impl Peripheral<P = impl InputPin> + 'd,
    ) -> Self {
        Self {
            trigger: Output::new(trig_pin, Level::Low),
            left_side: Input::new(ls_pin, Pull::Down),
            left_front: Input::new(lf_pin, Pull::Down),
            right_side: Input::new(rs_pin, Pull::Down),
            right_front: Input::new(rf_pin, Pull::Down),
        }
    }
}

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
