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
