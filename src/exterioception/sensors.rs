#![no_std]

use esp_backtrace as _;
use esp_hal::gpio::{Input, InputConfig, InputPin, Level, Output, OutputConfig, OutputPin, Pull};
use esp_hal::mcpwm::*;

pub struct IRSensor<'d> {
    trigger: Output<'d>,
    left_side: Input<'d>,
    left_front: Input<'d>,
    right_side: Input<'d>,
    right_front: Input<'d>,
}

impl<'d> IRSensor<'d> {
    pub fn new(
        mut trig_pin: impl OutputPin + 'd,
        mut lf_pin: impl InputPin + 'd,
        mut ls_pin: impl InputPin + 'd,
        mut rf_pin: impl InputPin + 'd,
        mut rs_pin: impl InputPin + 'd,
    ) -> Self {
        let in_cfg = InputConfig::default().with_pull(Pull::Down);
        Self {
            trigger: Output::new(trig_pin, Level::Low, OutputConfig::default()),
            left_side: Input::new(ls_pin, in_cfg),
            left_front: Input::new(lf_pin, in_cfg),
            right_side: Input::new(rs_pin, in_cfg),
            right_front: Input::new(rf_pin, in_cfg),
        }
    }
}
