#![no_std]

use esp_hal::gpio::{Input, InputConfig, InputPin, Pull};

pub struct MotEncoder<'d> {
    // two hall effect sensors to identify turning direction
    hal1_p1: Input<'d>,
    hal2_p1: Input<'d>,
    current_speed: i32,
}

impl<'d> MotEncoder<'d> {
    pub fn new(
        mut hal1_mot1_pin: impl InputPin + 'd,
        mut hal2_mot1_pin: impl InputPin + 'd,
    ) -> Self {
        todo!("Check which way the hall effect sensor pulls the signal");
        let in_cfg = InputConfig::default();
        let hal1_mot1 = Input::new(hal1_mot1_pin, in_cfg);
        let hal2_mot1 = Input::new(hal2_mot1_pin, in_cfg);
        Self {
            hal1_p1: hal1_mot1,
            hal2_p1: hal2_mot1,
            current_speed: 0,
        }
    }

    pub fn get_direction() {
        todo!("implement telling direction");
    }

    pub fn get_speed() {
        todo!("implement getting speed");
    }
}
