#![no_std]

use esp_backtrace as _;
use esp_hal::gpio::{Level, Output, OutputConfig, OutputPin};
use esp_hal::mcpwm::*;
use esp_hal::time::Rate;

pub struct Motor<'d, PWM>
where
    PWM: PwmPeripheral + 'd,
{
    // pin 1 & 2 for the motor
    mot_p1: operator::PwmPin<'d, PWM, 0, true>,
    mot_p2: operator::PwmPin<'d, PWM, 1, true>,
}

impl<'d, PWM> Motor<'d, PWM>
where
    PWM: PwmPeripheral + 'd,
{
    pub fn new(
        mut mot_pin_1: impl OutputPin + 'd,
        mut mot_pin_2: impl OutputPin + 'd,
        mut mcpwm_peripheral: PWM,
    ) -> Self {
        // create pins for motor
        let mot_p1 = Output::new(mot_pin_1, Level::Low, OutputConfig::default());
        let mot_p2 = Output::new(mot_pin_2, Level::Low, OutputConfig::default());

        // cofigure the clock and create mcpwm from peripheral
        let clk_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(32)).unwrap();
        let mut mot_ctrl = McPwm::new(mcpwm_peripheral, clk_cfg);

        // set the operators for the motor
        let mut mot_a = mot_ctrl
            .operator0
            .with_pin_a(mot_p1, operator::PwmPinConfig::UP_ACTIVE_HIGH);
        let mut mot_b = mot_ctrl
            .operator1
            .with_pin_a(mot_p2, operator::PwmPinConfig::UP_ACTIVE_HIGH);

        // set timer0 for the pwm & start it
        let timer_clock_cfg = clk_cfg
            .timer_clock_with_frequency(99, timer::PwmWorkingMode::Increase, Rate::from_khz(20))
            .unwrap();
        mot_ctrl.timer0.start(timer_clock_cfg);

        mot_a.set_timestamp(0);
        mot_b.set_timestamp(0);

        Self {
            mot_p1: mot_a,
            mot_p2: mot_b,
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
