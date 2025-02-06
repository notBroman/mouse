#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output};
use esp_hal::main;
use esp_hal::mcpwm::*;
use esp_hal::time::RateExtU32;
use log::info;

extern crate alloc;

pub struct Motor<'a, PWM>
where
    PWM: PwmPeripheral,
{
    mot_ctrl: McPwm<'a, PWM>,
    mot_pin_a: operator::PwmPin<'a, PWM, u8, bool>,
    mot_pin_b: operator::PwmPin<'a, PWM, u8, bool>,
}

impl<'a, PWM> Motor<'a, PWM>
where
    PWM: esp_hal::mcpwm::PwmPeripheral,
{
    pub fn new(
        mut mcpwm: McPwm<'a, PWM>,
        clk_cfg: PeripheralClockConfig,
        mot_r1: Output,
        mot_r2: Output,
    ) -> Self {
        // set the operator for the pwm
        let mut mot_ra = mcpwm
            .operator0
            .with_pin_a(mot_r1, operator::PwmPinConfig::UP_ACTIVE_HIGH);

        let mut mot_rb = mcpwm
            .operator1
            .with_pin_a(mot_r2, operator::PwmPinConfig::UP_ACTIVE_HIGH);

        // set timer0 for the pwm & start it
        let timer_clock_cfg = clk_cfg
            .timer_clock_with_frequency(99, timer::PwmWorkingMode::Increase, 20.kHz())
            .unwrap();
        mcpwm.timer0.start(timer_clock_cfg);
        Self {
            mot_ctrl: mcpwm,
            mot_pin_a: mot_ra,
            mot_pin_b: mot_rb,
        }
    }
}

#[main]
fn main() -> ! {
    // generator version: 0.2.2

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::logger::init_logger_from_env();

    esp_alloc::heap_allocator!(72 * 1024);

    // pins for the motors
    let mot_l1 = Output::new(peripherals.GPIO14, Level::Low);
    let mot_l2 = Output::new(peripherals.GPIO15, Level::Low);

    let mot_r1 = Output::new(peripherals.GPIO16, Level::Low);
    let mot_r2 = Output::new(peripherals.GPIO17, Level::Low);

    // clock for motors
    let clk_cfg = PeripheralClockConfig::with_frequency(32.MHz()).unwrap();
    let mut mot_ctrl = McPwm::new(peripherals.MCPWM0, clk_cfg);

    // config the timers
    mot_ra.set_timestamp(70);
    mot_rb.set_timestamp(0);

    let delay = Delay::new();
    loop {
        info!("Hello world!");
        delay.delay_millis(500);
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}
