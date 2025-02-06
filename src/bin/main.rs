#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputPin};
use esp_hal::main;
use esp_hal::mcpwm::*;
use esp_hal::peripheral::Peripheral;
use esp_hal::time::RateExtU32;
use log::info;

extern crate alloc;

pub struct Motor<'d, PWM> {
    mot_ctrl: McPwm<'d, PWM>,
    mot_p1: operator::PwmPin<'d, PWM, 0, true>,
    mot_p2: operator::PwmPin<'d, PWM, 1, true>,
}

impl<'d, PWM: PwmPeripheral> Motor<'d, PWM> {
    pub fn new(
        pwm_peripheral: impl Peripheral<P = PWM> + 'd,
        mot_pin_1: impl Peripheral<P = impl OutputPin> + 'd,
        mot_pin_2: impl Peripheral<P = impl OutputPin> + 'd,
    ) -> Self {
        todo!();
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

    let delay = Delay::new();
    loop {
        info!("Hello world!");
        delay.delay_millis(500);
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}
