#![no_std]
#![no_main]

use core::any::type_name;

use embassy_executor::Spawner;
use embassy_futures::{join::join, select::select};
use embassy_time::Timer;

use esp_alloc;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output};
use esp_hal::main;
use esp_hal::mcpwm::*;
use esp_hal::{rng::Rng, timer::timg::TimerGroup};
use log::info;

mod actuator;
use actuator::motor::Motor;

mod exterioception;
mod proprioception;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(72 * 1024);

    let systimer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    // create motors from pins
    let mut mot_r = Motor::new(peripherals.GPIO16, peripherals.GPIO17, peripherals.MCPWM0);
    let mut mot_l = Motor::new(peripherals.GPIO14, peripherals.GPIO14, peripherals.MCPWM1);

    //let _ = spawner.spawn(run());
    let _ = spawner.spawn(drive(mot_r, mot_l));
    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}

fn print_type_of<T>(_: &T) {
    info!("{}", type_name::<T>());
}

#[embassy_executor::task]
async fn run() {
    loop {
        info!("Hello World!");
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn drive(
    mut m1: Motor<'static, esp_hal::peripherals::MCPWM0<'static>>,
    mut m2: Motor<'static, esp_hal::peripherals::MCPWM1<'static>>,
) {
    m1.forward();
    m2.backwards();
    loop {
        Timer::after_secs(1).await;
    }
}
