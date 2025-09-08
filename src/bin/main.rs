#![no_std]
#![no_main]

use core::any::type_name;

use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_futures::{join::join, select::select};
use embassy_time::Timer;

use static_cell::StaticCell;
use trouble_host::prelude::*;
use trouble_host::Controller;

use esp_alloc;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output};
use esp_hal::main;
use esp_hal::mcpwm::*;
use esp_hal::time::RateExtU32;
use esp_hal::{rng::Rng, timer::timg::TimerGroup};
use esp_wifi::{ble::controller::BleConnector, init};
use log::{info, warn};

use mouse::*;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.2.2

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(72 * 1024);

    let systimer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    static RADIO: StaticCell<esp_wifi::EspWifiController<'static>> = StaticCell::new();
    let radio_tmp = init(
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .expect("Error getting wifi");
    let radio = RADIO.init(radio_tmp);

    // pins for the motors
    let mot_l1 = Output::new(peripherals.GPIO14, Level::Low);
    let mot_l2 = Output::new(peripherals.GPIO15, Level::Low);

    let mot_r1 = Output::new(peripherals.GPIO16, Level::Low);
    let mot_r2 = Output::new(peripherals.GPIO17, Level::Low);

    // clock for motors
    let clk_cfg = PeripheralClockConfig::with_frequency(32.MHz()).unwrap();
    let mut r_mot_ctrl = McPwm::new(peripherals.MCPWM0, clk_cfg);
    let mut l_mot_ctrl = McPwm::new(peripherals.MCPWM1, clk_cfg);

    // set the operator for the pwm for the right motor
    let mot_ra = r_mot_ctrl
        .operator0
        .with_pin_a(mot_r1, operator::PwmPinConfig::UP_ACTIVE_HIGH);
    let mot_rb = r_mot_ctrl
        .operator1
        .with_pin_a(mot_r2, operator::PwmPinConfig::UP_ACTIVE_HIGH);

    // set the operator for the pwm for the left motor
    let mot_la = l_mot_ctrl
        .operator0
        .with_pin_a(mot_l1, operator::PwmPinConfig::UP_ACTIVE_HIGH);
    let mot_lb = l_mot_ctrl
        .operator1
        .with_pin_a(mot_l2, operator::PwmPinConfig::UP_ACTIVE_HIGH);

    // set timer0 for the pwm & start it
    let timer_clock_cfg = clk_cfg
        .timer_clock_with_frequency(99, timer::PwmWorkingMode::Increase, 20.kHz())
        .unwrap();
    r_mot_ctrl.timer0.start(timer_clock_cfg);
    l_mot_ctrl.timer0.start(timer_clock_cfg);

    let mut mot_r = Motor::new(mot_ra, mot_rb, peripherals.GPIO47, peripherals.GPIO33);
    let mut mot_l = Motor::new(mot_la, mot_lb, peripherals.GPIO21, peripherals.GPIO26);

    spawner.spawn(drive(mot_r, mot_l));

    let connector = BleConnector::new(radio, peripherals.BT);
    let controller: ExternalController<_, 20> = ExternalController::new(connector);

    ble_peripheral_run(controller).await;
}

fn print_type_of<T>(_: &T) {
    info!("{}", type_name::<T>());
}

#[embassy_executor::task]
async fn drive(
    mut m1: Motor<'static, esp_hal::peripherals::MCPWM0>,
    mut m2: Motor<'static, esp_hal::peripherals::MCPWM1>,
) {
    m1.forward();
    m2.backwards();
    loop {
        Timer::after_secs(1).await;
    }
}

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;
/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att

// GATT Server definition
#[gatt_server()]
struct Server {
    battery_service: BatteryService,
}

/// Battery service
#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    /// Battery Level
    #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [0, 100])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, name = "hello", read, value = "Battery Level")]
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 10)]
    level: u8,
    #[characteristic(uuid = "408813df-5dd4-1f87-ec11-cdb001100000", write, read, notify)]
    status: bool,
}

pub async fn ble_peripheral_run<C>(controller: C)
where
    C: Controller,
{
    let address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    info!("Our Address: {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Trouble",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();

    let _ = join(ble_task(runner), async {
        loop {
            match advertise("Trouble Example", &mut peripheral, &server).await {
                Ok(conn) => {
                    // set up tasks when the connection is established to a central, so they don't run when no one is connected.
                    let a = gatt_events_task(&server, &conn);
                    let b = custom_task(&server, &conn, &stack);
                    // run until any task ends (usually because the connection has been closed),
                    // then return to advertising state.
                    select(a, b).await;
                }
                Err(e) => {
                    panic!("[adv] error: {:?}", e);
                }
            }
        }
    })
    .await;
}

async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[[0x0f, 0x18]]),
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    info!("[adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("[adv] connection established");
    Ok(conn)
}

async fn custom_task<C: Controller, P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    stack: &Stack<'_, C, P>,
) {
    let mut tick: u8 = 0;
    let level = server.battery_service.level;
    loop {
        tick = tick.wrapping_add(1);
        info!("[custom_task] notifying connection of tick {}", tick);
        if level.notify(conn, &tick).await.is_err() {
            info!("[custom_task] error notifying connection");
            break;
        };
        // read RSSI (Received Signal Strength Indicator) of the connection.
        if let Ok(rssi) = conn.raw().rssi(stack).await {
            info!("[custom_task] RSSI: {:?}", rssi);
        } else {
            info!("[custom_task] error getting RSSI");
            break;
        };
        Timer::after_secs(2).await;
    }
}

async fn gatt_events_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
) -> Result<(), Error> {
    let level = server.battery_service.level;
    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            GattConnectionEvent::Gatt { event } => {
                match &event {
                    GattEvent::Read(event) => {
                        if event.handle() == level.handle {
                            let value = server.get(&level);
                            info!("[gatt] Read Event to Level Characteristic: {:?}", value);
                        }
                    }
                    GattEvent::Write(event) => {
                        if event.handle() == level.handle {
                            info!(
                                "[gatt] Write Event to Level Characteristic: {:?}",
                                event.data()
                            );
                        }
                    }
                    _ => {}
                };
                // This step is also performed at drop(), but writing it explicitly is necessary
                // in order to ensure reply is sent.
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => warn!("[gatt] error sending response: {:?}", e),
                };
            }
            _ => {} // ignore other Gatt Connection Events
        }
    };
    info!("[gatt] disconnected: {:?}", reason);
    Ok(())
}

async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            panic!("[ble_task] error: {:?}", e);
        }
    }
}
