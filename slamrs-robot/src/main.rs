#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};

// use embassy_executor::_export::StaticCell;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config, IpListenEndpoint, Stack, StackResources};

use embassy_sync::channel::{Channel, Receiver, Sender};
use embedded_io_async::Write;
use esp32_hal as hal;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState};
use esp_wifi::{initialize, EspWifiInitFor};
use hal::clock::{ClockControl, CpuClock};
use hal::gpio::Unknown;
use hal::gpio::{AnyPin, GpioPin, Output, PushPull};
use hal::mcpwm::operator::PwmPinConfig;
use hal::mcpwm::timer::PwmWorkingMode;
use hal::mcpwm::{PeripheralClockConfig, MCPWM};
use hal::peripherals::{Interrupt, UART2};
use hal::uart::config::{DataBits, Parity, StopBits};
use hal::uart::{self, TxRxPins, Uart};
use hal::{embassy, peripherals::Peripherals, prelude::*, timer::TimerGroup, Rtc};
use hal::{interrupt, Rng, IO};

use slamrs_message::{CommandMessage, RobotMessage, ScanFrame};
use smoltcp::socket::tcp;
use static_cell::make_static;

type NeatoPwmPin = esp32_hal::mcpwm::operator::PwmPin<
    'static,
    GpioPin<Unknown, 27>,
    esp32_hal::peripherals::MCPWM0,
    0,
    true,
>;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

static LAST_RPM: AtomicU16 = AtomicU16::new(0);
static MOTOR_ON: AtomicBool = AtomicBool::new(false);

static CHANNEL: Channel<CriticalSectionRawMutex, ScanFrame, 10> = Channel::new();

// static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    // esp_wifi::init_heap();

    let peripherals = Peripherals::take();

    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    rtc.rwdt.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = io.pins.gpio2.into_push_pull_output().degrade();
    led.set_high().unwrap();

    let timer = TimerGroup::new(peripherals.TIMG1, &clocks).timer0;

    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiStaDevice).unwrap();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);

    embassy::init(&clocks, timer_group0.timer0);

    let config = Config::dhcpv4(Default::default());

    let seed = 1234; // very random, very secure seed

    // Init network stack
    let stack = &*make_static!(Stack::new(
        wifi_interface,
        config,
        make_static!(StackResources::<3>::new()),
        seed
    ));

    // setup serial port to talk to the Neato Lidar
    let pins = TxRxPins::new_tx_rx(
        io.pins.gpio17.into_push_pull_output(),
        io.pins.gpio16.into_floating_input(),
    );
    // setup serial port
    let config = uart::config::Config {
        baudrate: 115200,
        data_bits: DataBits::DataBits8,
        parity: Parity::ParityNone,
        stop_bits: StopBits::STOP1,
    };

    let mut uart2 = Uart::new_with_config(peripherals.UART2, config, Some(pins), &clocks);
    uart2
        .set_rx_fifo_full_threshold(32)
        .expect("Could not set rx fifo full threshold");

    //////////////////
    // initialize PWM motor controlling pin (GPIO 27)
    let pin = io.pins.gpio27;

    // initialize peripheral
    let clock_cfg = PeripheralClockConfig::with_frequency(&clocks, 40u32.MHz()).unwrap();
    let mut mcpwm = MCPWM::new(peripherals.MCPWM0, clock_cfg);

    // connect operator0 to timer0
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    // connect operator0 to pin
    let pwm_pin = mcpwm
        .operator0
        .with_pin_a(pin, PwmPinConfig::UP_ACTIVE_HIGH);

    interrupt::enable(Interrupt::UART2, interrupt::Priority::Priority3).unwrap();

    // start timer with timestamp values in the range of 0..=99 and a frequency of
    // 20 kHz
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, 20u32.kHz())
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    // channel to share messages

    let sender = CHANNEL.sender();
    let receiver = CHANNEL.receiver();

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(&stack)).ok();
    spawner.spawn(task(&stack, led, receiver)).ok();
    spawner.spawn(neato_serial_read(uart2, sender)).ok();
    spawner.spawn(motor_control(pwm_pin)).ok();

    loop {
        Timer::after(Duration::from_millis(1000)).await
    }
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    println!("start connection task");
    println!("Device capabilities: {:?}", controller.get_capabilities());
    loop {
        match esp_wifi::wifi::get_wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.into(),
                password: PASSWORD.into(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            println!("Starting wifi");
            controller.start().await.unwrap();
            println!("Wifi started!");
        }
        println!("About to connect...");

        match controller.connect().await {
            Ok(_) => println!("Wifi connected!"),
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    stack.run().await
}

async fn connected_loop(
    socket: &mut TcpSocket<'_>,
    receiver: &mut Receiver<'static, CriticalSectionRawMutex, ScanFrame, 10>,
) {
    let mut start = Instant::now();

    // let mut rx_buffer = [0u8; 2048];
    let mut tx_buffer = [0u8; 2048];

    while socket.state() == tcp::State::Established {
        // read incoming messages (to enable or disable the motor) (no bincode here for now)
        if socket.can_recv() {
            let mut buffer = [0; 16];
            // TODO: make the read less blocking?
            if let Ok(len) = socket.read(&mut buffer).await {
                // let (cmd, len): (CommandMessage, usize) =
                //     bincode::decode_from_slice(&rx_buffer[..], bincode::config::standard())
                //         .expect("Could not parse");

                // println!("{cmd:?}");

                if len > 0 {
                    match buffer[0] {
                        b'A' => {
                            MOTOR_ON.store(true, Ordering::Relaxed);
                            LAST_RPM.store(0, Ordering::Relaxed);
                        }
                        b'D' => {
                            MOTOR_ON.store(false, Ordering::Relaxed);
                        }
                        _ => {}
                    }
                }
            }
        }

        // ping message to keep the connection alive every second
        let now = Instant::now();
        if (now - start) > Duration::from_millis(1000) {
            start = now;

            if let Ok(len) = bincode::encode_into_slice(
                RobotMessage::Pong,
                &mut tx_buffer,
                bincode::config::standard(),
            ) {
                socket.write_all(&tx_buffer[0..len]).await.ok();
            }
        }

        // process any parsed packets and send them via the socket
        if let Ok(packet) = receiver.try_receive() {
            // println!("Sending: {:?}", &packet);
            if let Ok(len) = bincode::encode_into_slice(
                RobotMessage::ScanFrame(packet),
                &mut tx_buffer,
                bincode::config::standard(),
            ) {
                socket.write_all(&tx_buffer[0..len]).await.ok();
            }
        }

        Timer::after(Duration::from_millis(50)).await;
    }

    MOTOR_ON.store(false, Ordering::Relaxed);
}

/// Task that always reads the current PWM from the neato lidar,
#[embassy_executor::task]
async fn neato_serial_read(
    mut neato: Uart<'static, UART2>,
    sender: Sender<'static, CriticalSectionRawMutex, ScanFrame, 10>,
) {
    // to hold the whole packet
    let mut buffer = [0u8; 1980];
    let mut state = State::LookingForStart;
    let mut rpm_accumulator = 0i32;
    let mut rpm_average = 0i32;
    let mut last_byte = 0u8;

    let mut buff = [0u8; 512];

    loop {
        while let Ok(len) = embedded_io_async::Read::read(&mut neato, &mut buff).await {
            for i in 0..len {
                let read = buff[i];

                match state {
                    State::LookingForStart => {
                        if last_byte == 0xFA && read == 0xA0 {
                            buffer[0] = last_byte;
                            buffer[1] = read;
                            state = State::CollectingBytes { index: 2 };
                        }
                    }
                    State::CollectingBytes { index } => {
                        buffer[index] = read;

                        if index < buffer.len() - 1 {
                            state = State::CollectingBytes { index: index + 1 };
                        } else {
                            state = State::ParseRPM;
                        }
                    }
                    State::ParseRPM => {
                        let rpm_low = buffer[2];
                        let rpm_high = buffer[3];

                        let rpm = ((rpm_high as u16) << 8) | (rpm_low as u16);

                        // some exponential smoothing
                        rpm_accumulator += rpm as i32 - rpm_average as i32;
                        rpm_average = rpm_accumulator >> 2;

                        let rpm = (rpm_average / 64) as u16;

                        LAST_RPM.store(rpm, core::sync::atomic::Ordering::Relaxed);

                        println!("PACKET {}", rpm);

                        // write the full packet to a shared Channel or Pipe

                        sender
                            .try_send(ScanFrame {
                                scan_data: buffer,
                                odometry: [0.0, 0.0],
                                rpm,
                            })
                            .expect("Could not send parsed packet");

                        state = State::LookingForStart;
                    }
                }

                last_byte = read;

                // println!("0x{read:02x}");
            }
        }
    }
}

#[embassy_executor::task]
async fn motor_control(mut pwm_pin: NeatoPwmPin) {
    let mut pwm_current: i16 = 0;

    loop {
        Timer::after(Duration::from_millis(200)).await;

        let rpm_target = if MOTOR_ON.load(Ordering::Relaxed) {
            300
        } else {
            0
        };

        let last_rpm = LAST_RPM.load(Ordering::Relaxed);

        let error = rpm_target as i16 - last_rpm as i16;

        pwm_current += error;

        // cap the allowed PWM range
        pwm_current = pwm_current.clamp(0, 90 * 100);

        let mut pwm = pwm_current / 100;

        if rpm_target == 0 {
            pwm = 0;
        }

        pwm_pin.set_timestamp(pwm as u16);

        // println!(
        //     "Control, {} rpm, error={error}. New PWM = {}",
        //     last_rpm, pwm
        // );
    }
}
#[derive(Debug)]
enum State {
    LookingForStart,
    CollectingBytes { index: usize },
    ParseRPM,
}

#[embassy_executor::task]
async fn task(
    stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>,
    mut led: AnyPin<Output<PushPull>>,
    mut receiver: Receiver<'static, CriticalSectionRawMutex, ScanFrame, 10>,
) {
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    loop {
        if stack.is_link_up() {
            break;
        }
        led.set_high().unwrap();
        Timer::after(Duration::from_millis(250)).await;
        led.set_low().unwrap();
        Timer::after(Duration::from_millis(250)).await;
    }

    println!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            break;
        }
        led.set_high().unwrap();
        Timer::after(Duration::from_millis(100)).await;
        led.set_low().unwrap();
        Timer::after(Duration::from_millis(400)).await;
    }

    Timer::after(Duration::from_millis(1_000)).await;

    let mut socket = TcpSocket::new(&stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(embassy_time::Duration::from_secs(5)));

    loop {
        println!("Wait for connection...");
        let r = socket
            .accept(IpListenEndpoint {
                addr: None,
                port: 8080,
            })
            .await;
        println!("Connected...");
        led.set_high().unwrap();

        if let Err(e) = r {
            println!("connect error: {:?}", e);
            continue;
        }

        connected_loop(&mut socket, &mut receiver).await;

        let r = socket.flush().await;
        if let Err(e) = r {
            println!("flush error: {:?}", e);
        }
        Timer::after(Duration::from_millis(1000)).await;

        socket.close();
        println!("Connection terminated");
        led.set_low().unwrap();
        Timer::after(Duration::from_millis(1000)).await;

        socket.abort();
    }
}
