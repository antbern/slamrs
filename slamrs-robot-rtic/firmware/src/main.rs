#![no_main]
#![no_std]

mod encoder;
mod motor;
mod tasks;
mod util;
mod ws2812b;

// use rp_pico::hal as _;
use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(
    device = rp_pico::hal::pac,
    // Replace the `FreeInterrupt1, ...` with free interrupt vectors if software tasks are used
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [TIMER_IRQ_1],
    peripherals = true
)]
mod app {
    use crate::encoder;
    use crate::motor::{Motor, MotorDriver};
    use crate::tasks::esp::{init_esp, uart1_esp32};
    use crate::tasks::heartbeat::{heartbeat, Color, LedStatus, Speed};
    use crate::tasks::motors::motor_control_loop;
    use crate::tasks::neato::{neato_motor_control, uart0_neato};
    use crate::tasks::usb::{usb_irq, usb_sender};
    use crate::util::channel_send;

    use core::sync::atomic::Ordering;
    use defmt::{debug, error, info, warn};
    use embedded_hal::digital::v2::OutputPin;
    use futures::FutureExt;
    use library::event::Event;
    use library::neato::RunningParser;
    use library::parse_at::{AtParser, EspMessage};
    use library::slamrs_message::bincode;
    use library::slamrs_message::{CommandMessage, RobotMessage};
    use rp_pico::hal::gpio::PullNone;
    use rp_pico::hal::{
        self, clocks,
        dma::DMAExt,
        fugit::{ExtU64, RateExtU32},
        gpio::{self, bank0::*, FunctionSioOutput, PullDown},
        sio::Sio,
        uart::{DataBits, Reader, StopBits, Writer},
        watchdog::Watchdog,
        Clock,
    };
    use rp_pico::XOSC_CRYSTAL_FREQ;
    use rtic_monotonics::rp2040::*;

    use rtic_sync::portable_atomic::AtomicU8;
    // USB Device support
    use usb_device::{class_prelude::*, prelude::*};

    // USB Communications Class Device support
    use usbd_serial::SerialPort;

    type Uart1Pins = (
        hal::gpio::Pin<Gpio4, hal::gpio::FunctionUart, hal::gpio::PullNone>,
        hal::gpio::Pin<Gpio5, hal::gpio::FunctionUart, hal::gpio::PullNone>,
    );

    type Uart0Pins = (
        hal::gpio::Pin<Gpio16, hal::gpio::FunctionUart, hal::gpio::PullNone>,
        hal::gpio::Pin<Gpio17, hal::gpio::FunctionUart, hal::gpio::PullNone>,
    );

    type I2CPins = (
        hal::gpio::Pin<Gpio0, hal::gpio::FunctionI2C, hal::gpio::PullNone>,
        hal::gpio::Pin<Gpio1, hal::gpio::FunctionI2C, hal::gpio::PullNone>,
    );

    type I2CBus = hal::I2C<hal::pac::I2C0, I2CPins>;

    const ESP_CHANNEL_CAPACITY: usize = 32;
    const EVENT_CHANNEL_CAPACITY: usize = 32;
    pub type EspChannelReceiver =
        rtic_sync::channel::Receiver<'static, EspMessage, ESP_CHANNEL_CAPACITY>;

    const DATA_CHANNEL_CAPACITY: usize = 16;
    pub const DATA_PACKET_SIZE: usize = 64;

    const ROBOT_MESSAGE_CAPACITY: usize = 16;

    const MOTOR_STEPS_PER_REV: i32 = 2000;
    const MOTOR_WHEEL_DIAMETER: f32 = 0.06; // meters
    pub const MOTOR_STEPS_PER_METER: f32 =
        MOTOR_STEPS_PER_REV as f32 / (MOTOR_WHEEL_DIAMETER * core::f32::consts::PI);

    // Shared resources go here
    #[shared]
    struct Shared {
        /// Status
        led_status: LedStatus,

        /// The USB Serial Device mriver
        /// Shared between the USB interrupt and the USB sending task
        pub usb_serial: SerialPort<'static, hal::usb::UsbBus>,

        /// Flag indicating if the USB device is connected and active
        usb_active: bool,

        /// The motor controller
        motor_controller: MotorDriver<I2CBus>,

        /// The amount of downsampling to apply to the neato data, shared but with non-mutable
        /// access
        neato_downsampling: AtomicU8,

        /// speed in steps / second
        motor_speed_right: i32,
        motor_speed_left: i32,

        /// Motor PI parameters
        motor_pi_params: crate::tasks::motors::PiParameters,
    }

    // Local resources go here
    #[local]
    struct Local {
        /// for the heartbeat task
        led: gpio::Pin<Gpio10, FunctionSioOutput, PullDown>,
        led_rgb: crate::ws2812b::WS2812B,

        // the uart reader part used in the IRQ hardware task
        uart1_rx: Reader<hal::pac::UART1, Uart1Pins>,
        uart1_tx: Writer<hal::pac::UART1, Uart1Pins>,

        // pins used to reset the ESP
        esp_mode: gpio::Pin<Gpio24, FunctionSioOutput, PullDown>,
        esp_reset: gpio::Pin<Gpio19, FunctionSioOutput, PullDown>,

        // channel ends for transmitting esp messages
        esp_sender: rtic_sync::channel::Sender<'static, EspMessage, ESP_CHANNEL_CAPACITY>,
        esp_receiver: EspChannelReceiver,
        // channel for sending events from the ESP handler
        esp_event_sender: rtic_sync::channel::Sender<'static, Event, EVENT_CHANNEL_CAPACITY>,

        /// Channel receiver where all `Event` objects are sent
        event_receiver: rtic_sync::channel::Receiver<'static, Event, EVENT_CHANNEL_CAPACITY>,

        /// Channel for sending events from the data handler
        data_event_sender: rtic_sync::channel::Sender<'static, Event, EVENT_CHANNEL_CAPACITY>,

        /// Channel receiver where all data packets are sent
        data_receiver: rtic_sync::channel::Receiver<
            'static,
            (usize, [u8; DATA_PACKET_SIZE]),
            DATA_CHANNEL_CAPACITY,
        >,
        /// Channel sender for the ESP
        esp_data_sender: rtic_sync::channel::Sender<
            'static,
            (usize, [u8; DATA_PACKET_SIZE]),
            DATA_CHANNEL_CAPACITY,
        >,

        /// Sender for the robot messages
        robot_message_sender:
            rtic_sync::channel::Sender<'static, RobotMessage, ROBOT_MESSAGE_CAPACITY>,
        /// Receiver for the robot messages
        robot_message_receiver:
            rtic_sync::channel::Receiver<'static, RobotMessage, ROBOT_MESSAGE_CAPACITY>,

        usb_event_sender: rtic_sync::channel::Sender<'static, Event, EVENT_CHANNEL_CAPACITY>,
        usb_data_sender: rtic_sync::channel::Sender<
            'static,
            (usize, [u8; DATA_PACKET_SIZE]),
            DATA_CHANNEL_CAPACITY,
        >,

        /// The USB Device Driver (shared with the interrupt).
        usb_device: UsbDevice<'static, hal::usb::UsbBus>,

        /// Sender for the robot messages
        robot_message_sender_usb:
            rtic_sync::channel::Sender<'static, RobotMessage, ROBOT_MESSAGE_CAPACITY>,
        /// Receiver for the robot messages
        robot_message_receiver_usb:
            rtic_sync::channel::Receiver<'static, RobotMessage, ROBOT_MESSAGE_CAPACITY>,

        ///// Neato stuff
        // uart reader for the neato
        uart0_rx_neato: Reader<hal::pac::UART0, Uart0Pins>,
        neato_motor: Motor<I2CBus>,
        robot_message_sender_neato:
            rtic_sync::channel::Sender<'static, RobotMessage, ROBOT_MESSAGE_CAPACITY>,
        robot_message_sender_esp_neato:
            rtic_sync::channel::Sender<'static, RobotMessage, ROBOT_MESSAGE_CAPACITY>,

        ///// Motor speed controller
        motor_right: Motor<I2CBus>,
        motor_left: Motor<I2CBus>,
    }
    /// The USB bus, only needed for initializing the USB device and will never be accessed again
    static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            rp_pico::hal::sio::spinlock_reset();
        }

        info!("init");

        // TODO setup monotonic if used
        // Initialize the interrupt for the RP2040 timer and obtain the token
        // proving that we have.
        let rp2040_timer_token = rtic_monotonics::create_rp2040_monotonic_token!();
        // Configure the clocks, watchdog - The default is to generate a 125 MHz system clock
        Timer::start(ctx.device.TIMER, &mut ctx.device.RESETS, rp2040_timer_token); // default rp2040 clock-rate is 125MHz
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        let clocks = clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // Init LED pin
        let sio = Sio::new(ctx.device.SIO);
        let pins = hal::gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );
        let mut led = pins.gpio10.into_push_pull_output();
        led.set_low().unwrap();

        // initialize the pins for resetting the ESP, with HIGH as their current state
        let esp_reset = pins
            .gpio19
            .into_push_pull_output_in_state(gpio::PinState::High);
        let esp_mode = pins
            .gpio24
            .into_push_pull_output_in_state(gpio::PinState::High);

        // init the UART
        let uart_pins: Uart1Pins = (
            // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
            pins.gpio4.reconfigure(),
            // UART RX (characters received by RP2040) on pin 2 (GPIO1)
            pins.gpio5.reconfigure(),
        );
        let mut uart =
            hal::uart::UartPeripheral::new(ctx.device.UART1, uart_pins, &mut ctx.device.RESETS)
                .enable(
                    hal::uart::UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
                    clocks.peripheral_clock.freq(),
                )
                .unwrap();

        // TODO: should we setup DMA for reading the serial input??
        uart.set_fifos(true);
        uart.enable_rx_interrupt();

        let (rx, tx) = uart.split();

        let (usb_serial, usb_device) = {
            // Set up the USB driver
            let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
                ctx.device.USBCTRL_REGS,
                ctx.device.USBCTRL_DPRAM,
                clocks.usb_clock,
                true,
                &mut ctx.device.RESETS,
            ));
            unsafe {
                // Safety: This is safe as interrupts haven't been started yet
                USB_BUS = Some(usb_bus);
            }
            // Grab a reference to the USB Bus allocator. We are promising to the
            // compiler not to take mutable access to this global variable whilst this
            // reference exists!
            let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

            let serial = SerialPort::new(&bus_ref);

            // Create a USB device with a fake VID and PID
            let usb_dev = UsbDeviceBuilder::new(&bus_ref, UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(usbd_serial::USB_CLASS_CDC)
                .build();

            // Enable the USB interrupt
            unsafe {
                hal::pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
            };

            // No more USB code after this point in main! We can do anything we want in
            // here since USB is handled in the interrupt
            (serial, usb_dev)
        };

        // setup i2c for interacting with the motor controller
        // Configure two pins as being I²C, not GPIO
        let sda_pin: hal::gpio::Pin<_, hal::gpio::FunctionI2C, PullNone> = pins.gpio0.reconfigure();
        let scl_pin: hal::gpio::Pin<_, hal::gpio::FunctionI2C, PullNone> = pins.gpio1.reconfigure();

        // Create the I²C drive
        let i2c = hal::I2C::i2c0(
            ctx.device.I2C0,
            sda_pin,
            scl_pin,
            400.kHz(),
            &mut ctx.device.RESETS,
            &clocks.system_clock,
        );

        let mut controller = crate::motor::MotorDriver::new(i2c, 0x60, 100.0).unwrap();
        let motor = controller.motor(crate::motor::MotorId::M2).unwrap();

        // init the UART for the Neato
        let uart_pins: Uart0Pins = (
            // UART TX
            pins.gpio16.reconfigure(),
            // UART RX
            pins.gpio17.reconfigure(),
        );

        let mut uart_neato =
            hal::uart::UartPeripheral::new(ctx.device.UART0, uart_pins, &mut ctx.device.RESETS)
                .enable(
                    hal::uart::UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
                    clocks.peripheral_clock.freq(),
                )
                .unwrap();
        uart_neato.set_fifos(true);
        uart_neato.enable_rx_interrupt();
        // we only need the rx part of the uart
        let (rx_neato, _tx_neato) = uart_neato.split();

        // setup quadrature encoders for the motors
        let dma = ctx.device.DMA.split(&mut ctx.device.RESETS);
        encoder::initialize_encoders(
            ctx.device.PIO0,
            &mut ctx.device.RESETS,
            dma.ch0,
            dma.ch1,
            pins.gpio20.into_function(),
            pins.gpio21.into_function(),
            pins.gpio22.into_function(),
            pins.gpio23.into_function(),
        );

        let led_rgb = crate::ws2812b::WS2812B::new(
            ctx.device.PIO1,
            &mut ctx.device.RESETS,
            pins.gpio11.into_function(),
            false,
            clocks.system_clock.freq(),
        );

        let motor_right = controller.motor(crate::motor::MotorId::M1).unwrap();
        let motor_left = controller.motor(crate::motor::MotorId::M0).unwrap();

        // create a channel for communicating ESP messages
        let (esp_sender, esp_receiver) = rtic_sync::make_channel!(EspMessage, ESP_CHANNEL_CAPACITY);
        let (event_sender, event_receiver) =
            rtic_sync::make_channel!(Event, EVENT_CHANNEL_CAPACITY);

        // create a channel for comminicating data packets
        let (data_sender, data_receiver) =
            rtic_sync::make_channel!((usize, [u8; DATA_PACKET_SIZE]), DATA_CHANNEL_CAPACITY);

        let (robot_message_sender, robot_message_receiver) =
            rtic_sync::make_channel!(RobotMessage, ROBOT_MESSAGE_CAPACITY);
        let (robot_message_sender_usb, robot_message_receiver_usb) =
            rtic_sync::make_channel!(RobotMessage, ROBOT_MESSAGE_CAPACITY);

        neato_motor_control::spawn().ok();
        motor_control_loop::spawn().ok();
        data_handler::spawn().ok();
        event_loop::spawn().ok();
        init_esp::spawn().ok();
        usb_sender::spawn().ok();
        heartbeat::spawn().ok();
        (
            Shared {
                led_status: LedStatus::default(),
                usb_serial,
                usb_active: false,
                motor_controller: controller,
                neato_downsampling: AtomicU8::new(2),
                motor_speed_right: 0,
                motor_speed_left: 0,
                motor_pi_params: Default::default(),
            },
            Local {
                led_rgb,
                led,
                uart1_rx: rx,
                uart1_tx: tx,
                esp_mode,
                esp_reset,
                esp_sender,
                esp_receiver,
                esp_event_sender: event_sender.clone(),
                event_receiver,
                data_event_sender: event_sender.clone(),
                data_receiver,
                esp_data_sender: data_sender.clone(),
                robot_message_sender: robot_message_sender.clone(),
                robot_message_receiver,
                usb_data_sender: data_sender,
                usb_event_sender: event_sender,
                usb_device,
                robot_message_sender_usb: robot_message_sender_usb.clone(),
                robot_message_receiver_usb,
                uart0_rx_neato: rx_neato,
                neato_motor: motor,
                robot_message_sender_neato: robot_message_sender_usb,
                robot_message_sender_esp_neato: robot_message_sender,
                motor_right,
                motor_left,
            },
        )
    }

    // TODO create a task that listens for messages and status updates from the ESP task and the
    // serial communication task (for usability also over a serial connection)
    #[task(
        priority = 1,
        shared = [
            led_status,
            &neato_downsampling,
            motor_pi_params,
            motor_speed_right,
            motor_speed_left,
        ],
        local = [
            event_receiver,
            robot_message_sender,
            robot_message_sender_usb,
        ]
    )]
    async fn event_loop(mut cx: event_loop::Context) {
        let mut is_connected = false;
        loop {
            futures::select_biased! {

            _ = Timer::delay(1000.millis()).fuse() => {
                if is_connected {
                    // Send a ping message to the robot
                    channel_send(cx.local.robot_message_sender, RobotMessage::Pong, "event_loop");
                    channel_send(cx.local.robot_message_sender_usb, RobotMessage::Pong, "event_loop");
                }
            },
            event = cx.local.event_receiver.recv().fuse() => match event {
                Ok(event) => {
                    info!("Received event: {}", event);

                    match event {
                        Event::Connected => {
                            is_connected = true;
                            cx.shared.led_status.lock(|s| *s = LedStatus::Blinking(Color::Green, Speed::Fast));
                        }
                        Event::Disconnected => {
                            is_connected = false;
                            crate::tasks::neato::MOTOR_ON.store(false, Ordering::Relaxed);
                            cx.shared.led_status.lock(|s| *s = LedStatus::Blinking(Color::Green, Speed::Slow));
                        },
                        Event::Command(CommandMessage::NeatoOn) => {
                            crate::tasks::neato::MOTOR_ON.store(true, Ordering::Relaxed);
                            crate::tasks::neato::LAST_RPM.store(0, Ordering::Relaxed);
                        },
                        Event::Command(CommandMessage::NeatoOff) => {
                            crate::tasks::neato::MOTOR_ON.store(false, Ordering::Relaxed);
                        },
                        Event::Command(CommandMessage::SetDownsampling { every }) => {
                            cx.shared.neato_downsampling.store(every, Ordering::Relaxed);
                        },
                        Event::Command(CommandMessage::SetMotorPiParams { kp, ki }) => {
                                cx.shared.motor_pi_params.lock(| p| {
                                    p.kp = crate::tasks::motors::F32::from_num(kp);
                                    p.ki = crate::tasks::motors::F32::from_num(ki);
                                });
                        },
                        Event::Command(CommandMessage::Drive { left, right }) => {
                            cx.shared.motor_speed_right.lock(|speed|{
                                *speed = (right * MOTOR_STEPS_PER_REV as f32 / (MOTOR_WHEEL_DIAMETER * core::f32::consts::PI)) as i32;
                            });
                            cx.shared.motor_speed_left.lock(|speed|{
                                *speed = (left * MOTOR_STEPS_PER_REV as f32 / (MOTOR_WHEEL_DIAMETER * core::f32::consts::PI)) as i32;
                            });
                        },

                        _ => {}
                    }
                }
                Err(e) => {
                    warn!(
                        "Error receiveing event: {}",
                        match e {
                            rtic_sync::channel::ReceiveError::NoSender => "NoSender",
                            rtic_sync::channel::ReceiveError::Empty => "Empty",
                        }
                    );
                }
            },
            }
        }
    }

    /// This task receives data chunks and emitts [`Event`] to the [`event_loop`]
    #[task(
        priority = 1,
        local = [
            data_event_sender,
            data_receiver,
        ],
    )]
    async fn data_handler(cx: data_handler::Context) {
        // buffer to accumulate data packets
        let mut buffer: [u8; 256] = [0; 256];
        let mut index_end: usize = 0;

        loop {
            match cx.local.data_receiver.recv().await {
                Ok((size, data)) => {
                    let data = &data[..size];

                    // accumulate all received bytes into the buffer
                    if index_end + size > buffer.len() {
                        error!("Data packet is too large for the remaining space in the buffer, is this a bug? Skipping");
                        continue;
                    }
                    buffer[index_end..(index_end + size)].copy_from_slice(data);
                    index_end += size;

                    // iterate until we need more data
                    loop {
                        match library::slamrs_message::bincode::decode_from_slice::<CommandMessage, _>(
                            &buffer[..index_end], // always start at the beginning of the buffer
                            library::slamrs_message::bincode::config::standard(),
                        ) {
                            Ok((event, len)) => {
                                // shift the remaining data to the front of the buffer
                                buffer.copy_within(len..index_end, 0);
                                index_end -= len;

                                channel_send(
                                    cx.local.data_event_sender,
                                    Event::Command(event),
                                    "data_handler",
                                );
                            }
                            Err(bincode::error::DecodeError::UnexpectedEnd { .. }) => {
                                // do nothing, we need more data so break the inner loop
                                break;
                            }
                            Err(e) => {
                                error!("Failed to deserialize data: {}", defmt::Debug2Format(&e));
                            }
                        }
                    }
                }
                Err(e) => {
                    warn!(
                        "Error receiveing data packet: {}",
                        match e {
                            rtic_sync::channel::ReceiveError::NoSender => "NoSender",
                            rtic_sync::channel::ReceiveError::Empty => "Empty",
                        }
                    );
                }
            }
        }
        // continously read the data events and parse them to generate events for the event loop
    }

    extern "Rust" {
        // Task that initializes and handles the ESP WIFI connection
        #[task(
            priority = 1,
            shared = [led_status],
            local = [
                esp_mode,
                esp_reset,
                uart1_tx,
                esp_receiver,
                esp_event_sender,
                robot_message_receiver,
            ],
        )]
        async fn init_esp(_: init_esp::Context);

        // Hardware task that reads bytes from the UART an publishes messages!
        #[task(
            binds = UART1_IRQ,
            local = [
                uart1_rx,
                esp_sender,
                esp_data_sender,
                parser: AtParser<256> = AtParser::new(),
            ],
        )]
        fn uart1_esp32(cx: uart1_esp32::Context);

        // Hardware task that reads bytes from the USB and publishes messages!
        #[task(
            binds = USBCTRL_IRQ,
            shared = [usb_serial, usb_active],
            local = [
                usb_device,
                usb_event_sender,
                usb_data_sender,
            ],
        )]
        fn usb_irq(cx: usb_irq::Context);

        #[task(
            priority = 1,
            shared = [usb_serial, usb_active],
            local = [robot_message_receiver_usb],
        )]
        async fn usb_sender(cx: usb_sender::Context);

        // Hardware task that reads bytes from the Neato UART
        #[task(
            binds = UART0_IRQ,
            shared = [&neato_downsampling],
            local = [
                uart0_rx_neato,
                robot_message_sender_neato,
                robot_message_sender_esp_neato,
                parser: RunningParser = RunningParser::new(),
                rpm_accumulator: i32 = 0i32,
                rpm_average: i32 = 0i32,
                downsample_counter: u8 = 0u8,
                last_odometry_right: i32 = 0i32,
                last_odometry_left: i32 = 0i32,
         ],
        )]
        fn uart0_neato(cx: uart0_neato::Context);

        #[task(
            priority = 1,
            shared = [motor_controller],
            local = [
                neato_motor,
            ],
        )]
        async fn neato_motor_control(cx: neato_motor_control::Context);

        #[task(
            priority = 1,
            shared = [
                motor_controller,
                motor_speed_right,
                motor_speed_left,
                motor_pi_params,
            ],
            local = [
                motor_right,
                motor_left,
            ],
        )]
        async fn motor_control_loop(cx: motor_control_loop::Context);

        #[task(
            shared = [led_status],
            local = [
                led,
                led_rgb,
            ]
        )]
        async fn heartbeat(cx: heartbeat::Context);
    }
}
