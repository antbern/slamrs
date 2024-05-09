#![no_main]
#![no_std]

mod tasks;
mod util;

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
    use crate::tasks::esp::{init_esp, uart1_esp32};
    use crate::tasks::usb::usb_irq;
    use crate::util::channel_send;

    use defmt::{debug, error, info, warn};
    use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
    use futures::FutureExt;
    use library::event::Event;
    use library::parse_at::{AtParser, EspMessage};

    use library::slamrs_message::{CommandMessage, RobotMessage};
    use rp_pico::hal::{
        self, clocks,
        fugit::{ExtU64, RateExtU32},
        gpio::{
            self,
            bank0::{Gpio10, Gpio19, Gpio24, Gpio4, Gpio5},
            FunctionSioOutput, PullDown,
        },
        sio::Sio,
        uart::{DataBits, Reader, StopBits, Writer},
        watchdog::Watchdog,
        Clock,
    };
    use rp_pico::XOSC_CRYSTAL_FREQ;
    use rtic_monotonics::rp2040::*;

    // USB Device support
    use usb_device::{class_prelude::*, prelude::*};

    // USB Communications Class Device support
    use usbd_serial::SerialPort;

    type Uart1Pins = (
        hal::gpio::Pin<Gpio4, hal::gpio::FunctionUart, hal::gpio::PullNone>,
        hal::gpio::Pin<Gpio5, hal::gpio::FunctionUart, hal::gpio::PullNone>,
    );

    const ESP_CHANNEL_CAPACITY: usize = 32;
    const EVENT_CHANNEL_CAPACITY: usize = 32;
    pub type EspChannelReceiver =
        rtic_sync::channel::Receiver<'static, EspMessage, ESP_CHANNEL_CAPACITY>;

    const DATA_CHANNEL_CAPACITY: usize = 16;
    const DATA_PACKET_SIZE: usize = 64;

    const ROBOT_MESSAGE_CAPACITY: usize = 16;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        led: gpio::Pin<Gpio10, FunctionSioOutput, PullDown>,

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

        usb_state: UsbState<'static>,
        usb_event_sender: rtic_sync::channel::Sender<'static, Event, EVENT_CHANNEL_CAPACITY>,
    }

    static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    pub struct UsbState<'a> {
        /// The USB Device Driver (shared with the interrupt).
        pub usb_device: UsbDevice<'a, hal::usb::UsbBus>,

        /// The USB Serial Device Driver (shared with the interrupt).
        pub usb_serial: SerialPort<'a, hal::usb::UsbBus>,
    }

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

        let usb_state = {
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
                .device_class(2) // from: https://www.usb.org/defined-class-codes
                .build();

            // Enable the USB interrupt
            unsafe {
                hal::pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
            };

            // No more USB code after this point in main! We can do anything we want in
            // here since USB is handled in the interrupt
            UsbState {
                usb_device: usb_dev,
                usb_serial: serial,
            }
        };

        // create a channel for comminicating ESP messages
        let (esp_sender, esp_receiver) = rtic_sync::make_channel!(EspMessage, ESP_CHANNEL_CAPACITY);
        let (event_sender, event_receiver) =
            rtic_sync::make_channel!(Event, EVENT_CHANNEL_CAPACITY);

        // create a channel for comminicating data packets
        let (data_sender, data_receiver) =
            rtic_sync::make_channel!((usize, [u8; DATA_PACKET_SIZE]), DATA_CHANNEL_CAPACITY);

        let (robot_message_sender, robot_message_receiver) =
            rtic_sync::make_channel!(RobotMessage, ROBOT_MESSAGE_CAPACITY);

        data_handler::spawn().ok();
        event_loop::spawn().ok();
        init_esp::spawn().ok();
        heartbeat::spawn().ok();
        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                // Initialization of local resources go here
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
                esp_data_sender: data_sender,
                robot_message_sender,
                robot_message_receiver,
                usb_state,
                usb_event_sender: event_sender,
            },
        )
    }

    // TODO create a task that listens for messages and status updates from the ESP task and the
    // serial communication task (for usability also over a serial connection)
    #[task(priority = 1, local = [event_receiver, robot_message_sender])]
    async fn event_loop(cx: event_loop::Context) {
        let mut is_connected = false;
        loop {
            futures::select_biased! {

            _ = Timer::delay(1000.millis()).fuse() => {
                if is_connected {
                    // Send a ping message to the robot
                    channel_send(cx.local.robot_message_sender, RobotMessage::Pong, "event_loop");
                }
            },
            event = cx.local.event_receiver.recv().fuse() => match event {
                Ok(event) => {
                    // TODO! Handle the event
                    info!("Received event: {}", event);

                    match event {
                        Event::Connected => {is_connected = true;}
                        Event::Disconnected => {is_connected = false;}
                        // Event::Command(CommandMessage::Ping) => {
                        //     channel_send(
                        //         cx.local.robot_message_sender,
                        //         RobotMessage::Pong,
                        //         "event_loop",
                        //     );
                        // }
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
    #[task(priority = 1, local = [data_event_sender, data_receiver])]
    async fn data_handler(cx: data_handler::Context) {
        loop {
            match cx.local.data_receiver.recv().await {
                Ok((size, data)) => {
                    let data = &data[..size];
                    match library::slamrs_message::bincode::decode_from_slice::<CommandMessage, _>(
                        data,
                        library::slamrs_message::bincode::config::standard(),
                    ) {
                        Ok((event, len)) => {
                            if len != size {
                                warn!("Data packet was not fully consumed");
                            }
                            channel_send(
                                cx.local.data_event_sender,
                                Event::Command(event),
                                "data_handler",
                            );
                        }
                        Err(_e) => {
                            warn!("Failed to deserialize data");
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
            local = [
                usb_state,
                usb_event_sender,
                was_connected: bool = false,
            ],
        )]
        fn usb_irq(cx: usb_irq::Context);
    }

    #[task(local = [led])]
    async fn heartbeat(ctx: heartbeat::Context) {
        loop {
            // Flicker the built-in LED
            _ = ctx.local.led.toggle();

            // Delay for 1 second
            Timer::delay(1000.millis()).await;
        }
    }
}
