#![no_main]
#![no_std]

// use rp_pico::hal as _;
use defmt_rtt as _;
use panic_probe as _;

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    // TODO: Replace `some_hal::pac` with the path to the PAC
    device = rp_pico::hal::pac,
    // TODO: Replace the `FreeInterrupt1, ...` with free interrupt vectors if software tasks are used
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [TIMER_IRQ_1],
    peripherals = true
)]
mod app {

    use defmt::{debug, error, info, warn};
    use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
    use library::event::Event;
    use library::parse_at::{AtParser, EspMessage, ParsedMessage};

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
    use rtic_sync::channel::{Sender, TrySendError};

    type Uart1Pins = (
        hal::gpio::Pin<Gpio4, hal::gpio::FunctionUart, hal::gpio::PullNone>,
        hal::gpio::Pin<Gpio5, hal::gpio::FunctionUart, hal::gpio::PullNone>,
    );

    const ESP_CHANNEL_CAPACITY: usize = 32;
    const EVENT_CHANNEL_CAPACITY: usize = 32;
    type EspChannelReceiver =
        rtic_sync::channel::Receiver<'static, EspMessage, ESP_CHANNEL_CAPACITY>;

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

        // create a channel for comminicating ESP messages
        let (esp_sender, esp_receiver) = rtic_sync::make_channel!(EspMessage, ESP_CHANNEL_CAPACITY);
        let (event_sender, event_receiver) =
            rtic_sync::make_channel!(Event, EVENT_CHANNEL_CAPACITY);

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
                data_event_sender: event_sender,
            },
        )
    }

    // TODO create a task that listens for messages and status updates from the ESP task and the
    // serial communication task (for usability also over a serial connection)
    #[task(priority = 1, local = [event_receiver])]
    async fn event_loop(cx: event_loop::Context) {
        loop {
            match cx.local.event_receiver.recv().await {
                Ok(event) => {
                    // TODO! Handle the event
                    info!("Received event: {}", event);
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
            }
        }
    }

    /// This task receives data chunks and emitts [`Event`] to the [`event_loop`]
    #[task(priority = 1, local = [data_event_sender])]
    async fn data_handler(cx: data_handler::Context) {
        // Instantiate some kind of buffer

        // continously read the data events and parse them to generate events for the event loop
    }

    /// Task that initializes and handles the ESP wifi connection
    #[task(
        priority = 1,
        local = [
            esp_mode,
            esp_reset,
            uart1_tx,
            esp_receiver,
            esp_event_sender,
        ],
    )]
    async fn init_esp(cx: init_esp::Context) {
        info!("Reseting the ESP");
        cx.local.esp_mode.set_high().ok();
        cx.local.esp_reset.set_low().ok();

        Timer::delay(1.secs()).await;
        cx.local.esp_reset.set_high().ok();
        Timer::delay(1.secs()).await;

        // read messages from the device and advance the inner state machine

        wait_for_message(cx.local.esp_receiver, EspMessage::Ready).await;

        // configure some stuff
        cx.local.uart1_tx.write_full_blocking(b"AT+SYSMSG=0\r\n");

        enum State {
            Ready,
            WifiConnectedAndIp,
            Listening,
            ClientConnected,
        }

        let mut state = State::Ready;

        info!("Done, starting message loop");
        loop {
            while let Ok(m) = cx.local.esp_receiver.recv().await {
                info!("Got message: {}", m);
                match m {
                    EspMessage::GotIP => {
                        state = State::WifiConnectedAndIp;
                        // start the server

                        info!("Enabling Multiple Connections");
                        cx.local.uart1_tx.write_full_blocking(b"AT+CIPMUX=1\r\n");
                        wait_for_message(cx.local.esp_receiver, EspMessage::Ok).await;
                        Timer::delay(1.secs()).await;

                        cx.local
                            .uart1_tx
                            .write_full_blocking(b"AT+CIPSERVERMAXCONN=1\r\n");
                        wait_for_message(cx.local.esp_receiver, EspMessage::Ok).await;

                        info!("Starting server");
                        cx.local
                            .uart1_tx
                            .write_full_blocking(b"AT+CIPSERVER=1,80\r\n");
                        wait_for_message(cx.local.esp_receiver, EspMessage::Ok).await;

                        state = State::Listening;
                        info!("Listening");
                    }
                    EspMessage::ClientConnect => {
                        state = State::ClientConnected;
                        channel_send(cx.local.esp_event_sender, Event::Connected, "ESP");
                    }
                    EspMessage::ClientDisconnect => {
                        state = State::Listening;
                        channel_send(cx.local.esp_event_sender, Event::Disconnected, "ESP");
                    }
                    _ => {}
                }
            }
        }
    }

    async fn wait_for_message(receiver: &mut EspChannelReceiver, value: EspMessage) {
        while let Ok(m) = receiver.recv().await {
            if m == value {
                return;
            } else {
                warn!("got message {} while waiting for {}", m, value);
            }
        }
    }

    /// Helper function for trying to send something to a Sender MPSC channel, or print a warning
    /// message if an error occurred
    fn channel_send<T: defmt::Format, const N: usize>(
        sender: &mut Sender<'static, T, N>,
        value: T,
        context: &str,
    ) {
        match sender.try_send(value) {
            Err(TrySendError::Full(m)) => {
                warn!("ESP channel full, failed to send: {} ({})", m, context)
            }
            Err(TrySendError::NoReceiver(m)) => {
                warn!(
                    "ESP channel has no receiver, failed to send: {} ({})",
                    m, context
                )
            }
            _ => {}
        }
    }

    /// Hardware task that reads bytes from the UART an publishes messages!
    #[task(
        binds = UART1_IRQ,
        local = [
            uart1_rx,
            esp_sender,
            parser: AtParser<256> = AtParser::new(),
        ],
    )]
    fn uart1_esp32(cx: uart1_esp32::Context) {
        let sender = cx.local.esp_sender;
        let rx = cx.local.uart1_rx;
        cx.local.parser.consume(rx, move |message| match message {
            ParsedMessage::Simple(m) => channel_send(sender, m, "uart1_esp32"),
            ParsedMessage::ReceivedData(data) => {
                info!("got data: {}", data);
            }
        });
    }

    #[task(local = [led])]
    async fn heartbeat(ctx: heartbeat::Context) {
        // Loop forever.
        //
        // It is important to remember that tasks that loop
        // forever should have an `await` somewhere in that loop.
        //
        // Without the await, the task will never yield back to
        // the async executor, which means that no other lower or
        // equal  priority task will be able to run.
        loop {
            // Flicker the built-in LED
            _ = ctx.local.led.toggle();

            // Delay for 1 second
            Timer::delay(1000.millis()).await;
        }
    }
}
