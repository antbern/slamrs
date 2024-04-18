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
    use defmt::{error, info, warn};

    use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};

    use rp_pico::hal::{
        self, clocks,
        fugit::{ExtU64, RateExtU32},
        gpio::{
            self,
            bank0::{Gpio10, Gpio19, Gpio24, Gpio4, Gpio5},
            FunctionSioOutput, PullDown,
        },
        sio::Sio,
        uart::{DataBits, ReadErrorType, Reader, StopBits, Writer},
        watchdog::Watchdog,
        Clock,
    };
    use rp_pico::XOSC_CRYSTAL_FREQ;
    use rtic_monotonics::rp2040::*;

    type Uart1Pins = (
        hal::gpio::Pin<Gpio4, hal::gpio::FunctionUart, hal::gpio::PullNone>,
        hal::gpio::Pin<Gpio5, hal::gpio::FunctionUart, hal::gpio::PullNone>,
    );

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        // TODO: Add resources
        led: gpio::Pin<Gpio10, FunctionSioOutput, PullDown>,

        // the uart reader part used in the IRQ hardware task
        uart1_rx: Reader<hal::pac::UART1, Uart1Pins>,
        uart1_tx: Writer<hal::pac::UART1, Uart1Pins>,

        // pins used to reset the ESP
        esp_mode: gpio::Pin<Gpio24, FunctionSioOutput, PullDown>,
        esp_reset: gpio::Pin<Gpio19, FunctionSioOutput, PullDown>,
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
        uart.enable_rx_interrupt();

        let (rx, tx) = uart.split();

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
            },
        )
    }

    // Optional idle, can be removed if not needed.
    // #[idle]
    // fn idle(_: idle::Context) -> ! {
    //     info!("idle");
    //
    //     loop {
    //         continue;
    //     }
    // }

    /// Task that initializes and handles the ESP wifi connection
    #[task(priority = 1, local = [esp_mode, esp_reset, uart1_tx])]
    async fn init_esp(cx: init_esp::Context) {
        info!("Reseting the ESP");
        cx.local.esp_mode.set_high().ok();
        cx.local.esp_reset.set_low().ok();

        Timer::delay(1.secs()).await;
        cx.local.esp_reset.set_high().ok();
        Timer::delay(1.secs()).await;

        info!("Done");

        cx.local.uart1_tx.write_full_blocking(b"AT\r\n");
    }

    /// Hardware task that reads bytes from the UART an publishes messages!
    #[task(binds = UART1_IRQ, local = [uart1_rx])]
    fn uart1_esp32(cx: uart1_esp32::Context) {
        let mut buf = [0; 256];
        let mut index = 0;

        match cx.local.uart1_rx.read_raw(&mut buf) {
            Ok(n) => {
                // info!("RX: '{}'", core::str::from_utf8(&buf[..n]).unwrap());
                index += n;

                if index >= buf.len() - 1 {
                    warn!("Buffer reached end without finding a newline!");
                }

                loop {
                    // info!(
                    //     "Index: {}, Buffer: '{}'",
                    //     index,
                    //     from_utf8(&buf[..index]).unwrap()
                    // );
                    let mut found = false;

                    for i in 0..index.saturating_sub(1) {
                        if buf[i] == b'\r' && buf[i + 1] == b'\n' {
                            // found!, extract without the \r\n
                            let cmd = &buf[0..i];

                            // search the recently added bytes to find "\r\n"
                            info!(
                                "FOUND \\r\\n at {}: '{}'",
                                i,
                                core::str::from_utf8(&cmd).unwrap()
                            );

                            // reset the buffer by moving the remaining bytes to the front
                            let first_other_byte = i + 2;
                            // info!("copy range: {}", first_other_byte..index);
                            buf.copy_within(first_other_byte..index, 0);
                            index = index - first_other_byte;

                            found = true;
                            break; // break the for loop and restart it
                        }
                    }

                    if !found {
                        // break the outer loop if none was found
                        break;
                    }
                }
            }
            Err(nb::Error::WouldBlock) => info!("Would Block"),
            Err(nb::Error::Other(e)) => {
                match e.err_type {
                    ReadErrorType::Overrun => error!("Overrun"),
                    ReadErrorType::Break => error!("Break"),
                    ReadErrorType::Parity => error!("Parity"),
                    ReadErrorType::Framing => error!("Framing"),
                };
                error!("Data: '{}'", core::str::from_utf8(&e.discarded).unwrap());
            }
        }
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
