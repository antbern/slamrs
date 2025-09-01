use defmt::{debug, error, info, warn};
use embedded_hal::digital::OutputPin;
use futures::FutureExt;
use library::{
    event::Event,
    parse_at::{EspMessage, ParsedMessage},
    slamrs_message::RobotMessageBorrowed,
};

use rp2040_hal as hal;

use hal::{dma::SingleChannel, fugit::ExtU64};
use rtic::Mutex;
use rtic_monotonics::Monotonic;

use crate::{
    app::{dma3_esp, init_esp, uart1_esp32, DATA_PACKET_SIZE},
    tasks::heartbeat::{Color, LedStatus, Speed},
    util::{channel_send, wait_for_message},
    Mono,
};

static mut DMA_BUFFER: [u8; 2048] = [0u8; 2048];

/// Task that initializes and handles the ESP WIFI connection
pub async fn init_esp(mut cx: init_esp::Context<'_>) {
    info!("Reseting the ESP");

    cx.shared
        .led_status
        .lock(|s| *s = LedStatus::Blinking(Color::Blue, Speed::Fast));

    cx.local.esp_mode.set_high().ok();
    cx.local.esp_reset.set_low().ok();

    Mono::delay(1.secs()).await;
    cx.local.esp_reset.set_high().ok();
    Mono::delay(1.secs()).await;

    // take out the values we need to own for DMA to work

    let tx = cx.local.uart1_tx.as_ref().expect("should not be None");
    // read messages from the device and advance the inner state machine

    wait_for_message(cx.local.esp_receiver, EspMessage::Ready).await;

    // configure some stuff
    tx.write_full_blocking(b"AT+SYSMSG=0\r\n");
    wait_for_message(cx.local.esp_receiver, EspMessage::Ok).await;

    tx.write_full_blocking(b"AT+CWSTATE?\r\n");

    // enum State {
    //     Ready,
    //     WifiConnectedAndIp,
    //     Listening,
    //     ClientConnected,
    // }
    // let mut state = State::Ready;

    info!("Done, starting message loop");

    cx.shared
        .led_status
        .lock(|s| *s = LedStatus::Blinking(Color::Blue, Speed::Medium));

    let mut is_connected = false;

    loop {
        futures::select_biased! {
            value = cx.local.robot_message_receiver.recv().fuse() => {
                if let Ok(value) = value {
                    if !is_connected {
                        debug!("Not connected, dropping message");
                        continue;
                    }

                    info!("Sending: {:?}", value);
                    let start = Mono::now();

                    // convert to the type we can serialize
                    let message: &RobotMessageBorrowed = &(&value).into();

                    #[expect(clippy::deref_addrof)]
                    let buffer = unsafe { &mut *&raw mut DMA_BUFFER };
                    match library::slamrs_message::bincode::encode_into_slice(message, buffer, library::slamrs_message::bincode::config::standard()) {
                        Ok(len) => {
                            let elapsed = Mono::now() - start;
                            debug!("Encoded message with length: {} in {} micros", len, elapsed.to_micros());

                            // take the things we need to hold onto out
                            let tx = cx.local.uart1_tx.take().expect("should not be None");
                            let mut dma = cx.local.esp_tx_dma.take().expect("should not be None");

                            // send start command including ASCII formatted length
                            let mut len_str_buffer = [0u8; 10];
                            let len_str_length = library::util::format_base_10(len as u32, &mut len_str_buffer).unwrap();
                            tx.write_full_blocking(b"AT+CIPSEND=0,");
                            tx.write_full_blocking(&len_str_buffer[..len_str_length]);
                            tx.write_full_blocking(b"\r\n");
                            wait_for_message(cx.local.esp_receiver, EspMessage::Ok).await;
                            // wait_for_message(cx.local.esp_receiver, EspMessage::DataPrompt).await;

                            // send payload (with a baud rate of 115200, sending 1992 bytes takes around 170ms - so we use the DMA to do it non-blocking)
                            let start = Mono::now();

                            // make sure irq is cleared and empty any existing items
                            dma.check_irq0();
                            while cx.local.esp_receiver.try_recv().is_ok() {}

                            // start the DMA transfer
                            let tx_transfer = hal::dma::single_buffer::Config::new(dma, &buffer[..len], tx).start();
                            // wait for dma to finish, then we continue
                            let _ = cx.local.dma3_receiver.recv().await;
                            let (dma, _, tx) = tx_transfer.wait();


                            let elapsed = Mono::now() - start;
                            debug!("Writing data took: {} micros", elapsed.to_micros());
                            wait_for_message(cx.local.esp_receiver, EspMessage::SendOk).await;

                            // put them back again after using
                            *cx.local.uart1_tx = Some(tx);
                            *cx.local.esp_tx_dma = Some(dma)

                        }
                        Err(_e) => {
                            error!("Error encoding message");
                        }
                    }

                }
            },
            value = cx.local.esp_receiver.recv().fuse() => {
                if let Ok(m) = value {
                    let tx = cx.local.uart1_tx.as_ref().expect("should not be None");
                    info!("Got message: {}", m);
                    match m {
                        EspMessage::GotIP => {
                            cx.shared
                                .led_status
                                .lock(|s| *s = LedStatus::Blinking(Color::Cyan, Speed::Fast));
                            // state = State::WifiConnectedAndIp;
                            // enable mdns
                           tx
                                .write_full_blocking(b"AT+MDNS=1,\"robot\",\"_tcp\",8080\r\n");
                            wait_for_message(cx.local.esp_receiver, EspMessage::Ok).await;
                            // start the server

                            info!("Enabling Multiple Connections");
                            tx.write_full_blocking(b"AT+CIPMUX=1\r\n");
                            wait_for_message(cx.local.esp_receiver, EspMessage::Ok).await;
                            Mono::delay(1.secs()).await;

                            tx
                                .write_full_blocking(b"AT+CIPSERVERMAXCONN=1\r\n");
                            wait_for_message(cx.local.esp_receiver, EspMessage::Ok).await;

                            info!("Starting server");
                            tx
                                .write_full_blocking(b"AT+CIPSERVER=1,8080\r\n");
                            wait_for_message(cx.local.esp_receiver, EspMessage::Ok).await;

                            // state = State::Listening;
                            info!("Listening");

                            cx.shared
                                .led_status
                                .lock(|s| *s = LedStatus::Blinking(Color::Green, Speed::Slow));
                        }
                        EspMessage::ClientConnect => {
                            // state = State::ClientConnected;
                            is_connected = true;
                            channel_send(cx.local.esp_event_sender, Event::Connected, "ESP");
                        }
                        EspMessage::ClientDisconnect => {
                            // state = State::Listening;
                            is_connected = false;
                            channel_send(cx.local.esp_event_sender, Event::Disconnected, "ESP");
                        }
                        _ => {}
                    }
                }
            },
        };
    }
}

/// Hardware task that reads bytes from the UART and publishes messages!
pub fn uart1_esp32(cx: uart1_esp32::Context<'_>) {
    let sender = cx.local.esp_sender;
    let rx = cx.local.uart1_rx;
    cx.local.parser.consume(rx, move |message| match message {
        ParsedMessage::Simple(m) => channel_send(sender, m, "uart1_esp32"),
        ParsedMessage::ReceivedData(data) => {
            info!("got data: {}", data);
            // this is not very efficient , but it works for now
            let mut buffer = [0u8; DATA_PACKET_SIZE];
            if data.len() > buffer.len() {
                warn!("Data too long, ignoring");
                return;
            }
            buffer[..data.len()].copy_from_slice(data);
            channel_send(
                cx.local.esp_data_sender,
                (data.len(), buffer),
                "uart1_esp32",
            );
        }
    });
}

/// Hardware task that fires on DMA_IRQ_0 to notify that the dma transfer is done
pub fn dma3_esp(cx: dma3_esp::Context<'_>) {
    channel_send(cx.local.dma3_sender, (), "dma3_irq_esp");

    // SAFETY: we only clear the interrupt in the DMA controller
    unsafe {
        let dma = hal::pac::DMA::steal();
        use hal::dma::ChannelIndex;
        dma.ints1().write(|w| w.bits(1 << hal::dma::CH3::id()));
    };

    // clear the interrupt to avoid firing again
    hal::pac::NVIC::unpend(hal::pac::interrupt::DMA_IRQ_0);
}
