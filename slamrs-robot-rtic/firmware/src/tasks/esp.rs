use defmt::{error, info, warn};
use embedded_hal::digital::OutputPin;
use futures::FutureExt;
use library::{
    event::Event,
    parse_at::{EspMessage, ParsedMessage},
};
use rp_pico::hal::fugit::ExtU64;
use rtic::Mutex;
use rtic_monotonics::Monotonic;

use crate::{
    app::{init_esp, uart1_esp32, DATA_PACKET_SIZE},
    tasks::heartbeat::{Color, LedStatus, Speed},
    util::{channel_send, wait_for_message},
    Mono,
};

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

    // read messages from the device and advance the inner state machine

    wait_for_message(cx.local.esp_receiver, EspMessage::Ready).await;

    // configure some stuff
    cx.local.uart1_tx.write_full_blocking(b"AT+SYSMSG=0\r\n");
    wait_for_message(cx.local.esp_receiver, EspMessage::Ok).await;

    cx.local.uart1_tx.write_full_blocking(b"AT+CWSTATE?\r\n");

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
    loop {
        futures::select_biased! {
            value = cx.local.robot_message_receiver.recv().fuse() => {
                if let Ok(value) = value {
                    info!("Sending: {:?}", value);
                    let mut buffer = [0u8;2048];
                    match library::slamrs_message::bincode::encode_into_slice(value, &mut buffer, library::slamrs_message::bincode::config::standard()) {
                        Ok(len) => {
                            let mut len_buffer = [0u8; 10];
                            let len_length = library::util::format_base_10(len as u32, &mut len_buffer).unwrap();
                            info!("Encoded message: {:?} with length: {}", &buffer[..len], &len_buffer[..len_length]);
                            cx.local.uart1_tx.write_full_blocking(b"AT+CIPSEND=0,");
                            cx.local.uart1_tx.write_full_blocking(&len_buffer[..len_length]);
                            cx.local.uart1_tx.write_full_blocking(b"\r\n");
                            wait_for_message(cx.local.esp_receiver, EspMessage::Ok).await;
                            // wait_for_message(cx.local.esp_receiver, EspMessage::DataPrompt).await;
                            cx.local.uart1_tx.write_full_blocking(&buffer[..len]);
                            wait_for_message(cx.local.esp_receiver, EspMessage::SendOk).await;
                        }
                        Err(_e) => {
                            error!("Error encoding message");
                        }
                    }
                }
            },
            value = cx.local.esp_receiver.recv().fuse() => {
                if let Ok(m) = value {
                    info!("Got message: {}", m);
                    match m {
                        EspMessage::GotIP => {
                            cx.shared
                                .led_status
                                .lock(|s| *s = LedStatus::Blinking(Color::Cyan, Speed::Fast));
                            // state = State::WifiConnectedAndIp;
                            // enable mdns
                            cx.local
                                .uart1_tx
                                .write_full_blocking(b"AT+MDNS=1,\"robot\",\"_tcp\",8080\r\n");
                            wait_for_message(cx.local.esp_receiver, EspMessage::Ok).await;
                            // start the server

                            info!("Enabling Multiple Connections");
                            cx.local.uart1_tx.write_full_blocking(b"AT+CIPMUX=1\r\n");
                            wait_for_message(cx.local.esp_receiver, EspMessage::Ok).await;
                            Mono::delay(1.secs()).await;

                            cx.local
                                .uart1_tx
                                .write_full_blocking(b"AT+CIPSERVERMAXCONN=1\r\n");
                            wait_for_message(cx.local.esp_receiver, EspMessage::Ok).await;

                            info!("Starting server");
                            cx.local
                                .uart1_tx
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
                            channel_send(cx.local.esp_event_sender, Event::Connected, "ESP");
                        }
                        EspMessage::ClientDisconnect => {
                            // state = State::Listening;
                            channel_send(cx.local.esp_event_sender, Event::Disconnected, "ESP");
                        }
                        _ => {}
                    }
                }
            },
        };
    }
}

/// Hardware task that reads bytes from the UART an publishes messages!
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
