use defmt::info;
use embedded_hal::digital::v2::OutputPin;
use library::{
    event::Event,
    parse_at::{EspMessage, ParsedMessage},
};
use rp_pico::hal::fugit::ExtU64;
use rtic_monotonics::rp2040::Timer;

use crate::{
    app::{init_esp, uart1_esp32},
    util::{channel_send, wait_for_message},
};

/// Task that initializes and handles the ESP WIFI connection
pub async fn init_esp(cx: init_esp::Context<'_>) {
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
    wait_for_message(cx.local.esp_receiver, EspMessage::Ok).await;

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
                    // enable mdns
                    cx.local
                        .uart1_tx
                        .write_full_blocking(b"AT+MDNS=1,\"robot\",\"_tcp\",8080\r\n");
                    wait_for_message(cx.local.esp_receiver, EspMessage::Ok).await;
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
                        .write_full_blocking(b"AT+CIPSERVER=1,8080\r\n");
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

/// Hardware task that reads bytes from the UART an publishes messages!
pub fn uart1_esp32(cx: uart1_esp32::Context<'_>) {
    let sender = cx.local.esp_sender;
    let rx = cx.local.uart1_rx;
    cx.local.parser.consume(rx, move |message| match message {
        ParsedMessage::Simple(m) => channel_send(sender, m, "uart1_esp32"),
        ParsedMessage::ReceivedData(data) => {
            info!("got data: {}", data);
            // this is not very efficient , but it works for now
            let mut buffer = [0u8; 64];
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
