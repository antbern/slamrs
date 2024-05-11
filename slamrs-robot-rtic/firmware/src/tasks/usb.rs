use crate::{app::usb_irq, app::usb_sender, util::channel_send};
use defmt::{info, warn};
use library::event::Event;
use library::slamrs_message::CommandMessage;
use rtic::mutex_prelude::*;
use usb_device::prelude::*;

pub fn usb_irq(mut cx: usb_irq::Context) {
    let usb_dev = &mut cx.local.usb_device;

    (cx.shared.usb_serial, cx.shared.usb_active).lock(|serial, usb_active| {
        // check if we are conected or not and emit the right event
        let is_connected = serial.dtr() && usb_dev.state() == UsbDeviceState::Configured;
        if is_connected && !*usb_active {
            channel_send(cx.local.usb_event_sender, Event::Connected, "usb_irq");
        } else if !is_connected && *usb_active {
            channel_send(cx.local.usb_event_sender, Event::Disconnected, "usb_irq");
        }
        *usb_active = is_connected;

        // Poll the USB driver with all of our supported USB Classes
        if usb_dev.poll(&mut [serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    let data = &buf[..count];
                    match library::slamrs_message::bincode::decode_from_slice::<CommandMessage, _>(
                        data,
                        library::slamrs_message::bincode::config::standard(),
                    ) {
                        Ok((event, len)) => {
                            if len != count {
                                warn!("Data packet was not fully consumed");
                            }
                            channel_send(
                                cx.local.usb_event_sender,
                                Event::Command(event),
                                "usb_irq",
                            );
                        }
                        Err(_e) => {
                            warn!("Failed to deserialize data");
                        }
                    }
                }
            }
        }
    });
}

pub async fn usb_sender(mut cx: usb_sender::Context<'_>) {
    loop {
        match cx.local.robot_message_receiver_usb.recv().await {
            Ok(message) => {
                if !cx.shared.usb_active.lock(|usb_active| *usb_active) {
                    warn!("USB not active, dropping message");
                    continue;
                }

                let mut buffer = [0u8; 64];
                match library::slamrs_message::bincode::encode_into_slice(
                    message,
                    &mut buffer,
                    library::slamrs_message::bincode::config::standard(),
                ) {
                    Ok(len) => {
                        cx.shared.usb_serial.lock(|serial| {
                            let mut wr_ptr = &buffer[..len];
                            while !wr_ptr.is_empty() {
                                let _ = serial.write(wr_ptr).map(|len| {
                                    wr_ptr = &wr_ptr[len..];
                                });
                            }
                        });
                    }
                    Err(_e) => {
                        warn!("Error encoding message");
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
}
