use crate::{app::usb_irq, util::channel_send};
use defmt::warn;
use library::event::Event;
use library::slamrs_message::CommandMessage;
// USB Device support
use usb_device::prelude::*;

pub fn usb_irq(cx: usb_irq::Context) {
    let usb_dev = &mut cx.local.usb_state.usb_device;
    let serial = &mut cx.local.usb_state.usb_serial;

    // check if we are conected or not and emitt the right event
    let is_connected = serial.dtr() && usb_dev.state() == UsbDeviceState::Configured;
    if is_connected && !*cx.local.was_connected {
        channel_send(cx.local.usb_event_sender, Event::Connected, "usb_irq");
    } else if !is_connected && *cx.local.was_connected {
        channel_send(cx.local.usb_event_sender, Event::Disconnected, "usb_irq");
    }
    *cx.local.was_connected = is_connected;

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
                        channel_send(cx.local.usb_event_sender, Event::Command(event), "usb_irq");
                    }
                    Err(_e) => {
                        warn!("Failed to deserialize data");
                    }
                }
            }
        }
    }
}
