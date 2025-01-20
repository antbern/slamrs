use defmt::{error, warn};
use futures::FutureExt;
use library::parse_at::EspMessage;
use rp_pico::hal::fugit::ExtU64;
use rtic_monotonics::Monotonic as _;
use rtic_sync::channel::{Sender, TrySendError};

use crate::{app::EspChannelReceiver, Mono};

pub async fn wait_for_message(receiver: &mut EspChannelReceiver, wait_for: EspMessage) {
    // try to receive with timeout
    loop {
        futures::select_biased! {
            value = receiver.recv().fuse() => {
                match value {
                    Err(_) => {
                        error!("Error receiving from channel");
                        break; // TODO: can we handle errors in any other way?
                    },
                    Ok(m) if m == wait_for =>{
                        break;
                    },
                    Ok(EspMessage::Error) => {
                        error!("got error message while waiting for {}", wait_for);
                    },
                    Ok(other) => {
                        warn!("got message {} while waiting for {}", other, wait_for);
                    }
                }
            }
            _ = Mono::delay(1000u64.millis()).fuse() => {
                error!("Timeout while waiting for {}", wait_for);
                break;
            }
        }
    }
}

/// Helper function for trying to send something to a Sender MPSC channel, or print a warning
/// message if an error occurred
pub fn channel_send<T: defmt::Format, const N: usize>(
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
