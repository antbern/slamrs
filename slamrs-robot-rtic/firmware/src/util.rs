use defmt::{error, warn};
use library::parse_at::EspMessage;
use rtic_sync::channel::{Sender, TrySendError};

use crate::app::EspChannelReceiver;

pub async fn wait_for_message(receiver: &mut EspChannelReceiver, value: EspMessage) {
    while let Ok(m) = receiver.recv().await {
        if m == value {
            return;
        } else if m == EspMessage::Error {
            error!("got error message while waiting for {}", value);
        } else {
            warn!("got message {} while waiting for {}", m, value);
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
