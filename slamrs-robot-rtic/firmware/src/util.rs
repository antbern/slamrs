use defmt::{error, warn};
use library::parse_at::EspMessage;
use rp_pico::hal::uart::{ReadErrorType, UartDevice, ValidUartPinout};
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

/// Wrapper struct to implement the library read trait on the Uart Reader
pub struct Reader<'a, D: UartDevice, P: ValidUartPinout<D>> {
    inner: &'a mut rp_pico::hal::uart::Reader<D, P>,
}

impl<'a, D: UartDevice, P: ValidUartPinout<D>> Reader<'a, D, P> {
    pub fn wrap(inner: &'a mut rp_pico::hal::uart::Reader<D, P>) -> Self {
        Self { inner }
    }
}

impl<D: UartDevice, P: ValidUartPinout<D>> library::Read for Reader<'_, D, P> {
    type Error = ReadErrorType;

    fn read(&mut self) -> pwm_pca9685::nb::Result<u8, Self::Error> {
        let byte: &mut [u8] = &mut [0; 1];

        match self.inner.read_raw(byte) {
            Ok(_) => Ok(byte[0]),
            Err(e) => match e {
                nb::Error::Other(inner) => Err(nb::Error::Other(inner.err_type)),
                nb::Error::WouldBlock => Err(nb::Error::WouldBlock),
            },
        }
    }
}
