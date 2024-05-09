#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Event {
    Connected,
    Disconnected,
    Command(slamrs_message::CommandMessage),
}
