#![no_std]

// export `bincode` so that the same version is available to all users of this crate
pub use bincode;

use bincode::{Decode, Encode};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Encode, Decode, Debug, PartialEq)]
pub enum CommandMessage {
    Ping,
    NeatoOn,
    NeatoOff,
    Drive { left: f32, right: f32 },
}

#[derive(Copy, Clone, Encode, Decode, Debug)]
pub enum RobotMessage {
    ScanFrame(ScanFrame),
    Pong,
}

#[derive(Copy, Clone, Encode, Decode, Debug)]
pub struct ScanFrame {
    pub scan_data: [u8; 1980],
    pub odometry: [f32; 2],
    pub rpm: u16,
}
