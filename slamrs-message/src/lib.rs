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
    /// Set downsampling factor to only report every n-th scan frame
    SetDownsampling {
        every: u8,
    },
    Drive {
        left: f32,
        right: f32,
    },
    SetMotorPiParams {
        kp: f32,
        ki: f32,
    },
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Encode, Decode, Debug)]
pub enum RobotMessage {
    ScanFrame(ScanFrame),
    Pong,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Encode, Decode, Debug)]
pub struct ScanFrame {
    pub scan_data: [u8; 1980],
    pub odometry: [f32; 2],
    pub rpm: u16,
}

/// A borrowed version of the [`RobotMessage`] type that can be used to serialize the message without copying the data.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Encode, Debug)]
pub enum RobotMessageBorrowed<'a> {
    ScanFrame(ScanFrameBorrowed<'a>),
    Pong,
}

/// A borrowed version of the [`ScanFrame`] type that can be used to serialize the message without copying the data.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Encode, Debug)]
pub struct ScanFrameBorrowed<'a> {
    pub scan_data: &'a [u8; 1980],
    pub odometry: [f32; 2],
    pub rpm: u16,
}
