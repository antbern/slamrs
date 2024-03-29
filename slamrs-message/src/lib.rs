#![no_std]

use bincode::{Decode, Encode};

#[derive(Copy, Clone, Encode, Decode, Debug)]
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
