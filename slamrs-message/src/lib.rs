#![cfg_attr(not(test), no_std)]

// export `bincode` so that the same version is available to all users of this crate
pub use bincode;

pub use postcard;

use bincode::{Decode, Encode};
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Encode, Decode, Debug, PartialEq)]
pub enum CommandMessage {
    Ping,
    NeatoOn,
    NeatoOff,
    /// Set downampling factor to only report every n-th scan frame
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
#[derive(Copy, Clone, Encode, Decode, Debug, Serialize, Deserialize)]
pub enum RobotMessage {
    ScanFrame(ScanFrame),
    Pong,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Encode, Decode, Debug, Serialize, Deserialize)]
pub struct ScanFrame {
    #[serde(with = "serde_bytes")]
    pub scan_data: [u8; 1980],
    pub odometry: [f32; 2],
    pub rpm: u16,
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn test_encode_bincode() {
        let msg = RobotMessage::ScanFrame(ScanFrame {
            scan_data: [1; 1980],
            odometry: [0.0; 2],
            rpm: 10,
        });

        let mut buffer = [0; 2048];

        let n = bincode::encode_into_slice(msg, &mut buffer, bincode::config::standard()).unwrap();
        println!("Encoded ScanFrame: {:?} with length: {}", &buffer[..n], n);

        let msg = RobotMessage::Pong;

        let mut buffer = [0; 2048];

        let n = bincode::encode_into_slice(msg, &mut buffer, bincode::config::standard()).unwrap();
        println!("Encoded Pong: {:?} with length: {}", &buffer[..n], n);
    }

    #[test]
    fn test_encode_postcard() {
        let msg = RobotMessage::ScanFrame(ScanFrame {
            scan_data: [1; 1980],
            odometry: [0.0; 2],
            rpm: 10,
        });

        let mut buffer = [0; 2048];

        let message = postcard::to_slice(&msg, &mut buffer).unwrap();
        println!(
            "Encoded ScanFrame: {:?} with length: {}",
            &message,
            message.len()
        );

        let msg = RobotMessage::Pong;

        let mut buffer = [0; 2048];

        let message = postcard::to_slice(&msg, &mut buffer).unwrap();
        println!(
            "Encoded Pong: {:?} with length: {}",
            &message,
            message.len()
        );
    }
}
