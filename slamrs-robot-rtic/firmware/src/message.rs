//! Contains internal copy of the communication types for better efficiency

use library::pool::SharedBuffer;
use library::slamrs_message::{RobotMessageBorrowed, ScanFrameBorrowed};

#[derive(Clone, defmt::Format)]
pub enum RobotMessageInternal {
    ScanFrame(ScanFrameInternal),
    Pong,
}

#[derive(Clone, defmt::Format)]
pub struct ScanFrameInternal {
    pub scan_data: SharedBuffer<'static, 1980>,
    pub odometry: [f32; 2],
    pub rpm: u16,
}

impl<'a> From<&'a RobotMessageInternal> for RobotMessageBorrowed<'a> {
    fn from(msg: &'a RobotMessageInternal) -> Self {
        match msg {
            RobotMessageInternal::ScanFrame(frame) => RobotMessageBorrowed::ScanFrame(frame.into()),
            RobotMessageInternal::Pong => RobotMessageBorrowed::Pong,
        }
    }
}

impl<'a> From<&'a ScanFrameInternal> for ScanFrameBorrowed<'a> {
    fn from(frame: &'a ScanFrameInternal) -> Self {
        ScanFrameBorrowed {
            scan_data: frame.scan_data.as_ref(),
            odometry: frame.odometry,
            rpm: frame.rpm,
        }
    }
}
