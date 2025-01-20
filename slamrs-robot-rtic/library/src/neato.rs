use crate::pool::{BufferPool, OwnedBuffer, SharedBuffer};

enum RunningParserState {
    LookingForStart { previous_byte: u8 },
    CollectingBytes { index: usize },
}

pub struct RunningParser {
    buffer: Option<OwnedBuffer<'static, 1980>>,
    state: RunningParserState,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NeatoFrame {
    pub data: SharedBuffer<'static, 1980>,
}

impl RunningParser {
    pub const fn new() -> Self {
        Self {
            buffer: None,
            state: RunningParserState::LookingForStart { previous_byte: 0 },
        }
    }
    pub fn consume<R: embedded_hal_nb::serial::Read<u8>, const M: usize>(
        &mut self,
        reader: &mut R,
        pool: &'static BufferPool<1980, M>,
        mut callback: impl FnMut(NeatoFrame),
    ) {
        let buffer = self
            .buffer
            .get_or_insert_with(|| pool.acquire().expect("pool should not be empty"));

        loop {
            match reader.read() {
                Ok(byte) => {
                    use RunningParserState::*;
                    self.state = match self.state {
                        LookingForStart {
                            previous_byte: last_byte,
                        } => {
                            if last_byte == 0xFA && byte == 0xA0 {
                                buffer[0] = last_byte;
                                buffer[1] = byte;
                                CollectingBytes { index: 2 }
                            } else {
                                LookingForStart {
                                    previous_byte: byte,
                                }
                            }
                        }
                        CollectingBytes { index } => {
                            buffer[index] = byte;

                            if index < buffer.len() - 1 {
                                CollectingBytes { index: index + 1 }
                            } else {
                                // buffer is full -> parse and return it!

                                // replace old buffer with a new one
                                let full_buffer = core::mem::replace(
                                    buffer,
                                    pool.acquire().expect("pool should not be empty"),
                                );

                                callback(NeatoFrame {
                                    data: full_buffer.shared(),
                                });

                                // next restart looking for frame start
                                LookingForStart { previous_byte: 0 }
                            }
                        }
                    };
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {
                    // TODO: what to do here? Return?
                    break;
                }
            }
        }
    }
}

impl NeatoFrame {
    /// Parse the raw RPM value (RPM * 64) from the frame
    pub fn parse_rpm_raw(&self) -> u16 {
        let rpm_low = self.data[2];
        let rpm_high = self.data[3];

        ((rpm_high as u16) << 8) | (rpm_low as u16)
    }

    /// Parse the RPM value from the frame
    pub fn parse_rpm(&self) -> u16 {
        self.parse_rpm_raw() / 64
    }
}
