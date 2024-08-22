enum RunningParserState {
    LookingForStart { previous_byte: u8 },
    CollectingBytes { index: usize },
}

pub struct RunningParser {
    buffer: [u8; 1980],
    state: RunningParserState,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NeatoFrame<'a> {
    pub data: &'a [u8; 1980],
}

impl RunningParser {
    pub const fn new() -> Self {
        Self {
            buffer: [0u8; 1980],
            state: RunningParserState::LookingForStart { previous_byte: 0 },
        }
    }
    pub fn consume<R: embedded_hal_nb::serial::Read<u8>>(
        &mut self,
        reader: &mut R,
        mut callback: impl FnMut(NeatoFrame<'_>),
    ) {
        loop {
            match reader.read() {
                Ok(byte) => {
                    use RunningParserState::*;
                    self.state = match self.state {
                        LookingForStart {
                            previous_byte: last_byte,
                        } => {
                            if last_byte == 0xFA && byte == 0xA0 {
                                self.buffer[0] = last_byte;
                                self.buffer[1] = byte;
                                CollectingBytes { index: 2 }
                            } else {
                                LookingForStart {
                                    previous_byte: byte,
                                }
                            }
                        }
                        CollectingBytes { index } => {
                            self.buffer[index] = byte;

                            if index < self.buffer.len() - 1 {
                                CollectingBytes { index: index + 1 }
                            } else {
                                // buffer is full -> parse and return it!
                                callback(NeatoFrame { data: &self.buffer });

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

impl<'a> NeatoFrame<'a> {
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
