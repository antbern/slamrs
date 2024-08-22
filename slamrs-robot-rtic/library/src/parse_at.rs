use core::str::FromStr;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, PartialEq)]
pub enum ParsedMessage<'a> {
    Simple(EspMessage),
    ReceivedData(&'a [u8]),
}

/// A simple reference-less message received
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum EspMessage {
    Ok,
    Error,
    Ready,
    WifiConnected,
    GotIP,
    ClientConnect,
    ClientDisconnect,
    WifiStatus(WifiStatus),
    DataPrompt,
    SendOk,
}
/// Enum to describe the  Wifi status
/// From the docs:
/// 0: ESP32 station has not started any Wi-Fi connection.
/// 1: ESP32 station has connected to an AP, but does not get an IPv4 address yet.
/// 2: ESP32 station has connected to an AP, and got an IPv4 address.
/// 3: ESP32 station is in Wi-Fi connecting or reconnecting state.
/// 4: ESP32 station is in Wi-Fi disconnected state.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum WifiStatus {
    NotConnected,
    ConnectedNoIp,
    ConnectedWithIp,
    Connecting,
    Disconnected,
}

impl FromStr for EspMessage {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "OK" => Ok(EspMessage::Ok),
            "ERROR" => Ok(EspMessage::Error),
            "ready" => Ok(EspMessage::Ready),
            "WIFI CONNECTED" => Ok(EspMessage::WifiConnected),
            "WIFI GOT IP" => Ok(EspMessage::GotIP),
            "0,CONNECT" => Ok(EspMessage::ClientConnect),
            "0,CLOSED" => Ok(EspMessage::ClientDisconnect),
            ">" => Ok(EspMessage::DataPrompt),
            "SEND OK" => Ok(EspMessage::SendOk),
            other => {
                if other.starts_with("+CWSTATE:") {
                    if let Some(status) = other.chars().nth(9) {
                        return match status {
                            '0' => Ok(EspMessage::WifiStatus(WifiStatus::NotConnected)),
                            '1' => Ok(EspMessage::WifiStatus(WifiStatus::ConnectedNoIp)),
                            '2' => Ok(EspMessage::WifiStatus(WifiStatus::ConnectedWithIp)),
                            '3' => Ok(EspMessage::WifiStatus(WifiStatus::Connecting)),
                            '4' => Ok(EspMessage::WifiStatus(WifiStatus::Disconnected)),
                            _ => Err(()),
                        };
                    }
                }

                Err(())
            }
        }
    }
}

pub struct AtParser<const N: usize> {
    buffer: [u8; N],
    index: usize,
}

impl<const N: usize> AtParser<N> {
    pub const fn new() -> Self {
        Self {
            buffer: [0; N],
            index: 0,
        }
    }

    pub fn consume<R: embedded_hal_nb::serial::Read<u8>>(
        &mut self,
        reader: &mut R,
        callback: impl FnMut(ParsedMessage<'_>),
    ) {
        // first exhaust the reader, then try to parse the received bytes
        loop {
            match reader.read() {
                Ok(data) => {
                    self.buffer[self.index] = data;
                    self.index += 1;

                    if self.index >= self.buffer.len() {
                        // buffer is full, stop reading
                        break;
                    }
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {
                    // TODO: what to do here? Return?
                    break;
                }
            }
        }

        // we now have new data, parse the buffer!
        self.process_buffer(callback)
    }

    fn process_buffer(&mut self, mut callback: impl FnMut(ParsedMessage<'_>)) {
        loop {
            // info!(
            //     "Index: {}, Buffer: '{}'",
            //     index,
            //     from_utf8(&buf[..index]).unwrap()
            // );
            let mut found = false;

            let current_data = &self.buffer[0..self.index];

            // check if the current line starts with any URC (even though we haven't hit
            // \r\n yet
            if current_data.len() > 7 && &current_data[..7] == b"+IPD,0," {
                // info!("FOUND +IDP URC!");

                match parse_ipd(current_data) {
                    Ok((used, data)) => {
                        callback(ParsedMessage::ReceivedData(data));
                        // info!("Received data: {}", data);
                        // reset the buffer by moving the remaining bytes to the front
                        let first_other_byte = used;
                        self.buffer.copy_within(first_other_byte..self.index, 0);
                        self.index = self.index - first_other_byte;

                        found = true;
                    }
                    Err(_err) => {
                        // error!("Error parsing IPD: {}", err);
                    }
                }
            }

            if found {
                continue;
            }

            for i in 0..self.index.saturating_sub(1) {
                if self.buffer[i] == b'\r' && self.buffer[i + 1] == b'\n' {
                    // found!, extract without the \r\n
                    let cmd = &self.buffer[0..i];

                    // try to parse the string representation
                    if let Ok(s) = core::str::from_utf8(cmd) {
                        if !s.is_empty() {
                            if let Ok(m) = s.parse() {
                                callback(ParsedMessage::Simple(m));
                            } else {
                                #[cfg(feature = "defmt")]
                                defmt::warn!("unrecognized command'{}'", s);
                            }
                        }
                    } else {
                        // debug!("invalid utf8 received");
                    }

                    // reset the buffer by moving the remaining bytes to the front
                    let first_other_byte = i + 2;
                    // info!("copy range: {}", first_other_byte..index);
                    self.buffer.copy_within(first_other_byte..self.index, 0);
                    self.index = self.index - first_other_byte;

                    found = true;
                    break; // break the for loop and restart it
                }
            }

            if !found {
                // break the outer loop if none was found
                break;
            }
        }
    }
}

/// Tries to parse the +IPD message and returns a tuple with the number of bytes used as well
/// as a slice containing the data bytes.
pub fn parse_ipd<'a>(cmd: &'a [u8]) -> Result<(usize, &'a [u8]), &'static str> {
    let separator = cmd
        .iter()
        .enumerate()
        .find(|x| x.1 == &b':')
        .ok_or("No separator found")?
        .0;

    let length_str =
        core::str::from_utf8(&cmd[7..separator]).map_err(|_| "Length string not valid Utf8 ")?;

    let length_usize = length_str
        .parse::<usize>()
        .map_err(|_| "Length string is not valid usize")?;

    let remaining_data = &cmd[separator + 1..];
    if remaining_data.len() >= length_usize {
        Ok((
            7 + length_str.len() + 1 + length_usize,
            &remaining_data[..length_usize],
        ))
    } else {
        Err("All data not present")
    }
}

// #[cfg(all(test, target_arch = "x86_64"))]
// extern crate std;

// #[cfg(all(test, target_arch = "x86_64"))]
#[cfg(test)]
mod tests {
    use std::vec::Vec;

    use embedded_hal_nb::serial::Read;

    use super::*;

    struct VecReader {
        strings: Vec<Vec<u8>>,
        current_word: usize,
        current_byte: usize,
    }

    impl VecReader {
        fn new(strings: &[&str]) -> Self {
            let bytes = strings.into_iter().map(|s| s.bytes().collect()).collect();
            Self {
                strings: bytes,
                current_word: 0,
                current_byte: 0,
            }
        }
        fn is_exhausted(&self) -> bool {
            self.current_word >= self.strings.len()
        }
    }

    impl embedded_hal_nb::serial::ErrorType for VecReader {
        type Error = core::convert::Infallible;
    }

    impl embedded_hal_nb::serial::Read<u8> for VecReader {
        fn read(&mut self) -> nb::Result<u8, Self::Error> {
            if self.current_word < self.strings.len() {
                if self.current_byte >= self.strings[self.current_word].len() {
                    self.current_byte = 0;
                    self.current_word += 1;
                    Err(nb::Error::WouldBlock)
                } else {
                    let value = self.strings[self.current_word][self.current_byte];
                    self.current_byte += 1;
                    Ok(value)
                }
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }

    #[test]
    fn test_vecreader() {
        let mut reader = VecReader::new(&["one", "two", "e"]);
        assert_eq!(reader.read(), nb::Result::Ok(b'o'));
        assert_eq!(reader.read(), nb::Result::Ok(b'n'));
        assert_eq!(reader.read(), nb::Result::Ok(b'e'));
        assert_eq!(reader.read(), nb::Result::Err(nb::Error::WouldBlock));
        assert_eq!(reader.read(), nb::Result::Ok(b't'));
        assert_eq!(reader.read(), nb::Result::Ok(b'w'));
        assert_eq!(reader.read(), nb::Result::Ok(b'o'));
        assert_eq!(reader.read(), nb::Result::Err(nb::Error::WouldBlock));
        assert_eq!(reader.read(), nb::Result::Ok(b'e'));
        assert_eq!(reader.read(), nb::Result::Err(nb::Error::WouldBlock));
        assert_eq!(reader.read(), nb::Result::Err(nb::Error::WouldBlock));
    }

    #[test]
    fn test_parse_ipd() {
        let input = b"+IPD,0,5:hello";
        let (len, data) = parse_ipd(input).unwrap();
        assert_eq!(len, input.len());
        assert_eq!(data, b"hello");
    }

    #[test]
    fn test_parse_ipd_exess_data() {
        let input = b"+IPD,0,4:hello";
        let (len, data) = parse_ipd(input).unwrap();
        assert_eq!(len, input.len() - 1);
        assert_eq!(data, b"hell");
    }

    #[test]
    fn test_consume_strings() {
        let input = &[
            "OK\r\n",
            "ERROR\r\n",
            "ready\r\n",
            "WIFI CONNECTED\r\n",
            "WIFI",
            " CONNECTED\r\n",
            "WIFI GOT IP\r\n",
            "0,CONNECT\r\n",
            "0,CLOSED\r\n",
            "0,CLOSED\r",
            "\n",
            "OK\r\nERROR\r\n",
        ];

        let mut reader = VecReader::new(input);
        let mut found_values = Vec::new();

        let mut parser: AtParser<256> = AtParser::new();
        while !reader.is_exhausted() {
            parser.consume(&mut reader, |m| match m {
                ParsedMessage::Simple(m) => found_values.push(m),
                o => panic!("Unexpected parsed message: {:?}", o),
            });
        }
        assert_eq!(
            found_values,
            vec![
                EspMessage::Ok,
                EspMessage::Error,
                EspMessage::Ready,
                EspMessage::WifiConnected,
                EspMessage::WifiConnected,
                EspMessage::GotIP,
                EspMessage::ClientConnect,
                EspMessage::ClientDisconnect,
                EspMessage::ClientDisconnect,
                EspMessage::Ok,
                EspMessage::Error,
            ]
        );
    }
}
