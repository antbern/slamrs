#![no_std]
#![deny(unsafe_code)]

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

#[cfg(test)]
mod tests {
    use super::*;

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
}
