pub fn format_base_10(mut x: u32, buffer: &mut [u8]) -> Result<usize, ()> {
    let radix = 10;

    let mut i = 0;
    loop {
        let m = x % radix;
        x = x / radix;

        if i >= buffer.len() {
            return Err(());
        }

        // will panic if you use a bad radix (< 2 or > 36).
        buffer[i] = core::char::from_digit(m, radix).unwrap() as u8;
        i += 1;
        if x == 0 {
            break;
        }
    }
    buffer[..i].reverse();

    Ok(i)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_10() {
        let mut buffer = [0u8; 10];

        assert_eq!(format_base_10(1234, &mut buffer), Ok(4));
        assert_eq!(&buffer[..4], b"1234");
        assert_eq!(format_base_10(1000, &mut buffer), Ok(4));
        assert_eq!(&buffer[..4], b"1000");
        assert_eq!(format_base_10(0, &mut buffer), Ok(1));
        assert_eq!(&buffer[..1], b"0");
    }
}
