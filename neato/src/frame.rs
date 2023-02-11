use std::{fs::File, io::Read, path::PathBuf};

use common::robot::{Measurement, Observation};

#[derive(Clone, Copy)]
pub struct NeatoFrame {
    pub distance: [u16; 360],
    pub strength: [u16; 360],
    pub valid: [u8; 360],
}

#[derive(Debug, Copy, Clone)]
struct Data {
    valid: bool,
    strength_warning: bool,
    strength: u16,
    distance: u16,
}

#[derive(Debug, Copy, Clone)]
struct Packet {
    index: u8,
    speed: u16,
    data: [Data; 4],
    checksum: bool,
}

#[derive(Debug, Copy, Clone)]
struct Revolution {
    packets: [Option<Packet>; 90],
}

impl Default for Revolution {
    fn default() -> Self {
        Self {
            packets: [None; 90],
        }
    }
}

impl Revolution {
    fn to_readings(&self) -> NeatoFrame {
        // extract all packets in order and insert them into a simpler data structure

        let mut distance = [0u16; 360];
        let mut strength = [0u16; 360];
        let mut valid = [0; 360];
        // let mut speed = [0u16; 360];

        // println!("[");

        for (i, p) in self.packets.iter().enumerate() {
            if let Some(p) = p {
                for j in 0..4 {
                    distance[i * 4 + j] = p.data[j].distance;
                    strength[i * 4 + j] = p.data[j].strength;
                    valid[i * 4 + j] = p.data[j].valid as u8;
                }
            }
        }

        NeatoFrame {
            distance,
            strength,
            valid,
        }

        // println!("],");
    }
}

fn parse_data(b: &[u8]) -> anyhow::Result<Data> {
    assert!(b.len() == 4);

    Ok(Data {
        valid: (b[1] & (1 << 7)) == 0,
        strength_warning: (b[1] & (1 << 6)) == 0,
        distance: b[0] as u16 | (((b[1] as u16) & 0x3F) << 8),
        strength: ((b[3] as u16) << 8) | b[2] as u16,
    })
}

fn calculate_checksum_and_validate(b: &[u8]) -> anyhow::Result<bool> {
    assert!(b.len() == 22);

    // convert data to words, little-endian
    let mut words = Vec::with_capacity(b.len() / 2);
    for i in 0..((b.len() - 2) / 2) {
        words.push(((b[2 * i + 1] as u32) << 8) | b[2 * i] as u32)
    }

    let mut chk32 = 0;
    for &d in words.iter() {
        chk32 = (chk32 << 1) + d;
    }

    let checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
    let checksum = (checksum & 0x7FFF) as u16;

    let cs = ((b[21] as u16) << 8) | b[20] as u16;
    // println!("{checksum} == {cs}");

    Ok(checksum == cs)
}

fn parse_packet(b: &[u8]) -> anyhow::Result<Packet> {
    assert!(b.len() == 22);

    Ok(Packet {
        index: b[1],
        speed: ((b[3] as u16) << 8) | b[2] as u16,
        data: [
            parse_data(&b[4..8])?,
            parse_data(&b[8..12])?,
            parse_data(&b[12..16])?,
            parse_data(&b[16..20])?,
        ],
        checksum: calculate_checksum_and_validate(b)?,
    })
}

fn parse_packets<R: Read>(reader: &mut R) -> anyhow::Result<Vec<NeatoFrame>> {
    // read all the bytes into a buffer for now
    let mut buf = Vec::new();
    reader.read_to_end(&mut buf)?;

    // let mut buf = vec![0u8; 16000];
    // reader.read_exact(&mut buf)?;

    let mut frames = Vec::new();

    let mut i: usize = 0;

    let mut r = Revolution::default();
    // 0xA0 = 0
    // 0xF9 = 90
    let mut last_index = 0;

    while i < buf.len() {
        if buf[i] == 0xFA && (buf.len() - i) >= 22 {
            // parse a packet!

            let packet = &buf[i..(i + 22)];

            // print!("Found 0xFA: ");

            // for b in packet {
            //     print!(" {:02x}", *b);
            // }
            // println!();

            let p = parse_packet(packet)?;
            // println!("{p:?}");

            // insert into the current revolution
            if !p.checksum {
                // println!("Invalid checksum, skipping...");
                i += 1;
                continue;
            }

            if let None = p.index.checked_sub(0xA0) {
                println!("Subtract underflow: {p:?}");
                println!("Skipping...");
                i += 1;
                continue;
            }

            let index = p.index - 0xA0;
            if index < last_index {
                // wrapped around to new revolution, print and instantiate new one
                // print!("Revolution: ");

                frames.push(r.to_readings());

                r = Revolution::default();
            }

            r.packets[index as usize] = Some(p);
            last_index = index;
        }

        i += 1;
    }

    Ok(frames)
}

pub fn load_neato_binary(path: &PathBuf) -> anyhow::Result<Vec<NeatoFrame>> {
    // load all contents of the file
    // let contents = fs::read(path)?;
    let mut file = File::open(path)?;
    let p = parse_packets(&mut file)?;

    Ok(p)
}

impl From<NeatoFrame> for Observation {
    fn from(value: NeatoFrame) -> Self {
        let mut m: Vec<Measurement> = Vec::new();

        for i in 0..value.distance.len() {
            m.push(Measurement {
                angle: (i as f64).to_radians(),
                distance: value.distance[i] as f64 / 1000.0,
                strength: value.strength[i] as f64,
                valid: value.valid[i] != 0,
            })
        }

        Observation { measurements: m }
    }
}
