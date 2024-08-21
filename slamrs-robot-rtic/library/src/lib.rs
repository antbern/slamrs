#![cfg_attr(not(test), no_std)]
#![deny(unsafe_code)]

pub mod event;
pub mod neato;
pub mod parse_at;
pub mod util;

pub use slamrs_message;

pub trait Read {
    type Error;
    fn read(&mut self) -> nb::Result<u8, Self::Error>;
}
