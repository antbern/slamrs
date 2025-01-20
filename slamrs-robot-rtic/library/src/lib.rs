#![cfg_attr(not(test), no_std)]
#![deny(unsafe_code)]
#![allow(clippy::new_without_default)]

pub mod event;
pub mod neato;
pub mod parse_at;
pub mod pool;
pub mod util;

pub use slamrs_message;
