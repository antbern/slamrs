mod serial;
mod network;

pub use serial::{SerialConnection, SerialConnectionNodeConfig};
pub use network::{NetworkConnection, NetworkConnectionNodeConfig};

mod frame;

mod fileloader;
pub use fileloader::{FileLoader, FileLoaderNodeConfig};
