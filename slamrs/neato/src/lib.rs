mod network;
mod serial;

pub use network::{NetworkConnection, NetworkConnectionNodeConfig};
pub use serial::{SerialConnection, SerialConnectionNodeConfig};

mod frame;

mod fileloader;
pub use fileloader::{FileLoader, FileLoaderNodeConfig};
