use common::{node::Node, robot::Observation, world::WorldObj};
use pubsub::{PubSub, Publisher};
use std::{
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread::{self, JoinHandle},
};

use serial2::SerialPort;

use crate::frame::{self, NeatoFrame};

pub struct SerialConnection {
    state: State,
    selected_port: usize,
    pub_obs: Publisher<Observation>,
}

enum State {
    Idle,
    Running {
        handle: JoinHandle<()>,
        running: Arc<AtomicBool>,
    },
}

impl SerialConnection {
    pub fn new(pubsub: &mut PubSub) -> Self
    where
        Self: Sized,
    {
        Self {
            state: State::Idle,
            selected_port: 0,
            pub_obs: pubsub.publish("robot/observation"),
        }
    }
}

impl Node for SerialConnection {
    fn draw(&mut self, ui: &egui::Ui, _world: &mut WorldObj<'_>) {
        egui::Window::new("Serial Connection").show(ui.ctx(), |ui| {
            let ports = SerialPort::available_ports().unwrap();

            if !ports.is_empty() {
                ui.horizontal(|ui| {
                    use State::*;

                    match &self.state {
                        Idle => {
                            egui::ComboBox::from_label("Port")
                                .selected_text(format!("{:?}", self.selected_port))
                                .show_index(ui, &mut self.selected_port, ports.len(), |i| {
                                    ports[i].display().to_string()
                                });

                            if ui.button("Open").clicked() {
                                // start a thread
                                let selected_port = ports[self.selected_port].to_owned();
                                let running = Arc::new(AtomicBool::new(true));

                                let handle = thread::spawn({
                                    let running = running.clone();
                                    let pub_obs = self.pub_obs.clone();
                                    move || {
                                        serial_thread(&selected_port, running, pub_obs);
                                    }
                                });

                                self.state = Running { handle, running }
                            }
                        }
                        Running { handle, running } => {
                            if ui.button("Close").clicked() {
                                running.store(false, Ordering::Relaxed);
                                // handle.join();

                                self.state = Idle;
                            }
                        }
                    }
                });
            } else {
                ui.label("No ports available!");
            }
        });
    }
}

fn serial_thread(path: &PathBuf, running: Arc<AtomicBool>, pub_obs: Publisher<Observation>) {
    open_and_stream(path, running, pub_obs).expect("Error in serial thread");
}

fn open_and_stream(
    path: &PathBuf,
    running: Arc<AtomicBool>,
    mut pub_obs: Publisher<Observation>,
) -> anyhow::Result<()> {
    println!("Opening {path:?}");

    let port = SerialPort::open(path, 115200)?;

    port.write(&[b'A'])?;

    let mut buffer = [0u8; 1024];
    let mut parser = RunningParser::new();

    while running.load(Ordering::Relaxed) {
        // read bytes into the buffer
        match port.read(&mut buffer) {
            Ok(bytes) => {
                for &b in &buffer[..bytes] {
                    if let Some(f) = parser.update(b) {
                        // pusblish the frame!
                        pub_obs.publish(Arc::new(f.into()));
                    }
                }
            }
            Err(e) => {
                // skip TimedOut errors
                if e.kind() != std::io::ErrorKind::TimedOut {
                    return Err(e.into());
                }
            }
        }
    }

    // doesn't really matter if this succeeds or not since the connection might be broken already
    port.write(&[b'D'])?;

    println!("Closing!");

    drop(port);

    Ok(())
}

enum RunningParserState {
    LookingForStart { previous_byte: u8 },
    CollectingBytes { index: usize },
}

struct RunningParser {
    buffer: [u8; 1980],
    state: RunningParserState,
}

impl RunningParser {
    pub fn new() -> Self {
        Self {
            buffer: [0u8; 1980],
            state: RunningParserState::LookingForStart { previous_byte: 0 },
        }
    }

    pub fn update(&mut self, byte: u8) -> Option<NeatoFrame> {
        use RunningParserState::*;

        let mut result = None;
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
                    result = frame::parse_frame(&self.buffer).ok();

                    // next restart looking for frame start
                    LookingForStart { previous_byte: 0 }
                }
            }
        };

        result
    }
}
