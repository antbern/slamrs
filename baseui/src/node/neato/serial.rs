use common::{node::Node, world::WorldObj};
use pubsub::PubSub;
use std::{
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread::{self, JoinHandle},
};

use serial2::SerialPort;

pub struct SerialConnection {
    state: State,
    selected_port: usize,
}

enum State {
    Idle,
    Running {
        handle: JoinHandle<()>,
        running: Arc<AtomicBool>,
    },
}

impl Node for SerialConnection {
    fn new(pubsub: &mut PubSub) -> Self
    where
        Self: Sized,
    {
        Self {
            state: State::Idle,
            selected_port: 0,
        }
    }

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
                                    move || {
                                        serial_thread(&selected_port, running);
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

fn serial_thread(path: &PathBuf, running: Arc<AtomicBool>) {
    open_and_stream(path, running).expect("Error in serial thread");
}

fn open_and_stream(path: &PathBuf, running: Arc<AtomicBool>) -> anyhow::Result<()> {
    println!("Opening {path:?}");

    let port = SerialPort::open(path, 115200)?;

    port.write(&[b'A'])?;

    let mut buffer = [0; 8 * 256];
    while running.load(Ordering::Relaxed) {
        // do stuff

        match port.read(&mut buffer) {
            Ok(bytes) => {
                if bytes > 0 {
                    println!("{:?}", &buffer[..bytes])
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

    // doesn't really matter if this succeeds or not since the connection might be
    port.write(&[b'D'])?;

    println!("Closing!");

    drop(port);

    Ok(())
}
