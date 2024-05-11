use common::{
    node::{Node, NodeConfig},
    robot::Observation,
    world::WorldObj,
};
use eframe::egui;
use pubsub::{PubSub, Publisher};
use serde::Deserialize;
use slamrs_message::{bincode, CommandMessage, RobotMessage};
use std::{
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread::{self, JoinHandle},
};

use serial2::SerialPort;

use crate::frame;

pub struct SerialConnection {
    state: State,
    selected_port: usize,
    pub_obs: Publisher<Observation>,
}

enum State {
    Idle,
    Running {
        #[allow(unused)] // We need to hold on to this but are actually never using it directly
        handle: JoinHandle<()>,
        running: Arc<AtomicBool>,
    },
}

#[derive(Deserialize, Clone)]
pub struct SerialConnectionNodeConfig {
    topic_observation: String,
}

impl NodeConfig for SerialConnectionNodeConfig {
    fn instantiate(&self, pubsub: &mut PubSub) -> Box<dyn Node> {
        Box::new(SerialConnection {
            state: State::Idle,
            selected_port: 0,
            pub_obs: pubsub.publish(&self.topic_observation),
        })
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
                        Running { handle: _, running } => {
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

impl Drop for SerialConnection {
    fn drop(&mut self) {
        if let State::Running { handle: _, running } = &self.state {
            running.store(false, Ordering::Relaxed);
        }
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

    let mut port = SerialPort::open(path, 115200)?;

    bincode::encode_into_std_write(
        CommandMessage::NeatoOn,
        &mut port,
        bincode::config::standard(),
    )?;
    port.flush()?;

    while running.load(Ordering::Relaxed) {
        match bincode::decode_from_std_read(&mut port, bincode::config::standard()) {
            Ok(data) => match data {
                RobotMessage::ScanFrame(scan_frame) => {
                    let parsed = frame::parse_frame(&scan_frame.scan_data)?;
                    println!("Received: {:?}", &scan_frame.rpm);
                    pub_obs.publish(Arc::new(parsed.into()));
                }
                RobotMessage::Pong => {
                    println!("Received: Pong");

                    // send ping
                    bincode::encode_into_std_write(
                        CommandMessage::Ping,
                        &mut port,
                        bincode::config::standard(),
                    )?;
                }
            },
            // skip TimedOut errors
            Err(bincode::error::DecodeError::Io { inner, .. })
                if inner.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => {
                return Err(e.into());
            }
        }
    }

    // doesn't really matter if this succeeds or not since the connection might be broken already
    bincode::encode_into_std_write(CommandMessage::Ping, &mut port, bincode::config::standard())?;

    println!("Closing!");

    drop(port);

    Ok(())
}
