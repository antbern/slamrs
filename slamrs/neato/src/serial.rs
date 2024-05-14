use common::{
    node::{Node, NodeConfig},
    robot::{Command, Observation, Odometry},
    world::WorldObj,
};
use eframe::egui;
use pubsub::{PubSub, Publisher, Subscription};
use serde::Deserialize;
use slamrs_message::{bincode, CommandMessage, RobotMessage};
use std::{
    net::TcpStream,
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread::{self, JoinHandle},
};
use tracing::{error, info};

use serial2::SerialPort;

use crate::frame;

pub struct SerialConnection {
    state: State,
    serial_port_sected: bool,
    selected_port: usize,
    host: String,
    pub_obs: Publisher<(Observation, Odometry)>,
    sub_command: Subscription<Command>,
}

enum State {
    Idle,
    Running {
        handle: JoinHandle<()>,
        running: Arc<AtomicBool>,
        sender: std::sync::mpsc::Sender<CommandMessage>,
        speed: f32,
        kp: f32,
        ki: f32,
    },
}

#[derive(Deserialize, Clone)]
pub struct SerialConnectionNodeConfig {
    topic_observation: String,
    topic_command: String,
}

impl NodeConfig for SerialConnectionNodeConfig {
    fn instantiate(&self, pubsub: &mut PubSub) -> Box<dyn Node> {
        Box::new(SerialConnection {
            state: State::Idle,
            serial_port_sected: false,
            selected_port: 0,
            host: "robot:8080".into(),
            pub_obs: pubsub.publish(&self.topic_observation),
            sub_command: pubsub.subscribe(&self.topic_command),
        })
    }
}

impl Node for SerialConnection {
    fn draw(&mut self, ui: &egui::Ui, _world: &mut WorldObj<'_>) {
        egui::Window::new("Serial Connection").show(ui.ctx(), |ui| {
            use State::*;
            let mut new_state = None;
            match &mut self.state {
                Idle => {
                    let ports = SerialPort::available_ports().unwrap_or_default();
                    ui.horizontal(|ui| {
                        ui.vertical(|ui| {
                            ui.radio_value(&mut self.serial_port_sected, true, "Serial");
                            ui.radio_value(&mut self.serial_port_sected, false, "Network");
                        });

                        if self.serial_port_sected {
                            if !ports.is_empty() {
                                egui::ComboBox::from_label("Port")
                                    .selected_text(format!("{:?}", self.selected_port))
                                    .show_index(ui, &mut self.selected_port, ports.len(), |i| {
                                        ports[i].display().to_string()
                                    });
                            } else {
                                ui.label("No ports available!");
                            }
                        } else {
                            ui.label("Host");
                            ui.text_edit_singleline(&mut self.host);
                        }
                    });

                    if ui.button("Open").clicked() {
                        // start a thread
                        let connection_type = if self.serial_port_sected {
                            ConnectionType::Serial(ports[self.selected_port].to_owned())
                        } else {
                            ConnectionType::Tcp(self.host.to_owned())
                        };

                        let running = Arc::new(AtomicBool::new(true));
                        let (sender, receiver) = std::sync::mpsc::channel();
                        let handle = thread::spawn({
                            let running = running.clone();
                            let pub_obs = self.pub_obs.clone();
                            move || {
                                serial_thread(connection_type, running, pub_obs, receiver);
                            }
                        });

                        new_state = Some(Running {
                            handle,
                            running,
                            sender,
                            speed: 0.0,
                            kp: 0.5,
                            ki: 2.0,
                        })
                    }
                }
                Running {
                    handle,
                    running,
                    sender,
                    speed,
                    kp,
                    ki,
                } => {
                    // if the thread has stopped (or the user want to exit), change the state to idle
                    if ui.button("Close").clicked() || handle.is_finished() {
                        running.store(false, Ordering::Relaxed);
                        // handle.join();

                        new_state = Some(Idle);
                    }

                    if let Some(cmd) = self.sub_command.try_recv() {
                        sender
                            .send(CommandMessage::Drive {
                                left: cmd.speed_left,
                                right: cmd.speed_right,
                            })
                            .ok();
                    }

                    ui.vertical(|ui| {
                        if ui.button("Start Neato").clicked() {
                            sender.send(CommandMessage::NeatoOn).ok();
                        }
                        if ui.button("Stop Neato").clicked() {
                            sender.send(CommandMessage::NeatoOff).ok();
                        }
                        if ui
                            .add(egui::Slider::new(speed, -1.0..=1.0).text("Speed"))
                            .changed()
                        {
                            sender
                                .send(CommandMessage::Drive {
                                    left: *speed,
                                    right: *speed,
                                })
                                .ok();
                        }
                        if ui
                            .add(egui::Slider::new(kp, 0.0..=2.0).text("Kp"))
                            .changed()
                            || ui
                                .add(egui::Slider::new(ki, 0.0..=3.0).text("Ki"))
                                .changed()
                        {
                            sender
                                .send(CommandMessage::SetMotorPiParams { kp: *kp, ki: *ki })
                                .ok();
                        }
                    });
                }
            }

            if let Some(state) = new_state {
                self.state = state;
            }
        });
    }
}

impl Drop for SerialConnection {
    fn drop(&mut self) {
        if let State::Running {
            handle: _, running, ..
        } = &self.state
        {
            running.store(false, Ordering::Relaxed);
        }
    }
}

enum ConnectionType {
    Serial(PathBuf),
    Tcp(String),
}
fn serial_thread(
    connection_type: ConnectionType,
    running: Arc<AtomicBool>,
    pub_obs: Publisher<(Observation, Odometry)>,
    receiver: std::sync::mpsc::Receiver<CommandMessage>,
) {
    match connection_type {
        ConnectionType::Serial(path) => {
            info!("Opening {path:?}");

            match SerialPort::open(path, 115200) {
                Ok(port) => {
                    if let Err(e) = open_and_stream(port, running, pub_obs, receiver) {
                        error!("Error while streaming serial port:\n{:#}", e);
                    }
                }
                Err(e) => {
                    error!("Error opening serial port: {:?}", e);
                }
            };
        }
        ConnectionType::Tcp(host) => {
            info!("Connecting to {host}");

            match TcpStream::connect(host) {
                Ok(port) => {
                    if let Err(e) = open_and_stream(port, running, pub_obs, receiver) {
                        error!("Error while streaming network connection:\n{:#}", e);
                    }
                }
                Err(e) => {
                    error!("Error connecting: {:?}", e);
                }
            };
        }
    }
}

fn open_and_stream<C: ConnectionMedium>(
    mut connection: C,
    running: Arc<AtomicBool>,
    mut pub_obs: Publisher<(Observation, Odometry)>,
    receiver: std::sync::mpsc::Receiver<CommandMessage>,
) -> anyhow::Result<()> {
    connection.set_timeout_read(std::time::Duration::from_millis(200))?;

    bincode::encode_into_std_write(
        CommandMessage::SetDownsampling { every: 2 },
        &mut connection,
        bincode::config::standard(),
    )?;

    bincode::encode_into_std_write(
        CommandMessage::NeatoOn,
        &mut connection,
        bincode::config::standard(),
    )?;

    while running.load(Ordering::Relaxed) {
        while let Ok(cmd) = receiver.try_recv() {
            info!("Sending: {:?}", cmd);
            bincode::encode_into_std_write(cmd, &mut connection, bincode::config::standard())?;
        }

        match bincode::decode_from_std_read(&mut connection, bincode::config::standard()) {
            Ok(data) => match data {
                RobotMessage::ScanFrame(scan_frame) => {
                    let parsed = frame::parse_frame(&scan_frame.scan_data)?;
                    println!("Received: {:?}", &scan_frame.rpm);
                    let odometry = Odometry::new(scan_frame.odometry[0], scan_frame.odometry[1]);
                    pub_obs.publish(Arc::new((parsed.into(), odometry)));
                }
                RobotMessage::Pong => {
                    println!("Received: Pong");

                    // send ping
                    bincode::encode_into_std_write(
                        CommandMessage::Ping,
                        &mut connection,
                        bincode::config::standard(),
                    )?;
                }
            },
            // skip TimedOut errors
            Err(bincode::error::DecodeError::Io { inner, .. })
                if inner.kind() == std::io::ErrorKind::TimedOut
                    || inner.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(e) => {
                return Err(e.into());
            }
        }
    }

    // doesn't really matter if this succeeds or not since the connection might be broken already
    bincode::encode_into_std_write(
        CommandMessage::NeatoOff,
        &mut connection,
        bincode::config::standard(),
    )?;
    bincode::encode_into_std_write(
        CommandMessage::Drive {
            left: 0.0,
            right: 0.0,
        },
        &mut connection,
        bincode::config::standard(),
    )?;

    info!("Closing!");

    drop(connection);

    Ok(())
}

/// A trait for a connection that can read and write bytes, with timeout.
trait ConnectionMedium: std::io::Write + std::io::Read {
    /// Set the read timeout
    fn set_timeout_read(&mut self, timeout: std::time::Duration) -> std::io::Result<()>;
}

impl ConnectionMedium for SerialPort {
    fn set_timeout_read(&mut self, timeout: std::time::Duration) -> std::io::Result<()> {
        self.set_read_timeout(timeout)
    }
}

impl ConnectionMedium for std::net::TcpStream {
    fn set_timeout_read(&mut self, timeout: std::time::Duration) -> std::io::Result<()> {
        self.set_read_timeout(Some(timeout))
    }
}
