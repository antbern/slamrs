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
    io::Write,
    net::TcpStream,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread::{self, JoinHandle},
};

use crate::frame;

pub struct NetworkConnection {
    state: State,
    host: String,
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
pub struct NetworkConnectionNodeConfig {
    topic_observation: String,
}

impl NodeConfig for NetworkConnectionNodeConfig {
    fn instantiate(&self, pubsub: &mut PubSub) -> Box<dyn Node> {
        Box::new(NetworkConnection {
            state: State::Idle,
            host: "robot:8080".into(),
            pub_obs: pubsub.publish(&self.topic_observation),
        })
    }
}

impl Node for NetworkConnection {
    fn draw(&mut self, ui: &egui::Ui, _world: &mut WorldObj<'_>) {
        egui::Window::new("Network Connection").show(ui.ctx(), |ui| {
            ui.horizontal(|ui| {
                use State::*;

                match &self.state {
                    Idle => {
                        ui.label("Host");
                        ui.text_edit_singleline(&mut self.host);

                        if ui.button("Open").clicked() {
                            // start a thread

                            let running = Arc::new(AtomicBool::new(true));

                            let host = self.host.to_owned();

                            let handle = thread::spawn({
                                let running = running.clone();
                                let pub_obs = self.pub_obs.clone();
                                move || {
                                    network_thread(&host, running, pub_obs);
                                }
                            });

                            self.state = Running { handle, running }
                        }
                    }
                    Running { handle: _, running } => {
                        if ui.button("Close").clicked() || !running.load(Ordering::Relaxed) {
                            running.store(false, Ordering::Relaxed);

                            self.state = Idle;
                        }
                    }
                }
            });
        });
    }
}

impl Drop for NetworkConnection {
    fn drop(&mut self) {
        if let State::Running { handle: _, running } = &self.state {
            running.store(false, Ordering::Relaxed);
        }
    }
}

fn network_thread(host: &String, running: Arc<AtomicBool>, pub_obs: Publisher<Observation>) {
    if let Err(e) = open_and_stream(host, running.clone(), pub_obs) {
        // if the function returns an error, display it and change the state to idle
        running.store(false, Ordering::Relaxed);

        tracing::error!("{}", e);
    }
}

fn open_and_stream(
    host: &String,
    running: Arc<AtomicBool>,
    mut pub_obs: Publisher<Observation>,
) -> anyhow::Result<()> {
    println!("Connecting to {host:?}");

    let mut stream = TcpStream::connect(host)?;

    bincode::encode_into_std_write(
        CommandMessage::SetDownsampling { every: 4 },
        &mut stream,
        bincode::config::standard(),
    )?;

    bincode::encode_into_std_write(
        CommandMessage::NeatoOn,
        &mut stream,
        bincode::config::standard(),
    )?;
    bincode::encode_into_std_write(
        CommandMessage::Ping,
        &mut stream,
        bincode::config::standard(),
    )?;

    while running.load(Ordering::Relaxed) {
        // read bytes into the buffer

        let data: RobotMessage =
            bincode::decode_from_std_read(&mut stream, bincode::config::standard())?;

        match data {
            RobotMessage::ScanFrame(scan_frame) => {
                let parsed = frame::parse_frame(&scan_frame.scan_data)?;
                println!("Received: {:?}", &scan_frame.rpm);
                pub_obs.publish(Arc::new(parsed.into()));
            }
            RobotMessage::Pong => {
                println!("Received: Pong");
            }
        }

        // send ping
        bincode::encode_into_std_write(
            CommandMessage::Ping,
            &mut stream,
            bincode::config::standard(),
        )?;
    }

    // doesn't really matter if this succeeds or not since the connection might be broken already
    bincode::encode_into_std_write(
        CommandMessage::NeatoOff,
        &mut stream,
        bincode::config::standard(),
    )?;

    println!("Closing!");

    drop(stream);

    Ok(())
}
