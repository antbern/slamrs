use common::{
    node::{Node, NodeConfig},
    robot::Observation,
    world::WorldObj,
};
use eframe::egui;
use pubsub::{PubSub, Publisher};
use serde::Deserialize;
use slamrs_message::{CommandMessage, RobotMessage};
use std::io::prelude::*;
use std::{
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
            host: "192.168.1.114:8080".into(),
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

    stream.write_all(&[b'A'])?;
    //port.flush()?;

    // let mut buffer = [0u8; 1024];
    // let mut parser = RunningParser::new();

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
            RobotMessage::Pong => {}
        }

        // send ping
        bincode::encode_into_std_write(
            CommandMessage::Ping,
            &mut stream,
            bincode::config::standard(),
        )?;

        // println!("Received: {:?}", data);

        // // let (cmd, len):  =
        // //     bincode::decode_from_slice(&rx_buffer[..], bincode::config::standard())
        // //         .expect("Could not parse");

        // match stream.read(&mut buffer) {
        //     Ok(bytes) => {
        //         println!("{bytes} incoming bytes");
        //         // for &b in &buffer[..bytes] {
        //         //     if let Some(f) = parser.update(b) {
        //         //         // pusblish the frame!
        //         //         pub_obs.publish(Arc::new(f.into()));
        //         //     }
        //         // }
        //     }
        //     Err(e) => {
        //         // skip TimedOut errors
        //         if e.kind() != std::io::ErrorKind::TimedOut {
        //             return Err(e.into());
        //         }
        //     }
        // }
    }

    // doesn't really matter if this succeeds or not since the connection might be broken already
    stream.write_all(&[b'D'])?;

    println!("Closing!");

    drop(stream);

    Ok(())
}
