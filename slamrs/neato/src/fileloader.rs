use common::{
    node::{Node, NodeConfig},
    robot::{Observation, Pose},
    world::WorldObj,
};
use pubsub::{PubSub, Publisher};
use serde::Deserialize;
use std::sync::Arc;

use super::frame;
use eframe::egui;

pub struct FileLoader {
    picked_path: Option<String>,
    data: Option<Vec<Observation>>,
    selected_frame: usize,
    pub_frame: Publisher<Observation>,
    pub_pose: Publisher<Pose>,
}

#[derive(Clone, Deserialize)]
pub struct FileLoaderNodeConfig {
    topic_observation: String,
    topic_pose: String,
    // TODO: make it possible to specify a path to load automatically here
}

impl NodeConfig for FileLoaderNodeConfig {
    fn instantiate(&self, pubsub: &mut PubSub) -> Box<dyn Node> {
        Box::new(FileLoader {
            picked_path: None,
            data: None,
            selected_frame: 0,
            pub_frame: pubsub.publish(&self.topic_observation),
            pub_pose: pubsub.publish(&self.topic_pose),
        })
    }
}

impl Node for FileLoader {
    fn draw(&mut self, ui: &egui::Ui, _world: &mut WorldObj<'_>) {
        egui::Window::new("Neato File").show(ui.ctx(), |ui| {
            if ui.button("Open file…").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .set_directory(std::env::current_dir().unwrap())
                    .pick_file()
                {
                    self.picked_path = Some(path.display().to_string());

                    // do stuff here!
                    self.data = frame::load_neato_binary(&path)
                        .ok()
                        .map(|n| n.iter().map(|&o| o.into()).collect())
                }
            }

            if let Some(picked_path) = &self.picked_path {
                ui.horizontal(|ui| {
                    ui.label("Picked file:");
                    ui.monospace(picked_path);
                });
            }

            if let Some(data) = &self.data {
                ui.horizontal(|ui| {
                    ui.label("Data:");
                    ui.monospace(format!("Records: {}", data.len()));
                });

                let r = ui.add(
                    egui::Slider::new(&mut self.selected_frame, 0..=data.len() - 1)
                        .clamping(egui::SliderClamping::Always)
                        .integer()
                        .text("Scan"),
                );
                if r.changed() {
                    self.pub_frame
                        .publish(Arc::new(data[self.selected_frame].clone()));
                    self.pub_pose.publish(Arc::new(Pose::default()));
                }
            }
        });
    }
}
