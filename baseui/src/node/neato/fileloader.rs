use std::sync::Arc;

use crate::{node::Node, pubsub::Publisher};

use super::frame::{self, NeatoFrame};

pub struct FileLoader {
    picked_path: Option<String>,
    data: Option<Vec<NeatoFrame>>,
    selected_frame: usize,
    pub_frame: Publisher<NeatoFrame>,
}

impl Node for FileLoader {
    fn new(pubsub: &mut crate::pubsub::PubSub) -> Self
    where
        Self: Sized,
    {
        Self {
            picked_path: None,
            data: None,
            selected_frame: 0,
            pub_frame: pubsub.publish("scan"),
        }
    }

    fn draw(&mut self, ui: &egui::Ui, _world: &mut crate::app::WorldRenderer) {
        egui::Window::new("Neato File").show(ui.ctx(), |ui| {
            if ui.button("Open file…").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .set_directory(std::env::current_dir().unwrap())
                    .pick_file()
                {
                    self.picked_path = Some(path.display().to_string());

                    // do stuff here!
                    self.data = frame::load_neato_binary(&path).ok();
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
                        .clamp_to_range(true)
                        .integer()
                        .text("Scan"),
                );
                if r.changed() {
                    self.pub_frame.publish(Arc::new(data[self.selected_frame]));
                }
            }
        });
    }
}
