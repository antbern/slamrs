use std::sync::Arc;

use common::node::NodeConfig;
use common::{node::Node, robot::Command};
use eframe::egui;
use egui::{Button, Key, Rgba, RichText, Slider};
use pubsub::Publisher;
use serde::Deserialize;

pub struct GamepadNode {
    pub_cmd: Publisher<Command>,
    target_speed: f32,
    last_command: Command,
}

#[derive(Clone, Deserialize)]
pub struct GamepadNodeConfig {
    topic_command: String,
    max_speed: f32,
}

impl NodeConfig for GamepadNodeConfig {
    fn instantiate(&self, pubsub: &mut pubsub::PubSub) -> Box<dyn Node> {
        Box::new(GamepadNode {
            pub_cmd: pubsub.publish(&self.topic_command),
            target_speed: self.max_speed,
            last_command: Default::default(),
        })
    }
}

impl Node for GamepadNode {
    fn draw(&mut self, ui: &egui::Ui, _world: &mut common::world::WorldObj<'_>) {
        // then do UI (in case window is closed)
        egui::Window::new("Gamepad")
            .default_width(200.0)
            .show(ui.ctx(), |ui| {
                ui.add(Slider::new(&mut self.target_speed, 0.0..=0.5).text("Speed"));

                ui.label(
                    RichText::new(format!(
                        "Last Command:\nLeft: {:+.3} | Right: {:+.3}",
                        self.last_command.speed_left, self.last_command.speed_right
                    ))
                    .text_style(egui::TextStyle::Monospace),
                );
            });

        let cmd = Command {
            speed_left: 0.0,
            speed_right: 0.0,
        };

        if cmd != self.last_command {
            self.pub_cmd.publish(Arc::new(cmd));
            self.last_command = cmd;
        }
    }
}
