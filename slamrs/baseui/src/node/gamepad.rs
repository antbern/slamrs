use std::sync::Arc;

use common::node::NodeConfig;
use common::{node::Node, robot::Command};
use eframe::egui;
use egui::{RichText, Slider};
use gilrs::Gilrs;
use pubsub::Publisher;
use serde::Deserialize;

pub struct GamepadNode {
    pub_cmd: Publisher<Command>,
    target_speed: f32,
    gilrs: Gilrs,
    last_command: Command,
    last_x: f32,
    last_y: f32,
}

#[derive(Clone, Deserialize)]
pub struct GamepadNodeConfig {
    topic_command: String,
    max_speed: f32,
}

impl NodeConfig for GamepadNodeConfig {
    fn instantiate(&self, pubsub: &mut pubsub::PubSub) -> Box<dyn Node> {
        let gilrs = Gilrs::new().expect("should be able to open Gilrs");
        // start a background thread to poll for gamepad events

        Box::new(GamepadNode {
            pub_cmd: pubsub.publish(&self.topic_command),
            target_speed: self.max_speed,
            gilrs,
            last_command: Default::default(),
            last_x: 0.0,
            last_y: 0.0,
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

                ui.label("Connected Gamepads:");
                for (_id, gp) in self.gilrs.gamepads() {
                    ui.label(format!("- {}", gp.name()));
                }

                while let Some(gp) = self.gilrs.next_event() {
                    match gp.event {
                        gilrs::EventType::AxisChanged(axis, value, _) => match axis {
                            gilrs::Axis::LeftStickX => self.last_x = value,
                            gilrs::Axis::LeftStickY => self.last_y = value,
                            _ => {}
                        },
                        gilrs::EventType::Connected => log::info!("Gamepad connected: {}", gp.id),
                        gilrs::EventType::Disconnected => {
                            log::info!("Gamepad disconnected: {}", gp.id)
                        }
                        _ => {}
                    }
                    log::info!("Event: {:?}", gp);
                }

                ui.label(
                    RichText::new(format!(
                        "Last Command:\nLeft: {:+.3} | Right: {:+.3}",
                        self.last_command.speed_left, self.last_command.speed_right
                    ))
                    .text_style(egui::TextStyle::Monospace),
                );
            });

        // TODO: convert x-y to left-right speed using trigonometry?
        // use Y-value to decided the speed
        let speed = self.last_y * self.target_speed;

        // use X-value to decide the direction: 1.0 means full turn right (+- the speed)

        let cmd = Command {
            speed_left: (1.0 + self.last_x * 2.0).min(1.0) * speed,
            speed_right: (1.0 + self.last_x * -2.0).min(1.0) * speed,
        };
        if cmd != self.last_command {
            self.pub_cmd.publish(Arc::new(cmd));
            self.last_command = cmd;
        }
    }
}
