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
                }

                ui.label(
                    RichText::new(format!(
                        "Last Command:\nLeft: {:+.3} | Right: {:+.3}",
                        self.last_command.speed_left, self.last_command.speed_right
                    ))
                    .text_style(egui::TextStyle::Monospace),
                );
            });

        // convert x-y to left-right speed using the angle
        let r = (self.last_x.powi(2) + self.last_y.powi(2)).sqrt();

        let cmd = if r > 0.0 {
            let angle = self.last_y.atan2(self.last_x);

            // this is the maximum r for a given angle
            let max_r = r / (self.last_x.abs().max(self.last_y.abs()));

            // this is the actual throttle
            let magnitude = r / max_r;

            let turn_damping = 3.0; // increase for slower turns
            let left = magnitude * (angle.sin() + angle.cos() / turn_damping);
            let right = magnitude * (angle.sin() - angle.cos() / turn_damping);

            Command {
                speed_left: left * self.target_speed,
                speed_right: right * self.target_speed,
            }
        } else {
            Command {
                speed_left: 0.0,
                speed_right: 0.0,
            }
        };

        if cmd != self.last_command {
            self.pub_cmd.publish(Arc::new(cmd));
            self.last_command = cmd;
        }
    }
}
