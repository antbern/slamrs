use std::sync::Arc;

use common::node::NodeConfig;
use common::{node::Node, robot::Command};
use egui::{Button, Key, Rgba, RichText, Slider};
use pubsub::Publisher;
use serde::Deserialize;

pub struct ControlsNode {
    pub_cmd: Publisher<Command>,
    keyboard_enabled: bool,
    target_speed: f32,
    last_command: Command,
}

#[derive(PartialEq, Eq)]
enum Control {
    Stop,
    Up,
    UpLeft,
    UpRight,
    Down,
    DownLeft,
    DownRight,
    Left,
    Right,
}

#[derive(Clone, Deserialize)]
pub struct ControlsNodeConfig {
    topic_command: String,
    keyboard_enabled: bool,
    max_speed: f32,
}

impl NodeConfig for ControlsNodeConfig {
    fn instantiate(&self, pubsub: &mut pubsub::PubSub) -> Box<dyn Node> {
        Box::new(ControlsNode {
            pub_cmd: pubsub.publish(&self.topic_command),
            keyboard_enabled: self.keyboard_enabled,
            target_speed: self.max_speed,
            last_command: Default::default(),
        })
    }
}

impl Node for ControlsNode {
    fn draw(&mut self, ui: &egui::Ui, _world: &mut common::world::WorldObj<'_>) {
        use Control::*;

        let mut ctrl = Stop;

        if self.keyboard_enabled {
            let (up, left, down, right) = ui.ctx().input(|i| {
                (
                    i.key_down(Key::W),
                    i.key_down(Key::A),
                    i.key_down(Key::S),
                    i.key_down(Key::D),
                )
            });

            ctrl = if up && left {
                UpLeft
            } else if up && right {
                UpRight
            } else if up {
                Up
            } else if down && left {
                DownLeft
            } else if down && right {
                DownRight
            } else if down {
                Down
            } else if right {
                Right
            } else if left {
                Left
            } else {
                Stop
            }
        }

        // then do UI (in case window is closed)
        egui::Window::new("Controls")
            .default_width(200.0)
            .show(ui.ctx(), |ui| {
                ui.checkbox(&mut self.keyboard_enabled, "Enable Keyboard (WASD)");

                ui.add(Slider::new(&mut self.target_speed, 0.0..=0.5).text("Speed"));

                ui.horizontal(|ui| {
                    for (c, str) in [(Left, "<"), (Up, "^"), (Down, "v"), (Right, ">")] {
                        // if keyboard is used to activate this button, change the background color

                        let mut btn =
                            Button::new(RichText::new(str).text_style(egui::TextStyle::Monospace));

                        if ctrl == c {
                            btn = btn.fill(Rgba::from_rgb(0.0, 0.5, 0.5));
                        }

                        if ui.add(btn).is_pointer_button_down_on() {
                            ctrl = c;
                        }
                    }
                });

                ui.label(
                    RichText::new(format!(
                        "Last Command:\nLeft: {:+.3} | Right: {:+.3}",
                        self.last_command.speed_left, self.last_command.speed_right
                    ))
                    .text_style(egui::TextStyle::Monospace),
                );
            });

        let cmd = match ctrl {
            Stop => Command {
                speed_left: 0.0,
                speed_right: 0.0,
            },
            Up => Command {
                speed_left: self.target_speed,
                speed_right: self.target_speed,
            },
            UpLeft => Command {
                speed_left: self.target_speed / 3.0,
                speed_right: self.target_speed,
            },
            UpRight => Command {
                speed_left: self.target_speed,
                speed_right: self.target_speed / 3.0,
            },
            Down => Command {
                speed_left: -self.target_speed,
                speed_right: -self.target_speed,
            },
            DownLeft => Command {
                speed_left: -self.target_speed / 3.0,
                speed_right: -self.target_speed,
            },
            DownRight => Command {
                speed_left: -self.target_speed,
                speed_right: -self.target_speed / 3.0,
            },
            Left => Command {
                speed_left: -self.target_speed,
                speed_right: self.target_speed,
            },
            Right => Command {
                speed_left: self.target_speed,
                speed_right: -self.target_speed,
            },
        };

        if cmd != self.last_command {
            self.pub_cmd.publish(Arc::new(cmd));
            self.last_command = cmd;
        }
    }
}
