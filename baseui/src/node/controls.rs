use std::sync::Arc;

use common::{node::Node, robot::Command};
use egui::{Button, Key, Rgba, RichText, Slider};
use pubsub::Publisher;

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
    Down,
    Left,
    Right,
}

impl Node for ControlsNode {
    fn new(pubsub: &mut pubsub::PubSub) -> Self
    where
        Self: Sized,
    {
        Self {
            pub_cmd: pubsub.publish("robot/command"),
            keyboard_enabled: true,
            target_speed: 0.1,
            last_command: Default::default(),
        }
    }

    fn draw(&mut self, ui: &egui::Ui, _world: &mut common::world::WorldObj<'_>) {
        use Control::*;

        let mut ctrl = Stop;

        if self.keyboard_enabled {
            let input = ui.ctx().input();

            ctrl = if input.key_down(Key::W) {
                Up
            } else if input.key_down(Key::A) {
                Left
            } else if input.key_down(Key::S) {
                Down
            } else if input.key_down(Key::D) {
                Right
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
            Down => Command {
                speed_left: -self.target_speed,
                speed_right: -self.target_speed,
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
