use std::sync::Arc;

use common::{
    gaussian::Gaussian2D,
    node::{Node, NodeConfig},
    world::WorldObj,
};
use eframe::egui;
use egui::DragValue;

use pubsub::{PubSub, Publisher};
use serde::Deserialize;

pub struct GaussianRendering {
    publish: Publisher<Gaussian2D>,
    gaussian: Gaussian2D,
    p: f32,
}

#[derive(Clone, Deserialize)]
pub struct GaussianNodeConfig {
    topic: String,
}

impl NodeConfig for GaussianNodeConfig {
    fn instantiate(&self, pubsub: &mut PubSub) -> Box<dyn Node> {
        Box::new(GaussianRendering {
            publish: pubsub.publish(&self.topic),
            gaussian: Gaussian2D::default(),
            p: 0.95,
        })
    }
}

impl Node for GaussianRendering {
    fn draw(&mut self, ui: &egui::Ui, w: &mut WorldObj<'_>) {
        egui::Window::new("Gaussian").show(ui.ctx(), |ui| {
            ui.add(
                DragValue::new(&mut self.gaussian.mean.x)
                    .fixed_decimals(2)
                    .clamp_range(-1.0..=1.0)
                    .speed(0.01),
            );

            ui.add(
                DragValue::new(&mut self.gaussian.mean.y)
                    .fixed_decimals(2)
                    .clamp_range(-1.0..=1.0)
                    .speed(0.01),
            );

            ui.horizontal(|ui| {
                ui.add(
                    DragValue::new(&mut self.gaussian.covariance[(0, 0)])
                        .fixed_decimals(2)
                        .clamp_range(0.0..=5.0)
                        .speed(0.01),
                );
                // ui.add(
                //     DragValue::new(&mut self.gaussian.covariance[(0, 1)])
                //         .fixed_decimals(2)
                //         .clamp_range(0.0..=1.0)
                //         .speed(0.01),
                // );
            });
            ui.horizontal(|ui| {
                ui.add(
                    DragValue::new(&mut self.gaussian.covariance[(1, 0)])
                        .fixed_decimals(2)
                        .clamp_range(-5.0..=5.0)
                        .speed(0.01),
                );
                ui.add(
                    DragValue::new(&mut self.gaussian.covariance[(1, 1)])
                        .fixed_decimals(2)
                        .clamp_range(0.0..=5.0)
                        .speed(0.01),
                );
            });

            ui.add(
                DragValue::new(&mut self.p)
                    .fixed_decimals(2)
                    .clamp_range(0.0..=1.0)
                    .speed(0.01),
            );

            if ui.button("Publish").clicked() {
                self.publish.publish(Arc::new(self.gaussian));
            }
        });

        w.sr.gaussian2d(&self.gaussian.mean, &self.gaussian.covariance, self.p);
    }
}
