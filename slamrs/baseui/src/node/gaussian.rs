use std::{f32::consts::PI, sync::Arc};

use common::{
    gaussian::Gaussian2D,
    node::{Node, NodeConfig},
    world::WorldObj,
};
use egui::DragValue;
use graphics::{
    primitiverenderer::{Color, PrimitiveType},
    shaperenderer::ShapeRenderer,
};

use nalgebra::{Matrix2, Vector2};
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

        gaussian2d(&mut w.sr, &self.gaussian, self.p);
    }
}

/// Use the information in the Gaussian2D component to draw the correct ellipse around the uncertainty as well as a center piece
fn gaussian2d(sr: &mut ShapeRenderer, gaussian: &Gaussian2D, p: f32) {
    sr.begin(PrimitiveType::Filled);
    sr.circle(gaussian.mean.x, gaussian.mean.y, 0.01, Color::BLUE);
    sr.end();

    // Matlab reference (Source: https://www.xarg.org/2018/04/how-to-plot-a-covariance-error-ellipse/)
    // s = -2 * log(1 - p);
    // [V, D] = eig(Sigma * s);
    // t = linspace(0, 2 * pi);
    // a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];
    // plot(a(1, :) + mu(1), a(2, :) + mu(2));

    // update the ellipse radii

    let s = -2.0 * (1.0 - p).ln();

    let eigen = (gaussian.covariance * s).symmetric_eigen();

    let D = Matrix2::from_diagonal(&eigen.eigenvalues.map(|v| v.sqrt()));
    let V = eigen.eigenvectors;

    sr.begin(PrimitiveType::Line);
    let steps = 25;
    for i in 0..steps {
        let angle = i as f32 * PI * 2.0 / steps as f32;
        let start = gaussian.mean + (V * D) * Vector2::new(angle.cos(), angle.sin());

        let angle = ((i + 1) % steps) as f32 * PI * 2.0 / steps as f32;
        let end = gaussian.mean + (V * D) * Vector2::new(angle.cos(), angle.sin());
        sr.line(start.x, start.y, end.x, end.y, Color::BLACK);
    }

    sr.end();
}
