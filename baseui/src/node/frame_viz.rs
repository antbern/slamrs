use std::sync::Arc;

use common::{
    node::Node,
    robot::{Observation, Pose},
    world::WorldObj,
};
use egui::plot::{Bar, BarChart, Plot, PlotBounds};
use pubsub::{PubSub, Subscription};

use graphics::primitiverenderer::{Color, PrimitiveType};

pub struct FrameVizualizer {
    last_frame: Option<Arc<Observation>>,
    last_pose: Pose,
    sub: Subscription<Observation>,
    sub_pose: Subscription<Pose>,
}

impl Node for FrameVizualizer {
    fn new(pubsub: &mut PubSub) -> Self
    where
        Self: Sized,
    {
        Self {
            last_frame: None,
            last_pose: Default::default(),
            // sub: pubsub.subscribe("scan"),
            sub: pubsub.subscribe("robot/observation"),
            sub_pose: pubsub.subscribe("robot/pose"),
        }
    }

    fn draw(&mut self, ui: &egui::Ui, world: &mut WorldObj<'_>) {
        while let Some(v) = self.sub.try_recv() {
            self.last_frame = Some(v);
        }

        while let Some(v) = self.sub_pose.try_recv() {
            self.last_pose = *v; // Copy the value into local storage
        }

        if let Some(frame) = &self.last_frame {
            // draw the reading into the world

            let (ox, oy) = self.last_pose.into();

            world.sr.begin(PrimitiveType::Line);

            for m in frame.measurements.iter() {
                let (s, c) = (m.angle as f32).sin_cos();
                let d = m.distance as f32;
                let x = c * d;
                let y = s * d;

                // let color = if m.valid { Color::WHITE } else { Color::BLACK };
                let color = Color::BLACK;

                world.sr.line(ox, oy, ox + x, oy + y, color);
            }

            world.sr.end();

            world.sr.begin(PrimitiveType::Filled);

            for m in frame.measurements.iter() {
                let (s, c) = (m.angle as f32).sin_cos();
                let d = m.distance as f32;
                let x = c * d;
                let y = s * d;

                let color = Color::rgb(m.strength as f32 / 2000.0, 0.0, 0.0);
                world
                    .sr
                    .rect(ox + x - 0.005, oy + y - 0.005, 0.01, 0.01, color)
            }

            world.sr.end()
        }

        // window that shows the strength vs angle
        egui::Window::new("Frame Visualization").show(ui.ctx(), |ui| {
            let mut bars = Vec::new();

            if let Some(o) = &self.last_frame {
                for m in &o.measurements {
                    bars.push(Bar::new(m.angle.to_degrees(), m.strength))
                }
            };

            let chart = BarChart::new(bars).width(0.1).name("Strenght");

            Plot::new("Strenght")
                .view_aspect(2.0)
                .include_x(0.0)
                .include_y(0.0)
                .auto_bounds_x()
                .auto_bounds_y()
                .allow_scroll(false)
                .allow_drag(false)
                .allow_boxed_zoom(false)
                .show(ui, |plot_ui| {
                    plot_ui.bar_chart(chart);
                    plot_ui.set_plot_bounds(PlotBounds::from_min_max([0.0, 0.0], [360.0, 2000.0]));
                });
        });
    }
}
