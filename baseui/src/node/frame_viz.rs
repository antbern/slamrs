use std::sync::Arc;

use common::{
    node::Node,
    robot::{Observation, Pose},
    world::WorldObj,
};
use egui::{
    plot::{Bar, BarChart, Plot, PlotBounds, Points},
    CollapsingHeader,
};
use pubsub::{PubSub, Subscription};

use graphics::{
    primitiverenderer::{Color, PrimitiveType},
    shaperenderer::ShapeRenderer,
};

use super::visualize::{PoseVisualizeConfig, Visualize, VisualizeParametersUi};

pub struct FrameVizualizer {
    last_frame: Option<Arc<Observation>>,
    last_pose: Pose,
    sub: Subscription<Observation>,
    sub_pose: Subscription<Pose>,
    // last_pointmap: Option<Arc<Observation>>,
    // sub_pointmap: Subscription<Observation>,
    vis: Vec<Box<dyn SubViz>>,
}

trait SubViz {
    fn visualize(&self, sr: &mut ShapeRenderer);
    fn poll(&mut self);
    fn name(&self) -> &str;
    fn enabled(&mut self) -> &mut bool;
    fn config_ui(&mut self, ui: &mut egui::Ui);
}

struct SubscriptionVisualizer<
    T: Visualize<Parameters = C> + Send + Sync + 'static,
    C: VisualizeParametersUi,
> {
    subscription: Subscription<T>,
    latest_value: Option<Arc<T>>,
    config: C,
    enabled: bool,
    name: String,
}

impl<T: Visualize<Parameters = C> + Send + Sync + 'static, C: VisualizeParametersUi>
    SubscriptionVisualizer<T, C>
{
    pub fn new(subscription: Subscription<T>, config: C) -> Self {
        let name = format!("{} ({})", subscription.topic(), std::any::type_name::<T>());
        Self {
            subscription,
            latest_value: None,
            config,
            enabled: true,
            name,
        }
    }
}

impl<T: Visualize<Parameters = C> + Send + Sync + 'static, C: VisualizeParametersUi> SubViz
    for SubscriptionVisualizer<T, C>
{
    fn poll(&mut self) {
        while let Some(v) = self.subscription.try_recv() {
            self.latest_value = Some(v);
        }
    }

    fn visualize(&self, sr: &mut ShapeRenderer) {
        if let Some(latest_value) = &self.latest_value {
            latest_value.visualize(sr, &self.config);
        }
    }

    fn config_ui(&mut self, ui: &mut egui::Ui) {
        self.config.ui(ui)
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn enabled(&mut self) -> &mut bool {
        &mut self.enabled
    }
}

impl Node for FrameVizualizer {
    fn new(pubsub: &mut PubSub) -> Self
    where
        Self: Sized,
    {
        Self {
            last_frame: None,
            last_pose: Default::default(),
            // last_pointmap: None,
            // sub: pubsub.subscribe("scan"),
            sub: pubsub.subscribe("robot/observation"),
            sub_pose: pubsub.subscribe("robot/pose"),
            // sub_pointmap: pubsub.subscribe("pointmap"),
            vis: vec![Box::new(SubscriptionVisualizer::new(
                pubsub.subscribe::<Pose>("robot/pose"),
                PoseVisualizeConfig::default(),
            ))],
        }
    }

    fn draw(&mut self, ui: &egui::Ui, world: &mut WorldObj<'_>) {
        while let Some(v) = self.sub.try_recv() {
            self.last_frame = Some(v);
        }

        // while let Some(v) = self.sub_pose.try_recv() {
        //     self.last_pose = *v; // Copy the value into local storage
        // }

        // while let Some(v) = self.sub_pointmap.try_recv() {
        //     self.last_pointmap = Some(v); // Copy the value into local storage
        // }

        if let Some(frame) = &self.last_frame {
            // draw the reading into the world

            let (ox, oy) = self.last_pose.into();

            world.sr.begin(PrimitiveType::Line);

            for m in frame.measurements.iter() {
                let (s, c) = (m.angle as f32 + self.last_pose.theta).sin_cos();
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
                let (s, c) = (m.angle as f32 + self.last_pose.theta).sin_cos();
                let d = m.distance as f32;
                let x = c * d;
                let y = s * d;

                let color = Color::rgb(m.strength as f32 / 2000.0, 0.0, 0.0);
                world
                    .sr
                    .rect(ox + x - 0.005, oy + y - 0.005, 0.01, 0.01, color)
            }

            // dbg!(self.last_pose.theta);

            world.sr.arrow(
                self.last_pose.x,
                self.last_pose.y,
                self.last_pose.theta,
                0.1,
                Color::GREEN,
            );

            world.sr.end()
        }

        // if let Some(frame) = &self.last_pointmap {
        //     world.sr.begin(PrimitiveType::Filled);

        //     let ox = 0.0;
        //     let oy = 0.0;
        //     for m in frame.measurements.iter() {
        //         let (s, c) = (m.angle as f32).sin_cos();
        //         let d = m.distance as f32;
        //         let x = c * d;
        //         let y = s * d;

        //         let color = Color::rgb(m.strength as f32 / 2000.0, 0.0, 0.0);
        //         world
        //             .sr
        //             .rect(ox + x - 0.005, oy + y - 0.005, 0.01, 0.01, color)
        //     }
        //     world.sr.end();
        // }

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

            // Draw a strength-distance scatter plot
            let mut points = Vec::new();

            if let Some(o) = &self.last_frame {
                for m in &o.measurements {
                    points.push([m.strength, m.distance])
                }
            }

            // let line = Line::new(points);
            let points = Points::new(points);
            Plot::new("my_plot")
                .view_aspect(2.0)
                .show(ui, |plot_ui| plot_ui.points(points));

            ui.label("Visualizer");

            for v in self.vis.iter_mut() {
                ui.horizontal(|ui| {
                    ui.checkbox(v.enabled(), "");

                    CollapsingHeader::new(v.name())
                        .default_open(true)
                        .show(ui, |ui| v.config_ui(ui));
                });
            }
        });

        for v in self.vis.iter_mut() {
            v.poll();
            if *v.enabled() {
                v.visualize(world.sr);
            }
        }
    }
}
