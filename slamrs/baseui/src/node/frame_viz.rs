use std::sync::Arc;

use common::{
    node::{Node, NodeConfig},
    robot::{LandmarkObservations, Observation, Pose},
    world::WorldObj,
};
use egui::CollapsingHeader;
use pubsub::{PubSub, Subscription};

use graphics::shaperenderer::ShapeRenderer;
use serde::Deserialize;
use slam::{GridMapMessage, PointMap};

use super::visualize::{
    GridMapVisualizeConfig, LandmarkObservationVisualizeConfig, ObservationVisualizeConfig,
    PointMapVisualizeConfig, PoseVisualizeConfig, Visualize, VisualizeParametersUi,
};

pub struct FrameVizualizer {
    vis: Vec<Box<dyn SubViz>>,
}

trait SubViz {
    fn visualize(&self, sr: &mut ShapeRenderer);
    fn poll(&mut self);
    fn name(&self) -> &str;
    fn enabled(&mut self) -> &mut bool;
    fn config_ui(&mut self, ui: &mut egui::Ui);
}

#[allow(unused)]
pub enum SecondaryValue<T: Send + Sync + 'static + Clone> {
    None,
    Constant(T),
    Subscription(Subscription<T>),
}
struct SubscriptionVisualizer<
    T: Visualize<Parameters = C, Secondary = S> + Send + Sync + 'static,
    C: VisualizeParametersUi,
    S: Send + Sync + 'static + Clone,
> {
    subscription: Subscription<T>,
    secondary_value: SecondaryValue<S>,
    latest_value: Option<Arc<T>>,
    latest_secondary_value: Option<S>,
    config: C,
    enabled: bool,
    name: String,
}

impl<
        T: Visualize<Parameters = C, Secondary = S> + Send + Sync + 'static,
        C: VisualizeParametersUi,
        S: Send + Sync + 'static + Clone,
    > SubscriptionVisualizer<T, C, S>
{
    pub fn new(subscription: Subscription<T>, config: C) -> Self {
        Self::new_with_secondary(subscription, config, SecondaryValue::None)
    }

    pub fn new_with_secondary(
        subscription: Subscription<T>,
        config: C,
        secondary_value: SecondaryValue<S>,
    ) -> Self {
        let name = format!("{} ({})", subscription.topic(), std::any::type_name::<T>());
        Self {
            subscription,
            secondary_value,
            latest_value: None,
            latest_secondary_value: None,
            config,
            enabled: true,
            name,
        }
    }
}

impl<
        T: Visualize<Parameters = C, Secondary = S> + Send + Sync + 'static,
        C: VisualizeParametersUi,
        S: Send + Sync + 'static + Clone,
    > SubViz for SubscriptionVisualizer<T, C, S>
{
    fn poll(&mut self) {
        while let Some(v) = self.subscription.try_recv() {
            self.latest_value = Some(v);
        }

        match &mut self.secondary_value {
            SecondaryValue::None => self.latest_secondary_value = None,
            SecondaryValue::Constant(value) => self.latest_secondary_value = Some(value.clone()),
            SecondaryValue::Subscription(ref mut s) => {
                while let Some(v) = s.try_recv() {
                    self.latest_secondary_value = Some((*v).clone());
                }
            }
        }
    }

    fn visualize(&self, sr: &mut ShapeRenderer) {
        if let Some(latest_value) = &self.latest_value {
            latest_value.visualize(sr, &self.config, &self.latest_secondary_value);
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

#[derive(Clone, Deserialize, Debug)]
pub struct FrameVizualizerNodeConfig {
    topics: Vec<VizType>,
}

#[derive(Clone, Deserialize, Debug)]
enum VizType {
    Pose {
        topic: String,
        config: PoseVisualizeConfig,
    },
    Observation {
        topic: String,
        topic_pose: String,
        config: ObservationVisualizeConfig,
    },
    LandmarkObservation {
        topic: String,
        topic_pose: String,
        config: LandmarkObservationVisualizeConfig,
    },
    PointMap {
        topic: String,
        config: PointMapVisualizeConfig,
    },
    GridMap {
        topic: String,
        config: GridMapVisualizeConfig,
    },
}

impl VizType {
    fn instantiate(&self, pubsub: &mut PubSub) -> Box<dyn SubViz> {
        match self {
            VizType::Pose { topic, config } => Box::new(SubscriptionVisualizer::new(
                pubsub.subscribe::<Pose>(topic),
                config.clone(),
            )),
            VizType::Observation {
                topic,
                topic_pose,
                config,
            } => Box::new(SubscriptionVisualizer::new_with_secondary(
                pubsub.subscribe::<Observation>(topic),
                config.clone(),
                SecondaryValue::Subscription(pubsub.subscribe::<Pose>(topic_pose)),
            )),
            VizType::LandmarkObservation {
                topic,
                topic_pose,
                config,
            } => Box::new(SubscriptionVisualizer::new_with_secondary(
                pubsub.subscribe::<LandmarkObservations>(topic),
                config.clone(),
                SecondaryValue::Subscription(pubsub.subscribe::<Pose>(topic_pose)),
            )),
            VizType::PointMap { topic, config } => Box::new(SubscriptionVisualizer::new(
                pubsub.subscribe::<PointMap>(topic),
                config.clone(),
            )),
            VizType::GridMap { topic, config } => Box::new(SubscriptionVisualizer::new(
                pubsub.subscribe::<GridMapMessage>(topic),
                config.clone(),
            )),
        }
    }
}

impl NodeConfig for FrameVizualizerNodeConfig {
    fn instantiate(&self, pubsub: &mut PubSub) -> Box<dyn Node> {
        Box::new(FrameVizualizer {
            vis: self.topics.iter().map(|t| t.instantiate(pubsub)).collect(),
        })
    }
}

impl Node for FrameVizualizer {
    fn draw(&mut self, ui: &egui::Ui, world: &mut WorldObj<'_>) {
        // TODO: move this into the Visualizer directly?
        // window that shows the strength vs angle
        egui::Window::new("Visualizer").show(ui.ctx(), |ui| {
            /*
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
             */

            for v in self.vis.iter_mut() {
                ui.horizontal(|ui| {
                    ui.checkbox(v.enabled(), "");

                    CollapsingHeader::new(v.name())
                        // .default_open(true)
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
