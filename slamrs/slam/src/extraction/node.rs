use std::sync::Arc;

use common::{
    node::{Node, NodeConfig},
    robot::{LandmarkObservations, Observation, Odometry},
};
use eframe::egui;

use pubsub::{Publisher, Subscription};
use serde::Deserialize;

use super::extract::{extract_landmarks, Config};

pub struct LandmarkExtractionNode {
    sub_scan_odom: Subscription<(Observation, Odometry)>,
    pub_landmarks_odom: Publisher<(LandmarkObservations, Odometry)>,
    config: Config,
}

#[derive(Clone, Deserialize)]
pub struct LandmarkExtractionNodeConfig {
    topic_in_scan_odometry: String,
    topic_out_landmarks_odometry: String,
    config: Option<Config>,
}

impl NodeConfig for LandmarkExtractionNodeConfig {
    fn instantiate(&self, pubsub: &mut pubsub::PubSub) -> Box<dyn Node> {
        Box::new(LandmarkExtractionNode {
            sub_scan_odom: pubsub.subscribe(&self.topic_in_scan_odometry),
            pub_landmarks_odom: pubsub.publish(&self.topic_out_landmarks_odometry),
            config: self.config.clone().unwrap_or_default(),
        })
    }
}

impl Node for LandmarkExtractionNode {
    fn update(&mut self) {
        if let Some(o) = self.sub_scan_odom.try_recv() {
            let landmarks = extract_landmarks(&self.config, &o.0);

            self.pub_landmarks_odom
                .publish(Arc::new((LandmarkObservations { landmarks }, o.1)));
        }
    }

    fn draw(&mut self, ui: &egui::Ui, _world: &mut common::world::WorldObj<'_>) {
        egui::Window::new("Landmark Extraction").show(ui.ctx(), |ui| {
            ui.label("[WIP]");
            // TODO: add adjustments for the config here
        });
    }
}
