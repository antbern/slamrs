use std::sync::Arc;

use common::{
    node::{Node, NodeConfig},
    robot::{Observation, Pose},
};
use nalgebra::Vector2;
use pubsub::{Publisher, Subscription};
use serde::Deserialize;

use super::{
    map::GridData,
    math::Probability,
    slam::{GridMapSlam, GridMapSlamConfig},
};

pub struct GridMapSlamNode {
    sub_obs: Subscription<Observation>,
    pub_pose: Publisher<Pose>,
    pub_map: Publisher<GridMapMessage>,
    slam: GridMapSlam,
    config: GridMapSlamConfig,
}

#[derive(Deserialize)]
pub struct GridMapSlamNodeConfig {
    topic_pose: String,
    topic_observation: String,
    topic_map: String,
    config: GridMapSlamConfig,
}

impl NodeConfig for GridMapSlamNodeConfig {
    fn instantiate(&self, pubsub: &mut pubsub::PubSub) -> Box<dyn Node> {
        Box::new(GridMapSlamNode {
            sub_obs: pubsub.subscribe(&self.topic_observation),
            pub_pose: pubsub.publish(&self.topic_pose),
            pub_map: pubsub.publish(&self.topic_map),
            slam: GridMapSlam::new(&self.config),
            config: self.config.clone(),
        })
    }
}

impl Node for GridMapSlamNode {
    fn update(&mut self) {
        if let Some(o) = self.sub_obs.try_recv() {
            self.slam.update(&o);

            self.pub_pose.publish(Arc::new(self.slam.estimated_pose()));

            self.pub_map.publish(Arc::new(GridMapMessage {
                position: self.config.position,
                resolution: self.config.resolution,
                data: self.slam.estimated_likelihood(),
            }));
        }
    }

    fn draw(&mut self, ui: &egui::Ui, world: &mut common::world::WorldObj<'_>) {
        egui::Window::new("Slam").show(ui.ctx(), |ui| {
            ui.label("[WIP]");
        });
    }
}

pub struct GridMapMessage {
    pub position: Vector2<f32>,
    pub resolution: f32,
    pub data: GridData<Probability>,
}
