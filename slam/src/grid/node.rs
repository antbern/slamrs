use std::sync::Arc;

use common::{
    node::{Node, NodeConfig},
    robot::{Observation, Odometry, Pose},
};
use nalgebra::Vector2;
use pubsub::{Publisher, Subscription};
use serde::Deserialize;

use super::{
    map::GridData,
    slam::{GridMapSlam, GridMapSlamConfig},
};
use common::math::Probability;

pub struct GridMapSlamNode {
    sub_obs_odom: Subscription<(Observation, Odometry)>,
    pub_pose: Publisher<Pose>,
    pub_map: Publisher<GridMapMessage>,
    slam: GridMapSlam,
    config: GridMapSlamConfig,
}

#[derive(Deserialize)]
pub struct GridMapSlamNodeConfig {
    topic_pose: String,
    topic_observation_odometry: String,
    topic_map: String,
    config: GridMapSlamConfig,
}

impl NodeConfig for GridMapSlamNodeConfig {
    fn instantiate(&self, pubsub: &mut pubsub::PubSub) -> Box<dyn Node> {
        Box::new(GridMapSlamNode {
            sub_obs_odom: pubsub.subscribe(&self.topic_observation_odometry),
            pub_pose: pubsub.publish(&self.topic_pose),
            pub_map: pubsub.publish(&self.topic_map),
            slam: GridMapSlam::new(&self.config),
            config: self.config.clone(),
        })
    }
}

impl Node for GridMapSlamNode {
    fn update(&mut self) {
        if let Some(o) = self.sub_obs_odom.try_recv() {
            self.slam.update(&o.0, o.1);

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
