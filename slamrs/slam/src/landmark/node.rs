use std::sync::Arc;

use common::{
    node::{Node, NodeConfig},
    robot::{LandmarkObservations, Odometry, Pose},
};
use eframe::egui;

use pubsub::{Publisher, Subscription};
use serde::Deserialize;

use super::ekf::{EKFLandmarkSlam, EKFLandmarkSlamConfig, Landmark};

pub struct EKFLandmarkSlamNode {
    sub_obs_odom: Subscription<(LandmarkObservations, Odometry)>,
    pub_pose: Publisher<Pose>,
    pub_map: Publisher<LandmarkMapMessage>,
    slam: EKFLandmarkSlam,
    config: EKFLandmarkSlamConfig,
}

#[derive(Clone, Deserialize)]
pub struct EKFLandmarkSlamNodeConfig {
    topic_pose: String,
    topic_observation_landmark: String,
    topic_map: String,
    config: EKFLandmarkSlamConfig,
}

impl NodeConfig for EKFLandmarkSlamNodeConfig {
    fn instantiate(&self, pubsub: &mut pubsub::PubSub) -> Box<dyn Node> {
        Box::new(EKFLandmarkSlamNode {
            sub_obs_odom: pubsub.subscribe(&self.topic_observation_landmark),
            pub_pose: pubsub.publish(&self.topic_pose),
            pub_map: pubsub.publish(&self.topic_map),
            slam: EKFLandmarkSlam::new(&self.config),
            config: self.config.clone(),
        })
    }
}

impl Node for EKFLandmarkSlamNode {
    fn update(&mut self) {
        if let Some(o) = self.sub_obs_odom.try_recv() {
            self.slam.update(&o.0, o.1);

            self.pub_pose.publish(Arc::new(self.slam.estimated_pose()));

            let estimated_landmarks = self.slam.estimated_landmarks();

            // log::info!("Estimated landmarks: {:?}", estimated_landmarks);
            self.pub_map.publish(Arc::new(LandmarkMapMessage {
                landmarks: estimated_landmarks,
            }));
        }
    }

    fn draw(&mut self, ui: &egui::Ui, _world: &mut common::world::WorldObj<'_>) {
        egui::Window::new("EKF Landmark Slam").show(ui.ctx(), |ui| {
            ui.label("[WIP]");
        });
    }
}

pub struct LandmarkMapMessage {
    pub landmarks: Vec<Landmark>,
}
