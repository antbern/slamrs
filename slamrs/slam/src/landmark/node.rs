use std::sync::Arc;

use common::{
    node::{Node, NodeConfig},
    robot::{LandmarkObservations, Odometry, Pose},
};
use eframe::egui;

use graphics::primitiverenderer::Color;
use pubsub::{Publisher, Subscription};
use serde::Deserialize;

use nalgebra as na;

use super::ekf::{EKFLandmarkSlam, EKFLandmarkSlamConfig, Landmark};

pub struct EKFLandmarkSlamNode {
    sub_obs_odom: Subscription<(LandmarkObservations, Odometry)>,
    pub_pose: Publisher<Pose>,
    pub_map: Publisher<LandmarkMapMessage>,
    slam: EKFLandmarkSlam,
    #[allow(dead_code)]
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

            self.pub_map.publish(Arc::new(LandmarkMapMessage {
                landmarks: self.slam.estimated_landmarks(),
            }));
        }
    }

    fn draw(&mut self, ui: &egui::Ui, world: &mut common::world::WorldObj<'_>) {
        egui::Window::new("EKF Landmark Slam").show(ui.ctx(), |ui| {
            ui.label("[WIP]");

            let cov = self.slam.raw_covariance();
            let d: na::DMatrix<f32> = na::DMatrix::from_diagonal(&cov.diagonal().map(|v| v.sqrt()));
            if let Some(d_inv) = d.try_inverse() {
                let corr = &d_inv * cov * d_inv;

                world
                    .sr
                    .begin(graphics::primitiverenderer::PrimitiveType::Filled);
                let x_offset = 2.0;
                let y_offser = 0.0;
                let size = 0.08;
                for i in 0..corr.nrows() {
                    for j in 0..corr.ncols() {
                        let c = corr[(i, j)];
                        let color = if c > 0.0 {
                            Color::rgb(0.0, c, 0.0)
                        } else if c == 0.0 {
                            Color::WHITE
                        } else {
                            Color::rgb(-c, 0.0, 0.0)
                        };
                        let x = x_offset + i as f32 * size;
                        let y = y_offser + j as f32 * size;

                        let x = if i > 2 { x + size / 3.0 } else { x };
                        let y = if j > 2 { y + size / 3.0 } else { y };
                        world.sr.rect(x, y, size, size, color);
                    }
                }

                world.sr.end();
            }
        });
    }
}

pub struct LandmarkMapMessage {
    pub landmarks: Vec<Landmark>,
}
