use common::robot::{LandmarkObservations, Odometry, Pose};

use nalgebra as na;
use serde::Deserialize;

#[derive(Clone, Debug, Deserialize)]
pub struct EKFLandmarkSlamConfig {}

#[derive(Debug)]
pub struct EKFLandmarkSlam {
    landmarks: Vec<Landmark>,
}

impl EKFLandmarkSlam {
    pub fn new(_config: &EKFLandmarkSlamConfig) -> Self {
        // TODO
        Self {
            landmarks: vec![
                Landmark {
                    mean: na::Vector2::new(0.0, 0.0),
                    covariance: na::Matrix2::identity() * 0.1,
                },
                Landmark {
                    mean: na::Vector2::new(1.0, 0.0),
                    covariance: na::Matrix2::identity() * 0.1,
                },
            ],
        }
    }

    pub fn update(&mut self, _observation: &LandmarkObservations, _odometry: Odometry) {
        // todo
    }

    pub fn estimated_pose(&self) -> Pose {
        // TODO
        Pose::default()
    }

    pub fn estimated_landmarks(&self) -> &[Landmark] {
        &self.landmarks
    }
}

#[derive(Clone, Debug)]
pub struct Landmark {
    pub mean: na::Vector2<f32>,
    pub covariance: na::Matrix2<f32>,
}
