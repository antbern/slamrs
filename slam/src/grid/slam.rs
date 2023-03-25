use common::robot::{Observation, Pose};
use nalgebra::Vector2;
use serde::Deserialize;

use super::{
    map::{GridData, Map},
    math::Probability,
};

pub struct GridMapSlam {
    map: Map,
}

#[derive(Deserialize, Clone)]
pub struct GridMapSlamConfig {
    pub position: Vector2<f32>,
    pub width: f32,
    pub height: f32,
    pub resolution: f32,
}

impl GridMapSlam {
    pub fn new(config: &GridMapSlamConfig) -> Self {
        GridMapSlam {
            map: Map::new(
                config.position,
                config.width,
                config.height,
                config.resolution,
            ),
        }
    }

    pub fn update(&mut self, observation: &Observation) {
        self.map.integrate(observation, Pose::default());
    }

    pub fn estimated_pose(&self) -> Pose {
        Pose::default()
    }

    pub fn estimated_likelihood(&self) -> GridData<Probability> {
        self.map.likelihood()
    }

    pub fn map_position(&self) -> Vector2<f32> {
        self.map.position()
    }
}
