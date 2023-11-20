use common::robot::{Observation, Odometry, Pose};
use nalgebra::Vector2;
use serde::Deserialize;

use super::{
    map::{GridData, Map},
    particle::ParticleFilter,
};

use common::math::Probability;
type PoseMap = (Pose, Map);

pub struct GridMapSlam {
    // map: Map,
    filter: ParticleFilter<PoseMap>,
}

#[derive(Deserialize, Clone)]
pub struct GridMapSlamConfig {
    pub position: Vector2<f32>,
    pub width: f32,
    pub height: f32,
    pub resolution: f32,
    n_particles: usize,
}

impl GridMapSlam {
    pub fn new(config: &GridMapSlamConfig) -> Self {
        GridMapSlam {
            filter: ParticleFilter::new(
                config.n_particles,
                (
                    Pose::default(),
                    Map::new(
                        config.position,
                        config.width,
                        config.height,
                        config.resolution,
                    ),
                ),
            ),
        }
    }

    pub fn update(&mut self, z: &Observation, u: Odometry) {
        // self.map.integrate(observation, Pose::default());

        let update_map = true;

        self.filter.update(|(pose, map)| {
            let initial_pose = *pose;

            // first sample a new pose from the motion model based on the given controls (odometry)
            let new_pose = u.sample(initial_pose);

            // calculate the weight of this particle as p(z|x,m)
            let likelihood = map.likelihood();

            // OPTIONAL: optimize pose position to maximize measurement likelihood (scan-matching?)

            let weight = map.probability_of(z, new_pose) * u.probabiliy_of(initial_pose, new_pose);
            // * SensorModel.probHeading(z.getHeading(), pose.theta); // Compass measurements are not included

            if update_map {
                // integrate the measurement into the particles map
                map.integrate(z, new_pose);
            }
            *pose = new_pose;

            weight.prob().value()
        });

        self.filter.resample();
    }

    pub fn estimated_pose(&self) -> Pose {
        self.filter
            .particle_value(self.filter.strongest_particle_idx())
            .0
    }

    pub fn estimated_likelihood(&self) -> GridData<Probability> {
        self.filter
            .particle_value(self.filter.strongest_particle_idx())
            .1
            .likelihood()
    }

    pub fn map_position(&self) -> Vector2<f32> {
        // TODO: the position never changes for the particles...
        self.filter
            .particle_value(self.filter.strongest_particle_idx())
            .1
            .position()
    }
}
