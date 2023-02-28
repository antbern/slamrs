use std::sync::Arc;

use common::robot::{Command, Measurement, Observation, Pose};
use egui::mutex::RwLock;
use nalgebra::Point2;
use pubsub::{Publisher, Subscription};

use crate::scene::ray::{Intersect, Ray, Scene};

pub struct Simulator {
    pub_obs: Publisher<Observation>,
    pub_pose: Publisher<Pose>,
    sub_cmd: Subscription<Command>,
    scene: Arc<RwLock<Scene>>,
    parameters: SimParameters,
    pose: Pose,
    // robot pose, last command, wheel velocities etc that need to be shared between the background thread and the UI
    active: bool,
}

#[derive(Clone, Copy)]
pub struct SimParameters {
    wheel_base: f32,
}

impl Default for SimParameters {
    fn default() -> Self {
        Self { wheel_base: 0.1 }
    }
}

impl Simulator {
    pub fn new(
        pub_obs: Publisher<Observation>,
        pub_pose: Publisher<Pose>,
        sub_cmd: Subscription<Command>,
        scene: Arc<RwLock<Scene>>,
        parameters: SimParameters,
    ) -> Self {
        Self {
            pub_obs,
            pub_pose,
            sub_cmd,
            scene,
            parameters,
            pose: Pose::default(),
            active: true,
        }
    }

    pub fn set_parameters(&mut self, parameters: SimParameters) {
        self.parameters = parameters;
    }

    pub fn tick(&mut self, delta: f32) {
        // TODO: handle all incoming messages

        // TODO: simulation logic goes here

        while let Some(c) = self.sub_cmd.try_recv() {
            dbg!(c);
        }

        if self.active {
            // take a reading and send it to the drawing node
            let mut meas: Vec<Measurement> = Vec::with_capacity(10);
            let origin = Point2::new(self.pose.x, self.pose.y);

            for angle in 0..360 {
                let angle = (angle as f32).to_radians();
                // let angle = 0.0;
                if let Some(v) = self
                    .scene
                    .read()
                    .intersect(&Ray::from_point_angle(origin, angle))
                {
                    if v < 1.0 {
                        meas.push(Measurement {
                            angle: angle as f64,
                            distance: v as f64,
                            strength: 1.0,
                            valid: true,
                        });
                    }
                }
            }

            self.pub_obs
                .publish(Arc::new(Observation { measurements: meas }));

            self.pub_pose.publish(Arc::new(Pose {
                x: origin.x,
                y: origin.y,
                theta: 0.0,
            }));
        }
    }
}
