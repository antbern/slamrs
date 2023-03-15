use std::sync::Arc;

use common::robot::{Command, Measurement, Observation, Pose};
use egui::mutex::RwLock;
use nalgebra::{Point2, Vector2};
use pubsub::{Publisher, Subscription};

use crate::scene::ray::{Intersect, Ray, Scene};

pub struct Simulator {
    pub_obs: Publisher<Observation>,
    pub_pose: Publisher<Pose>,
    sub_cmd: Subscription<Command>,
    scene: Arc<RwLock<Scene>>,
    parameters: SimParameters,
    pose: Pose,
    wheel_velocity: Vector2<f32>,
    // robot pose, last command, wheel velocities etc that need to be shared between the background thread and the UI
    active: bool,
    scan_update_timer: f32,
    scan_counter: usize,
}

#[derive(Clone, Copy)]
pub struct SimParameters {
    wheel_base: f32,
    update_period: f32, // 1/Hz
}

impl Default for SimParameters {
    fn default() -> Self {
        Self {
            wheel_base: 0.1,
            update_period: 0.2,
        }
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
            wheel_velocity: Vector2::zeros(),
            active: true,
            scan_update_timer: 0.0,
            scan_counter: 0,
        }
    }

    pub fn set_parameters(&mut self, parameters: SimParameters) {
        self.parameters = parameters;
    }

    pub fn get_pose(&self) -> Pose {
        self.pose
    }

    pub fn tick(&mut self, dt: f32) {
        // TODO: handle all incoming messages

        // TODO: simulation logic goes here

        while let Some(c) = self.sub_cmd.try_recv() {
            self.wheel_velocity = Vector2::new(c.speed_left, c.speed_right);
        }

        if self.active {
            self.scan_update_timer += dt;

            if self.scan_update_timer > self.parameters.update_period {
                self.scan_update_timer -= self.parameters.update_period;

                // take a reading and send it to the drawing node
                let mut meas: Vec<Measurement> = Vec::with_capacity(10);
                let origin = Point2::new(self.pose.x, self.pose.y);

                for angle in 0..360 {
                    let angle = (angle as f32).to_radians();

                    // let angle = 0.0;
                    if let Some(v) = self
                        .scene
                        .read()
                        .intersect(&Ray::from_point_angle(origin, angle + self.pose.theta))
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

                self.pub_obs.publish(Arc::new(Observation {
                    id: self.scan_counter,
                    measurements: meas,
                }));
                self.scan_counter += 1;
            }

            // make the robot move
            self.motion_model(self.wheel_velocity.x * dt, self.wheel_velocity.y * dt);
        }
    }

    fn motion_model(&mut self, sl: f32, sr: f32) {
        // from https://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html
        let sbar = (sr + sl) / 2.0;
        self.pose.theta += (sr - sl) / self.parameters.wheel_base;
        self.pose.x += sbar * self.pose.theta.cos();
        self.pose.y += sbar * self.pose.theta.sin();
    }
}
