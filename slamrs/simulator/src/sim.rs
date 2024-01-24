use std::sync::Arc;

use common::robot::{
    Command, LandmarkObservation, LandmarkObservations, Measurement, Observation, Odometry, Pose,
};
use egui::mutex::RwLock;
use nalgebra::{Point2, Vector2};
use pubsub::{Publisher, Subscription};
use serde::Deserialize;

use crate::scene::ray::{Intersect, Ray, Scene};

pub struct Simulator {
    pub_obs_scanner: Option<Publisher<(Observation, Odometry)>>,
    pub_obs_landmarks: Option<Publisher<(LandmarkObservations, Odometry)>>,
    pub_pose: Option<Publisher<Pose>>,
    sub_cmd: Subscription<Command>,
    scene: Arc<RwLock<Scene>>,
    parameters: SimParameters,
    pose: Pose,
    wheel_velocity: Vector2<f32>,
    active: bool,
    scan_update_timer: f32,
    scan_counter: usize,
    wheel_motion_accumulator: (f32, f32),
}

#[derive(Clone, Copy, Deserialize)]
pub struct SimParameters {
    /// The wheel base (in meters) of the differential robot used in the simulator, i.e,
    /// the distance between the wheels.
    pub(crate) wheel_base: f32,

    /// The update period (in ms) of the laser range scanner, i.e., 1/Hz.
    pub(crate) update_period: f32,

    /// Laser range scanner maximum distance in meters.
    pub(crate) scanner_range: f32,
}

impl Default for SimParameters {
    fn default() -> Self {
        Self {
            wheel_base: 0.1,
            update_period: 0.2,
            scanner_range: 1.0,
        }
    }
}

impl Simulator {
    pub fn new(
        pub_obs_scanner: Option<Publisher<(Observation, Odometry)>>,
        pub_obs_landmarks: Option<Publisher<(LandmarkObservations, Odometry)>>,
        pub_pose: Option<Publisher<Pose>>,
        sub_cmd: Subscription<Command>,
        scene: Arc<RwLock<Scene>>,
        parameters: SimParameters,
    ) -> Self {
        Self {
            pub_obs_scanner,
            pub_obs_landmarks,
            pub_pose,
            sub_cmd,
            scene,
            parameters,
            pose: Pose::default(),
            wheel_velocity: Vector2::zeros(),
            active: true,
            scan_update_timer: 0.0,
            scan_counter: 0,
            wheel_motion_accumulator: (0.0, 0.0),
        }
    }

    pub fn parameters_mut(&mut self) -> &mut SimParameters {
        &mut self.parameters
    }

    pub fn get_pose(&self) -> Pose {
        self.pose
    }

    pub fn tick(&mut self, dt: f32) {
        // consume any incoming motion commands
        while let Some(c) = self.sub_cmd.try_recv() {
            self.wheel_velocity = Vector2::new(c.speed_left, c.speed_right);
        }

        if self.active {
            // make the robot move
            self.motion_model(self.wheel_velocity.x * dt, self.wheel_velocity.y * dt);

            self.wheel_motion_accumulator.0 += self.wheel_velocity.x * dt;
            self.wheel_motion_accumulator.1 += self.wheel_velocity.y * dt;

            // if it's time for a scan, perform it!
            self.scan_update_timer += dt;
            if self.scan_update_timer > self.parameters.update_period {
                self.scan_update_timer -= self.parameters.update_period;

                // new scan will be taken, prepare an odometry measurement
                let odometry = Odometry::new(
                    self.wheel_motion_accumulator.0,
                    self.wheel_motion_accumulator.1,
                );

                // reset the accumulator
                self.wheel_motion_accumulator = (0.0, 0.0);

                if let Some(pub_pose) = &mut self.pub_pose {
                    pub_pose.publish(Arc::new(self.pose));
                }

                // if the laser scanner is enabled, perform a scan
                if let Some(pub_obs) = &mut self.pub_obs_scanner {
                    // take a reading and send it to the drawing node
                    let mut meas: Vec<Measurement> = Vec::with_capacity(360);
                    let origin = Point2::new(self.pose.x, self.pose.y);

                    for angle in 0..360 {
                        let angle = (angle as f32).to_radians();

                        // let angle = 0.0;
                        if let Some(v) = self
                            .scene
                            .read()
                            .intersect(&Ray::from_origin_angle(origin, angle + self.pose.theta))
                        {
                            if v < self.parameters.scanner_range {
                                meas.push(Measurement {
                                    angle: angle as f64,
                                    distance: v as f64,
                                    strength: 1.0,
                                    valid: true,
                                });
                            } else {
                                meas.push(Measurement {
                                    angle: angle as f64,
                                    distance: self.parameters.scanner_range as f64,
                                    strength: 1.0,
                                    valid: false, // Treat the valid flag as a hit/no hit for now
                                });
                            }
                        }
                    }

                    pub_obs.publish(Arc::new((
                        Observation {
                            id: self.scan_counter,
                            measurements: meas,
                        },
                        odometry.clone(),
                    )));

                    self.scan_counter += 1;
                }

                // if the landmark sensor is enabled, perform a scan
                if let Some(pub_obs) = &mut self.pub_obs_landmarks {
                    let mut observations = Vec::new();

                    // go through all the landmarks and find the ones that are in the field of view infrontof the robot

                    for l in self.scene.read().landmarks() {
                        let dist_sq = (self.pose.x - l.x).powi(2) + (self.pose.y - l.y).powi(2);
                        if dist_sq > self.parameters.scanner_range {
                            continue;
                        }

                        // within range, create observation
                        let angle = (l.y - self.pose.y).atan2(l.x - self.pose.x);

                        // TODO: filter based on angle difference

                        observations.push(LandmarkObservation {
                            angle: angle - self.pose.theta,
                            distance: dist_sq.sqrt(),
                        })
                    }

                    tracing::debug!("Publishing landmarks: {observations:?}");

                    pub_obs.publish(Arc::new((
                        LandmarkObservations {
                            landmarks: observations,
                        },
                        odometry.clone(),
                    )));
                }
            }
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
