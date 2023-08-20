use nalgebra::{Matrix2xX, Vector2, Vector3};

use crate::math::{self, LogProbability};
use rand::distributions::Distribution;
use statrs::distribution::{Continuous, Normal};

/// The pose of a robot in the 2D plane.
#[derive(Copy, Clone, Default)]
pub struct Pose {
    /// The x position of the robot
    pub x: f32,

    /// The y position of the robot
    pub y: f32,

    /// The rotation of the robot, measured in radians counter-clockwise from the positive x-axis.
    pub theta: f32,
}

impl From<Pose> for (f32, f32) {
    fn from(val: Pose) -> Self {
        (val.x, val.y)
    }
}

impl From<Pose> for Vector3<f32> {
    fn from(value: Pose) -> Self {
        Vector3::new(value.x, value.y, value.theta)
    }
}

impl From<Vector3<f32>> for Pose {
    fn from(value: Vector3<f32>) -> Self {
        Pose {
            x: value[0],
            y: value[1],
            theta: value[2],
        }
    }
}

impl Pose {
    pub fn xy(&self) -> Vector2<f32> {
        Vector2::new(self.x, self.y)
    }
}

/// Contains all data for a single lidar scan (a complete revolution)
/// Note that these measurements are in the robots local coordinate system.
#[derive(Clone)]
pub struct Observation {
    pub id: usize,
    pub measurements: Vec<Measurement>,
}

impl Observation {
    pub fn to_points(&self, origin: Pose) -> Vec<Vector2<f32>> {
        self.measurements
            .iter()
            .filter(|&m| m.valid)
            .map(|m| {
                Vector2::new(
                    origin.x + (origin.theta + m.angle as f32).cos() * m.distance as f32,
                    origin.y + (origin.theta + m.angle as f32).sin() * m.distance as f32,
                )
            })
            .collect()
    }

    pub fn to_matrix(&self, origin: Pose) -> Matrix2xX<f32> {
        // Not the most efficient implementation (since it creates two Vec's, but it works)
        let vectors: Vec<Vector2<f32>> = self.to_points(origin);
        if !vectors.is_empty() {
            Matrix2xX::from_columns(&vectors)
        } else {
            Matrix2xX::zeros(0)
        }
    }
}

#[derive(Clone, Copy)]
pub struct Measurement {
    /// The angle this measurement was acquired at (relative to the sensor zero) in radians.
    pub angle: f64,

    /// The distance measured in meters.
    pub distance: f64,

    /// The strength of the measurement (if applicable)
    pub strength: f64,

    /// If this measurement is valid (information provided by the sensor itself)
    pub valid: bool,
}

/// Observed (measured) motion of the left and right wheel
#[derive(Debug, Clone, Copy)]
pub struct Odometry {
    /// The distance in meters that the left wheel of the robot travelled since last odometry reading.
    pub distance_left: f32,

    /// The distance in meters that the right wheel of the robot travelled since last odometry reading.
    pub distance_right: f32,

    /// Distribution that describes how far the center has moved
    distribution_center: Normal,
    /// Distribution that describes the angle moved (in radians)
    distribution_theta: Normal,
}

pub const WHEEL_DISTANCE: f32 = 0.1;

impl Default for Odometry {
    fn default() -> Self {
        Self::new(0.0, 0.0)
    }
}

impl Odometry {
    pub fn new(distance_left: f32, distance_right: f32) -> Self {
        let delta_center = ((distance_left + distance_right) / 2.0) as f64;
        let delta_theta = ((distance_right - distance_left) / WHEEL_DISTANCE) as f64;

        // simple model for the expected variation in measurement vs world:
        // Some fixed (minimum) variation + a part proportional to the change in value
        let delta_center_std = (0.01 + delta_center.abs() * 0.05) / 2.0;
        let delta_theta_std = f64::to_radians(5.0) + 0.1 * delta_theta.abs();

        Self {
            distribution_center: Normal::new(delta_center, delta_center_std)
                .expect("Create normal distribution for center"),
            distribution_theta: Normal::new(delta_theta, delta_theta_std)
                .expect("Create normal distribution for theta"),
            distance_left,
            distance_right,
        }
    }

    pub fn probabiliy_of(&self, initial_pose: Pose, new_pose: Pose) -> LogProbability {
        // hard code the parameters here for now. Future improvements should add
        // a MotionModel to handle the forward and inverse case.

        let center_distance =
            ((initial_pose.x - new_pose.x).powi(2) + (initial_pose.y - new_pose.y).powi(2)).sqrt();

        // TODO: should probably normalize the angle here... or at least calculate the shortest
        // distance between the angles
        let angle_distance = math::angle_diff(initial_pose.theta.into(), new_pose.theta.into());
        
        // Since the pdf is not really a probability, we will do an unchecked initialization here
        // TODO: improve!
        LogProbability::new_unchecked(self.distribution_center.pdf(center_distance as f64))
            * LogProbability::new_unchecked(self.distribution_theta.pdf(angle_distance as f64))
    }

    /// Samples from the motion model with the specific initial Pose
    pub fn sample(&self, initial_pose: Pose) -> Pose {
        // take a sample from this very simple motion model

        let mut rng = rand::thread_rng();

        let center_distance = self.distribution_center.sample(&mut rng) as f32;
        let theta = initial_pose.theta + self.distribution_theta.sample(&mut rng) as f32;

        Pose {
            theta,
            x: initial_pose.x + theta.cos() * center_distance,
            y: initial_pose.y + theta.sin() * center_distance,
        }
    }
}

/// A Command to move the robot by setting the desired left and right wheel speed.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Command {
    /// The target speed in meters/second that the left wheel of the robot should move.
    pub speed_left: f32,

    /// The target speed in meters/second that the right wheel of the robot should move.
    pub speed_right: f32,
}
