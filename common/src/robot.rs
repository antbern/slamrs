use nalgebra::{Matrix2xX, Point2, Vector2, Vector3};

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

/// Contains all data for a single lidar scan (a complete revolution)
#[derive(Clone)]
pub struct Observation {
    pub id: usize,
    pub measurements: Vec<Measurement>,
}

impl Observation {
    pub fn to_points(&self, origin: Pose) -> Vec<Point2<f32>> {
        self.measurements
            .iter()
            .filter(|&m| m.valid)
            .map(|m| {
                Point2::new(
                    origin.x + (origin.theta + m.angle as f32).cos() * m.distance as f32,
                    origin.y + (origin.theta + m.angle as f32).sin() * m.distance as f32,
                )
            })
            .collect()
    }

    pub fn to_matrix(&self, origin: Pose) -> Matrix2xX<f32> {
        // Not the most efficient implementation (since it creates two Vec's, but it works)
        let vectors: Vec<Vector2<f32>> = self.to_points(origin).iter().map(|p| p.coords).collect();
        Matrix2xX::from_columns(&vectors)
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
#[derive(Debug, Clone, Copy, Default)]
pub struct Odometry {
    /// The distance in meters that the left wheel of the robot travelled since last odometry reading.
    pub distance_left: f32,

    /// The distance in meters that the right wheel of the robot travelled since last odometry reading.
    pub distance_right: f32,
}

/// A Command to move the robot by setting the desired left and right wheel speed.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Command {
    /// The target speed in meters/second that the left wheel of the robot should move.
    pub speed_left: f32,

    /// The target speed in meters/second that the right wheel of the robot should move.
    pub speed_right: f32,
}
