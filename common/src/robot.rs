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

/// Contains all data for a single lidar scan (a complete revolution)
pub struct Observation {
    pub measurements: Vec<Measurement>,
}

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

