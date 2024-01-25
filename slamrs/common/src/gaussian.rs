use nalgebra::{Matrix2, Vector2};

#[derive(Debug, Clone, Copy)]
pub struct Gaussian2D {
    pub mean: Vector2<f32>,
    pub covariance: Matrix2<f32>,
}

impl Default for Gaussian2D {
    fn default() -> Self {
        Self {
            mean: Vector2::new(0.0, 0.0),
            covariance: Matrix2::identity(),
        }
    }
}
