use eframe::glow;
use std::f32::consts::PI;

use nalgebra::{Matrix2, Vector2};

use crate::primitiverenderer::Color;

use super::primitiverenderer::{PrimitiveRenderer, PrimitiveType, Vertex2C};

pub struct ShapeRenderer {
    pr: PrimitiveRenderer,
    current_shape_type: Option<PrimitiveType>,
}

// TODO: this could build on some trait for adding vertices that the primitive renderer implements

impl ShapeRenderer {
    pub fn new(gl: &glow::Context) -> Self {
        Self {
            pr: PrimitiveRenderer::new(gl, 1000000),
            current_shape_type: None,
        }
    }

    pub fn set_mvp(&mut self, mvp: nalgebra::Matrix4<f32>) {
        self.pr.set_mvp(mvp);
    }

    pub fn begin(&mut self, pt: PrimitiveType) {
        self.current_shape_type = Some(pt);
        self.pr.begin(pt);
    }

    pub fn end(&mut self) {
        self.pr.end();
        self.current_shape_type = None;
    }

    pub fn flush(&mut self, gl: &glow::Context) {
        self.pr.flush(gl);
    }

    fn check(&mut self, desired_type: PrimitiveType, other: PrimitiveType, _n_vertices: usize) {
        if let Some(pt) = self.current_shape_type {
            // do we need to "restart" ?
            if pt != desired_type && pt != other {
                self.end();
                self.begin(desired_type);
            }
        //  else if (renderer.getVertexCount() + numberOfNewVertices > renderer.getMaxVertices()) {
        // 	ShapeType type = currentShapeType;
        // 	end();
        // 	begin(type);
        // }
        } else {
            panic!("begin() must be called first");
        }
    }

    pub fn line(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, color: Color) {
        self.check(PrimitiveType::Line, PrimitiveType::Point, 2);

        self.pr.xyc(x1, y1, color);
        self.pr.xyc(x2, y2, color);
    }

    pub fn rect(&mut self, x: f32, y: f32, width: f32, height: f32, color: Color) {
        self.check(
            PrimitiveType::Line,
            PrimitiveType::Filled,
            8, /*not really always 8 though */
        );

        match self.current_shape_type {
            Some(PrimitiveType::Line) => {
                self.pr.xyc(x, y, color);
                self.pr.xyc(x + width, y, color);
                self.pr.xyc(x + width, y, color);
                self.pr.xyc(x + width, y + height, color);
                self.pr.xyc(x + width, y + height, color);
                self.pr.xyc(x, y + height, color);
                self.pr.xyc(x, y + height, color);
                self.pr.xyc(x, y, color);
            }
            Some(PrimitiveType::Filled) => {
                self.pr.xyc(x, y, color);
                self.pr.xyc(x + width, y, color);
                self.pr.xyc(x + width, y + height, color);
                self.pr.xyc(x + width, y + height, color);
                self.pr.xyc(x, y + height, color);
                self.pr.xyc(x, y, color);
            }
            _ => {}
        }
    }

    pub fn circle(&mut self, x: f32, y: f32, radius: f32, color: Color) {
        // calculate the number of segments needed for a "good" circle
        let number_of_segments = 1.max((4.0 * 12.0 * radius.cbrt()) as usize);
        self._circle(x, y, radius, color, number_of_segments);
    }

    fn _circle(&mut self, x: f32, y: f32, radius: f32, color: Color, number_of_segments: usize) {
        // the angle between each circle segment
        let angle_per_segment = 2.0 * std::f32::consts::PI / number_of_segments as f32;

        // precompute sin and cos
        let (s, c) = angle_per_segment.sin_cos();

        // starting point
        let mut px: f32 = radius;
        let mut py: f32 = 0.0;

        match self.current_shape_type {
            Some(PrimitiveType::Line) => {
                // check(ShapeType.LINE, null, numberOfSegments * 2 + 2);

                // place one vertex for each segment
                for _ in 0..number_of_segments {
                    self.pr.xyc(x + px, y + py, color);

                    // rotate point using the "rotation matrix" multiplication to get to the next
                    (px, py) = (c * px - s * py, s * px + c * py);

                    self.pr.xyc(x + px, y + py, color);
                }
            }
            Some(PrimitiveType::Filled) => {
                // check(ShapeType.LINE, ShapeType.FILLED, numberOfSegments * 3 + 3);

                // place one vertex for each segment
                for _ in 0..number_of_segments {
                    self.pr.xyc(x, y, color);
                    self.pr.xyc(x + px, y + py, color);

                    // rotate point using the "rotation matrix" multiplication to get to the next
                    (px, py) = (c * px - s * py, s * px + c * py);

                    self.pr.xyc(x + px, y + py, color);
                }
            }
            _ => {}
        }
    }

    pub fn arrow(&mut self, x: f32, y: f32, angle_rad: f32, radius: f32, color: Color) {
        // pre compute sin and cos for the rotation
        let (s, c) = angle_rad.sin_cos();

        // pre-computed sine and cosine values for the "back-wing" of the arrow
        let (a_sin, a_cos) = 45f32.sin_cos();

        // Used Wolfram Alpha for the following trigonometric identities for the corner points:
        // cos(t+pi-a) = -sin(a)sin(t)-cos(a)cos(t)
        // sin(t+pi-a) = sin(a)cos(t)-cos(a)sin(t)
        // cos(t+pi+a) = sin(a)sin(t)-cos(a)cos(t)
        // sin(t+pi+a) = sin(a)-cos(t)-cos(a)sin(t)

        // pre compute the factors above for the position of the corner points
        let left_cos = -a_sin * s - a_cos * c;
        let left_sin = a_sin * c - a_cos * s;
        let right_cos = a_sin * s - a_cos * c;
        let right_sin = a_sin * -c - a_cos * s;

        match self.current_shape_type {
            Some(PrimitiveType::Filled) => {
                // check(ShapeType.FILLED, null, 3 * 2);

                // front
                self.pr.xyc(x + c * radius, y + s * radius, color);

                // back left
                self.pr
                    .xyc(x + left_cos * radius, y + left_sin * radius, color);

                // back middle
                self.pr
                    .xyc(x - c * (radius / 3.0), y - s * (radius / 3.0), color);

                // back middle (again, starting a new triangle)
                self.pr
                    .xyc(x - c * (radius / 3.0), y - s * (radius / 3.0), color);

                // back right
                self.pr
                    .xyc(x + right_cos * radius, y + right_sin * radius, color);

                // front
                self.pr.xyc(x + c * radius, y + s * radius, color);
            }
            Some(PrimitiveType::Line) => {
                // check(ShapeType.LINE, ShapeType.POINT, 4 * 2);

                // front
                self.pr.xyc(x + c * radius, y + s * radius, color);

                // back left
                self.pr
                    .xyc(x + left_cos * radius, y + left_sin * radius, color);

                // back left (again, starting a new line)
                self.pr
                    .xyc(x + left_cos * radius, y + left_sin * radius, color);

                // back middle
                self.pr
                    .xyc(x - c * (radius / 3.0), y - s * (radius / 3.0), color);

                // back middle (again, starting a new line)
                self.pr
                    .xyc(x - c * (radius / 3.0), y - s * (radius / 3.0), color);

                // back right
                self.pr
                    .xyc(x + right_cos * radius, y + right_sin * radius, color);

                // back right (again, starting a new line)
                self.pr
                    .xyc(x + right_cos * radius, y + right_sin * radius, color);

                // front
                self.pr.xyc(x + c * radius, y + s * radius, color);
            }
            _ => {}
        }
    }

    /// Use the information in the Gaussian2D component to draw the correct ellipse around the uncertainty as well as a center piece
    pub fn gaussian2d(&mut self, mean: &Vector2<f32>, covariance: &Matrix2<f32>, p: f32) {
        self.begin(PrimitiveType::Filled);
        self.circle(mean.x, mean.y, 0.01, Color::BLUE);
        self.end();

        // Matlab reference (Source: https://www.xarg.org/2018/04/how-to-plot-a-covariance-error-ellipse/)
        // s = -2 * log(1 - p);
        // [V, D] = eig(Sigma * s);
        // t = linspace(0, 2 * pi);
        // a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];
        // plot(a(1, :) + mu(1), a(2, :) + mu(2));

        // update the ellipse radii

        let s = -2.0 * (1.0 - p).ln();

        let eigen = (covariance * s).symmetric_eigen();

        let d = Matrix2::from_diagonal(&eigen.eigenvalues.map(|v| v.sqrt()));
        let v = eigen.eigenvectors;

        self.begin(PrimitiveType::Line);
        let steps = 25;
        for i in 0..steps {
            let angle = i as f32 * PI * 2.0 / steps as f32;
            let start = mean + (v * d) * Vector2::new(angle.cos(), angle.sin());

            let angle = ((i + 1) % steps) as f32 * PI * 2.0 / steps as f32;
            let end = mean + (v * d) * Vector2::new(angle.cos(), angle.sin());
            self.line(start.x, start.y, end.x, end.y, Color::BLACK);
        }

        self.end();
    }

    pub fn destroy(&self, gl: &glow::Context) {
        self.pr.destroy(gl);
    }
}
