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

    // pre-computed sine and cosine values for the "back-wing" of the arrow
    // const arrowAngle: f32 = 45f32.to_radians();
    // const aSin: f32 = arrowAngle.sin();
    // const aCos: f32 = arrowAngle.cos();

    pub fn arrow(&mut self, x: f32, y: f32, angle_rad: f32, radius: f32, color: Color) {
        // pre compute sin and cos for the rotation
        let (s, c) = angle_rad.sin_cos();

        let (aSin, aCos) = 45f32.sin_cos();

        // Used Wolfram Alpha for the following trigonometric identities for the corner points:
        // cos(t+pi-a) = -sin(a)sin(t)-cos(a)cos(t)
        // sin(t+pi-a) = sin(a)cos(t)-cos(a)sin(t)
        // cos(t+pi+a) = sin(a)sin(t)-cos(a)cos(t)
        // sin(t+pi+a) = sin(a)-cos(t)-cos(a)sin(t)

        // pre compute the factors above for the position of the corner points
        let leftCos = -aSin * s - aCos * c;
        let leftSin = aSin * c - aCos * s;
        let rightCos = aSin * s - aCos * c;
        let rightSin = aSin * -c - aCos * s;

        match self.current_shape_type {
            Some(PrimitiveType::Filled) => {
                // check(ShapeType.FILLED, null, 3 * 2);

                // front
                self.pr.xyc(x + c * radius, y + s * radius, color);

                // back left
                self.pr
                    .xyc(x + leftCos * radius, y + leftSin * radius, color);

                // back middle
                self.pr
                    .xyc(x - c * (radius / 3.0), y - s * (radius / 3.0), color);

                // back middle (again, starting a new triangle)
                self.pr
                    .xyc(x - c * (radius / 3.0), y - s * (radius / 3.0), color);

                // back right
                self.pr
                    .xyc(x + rightCos * radius, y + rightSin * radius, color);

                // front
                self.pr.xyc(x + c * radius, y + s * radius, color);
            }
            Some(PrimitiveType::Line) => {
                // check(ShapeType.LINE, ShapeType.POINT, 4 * 2);

                // front
                self.pr.xyc(x + c * radius, y + s * radius, color);

                // back left
                self.pr
                    .xyc(x + leftCos * radius, y + leftSin * radius, color);

                // back left (again, starting a new line)
                self.pr
                    .xyc(x + leftCos * radius, y + leftSin * radius, color);

                // back middle
                self.pr
                    .xyc(x - c * (radius / 3.0), y - s * (radius / 3.0), color);

                // back middle (again, starting a new line)
                self.pr
                    .xyc(x - c * (radius / 3.0), y - s * (radius / 3.0), color);

                // back right
                self.pr
                    .xyc(x + rightCos * radius, y + rightSin * radius, color);

                // back right (again, starting a new line)
                self.pr
                    .xyc(x + rightCos * radius, y + rightSin * radius, color);

                // front
                self.pr.xyc(x + c * radius, y + s * radius, color);
            }
            _ => {}
        }
    }

    /*
    pub fn test(&mut self) {
        let c1 = Color::rgba(1.0, 0.0, 0.0, 1.0);
        let c2 = Color::rgba(0.0, 1.0, 0.0, 1.0);
        let c3 = Color::rgba(0.0, 0.0, 1.0, 1.0);

        let mut g = self.pr.begin2(PrimitiveType::Filled);

        g.xyc(0.0, 1.0, c1);
        g.xyc(-1.0, -1.0, c2);
        g.xyc(1.0, -1.0, c3);

        g.end();
        let mut g2 = self.pr.begin2(PrimitiveType::Line);

        // let g = self.pr.begin2(PrimitiveType::Line);

        // self.pr.xyc(0.0, 1.0 + 0.1, c1);
        // self.pr.xyc(-1.0 - 0.1, -1.0 - 0.1, c2);

        // self.pr.xyc(-1.0 - 0.1, -1.0 - 0.1, c2);
        // self.pr.xyc(1.0 + 0.1, -1.0 - 0.1, c3);

        // self.pr.xyc(1.0 + 0.1, -1.0 - 0.1, c3);
        // self.pr.xyc(0.0, 1.0 + 0.1, c1);

        // self.pr.end();
    }
    */

    pub fn destroy(&self, gl: &glow::Context) {
        self.pr.destroy(gl);
    }
}
