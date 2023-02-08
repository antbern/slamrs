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
