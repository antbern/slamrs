use serde::Deserialize;

use super::ray::Draw;

#[derive(Debug, Clone, Copy, Deserialize)]
pub struct Landmark {
    pub x: f32,
    pub y: f32,
}

impl Draw for Landmark {
    fn draw(
        &self,
        r: &mut graphics::shaperenderer::ShapeRenderer,
        color: graphics::primitiverenderer::Color,
    ) {
        r.circle(self.x, self.y, 0.05, color);
    }
}
