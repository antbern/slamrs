use common::robot::Pose;
use graphics::{
    primitiverenderer::{Color, PrimitiveType},
    shaperenderer::ShapeRenderer,
};

pub trait Visualize {
    // type Parameters;
    fn visualize(&self, sr: &mut ShapeRenderer);
}

impl Visualize for Pose {
    fn visualize(&self, sr: &mut graphics::shaperenderer::ShapeRenderer) {
        sr.begin(PrimitiveType::Filled);
        sr.arrow(self.x + 0.1, self.y, self.theta, 0.1, Color::RED);
        sr.end()
    }
}
