use graphics::shaperenderer::ShapeRenderer;
use nalgebra::Point2;

pub struct WorldObj<'a> {
    pub sr: &'a mut ShapeRenderer,
    pub last_mouse_pos: Point2<f32>,
}
