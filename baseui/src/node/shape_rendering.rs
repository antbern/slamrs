use common::{node::Node, world::WorldObj};
use graphics::primitiverenderer::{Color, PrimitiveType};

use pubsub::PubSub;
pub struct ShapeRendering {}

impl Node for ShapeRendering {
    fn new(_pubsub: &mut PubSub) -> Self {
        ShapeRendering {}
    }

    fn draw(&mut self, ui: &egui::Ui, w: &mut WorldObj<'_>) {
        let c1 = Color::rgba(1.0, 0.0, 0.0, 1.0);
        let c2 = Color::rgba(0.0, 1.0, 0.0, 1.0);
        let c3 = Color::rgba(0.0, 0.0, 1.0, 1.0);

        w.sr.begin(PrimitiveType::Filled);
        for x in 0..255 {
            for y in 0..255 {
                let c = Color::rgba_u8(x, y, 128, 0xff);
                w.sr.rect(
                    x as f32 / 255.0,
                    y as f32 / 255.0,
                    1.0 / 255.0,
                    1.0 / 255.0,
                    c,
                );
            }
        }

        w.sr.end();

        // self.sr.test();

        // self.pr.begin(PrimitiveType::Filled);

        // self.pr.xyc(0.0, 1.0, c1);
        // self.pr.xyc(-1.0, -1.0, c2);
        // self.pr.xyc(1.0, -1.0, c3);

        // self.pr.end();

        // self.pr.begin(PrimitiveType::Line);

        // self.pr.xyc(0.0, 1.0 + 0.1, c1);
        // self.pr.xyc(-1.0 - 0.1, -1.0 - 0.1, c2);

        // self.pr.xyc(-1.0 - 0.1, -1.0 - 0.1, c2);
        // self.pr.xyc(1.0 + 0.1, -1.0 - 0.1, c3);

        // self.pr.xyc(1.0 + 0.1, -1.0 - 0.1, c3);
        // self.pr.xyc(0.0, 1.0 + 0.1, c1);

        // self.pr.end();
    }
}
