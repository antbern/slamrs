use eframe::egui;
use std::f32::consts::{FRAC_PI_3, FRAC_PI_6, PI};

use common::{
    node::{Node, NodeConfig},
    world::WorldObj,
};
use graphics::primitiverenderer::{Color, PrimitiveType};

use pubsub::PubSub;
use serde::Deserialize;
pub struct ShapeRendering {}

#[derive(Clone, Deserialize)]
pub struct ShapeRenderingNodeConfig {}

impl NodeConfig for ShapeRenderingNodeConfig {
    fn instantiate(&self, _pubsub: &mut PubSub) -> Box<dyn Node> {
        Box::new(ShapeRendering {})
    }
}

impl Node for ShapeRendering {
    fn draw(&mut self, _ui: &egui::Ui, w: &mut WorldObj<'_>) {
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

        w.sr.begin(PrimitiveType::Line);
        w.sr.circle(0.1, -0.1, 0.05, Color::RED);
        w.sr.end();

        w.sr.begin(PrimitiveType::Filled);
        w.sr.circle(-0.1, -0.1, 0.05, Color::GREEN);
        w.sr.end();

        w.sr.begin(PrimitiveType::Line);
        w.sr.arrow(-0.1, 0.1, PI + FRAC_PI_3, 0.1, Color::RED);
        w.sr.end();

        w.sr.begin(PrimitiveType::Filled);
        w.sr.arrow(0.1, 0.1, FRAC_PI_6, 0.1, Color::GREEN);
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
