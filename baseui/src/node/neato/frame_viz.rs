use std::sync::Arc;

use pubsub::{PubSub, Subscription};

use crate::{
    graphics::primitiverenderer::{Color, PrimitiveType},
    node::Node,
};

use super::frame::NeatoFrame;

pub struct FrameVizualizer {
    last_frame: Option<Arc<NeatoFrame>>,
    sub: Subscription<NeatoFrame>,
}

impl Node for FrameVizualizer {
    fn new(pubsub: &mut PubSub) -> Self
    where
        Self: Sized,
    {
        Self {
            last_frame: None,
            sub: pubsub.subscribe("scan"),
        }
    }

    fn draw(&mut self, _ui: &egui::Ui, world: &mut crate::app::WorldRenderer) {
        if let Some(v) = self.sub.try_recv() {
            self.last_frame = Some(v);
        }

        if let Some(frame) = &self.last_frame {
            // draw the reading into the world

            world.sr.begin(PrimitiveType::Line);

            for i in 0..360 {
                let (s, c) = (i as f32).to_radians().sin_cos();
                let d = frame.distance[i] as f32 / 1000.0;
                let x = c * d;
                let y = s * d;

                let color = if frame.valid[i] == 0 {
                    Color::WHITE
                } else {
                    Color::BLACK
                };

                world.sr.line(0.0, 0.0, x, y, color);
            }

            world.sr.end();

            world.sr.begin(PrimitiveType::Filled);

            for i in 0..360 {
                let (s, c) = (i as f32).to_radians().sin_cos();
                let d = frame.distance[i] as f32 / 1000.0;
                let x = c * d;
                let y = s * d;

                let color = Color::rgb(frame.strength[i] as f32 / 2000.0, 0.0, 0.0);
                world.sr.rect(x - 0.01, y - 0.01, 0.02, 0.02, color)
            }

            world.sr.end()
        }
    }
}
