use std::sync::Arc;

use common::{
    node::Node,
    robot::{Observation, Pose},
    world::WorldObj,
};
use pubsub::{PubSub, Subscription};

use graphics::primitiverenderer::{Color, PrimitiveType};

pub struct FrameVizualizer {
    last_frame: Option<Arc<Observation>>,
    last_pose: Pose,
    sub: Subscription<Observation>,
    sub_pose: Subscription<Pose>,
}

impl Node for FrameVizualizer {
    fn new(pubsub: &mut PubSub) -> Self
    where
        Self: Sized,
    {
        Self {
            last_frame: None,
            last_pose: Default::default(),
            // sub: pubsub.subscribe("scan"),
            sub: pubsub.subscribe("robot/observation"),
            sub_pose: pubsub.subscribe("robot/pose"),
        }
    }

    fn draw(&mut self, _ui: &egui::Ui, world: &mut WorldObj<'_>) {
        if let Some(v) = self.sub.try_recv() {
            self.last_frame = Some(v);
        }

        if let Some(v) = self.sub_pose.try_recv() {
            self.last_pose = *v; // Copy the value into local storage
        }

        if let Some(frame) = &self.last_frame {
            // draw the reading into the world

            let (ox, oy) = self.last_pose.into();

            world.sr.begin(PrimitiveType::Line);

            for m in frame.measurements.iter() {
                let (s, c) = (m.angle as f32).sin_cos();
                let d = m.distance as f32;
                let x = c * d;
                let y = s * d;

                // let color = if m.valid { Color::WHITE } else { Color::BLACK };
                let color = Color::BLACK;

                world.sr.line(ox, oy, ox + x, oy + y, color);
            }

            world.sr.end();

            world.sr.begin(PrimitiveType::Filled);

            for m in frame.measurements.iter() {
                let (s, c) = (m.angle as f32).sin_cos();
                let d = m.distance as f32;
                let x = c * d;
                let y = s * d;

                let color = Color::rgb(m.strength as f32 / 1.0, 0.0, 0.0);
                world
                    .sr
                    .rect(ox + x - 0.01, oy + y - 0.01, 0.02, 0.02, color)
            }

            world.sr.end()
        }
    }
}
