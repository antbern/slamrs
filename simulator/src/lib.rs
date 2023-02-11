use std::sync::Arc;

use common::{
    node::Node,
    robot::{Measurement, Observation, Pose},
};
use graphics::primitiverenderer::{Color, PrimitiveType};
use nalgebra::{Point2, Vector2};
use pubsub::Publisher;
use scene::ray::{Draw, Intersect, LineSegment, Ray, Scene};

mod scene;
mod sensor;
pub struct Simulator {
    scene: Scene,
    pub_obs: Publisher<Observation>,
    pub_pose: Publisher<Pose>,
    active: bool,
}

impl Node for Simulator {
    fn new(pubsub: &mut pubsub::PubSub) -> Self
    where
        Self: Sized,
    {
        let mut scene = Scene::new();

        scene
            .add_rect(Point2::new(-1.0, -1.0), Vector2::new(2.0, 2.0))
            .add_rect(Point2::new(-0.1, -0.4), Vector2::new(0.5, 0.1))
            .add_rect(Point2::new(-0.6, 0.4), Vector2::new(0.2, 0.5))
            .add(Box::new(LineSegment::new(-0.4, -0.4, 0.4, 0.4)));

        Self {
            scene,
            pub_obs: pubsub.publish("robot/observation"),
            pub_pose: pubsub.publish("robot/pose"),
            active: false,
        }
    }

    fn draw(&mut self, ui: &egui::Ui, world: &mut common::world::WorldObj<'_>) {
        egui::Window::new("Simulator").show(ui.ctx(), |ui| {
            ui.label("Used to simulate different LIDAR sensors and environment shapes.");

            ui.checkbox(&mut self.active, "Active");
        });

        if self.active {
            // take a reading and send it to the drawing node
            let mut meas: Vec<Measurement> = Vec::with_capacity(10);
            let origin = world.last_mouse_pos;

            for angle in 0..360 {
                let angle = (angle as f32).to_radians();
                // let angle = 0.0;
                if let Some(v) = self.scene.intersect(&Ray::from_point_angle(origin, angle)) {
                    if v < 1.0 {
                        meas.push(Measurement {
                            angle: angle as f64,
                            distance: v as f64,
                            strength: 1.0,
                            valid: true,
                        });
                    }
                }
            }

            self.pub_obs
                .publish(Arc::new(Observation { measurements: meas }));

            self.pub_pose.publish(Arc::new(Pose {
                x: origin.x,
                y: origin.y,
                theta: 0.0,
            }));
        }

        // draw the scene itself
        world.sr.begin(PrimitiveType::Line);
        self.scene.draw(world.sr, Color::BLACK);
        world.sr.end();
    }

    fn terminate(&mut self) {}
}
