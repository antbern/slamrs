use std::sync::Arc;

use common::{
    node::Node,
    robot::{Observation, Pose},
};
use egui::{Label, RichText, Sense};
use pointmap::PointMap;
use pubsub::{Publisher, Subscription};
use scan_matching::ScanMatcher;

mod icp;
mod pointmap;
mod scan_matching;

pub struct SlamNode {
    sub_obs: Subscription<Observation>,
    pub_pose: Publisher<Pose>,
    // pub_point_map: Publisher<Observation>,
    pose_est: Pose,
    matcher: ScanMatcher,
    point_map: PointMap,
}

impl Node for SlamNode {
    fn new(pubsub: &mut pubsub::PubSub) -> Self
    where
        Self: Sized,
    {
        SlamNode {
            sub_obs: pubsub.subscribe("robot/observation"),
            pub_pose: pubsub.publish("robot/pose"),
            // pub_point_map: pubsub.publish("pointmap"),
            pose_est: Pose::default(),
            matcher: ScanMatcher::new(),
            point_map: PointMap::new(),
        }
    }

    fn draw(&mut self, ui: &egui::Ui, world: &mut common::world::WorldObj<'_>) {
        egui::Window::new("Slam").show(ui.ctx(), |ui| {
            ui.label("Slam Stuff");

            ui.horizontal(|ui| {
                ui.label("SM: ");
                if ui
                    .add(
                        Label::new(RichText::new(self.matcher.stats().to_string()).monospace())
                            .sense(Sense::click()),
                    )
                    .clicked()
                {
                    self.matcher.stats().reset();
                }
            });

            self.point_map.draw(ui, world);
        });

        // TODO: move all processing to separate thread later, do it here for now (but only one observation per frame)
        if let Some(o) = self.sub_obs.try_recv() {
            self.point_map.update(&o);

            self.pub_pose
                .publish(Arc::new(self.point_map.estimated_pose()));
        }
    }
}
