use std::sync::Arc;

use common::{
    node::{Node, NodeConfig},
    robot::{Observation, Pose},
};
use egui::{Label, RichText, Sense};
use pointmap::IcpPointMapper;
use pubsub::{Publisher, Subscription};
use scan_matching::ScanMatcher;
use serde::Deserialize;

mod icp;
mod pointmap;
mod scan_matching;

pub use pointmap::PointMap;
pub struct SlamNode {
    sub_obs: Subscription<Observation>,
    pub_pose: Publisher<Pose>,
    pub_point_map: Publisher<PointMap>,
    matcher: ScanMatcher,
    point_map: IcpPointMapper,
}

#[derive(Deserialize)]
pub struct SlamNodeConfig {
    topic_pose: String,
    topic_observation: String,
    topic_pointmap: String,
}

impl NodeConfig for SlamNodeConfig {
    fn instantiate(&self, pubsub: &mut pubsub::PubSub) -> Box<dyn Node> {
        Box::new(SlamNode {
            sub_obs: pubsub.subscribe(&self.topic_observation),
            pub_pose: pubsub.publish(&self.topic_pose),
            pub_point_map: pubsub.publish(&self.topic_pointmap),
            matcher: ScanMatcher::new(),
            point_map: IcpPointMapper::new(),
        })
    }
}

impl Node for SlamNode {
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

            self.pub_point_map
                .publish(Arc::new(self.point_map.pointmap()));
        }
    }
}
