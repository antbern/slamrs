use std::sync::Arc;

use common::{
    node::Node,
    robot::{Observation, Pose},
};
use pubsub::{Publisher, Subscription};
use scan_matching::ScanMatcher;

mod scan_matching;

pub struct SlamNode {
    sub_obs: Subscription<Observation>,
    pub_pose: Publisher<Pose>,
    pose_est: Pose,
    matcher: ScanMatcher,
}

impl Node for SlamNode {
    fn new(pubsub: &mut pubsub::PubSub) -> Self
    where
        Self: Sized,
    {
        SlamNode {
            sub_obs: pubsub.subscribe("robot/observation"),
            pub_pose: pubsub.publish("robot/pose"),
            pose_est: Pose::default(),
            matcher: ScanMatcher::new(),
        }
    }

    fn draw(&mut self, ui: &egui::Ui, _world: &mut common::world::WorldObj<'_>) {
        egui::Window::new("Slam").show(ui.ctx(), |ui| {
            ui.label("Slam Stuff");
        });

        // TODO: move all processing to separate thread later, do it here for now (but only one observation per frame)
        if let Some(o) = self.sub_obs.try_recv() {
            let newpose = self.matcher.update(&o, self.pose_est);

            self.pose_est = newpose;
            // self.pose_est.x += 0.1 * 1.0 / 60.0;

            self.pub_pose.publish(Arc::new(self.pose_est));
        }
    }
}
