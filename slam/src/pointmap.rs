use std::{sync::Arc, time::Instant};

use common::{
    node::{Node, NodeConfig},
    robot::{Observation, Pose},
    PerfStats,
};
use egui::{Label, RichText, Sense};

use nalgebra::Matrix2xX;
use pubsub::{Publisher, Subscription};
use serde::Deserialize;

use crate::icp::{self, IcpParameters};

pub struct PointMap(pub Matrix2xX<f32>);
#[derive(Default)]
pub struct IcpPointMapper {
    map_points: Option<Matrix2xX<f32>>,
    pose_est: Pose,
    perf_stats: PerfStats,
    icp_parameters: IcpParameters,
}

impl IcpPointMapper {
    pub fn new(icp_parameters: IcpParameters) -> Self {
        Self {
            icp_parameters,
            ..Self::default()
        }
    }

    pub fn update(&mut self, observation: &Observation) {
        let start = Instant::now();

        let newp = observation.to_matrix(Pose::default());

        if self.map_points.is_none() {
            self.map_points = Some(newp);
            return;
        }

        if let Some(mut map_points) = self.map_points.take() {
            // match the new scan with the previous to get an estimate of the movement
            let result = icp::icp_point_to_normal(
                &newp,
                &map_points,
                self.pose_est.into(),
                self.icp_parameters,
            );

            self.pose_est = Pose::from(result.transformation);

            let new_points = result.transformed_points;

            // make space for all new columns (NOTE: this will probably reallocate!)
            let n_map_points = map_points.ncols();
            map_points = map_points.insert_columns(n_map_points, new_points.ncols(), 0.0);

            // insert their values
            map_points
                .columns_mut(n_map_points, new_points.ncols())
                .copy_from(&new_points);

            // TODO implement some kind of sub-sampling here (otherwise the points will grow to be too many!)

            println!(
                "Map updated from {} -> {} points",
                n_map_points,
                map_points.ncols()
            );

            self.map_points = Some(map_points)
        }

        self.perf_stats.update(start.elapsed());
    }

    pub fn estimated_pose(&self) -> Pose {
        self.pose_est
    }

    pub fn pointmap(&self) -> PointMap {
        if let Some(m) = &self.map_points {
            PointMap(m.to_owned())
        } else {
            PointMap(Matrix2xX::zeros(0))
        }
    }

    pub fn stats(&mut self) -> &mut PerfStats {
        &mut self.perf_stats
    }
}

pub struct IcpPointMapNode {
    sub_obs: Subscription<Observation>,
    pub_pose: Publisher<Pose>,
    pub_point_map: Publisher<PointMap>,
    point_map: IcpPointMapper,
}

#[derive(Clone, Deserialize)]
pub struct IcpPointMapNodeConfig {
    topic_pose: String,
    topic_observation: String,
    topic_pointmap: String,
    icp: IcpParameters,
}

impl NodeConfig for IcpPointMapNodeConfig {
    fn instantiate(&self, pubsub: &mut pubsub::PubSub) -> Box<dyn Node> {
        Box::new(IcpPointMapNode {
            sub_obs: pubsub.subscribe(&self.topic_observation),
            pub_pose: pubsub.publish(&self.topic_pose),
            pub_point_map: pubsub.publish(&self.topic_pointmap),
            point_map: IcpPointMapper::new(self.icp),
        })
    }
}

impl Node for IcpPointMapNode {
    fn update(&mut self) {
        // TODO: move all processing to separate thread later, do it here for now (but only one observation per frame)
        if let Some(o) = self.sub_obs.try_recv() {
            self.point_map.update(&o);

            self.pub_pose
                .publish(Arc::new(self.point_map.estimated_pose()));

            self.pub_point_map
                .publish(Arc::new(self.point_map.pointmap()));
        }
    }

    fn draw(&mut self, ui: &egui::Ui, _world: &mut common::world::WorldObj<'_>) {
        egui::Window::new("IcpPointMapNode").show(ui.ctx(), |ui| {
            ui.label("Point Map: ");
            ui.horizontal(|ui| {
                if ui
                    .add(
                        Label::new(RichText::new(self.point_map.stats().to_string()).monospace())
                            .sense(Sense::click()),
                    )
                    .clicked()
                {
                    self.point_map.stats().reset();
                }
            });
        });
    }
}
