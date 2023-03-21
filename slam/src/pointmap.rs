use std::time::Instant;

use common::{
    robot::{Observation, Pose},
    world::WorldObj,
    PerfStats,
};
use egui::{Label, RichText, Sense};
use graphics::primitiverenderer::{Color, PrimitiveType};
use nalgebra::Matrix2xX;

use crate::icp;

pub struct PointMap(pub Matrix2xX<f32>);
#[derive(Default)]
pub struct IcpPointMapper {
    map_points: Option<Matrix2xX<f32>>,
    pose_est: Pose,
    perf_stats: PerfStats,
}
// TODO: make this into its own node, that takes in observations and outputs pose estimations and point cloud map

impl IcpPointMapper {
    pub fn new() -> Self {
        Self::default()
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
            let result = icp::icp_point_to_normal(&newp, &map_points, self.pose_est.into(), 20);

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

    pub fn draw(&mut self, ui: &mut egui::Ui, world: &mut WorldObj<'_>) {
        ui.label("Point Map: ");
        ui.horizontal(|ui| {
            if ui
                .add(
                    Label::new(RichText::new(self.perf_stats.to_string()).monospace())
                        .sense(Sense::click()),
                )
                .clicked()
            {
                self.perf_stats.reset();
            }
        });
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
}
