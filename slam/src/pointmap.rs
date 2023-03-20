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

const MAP_POINTS_SIZE: f32 = 0.01;
#[derive(Default)]
pub struct PointMap {
    map_points: Option<Matrix2xX<f32>>,
    pose_est: Pose,
    perf_stats: PerfStats,
}

impl PointMap {
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

        if let Some(map_points) = &self.map_points {
            world.sr.begin(PrimitiveType::Filled);

            for p in map_points.column_iter() {
                world.sr.rect(
                    p.x - MAP_POINTS_SIZE / 2.0,
                    p.y - MAP_POINTS_SIZE / 2.0,
                    MAP_POINTS_SIZE,
                    MAP_POINTS_SIZE,
                    Color::BLACK,
                )
            }

            world.sr.arrow(
                self.pose_est.x,
                self.pose_est.y,
                self.pose_est.theta,
                0.1,
                Color::GREEN,
            );

            world.sr.end();
        }
    }

    pub fn estimated_pose(&self) -> Pose {
        self.pose_est
    }
}
