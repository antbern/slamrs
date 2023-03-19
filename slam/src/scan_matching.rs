#![allow(non_snake_case)]

//! Module for performing scan matchin. Inspiration and code taken from:
//! * https://nbviewer.org/github/niosus/notebooks/blob/master/icp.ipynb
//!

use std::{sync::Arc, time::Duration};

use common::{
    robot::{Measurement, Observation, Pose},
    PerfStats,
};

use nalgebra::{Matrix2xX, Point2, Vector2};
use pubsub::Publisher;

use crate::icp;

// TODO: make this into a scan matcher for generating odometry generator
pub(crate) struct ScanMatcher {
    previous_scan: Option<Observation>,
    stats: PerfStats,
}

impl ScanMatcher {
    pub(crate) fn new() -> Self {
        Self {
            previous_scan: None,
            stats: PerfStats::new(),
        }
    }

    pub(crate) fn update(
        &mut self,
        scan: &Observation,
        pose: Pose,
        pub_pointmap: &mut Publisher<Observation>,
    ) -> Pose {
        if self.previous_scan.is_none() {
            self.previous_scan = Some(scan.clone());
            return pose;
        }

        if let Some(mut prev) = self.previous_scan.take() {
            let (p, time, transformed) = scan_match(&prev, scan, pose);

            // TODO: subsample this a bit now and then... or just add every Nth point etc
            prev.measurements.extend(transformed.iter().step_by(3));

            pub_pointmap.publish(Arc::new(prev.clone()));

            self.previous_scan = Some(prev);

            self.stats.update(time);
            p
        } else {
            pose
        }
    }

    pub fn stats(&mut self) -> &mut PerfStats {
        &mut self.stats
    }
}

fn scan_match(
    previous: &Observation,
    new: &Observation,
    start: Pose,
) -> (Pose, Duration, Vec<Measurement>) {
    // convert the observations into two arrays of points for easier manipulation

    println!(
        "Matching scan {} ({}) to {} ({})",
        previous.id,
        previous.measurements.len(),
        new.id,
        new.measurements.len()
    );

    let previous = &previous.to_matrix(Pose::default());
    let newp = &new.to_matrix(Pose::default());

    let startv = start.into();

    // match the new scan with the previous to get an estimate of the movement
    let result = icp::icp_point_to_normal(&newp, &previous, startv, 20);

    let transformed_points = result
        .transformed_points
        .column_iter()
        .map(|p| Measurement {
            angle: f32::atan2(p.y, p.x) as f64,
            distance: p.norm() as f64,
            strength: 0.0,
            valid: true,
        })
        .collect();

    (
        Pose::from(result.transformation),
        result.execution_time,
        transformed_points,
    )
}
