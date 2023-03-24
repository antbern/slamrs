#![allow(non_snake_case)]

//! Module for performing scan matching using Iterative Closest Point (ICP). Inspiration and code taken from:
//! * https://nbviewer.org/github/niosus/notebooks/blob/master/icp.ipynb
//!

use std::time::{Duration, Instant};

use kd_tree::KdMap;
use nalgebra::{Matrix1, Matrix2, Matrix2x3, Matrix2xX, Matrix3, Vector2, Vector3};
use serde::Deserialize;

/// Specifies parameters to use during the ICP computation.
#[derive(Deserialize, Clone, Copy)]
pub struct IcpParameters {
    pub correspondence_weights: CorrespondenceWeight,
    pub iterations: usize,
}

impl Default for IcpParameters {
    fn default() -> Self {
        Self {
            correspondence_weights: CorrespondenceWeight::Uniform,
            iterations: 10,
        }
    }
}

#[derive(Deserialize, Clone, Copy)]
pub enum CorrespondenceWeight {
    /// All weights are 1.0
    Uniform,

    /// Weight is a step function. Below the threshold (in error norm terms) the weight is 1.0. Above it is 0.0.
    Step { threshold: f32 },
}

impl CorrespondenceWeight {
    fn weight(&self, error: Matrix1<f32>) -> f32 {
        match self {
            CorrespondenceWeight::Uniform => 1.0,
            CorrespondenceWeight::Step { threshold } => {
                if error.norm_squared() < threshold * threshold {
                    1.0
                } else {
                    0.0
                }
            }
        }
    }
}

#[derive(Debug)]
pub struct IcpResult {
    pub transformation: Vector3<f32>,
    pub transformed_points: Matrix2xX<f32>,
    pub chi_values: Vec<f32>,
    pub execution_time: Duration,
}

fn matrix_to_kdmap(matrix: &Matrix2xX<f32>) -> KdMap<[f32; 2], usize> {
    let s: Vec<([f32; 2], usize)> = matrix
        .column_iter()
        .enumerate()
        .map(|(i, c)| ([c.x, c.y], i))
        .collect();
    KdMap::build_by_ordered_float(s)
}

fn transform_points(points: &Matrix2xX<f32>, x: Vector3<f32>) -> Matrix2xX<f32> {
    // perform rotation
    let mut p = R(x[2]) * points;

    // add translation part
    p.row_mut(0).add_scalar_mut(x[0]);
    p.row_mut(1).add_scalar_mut(x[1]);

    p
}

/// Returns the pose required to translate points to be as close to the reference points as possible.
pub fn icp_point_to_normal(
    points: &Matrix2xX<f32>,
    reference_points: &Matrix2xX<f32>,
    initial_pose: Vector3<f32>,
    params: IcpParameters,
) -> IcpResult {
    let start_time = Instant::now();

    let mut x = initial_pose;

    let q_normals = compute_normals(reference_points);
    let q_tree = matrix_to_kdmap(reference_points);

    let mut chi_values: Vec<f32> = Vec::with_capacity(params.iterations);
    for _ in 0..params.iterations {
        // transform the original points by the accumulated x
        let p_copy = transform_points(points, x);

        let correspondences = find_correspondences(&p_copy, &q_tree);

        // let s = prepare_system(x, p, q, &correspondences);
        let s = prepare_system_normals(
            x,
            points,
            reference_points,
            &correspondences,
            &q_normals,
            &params,
        );

        let dx = least_squares(s.hessian, s.gradient);
        x += dx;

        // normalize the angle
        x[2] = f32::atan2(x[2].sin(), x[2].cos());

        // log metrics
        chi_values.push(s.chi);
    }

    IcpResult {
        transformation: x,
        transformed_points: transform_points(points, x),
        chi_values,
        execution_time: start_time.elapsed(),
    }
}

/// For each point in `p`, finds the closest point in `q` using euclidean distance. Returns tuples of (p,q) indices with the correspondences
fn find_correspondences(p: &Matrix2xX<f32>, q: &KdMap<[f32; 2], usize>) -> Vec<(usize, usize)> {
    let mut c = Vec::with_capacity(p.len());

    if p.is_empty() || q.is_empty() {
        return c;
    }

    for (i_p, p_p) in p.column_iter().enumerate() {
        let nearest = q
            .nearest(&[p_p.x, p_p.y])
            .expect("Could not find nearest neighbor in Kd-tree");

        c.push((i_p, nearest.item.1));
    }
    c
}

fn dR(theta: f32) -> Matrix2<f32> {
    Matrix2::new(-theta.sin(), -theta.cos(), theta.cos(), -theta.sin())
}

fn R(theta: f32) -> Matrix2<f32> {
    Matrix2::new(theta.cos(), -theta.sin(), theta.sin(), theta.cos())
}

fn jacobian(x: Vector3<f32>, p_point: Vector2<f32>) -> Matrix2x3<f32> {
    let mut J = Matrix2x3::identity();
    let t = dR(x[2]) * p_point;
    J.set_column(2, &Vector2::new(t.x, t.y));
    J
}

fn error(x: Vector3<f32>, p_point: Vector2<f32>, q_point: Vector2<f32>) -> Vector2<f32> {
    let r = R(x[2]);
    let tr = x.xy();

    let prediction = r * p_point + tr;

    prediction - q_point
}

struct PreparedSystem {
    hessian: Matrix3<f32>,
    gradient: Vector3<f32>,
    chi: f32,
}

// Computes the hessian and the gradient of the system based on the provided correspondences and the transformation (translation + rotation) x
fn prepare_system(
    x: Vector3<f32>,
    p: &Matrix2xX<f32>,
    q: &Matrix2xX<f32>,
    c: &[(usize, usize)],
    params: &IcpParameters,
) -> PreparedSystem {
    let mut H = Matrix3::zeros();
    let mut g = Vector3::zeros();
    let mut chi = 0f32;

    for &(i, j) in c {
        let p_point = p.column(i);
        let q_point = q.column(j);

        let e = error(x, p_point.into(), q_point.into());
        let weight = params.correspondence_weights.weight(e.transpose() * e); // TODO
        let J = jacobian(x, p_point.into());

        H += weight * J.transpose() * J;
        g += weight * J.transpose() * e;

        chi += e.dot(&e);
    }

    PreparedSystem {
        hessian: H,
        gradient: g,
        chi,
    }
}

fn least_squares(hessian: Matrix3<f32>, gradient: Vector3<f32>) -> Vector3<f32> {
    lstsq::lstsq(&hessian, &(-gradient), 1e-8)
        .expect("Could not solve least squares")
        .solution
}

fn least_squares_lm(hessian: Matrix3<f32>, gradient: Vector3<f32>) -> Vector3<f32> {
    let lambda: f32 = 0.0;
    let lhs = hessian + lambda * Matrix3::identity();
    let rhs = -gradient;

    let r = lstsq::lstsq(&lhs, &rhs, 1e-8).expect("Could not solve least squares");
    let dx = r.solution;

    dx
}

fn compute_normals(points: &Matrix2xX<f32>) -> Matrix2xX<f32> {
    // let mut normals = Vec::with_capacity(points.len());

    let mut normals = Matrix2xX::zeros(points.ncols());

    // normals.push(Vector2::zeros()); // no normal for the endpoints

    // we cannot compute normals unless we have at least 3 points
    if normals.ncols() <= 2 {
        return normals;
    }

    for i in 1..(points.ncols() - 1) {
        let prev_point = points.column(i - 1);
        let next_point = points.column(i + 1);

        let diff = next_point - prev_point;

        let normal = Vector2::new(-diff.y, diff.x).normalize();

        normals.set_column(i, &normal);

        // normals.push(normal);
    }

    // normals.push(Vector2::zeros()); // no normal for the endpoints

    normals
}

fn prepare_system_normals(
    x: Vector3<f32>,
    p: &Matrix2xX<f32>,
    q: &Matrix2xX<f32>,
    c: &[(usize, usize)],
    q_normals: &Matrix2xX<f32>,
    params: &IcpParameters,
) -> PreparedSystem {
    let mut H = Matrix3::zeros();
    let mut g = Vector3::zeros();
    let mut chi = 0f32;

    for &(i, j) in c {
        let p_point = p.column(i);
        let q_point = q.column(j);
        let q_normal = q_normals.column(j);

        let e = q_normal.transpose() * error(x, p_point.into(), q_point.into());
        let weight = params.correspondence_weights.weight(e);
        let J = q_normal.transpose() * jacobian(x, p_point.into());

        H += weight * J.transpose() * J;
        g += weight * J.transpose() * e;

        chi += e.dot(&e);
    }

    PreparedSystem {
        hessian: H,
        gradient: g,
        chi,
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn it_works() {
        let p = Matrix2xX::from_columns(&[
            Vector2::new(0.0, 2.0),
            Vector2::new(0.0, 1.0),
            Vector2::new(0.0, 0.0),
            Vector2::new(0.0, -1.0),
            Vector2::new(0.0, -2.0),
        ]);

        let q = Matrix2xX::from_columns(&[
            Vector2::new(1.0, 2.0),
            Vector2::new(1.0, 1.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(1.0, -1.0),
            Vector2::new(1.0, -2.0),
        ]);

        let r = icp_point_to_normal(
            &p,
            &q,
            Vector3::zeros(),
            IcpParameters {
                correspondence_weights: CorrespondenceWeight::Uniform,
                iterations: 10,
            },
        );

        assert_relative_eq!(r.transformation, Vector3::new(1.0, 0.0, 0.0));

        // assert_eq!(result, 4);
    }
}
