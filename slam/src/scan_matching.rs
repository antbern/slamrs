#![allow(non_snake_case)]

//! Module for performing scan matchin. Inspiration and code taken from:
//! * https://nbviewer.org/github/niosus/notebooks/blob/master/icp.ipynb
//!

use common::robot::{Observation, Pose};
use nalgebra::{Matrix2, Matrix2x3, Matrix3, Point2, Vector2, Vector3};

pub(crate) struct ScanMatcher {
    previous_scan: Option<Observation>,
}

impl ScanMatcher {
    pub(crate) fn new() -> Self {
        Self {
            previous_scan: None,
        }
    }

    pub(crate) fn update(&mut self, scan: &Observation, pose: Pose) -> Pose {
        if let Some(prev) = self.previous_scan.replace(scan.clone()) {
            scan_match(prev, scan.clone(), pose)
        } else {
            pose
        }
    }
}

fn scan_match(a: Observation, b: Observation, start: Pose) -> Pose {
    // convert the observations into two arrays of points for easier manipulation
    // let pa = a.to_points(start);
    // let pb = b.to_points(start);

    let pa = a.to_points(Pose::default());
    let pb = b.to_points(Pose::default());

    // dbg!(&pa[0..2]);
    // dbg!(&pb[0..2]);

    // let c = find_correspondences(&pa, &pb);

    // perform scan matching algorithm
    // start

    let r = icp_least_squares(&pb, &pa, None, 10);

    Pose {
        x: start.x + r[0],
        y: start.y + r[1],
        theta: start.theta + r[2],
    }
}

/// For each point in `p`, finds the closest point in `q` using euclidean distance. Returns tuples of (p,q) indices with the correspondences
fn find_correspondences(p: &[Point2<f32>], q: &[Point2<f32>]) -> Vec<(usize, usize)> {
    let mut c = Vec::with_capacity(p.len());

    for (i_p, p_p) in p.iter().enumerate() {
        let mut min_dist = f32::INFINITY;
        let mut min_idx = 0usize;

        for (i_q, p_q) in q.iter().enumerate() {
            let d2 = (p_q - p_p).norm_squared();
            if d2 <= min_dist {
                min_dist = d2;
                min_idx = i_q;
            }
        }

        c.push((i_p, min_idx));
    }
    c
}

fn dR(theta: f32) -> Matrix2<f32> {
    Matrix2::new(-theta.sin(), -theta.cos(), theta.cos(), -theta.sin())
}

fn R(theta: f32) -> Matrix2<f32> {
    Matrix2::new(theta.cos(), -theta.sin(), theta.sin(), theta.cos())
}

fn jacobian(x: Vector3<f32>, p_point: Point2<f32>) -> Matrix2x3<f32> {
    let mut J = Matrix2x3::identity();
    let t = dR(x[2]) * p_point;
    J.set_column(2, &Vector2::new(t.x, t.y));
    J
}
// def jacobian(x, p_point):
//     theta = x[2]
//     J = np.zeros((2, 3))
//     J[0:2, 0:2] = np.identity(2)
//     J[0:2, [2]] = dR(0).dot(p_point)
//     return J

fn error(x: Vector3<f32>, p_point: Point2<f32>, q_point: Point2<f32>) -> Vector2<f32> {
    let r = R(x[2]);
    let tr = x.xy();

    let prediction = r * p_point + tr;

    prediction - q_point
}
// def error(x, p_point, q_point):
//     rotation = R(x[2])
//     translation = x[0:2]
//     prediction = rotation.dot(p_point) + translation
//     return prediction - q_point

struct PreparedSystem {
    hessian: Matrix3<f32>,
    gradient: Vector3<f32>,
    chi: f32,
}

// Computes the hessian and the gradient of the system based on the provided correspondences and the transformation (translation + rotation) x
fn prepare_system(
    x: Vector3<f32>,
    p: &[Point2<f32>],
    q: &[Point2<f32>],
    c: &[(usize, usize)],
) -> PreparedSystem {
    let mut H = Matrix3::zeros();
    let mut g = Vector3::zeros();
    let mut chi = 0f32;

    for &(i, j) in c {
        let p_point = p[i];
        let q_point = q[j];

        let e = error(x, p_point, q_point);
        let weight = 1.0; // TODO
        let J = jacobian(x, p_point);

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

// def prepare_system(x, P, Q, correspondences, kernel=lambda distance: 1.0):
//     H = np.zeros((3, 3))
//     g = np.zeros((3, 1))
//     chi = 0
//     for i, j in correspondences:
//         p_point = P[:, [i]]
//         q_point = Q[:, [j]]
//         e = error(x, p_point, q_point)
//         weight = kernel(e) # Please ignore this weight until you reach the end of the notebook.
//         J = jacobian(x, p_point)
//         H += weight * J.T.dot(J)
//         g += weight * J.T.dot(e)
//         chi += e.T * e
//     return H, g, chi

/// Returns the pose required to translate points p to be as close to points q as possible.
fn icp_least_squares(
    p: &[Point2<f32>],
    q: &[Point2<f32>],
    initial_transformation: Option<Vector3<f32>>,
    iterations: usize,
) -> Vector3<f32> {
    let mut x = initial_transformation.unwrap_or(Vector3::zeros());
    let mut chi_values: Vec<f32> = Vec::new();

    let mut p_copy = p.to_owned(); // copy to modify along the way

    for _ in 0..iterations {
        let correspondences = find_correspondences(&p_copy, q);
        // dbg!(correspondences.len());

        let s = prepare_system(x, p, q, &correspondences);

        let r =
            lstsq::lstsq(&s.hessian, &(-s.gradient), 1e-8).expect("Could not solve least squares");
        let dx = r.solution;
        x += dx;

        // normalize the angle
        x[2] = f32::atan2(x[2].sin(), x[2].cos());

        // transform the original points by the accumulated x
        let rot = R(x[2]);
        let t = x.xy();
        p_copy = p.iter().map(|o| rot * o + t).collect();

        // log metrics
        chi_values.push(s.chi);
    }

    dbg!(&chi_values);
    dbg!(&x);
    dbg!(x[2].to_degrees());
    x
}

// def icp_least_squares(P, Q, iterations=30, kernel=lambda distance: 1.0):
//     x = np.zeros((3, 1))
//     chi_values = []
//     x_values = [x.copy()]  # Initial value for transformation.
//     P_values = [P.copy()]
//     P_copy = P.copy()
//     corresp_values = []
//     for i in range(iterations):
//         rot = R(x[2])
//         t = x[0:2]
//         correspondences = get_correspondence_indices(P_copy, Q)
//         corresp_values.append(correspondences)
//         H, g, chi = prepare_system(x, P, Q, correspondences, kernel)
//         dx = np.linalg.lstsq(H, -g, rcond=None)[0]
//         x += dx
//         x[2] = atan2(sin(x[2]), cos(x[2])) # normalize angle
//         chi_values.append(chi.item(0))
//         x_values.append(x.copy())
//         rot = R(x[2])
//         t = x[0:2]
//         P_copy = rot.dot(P.copy()) + t
//         P_values.append(P_copy)
//     corresp_values.append(corresp_values[-1])
//     return P_values, chi_values, corresp_values

pub fn add(left: usize, right: usize) -> usize {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
