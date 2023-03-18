#![allow(non_snake_case)]

//! Module for performing scan matchin. Inspiration and code taken from:
//! * https://nbviewer.org/github/niosus/notebooks/blob/master/icp.ipynb
//!

use common::robot::{Observation, Pose};
use nalgebra::{Matrix1, Matrix2, Matrix2x3, Matrix2xX, Matrix3, Point2, Vector2, Vector3};

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

fn points_to_matrix(points: &[Point2<f32>]) -> Matrix2xX<f32> {
    let vectors: Vec<Vector2<f32>> = points.iter().map(|p| p.coords).collect();
    Matrix2xX::from_columns(&vectors)
}

fn scan_match(previous: Observation, new: Observation, start: Pose) -> Pose {
    // convert the observations into two arrays of points for easier manipulation

    println!("Matching scan {} to {}", previous.id, new.id);

    let previous = points_to_matrix(&previous.to_points(Pose::default()));
    let new = points_to_matrix(&new.to_points(Pose::default()));

    // match the new scan with the previous to get an estimate of the movement
    let m = icp_least_squares(&new, &previous, None, 20);

    // the translation need to be converted from local to global space before being applied
    let s = R(start.theta) * m.xy();

    Pose {
        x: start.x + s.x,
        y: start.y + s.y,
        theta: start.theta + m[2],
    }
}

/// For each point in `p`, finds the closest point in `q` using euclidean distance. Returns tuples of (p,q) indices with the correspondences
fn find_correspondences(p: &Matrix2xX<f32>, q: &Matrix2xX<f32>) -> Vec<(usize, usize)> {
    let mut c = Vec::with_capacity(p.len());

    if p.is_empty() || q.is_empty() {
        return c;
    }

    for (i_p, p_p) in p.column_iter().enumerate() {
        let mut min_dist = f32::INFINITY;
        let mut min_idx = 0usize;

        for (i_q, p_q) in q.column_iter().enumerate() {
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

fn jacobian(x: Vector3<f32>, p_point: Vector2<f32>) -> Matrix2x3<f32> {
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

fn error(x: Vector3<f32>, p_point: Vector2<f32>, q_point: Vector2<f32>) -> Vector2<f32> {
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

fn weight(error: Matrix1<f32>) -> f32 {
    if error.norm() < 10.0 {
        1.0
    } else {
        0.0
    }
}

// Computes the hessian and the gradient of the system based on the provided correspondences and the transformation (translation + rotation) x
fn prepare_system(
    x: Vector3<f32>,
    p: &Matrix2xX<f32>,
    q: &Matrix2xX<f32>,
    c: &[(usize, usize)],
) -> PreparedSystem {
    let mut H = Matrix3::zeros();
    let mut g = Vector3::zeros();
    let mut chi = 0f32;

    for &(i, j) in c {
        let p_point = p.column(i);
        let q_point = q.column(j);

        let e = error(x, p_point.into(), q_point.into());
        let weight = weight(e.transpose() * e); // TODO
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
    p: &Matrix2xX<f32>,
    q: &Matrix2xX<f32>,
    initial_transformation: Option<Vector3<f32>>,
    iterations: usize,
) -> Vector3<f32> {
    let mut x = initial_transformation.unwrap_or(Vector3::zeros());

    let q_normals = compute_normals(q);

    let mut chi_values: Vec<f32> = Vec::with_capacity(iterations);
    for _ in 0..iterations {
        // transform the original points by the accumulated x
        let mut p_copy = R(x[2]) * p;
        p_copy.row_mut(0).add_scalar_mut(x[0]);
        p_copy.row_mut(1).add_scalar_mut(x[1]);

        let correspondences = find_correspondences(&p_copy, q);

        // let s = prepare_system(x, p, q, &correspondences);
        let s = prepare_system_normals(x, p, q, &correspondences, &q_normals);

        let r =
            lstsq::lstsq(&s.hessian, &(-s.gradient), 1e-8).expect("Could not solve least squares");
        let dx = r.solution;
        x += dx;

        // normalize the angle
        x[2] = f32::atan2(x[2].sin(), x[2].cos());

        // log metrics
        chi_values.push(s.chi);
    }

    // dbg!(&chi_values.last());
    // dbg!(&x);
    // dbg!(x[2].to_degrees());
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

fn compute_normals(points: &Matrix2xX<f32>) -> Matrix2xX<f32> {
    // let mut normals = Vec::with_capacity(points.len());

    let mut normals = Matrix2xX::zeros(points.ncols());

    // normals.push(Vector2::zeros()); // no normal for the endpoints

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

// def compute_normals(points, step=1):
//     normals = [np.array([[0, 0]])]
//     normals_at_points = []
//     for i in range(step, points.shape[1] - step):
//         prev_point = points[:, i - step]
//         next_point = points[:, i + step]
//         curr_point = points[:, i]
//         dx = next_point[0] - prev_point[0]
//         dy = next_point[1] - prev_point[1]
//         normal = np.array([[0, 0],[-dy, dx]])
//         normal = normal / np.linalg.norm(normal)
//         normals.append(normal[[1], :])
//         normals_at_points.append(normal + curr_point)
//     normals.append(np.array([[0, 0]]))
//     return normals, normals_at_points

fn prepare_system_normals(
    x: Vector3<f32>,
    p: &Matrix2xX<f32>,
    q: &Matrix2xX<f32>,
    c: &[(usize, usize)],
    q_normals: &Matrix2xX<f32>,
) -> PreparedSystem {
    let mut H = Matrix3::zeros();
    let mut g = Vector3::zeros();
    let mut chi = 0f32;

    for &(i, j) in c {
        let p_point = p.column(i);
        let q_point = q.column(j);
        let q_normal = q_normals.column(j);

        let e = q_normal.transpose() * error(x, p_point.into(), q_point.into());
        let weight = weight(e); // TODO
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
    use approx::relative_eq;

    use super::*;

    #[test]
    fn it_works() {
        let p = vec![
            Point2::new(0.0, 2.0),
            Point2::new(0.0, 1.0),
            Point2::new(0.0, 0.0),
            Point2::new(0.0, -1.0),
            Point2::new(0.0, -2.0),
        ];

        let q = vec![
            Point2::new(1.0, 2.0),
            Point2::new(1.0, 1.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, -1.0),
            Point2::new(1.0, -2.0),
        ];

        let r = icp_least_squares(&points_to_matrix(&p), &points_to_matrix(&q), None, 10);
        // relative_eq!(r.x, 1.0);
        // relative_eq!(r.y, 0.0);
        // relative_eq!(r.z, 0.0);

        relative_eq!(r, Vector3::new(1.0, 0.0, 0.0));

        // assert_eq!(result, 4);
    }
}
