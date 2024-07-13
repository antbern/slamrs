use common::robot::{LandmarkObservations, Odometry, Pose};

use nalgebra as na;
use serde::Deserialize;

#[derive(Clone, Debug, Deserialize)]
pub struct EKFLandmarkSlamConfig {}

#[derive(Debug)]
pub struct EKFLandmarkSlam {
    // pose_mean: na::Vector3<f32>,
    // pose_covariance: na::Matrix3<f32>,
    state_mean: na::DVector<f32>,
    state_covariance: na::DMatrix<f32>,
    num_landmarks: usize,
    landmark_seen: Vec<bool>,
    // landmarks: Vec<Landmark>,
}

impl EKFLandmarkSlam {
    pub fn new(_config: &EKFLandmarkSlamConfig) -> Self {
        let num_landmarks = 10;

        // mean starts out as zero for both pose and pandmark positions
        let state_mean = na::DVector::zeros(3 + 2 * num_landmarks);

        // "infinite" covariance for landmarks
        let mut state_covariance =
            na::DMatrix::identity(3 + 2 * num_landmarks, 3 + 2 * num_landmarks) * 1000.0;

        // covariance for the robot pose is zero
        state_covariance[(0, 0)] = 0.0;
        state_covariance[(1, 1)] = 0.0;
        state_covariance[(2, 2)] = 0.0;

        // TODO
        Self {
            state_mean,
            state_covariance,
            num_landmarks,
            landmark_seen: vec![false; num_landmarks],
            // pose_mean: na::Vector3::zeros(),
            // pose_covariance: na::Matrix3::zeros(),
            // landmarks: vec![
            //     Landmark {
            //         mean: na::Vector2::zeros(),
            //         covariance: na::Matrix2::identity() * 1000.0, // "infinite" covariance
            //     };
            //     20
            // ], // ,
            //     Landmark {
            //         mean: na::Vector2::new(1.0, 0.0),
            //         covariance: na::Matrix2::identity() * 0.1,
            //     },
            // ],
        }
    }

    // fn landmark_mean(&self, landmark_index: usize) -> na::VectorView2<f32> {
    //     assert!(landmark_index < self.num_landmarks);
    //     self.state_mean.fixed_rows::<2>(3 + 2 * landmark_index)
    // }

    pub fn update(&mut self, observation: &LandmarkObservations, odometry: Odometry) {
        // todo

        /////// Update the robot location using the motion model

        let omega_dt = (odometry.distance_right - odometry.distance_left) / odometry.wheel_distance;
        let v_dt = (odometry.distance_left + odometry.distance_right) / 2.0;

        // Velocity-based motion model from : https://youtu.be/5Pu558YtjYM?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_&t=1849
        let (gxytheta, gx_jacobian) = if omega_dt != 0.0 {
            let v_over_omega = v_dt / omega_dt; // the delta t cancels out
            let theta = self.state_mean[2];
            let gxytheta = na::Vector3::new(
                -v_over_omega * theta.sin() + v_over_omega * (theta + omega_dt).sin(),
                v_over_omega * theta.cos() - v_over_omega * (theta + omega_dt).cos(),
                omega_dt,
            );
            // conpute the Jacobian of the motion model (only affects the top 3x3 block of the covariance matrix)
            let gx = na::Matrix3::new(
                1.0,
                0.0,
                -v_over_omega * theta.cos() + v_over_omega * (theta + omega_dt).cos(),
                0.0,
                1.0,
                -v_over_omega * theta.sin() + v_over_omega * (theta + omega_dt).sin(),
                0.0,
                0.0,
                1.0,
            );
            (gxytheta, gx)
        } else {
            // no rotation, just straight line motion
            let theta = self.state_mean[2];
            let gxytheta = na::Vector3::new(v_dt * theta.cos(), v_dt * theta.sin(), 0.0);
            let gx = na::Matrix3::new(
                1.0,
                0.0,
                -v_dt * theta.sin(),
                0.0,
                1.0,
                v_dt * theta.cos(),
                0.0,
                0.0,
                1.0,
            );
            (gxytheta, gx)
        };

        // apply the motion model to get the new mean, wrap angle
        let mut mu_bar = self.state_mean.clone();
        mu_bar[0] += gxytheta[0];
        mu_bar[1] += gxytheta[1];
        mu_bar[2] = na::wrap(
            mu_bar[2] + gxytheta[2],
            -std::f32::consts::PI,
            std::f32::consts::PI,
        );

        let mut g: na::DMatrix<f32> =
            na::DMatrix::identity(3 + 2 * self.num_landmarks, 3 + 2 * self.num_landmarks);
        g.fixed_view_mut::<3, 3>(0, 0).copy_from(&gx_jacobian);
        let g = g;

        // r is the motion noise (variance) in the motion model: x (m), y (m), theta (radians)
        let sigma = na::Vector3::new(0.02, 0.02, 5.0_f32.to_radians());
        let r = na::Matrix3::from_diagonal(&sigma.component_mul(&sigma));

        // compute sigma bar (todo update blocks individually for better computational complexity, see video at 37:00)
        let mut sigma_bar = &g * &self.state_covariance * g.transpose();
        let mut a = sigma_bar.fixed_view_mut::<3, 3>(0, 0);
        a += r;

        //
        ///// Do the update / correction step

        for l in observation.landmarks.iter() {
            // data association
            let Some(landmark_idx) = l.association else {
                continue;
            };

            if !self.landmark_seen[landmark_idx] {
                self.landmark_seen[landmark_idx] = true;
                log::info!("landmark seen for first time: {}", landmark_idx);

                // initialize as if the landmark is exactly what we would expect
                mu_bar[3 + 2 * landmark_idx] = mu_bar[0] + l.distance * (mu_bar[2] + l.angle).cos();
                mu_bar[3 + 2 * landmark_idx + 1] =
                    mu_bar[1] + l.distance * (mu_bar[2] + l.angle).sin();
            }

            // predict the observed landmark location

            let dx = mu_bar[3 + 2 * landmark_idx] - mu_bar[0];
            let dy = mu_bar[3 + 2 * landmark_idx + 1] - mu_bar[1];
            let q = dx * dx + dy * dy;
            let sqrt_q = q.sqrt();

            // compute the expected observation
            let z_bar = na::Vector2::new(sqrt_q, dy.atan2(dx) - mu_bar[2]);
            let z = na::Vector2::new(l.distance, l.angle);

            // compute the jacobian of the expected observation wrt the state and the location of
            // the landmark

            let h_jacobian_low = na::Matrix2x5::new(
                -sqrt_q * dx,
                -sqrt_q * dy,
                0.0,
                sqrt_q * dx,
                sqrt_q * dy,
                dy,
                -dx,
                -q,
                -dy,
                dx,
            );

            // transformation matrix to get to the full state
            let fxj = {
                let mut fxj = na::DMatrix::zeros(5, 3 + 2 * self.num_landmarks);
                fxj[(0, 0)] = 1.0;
                fxj[(1, 1)] = 1.0;
                fxj[(2, 2)] = 1.0;
                fxj[(3, 3 + 2 * landmark_idx)] = 1.0;
                fxj[(4, 3 + 2 * landmark_idx + 1)] = 1.0;
                fxj
            };

            let h_jacobian = h_jacobian_low * fxj;

            // variance in the observation: distance (meters) and angle (radians)
            let sigma = na::Matrix2::from_diagonal(&na::Vector2::new(0.03, 3.0_f32.to_radians()));
            let q = na::Matrix2::from(sigma.component_mul(&sigma));

            // compute the kalman gain for this observation
            let k = &sigma_bar
                * h_jacobian.transpose()
                * (&h_jacobian * &sigma_bar * h_jacobian.transpose() + q)
                    .try_inverse()
                    .unwrap();

            // compute the diff and normalize the angle
            let mut diff = z - z_bar;
            diff[1] = na::wrap(diff[1], -std::f32::consts::PI, std::f32::consts::PI);

            mu_bar += &k * diff;

            // wrap angle
            mu_bar[2] = na::wrap(mu_bar[2], -std::f32::consts::PI, std::f32::consts::PI);

            // update the covariance
            sigma_bar =
                (na::DMatrix::identity(3 + 2 * self.num_landmarks, 3 + 2 * self.num_landmarks)
                    - &k * &h_jacobian)
                    * &sigma_bar;
        }

        self.state_mean = mu_bar;
        self.state_covariance = sigma_bar;
    }

    pub fn estimated_pose(&self) -> Pose {
        Pose {
            x: self.state_mean[0],
            y: self.state_mean[1],
            theta: self.state_mean[2],
        }
    }

    pub fn estimated_landmarks(&self) -> Vec<Landmark> {
        let mut l = self
            .landmark_seen
            .iter()
            .enumerate()
            .filter(|(_, &seen)| seen)
            .map(|(i, _)| {
                let x = self.state_mean[3 + 2 * i];
                let y = self.state_mean[3 + 2 * i + 1];
                let mean = na::Vector2::new(x, y);
                let covariance = self
                    .state_covariance
                    .fixed_view::<2, 2>(3 + 2 * i, 3 + 2 * i);
                Landmark {
                    mean,
                    covariance: covariance.into(),
                }
            })
            .collect::<Vec<Landmark>>();
        // TODO: remove when we have way to send an estimated pose with uncertainty
        l.push(Landmark {
            mean: self.state_mean.fixed_rows::<2>(0).into(),
            covariance: self.state_covariance.fixed_view::<2, 2>(0, 0).into(),
        });
        l
    }
}

#[derive(Clone, Debug)]
pub struct Landmark {
    pub mean: na::Vector2<f32>,
    pub covariance: na::Matrix2<f32>,
}
