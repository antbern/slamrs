use crate::{app::motor_control_loop, Mono};
use defmt::warn;
use fixed::{types::extra::U16, FixedI32};
use rp_pico::hal::fugit::ExtU32;
use rtic::Mutex;
use rtic_monotonics::Monotonic;

/// The fixed point type used for the PI controller
pub type F32 = FixedI32<U16>;

const CONTROL_LOOP_PERIOD_MS: u32 = 100; // ms

const MAX_VALUE: F32 = F32::const_from_int(4095);

pub async fn motor_control_loop(mut cx: motor_control_loop::Context<'_>) {
    let mut speed_right = SpeedEstimator::new(crate::encoder::get_encoder_value_right());
    let mut speed_left = SpeedEstimator::new(crate::encoder::get_encoder_value_left());

    let mut pi_right = PiController::new();
    let mut pi_left = PiController::new();

    let mut next_iteration_instant = Mono::now();
    loop {
        next_iteration_instant += CONTROL_LOOP_PERIOD_MS.millis();
        if next_iteration_instant < Mono::now() {
            warn!("Motor control loop is running behind");
            next_iteration_instant = Mono::now();
        }
        Mono::delay_until(next_iteration_instant).await;

        // do the actual control loop logic with a PI controller

        // get the target speed
        let target_right: F32 = F32::from_num(cx.shared.motor_speed_right.lock(|s| *s));
        let target_left: F32 = F32::from_num(cx.shared.motor_speed_left.lock(|s| *s));

        // estimate the current speed
        let current_speed_right: F32 =
            speed_right.update(crate::encoder::get_encoder_value_right());
        let current_speed_left: F32 = speed_left.update(crate::encoder::get_encoder_value_left());

        // get the current PI parameters
        let (kp, ki) = cx.shared.motor_pi_params.lock(|p| (p.kp, p.ki));
        let ki2: F32 = ki * CONTROL_LOOP_PERIOD_MS as i32 / 1000;

        // PI controller
        let out_right: F32 = pi_right.update(target_right, current_speed_right, kp, ki2);
        let out_left: F32 = pi_left.update(target_left, current_speed_left, kp, ki2);

        // apply the motor output
        let mut motor_output_right: i16 = out_right.to_num();
        if motor_output_right.abs() < 100 {
            motor_output_right = 0;
        }
        let mut motor_output_left: i16 = out_left.to_num();
        if motor_output_left.abs() < 100 {
            motor_output_left = 0;
        }
        cx.shared.motor_controller.lock(|mc| {
            cx.local
                .motor_right
                .set_speed_signed(mc, motor_output_right)
                .unwrap();
            cx.local
                .motor_left
                .set_speed_signed(mc, motor_output_left)
                .unwrap();
        });
    }
}

struct SpeedEstimator {
    last_position: i32,
}
impl SpeedEstimator {
    pub fn new(initial_position: i32) -> Self {
        Self {
            last_position: initial_position,
        }
    }

    /// Update the speed estimator with the current position, returns the estimated
    /// speed in encoder ticks per second.
    pub fn update(&mut self, position: i32) -> F32 {
        let diff = position - self.last_position;
        self.last_position = position;
        F32::from_num(diff * 1000 / CONTROL_LOOP_PERIOD_MS as i32)
    }
}

struct PiController {
    x_integral: F32,
    sat: i8,
}
impl PiController {
    fn new() -> Self {
        Self {
            x_integral: F32::from_num(0),
            sat: 0,
        }
    }
    /// Update the PI controller with the current error and return the new output.
    fn update(&mut self, target: F32, current: F32, kp: F32, ki2: F32) -> F32 {
        let error: F32 = target - current;

        if (self.sat < 0 && error < 0) || (self.sat > 0 && error > 0) {
            // Anti wind-up: do nothing if there is saturation and the error is in the same direction
        } else {
            self.x_integral = self.x_integral + ki2 * error;
            (self.x_integral, self.sat) = satlimit(self.x_integral, -MAX_VALUE, MAX_VALUE);
        }

        limit(kp * error + self.x_integral, -MAX_VALUE, MAX_VALUE)
    }
}

pub struct PiParameters {
    pub kp: F32,
    pub ki: F32,
}
impl Default for PiParameters {
    fn default() -> Self {
        Self {
            kp: F32::from_num(0.5),
            ki: F32::from_num(2.0),
        }
    }
}

fn satlimit<T: Ord>(x: T, min: T, max: T) -> (T, i8) {
    if x < min {
        return (min, -1);
    } else if x > max {
        return (max, 1);
    } else {
        return (x, 0);
    }
}
fn limit<T: Ord>(x: T, min: T, max: T) -> T {
    if x < min {
        return min;
    } else if x > max {
        return max;
    } else {
        return x;
    }
}
