use crate::{app::motor_control_loop, motor::MotorDirection};
use defmt::{info, warn};
use fixed::{types::extra::U16, FixedI32};
use rp_pico::hal::fugit::ExtU32;
use rtic::Mutex;
use rtic_monotonics::{rp2040::Timer, Monotonic};

/// The fixed point type used for the PI controller
pub type F32 = FixedI32<U16>;

const CONTROL_LOOP_PERIOD_MS: u32 = 100; // ms

const MAX_VALUE: F32 = F32::const_from_int(4095);

pub async fn motor_control_loop(mut cx: motor_control_loop::Context<'_>) {
    // instantiate state variables
    let mut sat: i8 = 0;
    let mut x_integral: F32 = F32::from_num(0);

    let mut previous_motor_position = crate::encoder::get_encoder_value();

    let mut next_iteration_instant = Timer::now();
    loop {
        next_iteration_instant += CONTROL_LOOP_PERIOD_MS.millis();
        if next_iteration_instant < Timer::now() {
            warn!("Motor control loop is running behind");
            next_iteration_instant = Timer::now();
        }
        Timer::delay_until(next_iteration_instant).await;

        // do the actual control loop logic with a PI controller
        let target = cx.shared.motor_speed.lock(|motor_speed| *motor_speed);

        // estimate the current speed
        let motor_position = crate::encoder::get_encoder_value();
        let diff = motor_position - previous_motor_position;
        previous_motor_position = motor_position;
        let current_speed = diff as i32 * 1000 / CONTROL_LOOP_PERIOD_MS as i32;

        // get the current PI parameters
        let (kp, ki) = cx.shared.motor_pi_params.lock(|p| (p.kp, p.ki));
        let ki2: F32 = ki * CONTROL_LOOP_PERIOD_MS as i32 / 1000;

        // PI controller
        let error: F32 = F32::from_num(target - current_speed);

        if (sat < 0 && error < 0) || (sat > 0 && error > 0) {
            // Anti wind-up: do nothing if there is saturation and the error is in the same direction
        } else {
            x_integral = x_integral + ki2 * error;
            (x_integral, sat) = satlimit(x_integral, -MAX_VALUE, MAX_VALUE);
        }

        let x = limit(kp * error + x_integral, -MAX_VALUE, MAX_VALUE);

        info!(
            "Motor control loop: target={}, current={}, error={}, integral={}, output={}",
            target,
            current_speed,
            error.to_num::<i32>(),
            x_integral.to_num::<i32>(),
            x.to_num::<i32>()
        );
        // info!("posituion: {}", motor_position);

        // apply the motor output
        let motor_output: i32 = x.to_num();
        let (direction, speed) = if motor_output > 0 {
            (MotorDirection::Forward, motor_output as u16)
        } else if motor_output < 0 {
            (MotorDirection::Backward, (-motor_output) as u16)
        } else {
            (MotorDirection::Brake, 0)
        };
        cx.shared.motor_controller.lock(|mc| {
            cx.local.motor_right.set_direction(mc, direction).unwrap();
            cx.local.motor_right.set_speed(mc, speed).unwrap();
        });
    }
}

pub struct PiParameters {
    pub kp: F32,
    pub ki: F32,
}
impl Default for PiParameters {
    fn default() -> Self {
        Self {
            kp: F32::from_num(0.1),
            ki: F32::from_num(0.1),
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
