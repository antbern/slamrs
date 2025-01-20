use crate::{
    app::{neato_motor_control, uart0_neato},
    message::{RobotMessageInternal, ScanFrameInternal},
    motor::MotorDirection,
    Mono,
};
use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use defmt::info;
use rp_pico::hal::fugit::ExtU64;
use rtic::Mutex;
use rtic_monotonics::Monotonic;

/// Atomic variables to control the on/off state of the motor and the last measured RPM
pub static MOTOR_ON: AtomicBool = AtomicBool::new(false);
pub static LAST_RPM: AtomicU16 = AtomicU16::new(0);

pub async fn neato_motor_control(mut cx: neato_motor_control::Context<'_>) {
    // initialize the motor
    cx.shared.motor_controller.lock(|mc| {
        cx.local
            .neato_motor
            .set_direction(mc, MotorDirection::Forward)
            .unwrap();
    });

    let mut pwm_current: i32 = 0;
    loop {
        Mono::delay(200.millis()).await;

        let rpm_target = if MOTOR_ON.load(Ordering::Relaxed) {
            300
        } else {
            0
        };

        let last_rpm = LAST_RPM.load(Ordering::Relaxed);

        let error = rpm_target as i16 - last_rpm as i16;

        pwm_current += error as i32 / 4;

        // cap the allowed PWM range to 0-90% (in units of 100)
        // pwm_current = pwm_current.clamp(0, 90 * 100);
        pwm_current = pwm_current.clamp(0, 4095);

        // scale the PWM value from 0-100 *100 to the range 0-4095
        // let mut pwm = (pwm_current as u32 * 4095 / (100 * 100)) as u16;
        let mut pwm = pwm_current as u16;

        if rpm_target == 0 {
            pwm = 0;
        }

        cx.shared.motor_controller.lock(|mc| {
            cx.local.neato_motor.set_speed(mc, pwm).unwrap();
        });

        // info!(
        //     "Control, {} rpm, error={}. New PWM = {}",
        //     last_rpm, error, pwm
        // );
    }
}

pub fn uart0_neato(cx: uart0_neato::Context<'_>) {
    cx.local.parser.consume(cx.local.uart0_rx_neato, |data| {
        // some exponential smoothing on the raw (*64) RPM value
        let rpm = data.parse_rpm_raw();
        *cx.local.rpm_accumulator += rpm as i32 - (*cx.local.rpm_average);
        *cx.local.rpm_average = *cx.local.rpm_accumulator >> 2;
        let rpm = (*cx.local.rpm_average / 64) as u16;
        LAST_RPM.store(rpm, core::sync::atomic::Ordering::Relaxed);

        info!("neato rpm: {:?}", rpm);
        // TODO: should we add a data validation check?
        if rpm < 250 && rpm > 350 {
            // THIS WILL NEVER BE TRUE LOL
            return;
        }

        *cx.local.downsample_counter += 1;
        if *cx.local.downsample_counter > cx.shared.neato_downsampling.load(Ordering::Relaxed) {
            *cx.local.downsample_counter = 0;
        } else {
            return;
        }

        // get the odometry change since the last scan
        let odometry_right = crate::encoder::get_encoder_value_right();
        let odometry_left = crate::encoder::get_encoder_value_left();
        let odometry_diff_right = odometry_right - *cx.local.last_odometry_right;
        let odometry_diff_left = odometry_left - *cx.local.last_odometry_left;
        *cx.local.last_odometry_right = odometry_right;
        *cx.local.last_odometry_left = odometry_left;

        // convert the odometry to meters
        let odometry_right = odometry_diff_right as f32 / crate::app::MOTOR_STEPS_PER_METER;
        let odometry_left = odometry_diff_left as f32 / crate::app::MOTOR_STEPS_PER_METER;

        // need to copy the data to a new array because the data is borrowed from the parser
        let mut buffer = crate::app::BUFFER_POOL
            .acquire()
            .expect("buffer pool should not be empty");

        buffer.copy_from_slice(data.data);
        let buffer = buffer.shared();

        // send frame to the host
        crate::util::channel_send(
            cx.local.robot_message_sender_neato,
            RobotMessageInternal::ScanFrame(ScanFrameInternal {
                scan_data: buffer.clone(),
                odometry: [odometry_left, odometry_right],
                rpm,
            }),
            "uart0_neato",
        );

        // send frame to the host
        crate::util::channel_send(
            cx.local.robot_message_sender_esp_neato,
            RobotMessageInternal::ScanFrame(ScanFrameInternal {
                scan_data: buffer,
                odometry: [odometry_left, odometry_right],
                rpm,
            }),
            "uart0_neato",
        );
    });
}
