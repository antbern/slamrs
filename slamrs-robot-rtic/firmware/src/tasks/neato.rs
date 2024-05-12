use crate::{
    app::{neato_motor_control, uart0_neato},
    motor::MotorDirection,
};
use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use cortex_m::peripheral::dwt;
use defmt::{info, warn};
use library::slamrs_message::{RobotMessage, ScanFrame};
use rp_pico::hal::fugit::ExtU64;
use rtic::Mutex;
use rtic_monotonics::rp2040::Timer;

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
        Timer::delay(200.millis()).await;

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
        *cx.local.rpm_accumulator += rpm as i32 - *cx.local.rpm_average as i32;
        *cx.local.rpm_average = *cx.local.rpm_accumulator >> 2;
        let rpm = (*cx.local.rpm_average / 64) as u16;
        LAST_RPM.store(rpm, core::sync::atomic::Ordering::Relaxed);

        info!("neato rpm: {:?}", rpm);
        // TODO: should we add a data validation check?
        if rpm < 250 && rpm > 350 {
            return;
        }

        *cx.local.downsample_counter += 1;
        if *cx.local.downsample_counter > cx.shared.neato_downsampling.load(Ordering::Relaxed) {
            *cx.local.downsample_counter = 0;
        } else {
            return;
        }

        // need to copy the data to a new array because the data is borrowed from the parser
        let mut scan_data = [0; 1980];
        scan_data.copy_from_slice(data.data);

        // send frame to the host
        crate::util::channel_send(
            cx.local.robot_message_sender_neato,
            RobotMessage::ScanFrame(ScanFrame {
                scan_data,
                odometry: [0.0; 2], // TODO: add odometry
                rpm,
            }),
            "uart0_neato",
        );

        // need to copy the data to a new array because the data is borrowed from the parser
        let mut scan_data = [0; 1980];
        scan_data.copy_from_slice(data.data);

        // send frame to the host
        crate::util::channel_send(
            cx.local.robot_message_sender_esp_neato,
            RobotMessage::ScanFrame(ScanFrame {
                scan_data,
                odometry: [0.0; 2], // TODO: add odometry
                rpm,
            }),
            "uart0_neato",
        );
    });
}
