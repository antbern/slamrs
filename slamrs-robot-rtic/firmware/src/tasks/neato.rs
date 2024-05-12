use crate::{
    app::{neato_motor_control, uart0_neato},
    motor::MotorDirection,
};
use defmt::{info, warn};
use embedded_hal::serial::Read;
use library::slamrs_message::{RobotMessage, ScanFrame};
use rp_pico::hal::fugit::ExtU64;
use rtic::Mutex;
use rtic_monotonics::rp2040::Timer;

pub async fn neato_motor_control(mut cx: neato_motor_control::Context<'_>) {
    // start the motor
    cx.shared.motor_controller.lock(|mc| {
        cx.local
            .neato_motor
            .set_direction(mc, MotorDirection::Forward)
            .unwrap();
        cx.local.neato_motor.set_speed(mc, 500).unwrap();
    });

    // wait for 1 second
    Timer::delay(10000.millis()).await;

    // stop
    cx.shared.motor_controller.lock(|mc| {
        cx.local.neato_motor.set_speed(mc, 0).unwrap();
    });
}
pub fn uart0_neato(cx: uart0_neato::Context<'_>) {
    cx.local.parser.consume(cx.local.uart0_rx_neato, |data| {
        let rpm = data.parse_rpm();
        info!("neato rpm: {:?}", rpm);
        // TODO: should we add a data validation check?
        if rpm < 250 && rpm > 350 {
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
    });
}
