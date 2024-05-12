use crate::{
    app::{neato_motor_control, uart0_neato},
    motor::MotorDirection,
};
use defmt::{info, warn};
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
    Timer::delay(1000.millis()).await;

    // stop
    cx.shared.motor_controller.lock(|mc| {
        cx.local.neato_motor.set_speed(mc, 0).unwrap();
    });
}
pub fn uart0_neato(cx: uart0_neato::Context<'_>) {}
