use crate::app::heartbeat;
use embedded_hal::digital::v2::ToggleableOutputPin;
use rp_pico::hal::fugit::ExtU64;
use rtic::Mutex;
use rtic_monotonics::{rp2040::Timer, Monotonic};

#[derive(defmt::Format, Copy, Clone)]
pub enum Color {
    Red,
    Green,
    Blue,
    Yellow,
    Cyan,
    Magenta,
    White,
}
impl Color {
    fn rgb(&self) -> (u8, u8, u8) {
        match self {
            Color::Red => (255, 0, 0),
            Color::Green => (0, 255, 0),
            Color::Blue => (0, 0, 255),
            Color::Yellow => (255, 255, 0),
            Color::Cyan => (0, 255, 255),
            Color::Magenta => (255, 0, 255),
            Color::White => (255, 255, 255),
        }
    }
}

#[derive(defmt::Format, Copy, Clone)]
pub enum Speed {
    Slow,
    Medium,
    Fast,
}
impl Speed {
    fn iterations_at_10hz(&self) -> u32 {
        match self {
            Speed::Slow => 10,
            Speed::Medium => 5,
            Speed::Fast => 1,
        }
    }
}

#[derive(defmt::Format, Copy, Clone)]
pub enum LedStatus {
    Off,
    On(Color),
    Blinking(Color, Speed),
}
impl Default for LedStatus {
    fn default() -> Self {
        LedStatus::Off
    }
}

pub async fn heartbeat(mut cx: heartbeat::Context<'_>) {
    cx.local.led_rgb.set_color(0, 0, 0);

    let mut counter = 0;
    let mut was_on = false;

    // 10hz loop
    let mut next_iteration_instant = Timer::now();
    loop {
        next_iteration_instant += 100.millis();
        Timer::delay_until(next_iteration_instant).await;

        let state = cx.shared.led_status.lock(|s| *s);

        match state {
            LedStatus::Off => {
                cx.local.led_rgb.set_color(0, 0, 0);
                was_on = false;
            }
            LedStatus::On(color) => {
                let (r, g, b) = color.rgb();
                cx.local.led_rgb.set_color(r, g, b);
                was_on = true;
            }
            LedStatus::Blinking(color, speed) => {
                let (r, g, b) = color.rgb();
                let iterations = speed.iterations_at_10hz();
                if counter % iterations == 0 {
                    was_on = !was_on;
                    if was_on {
                        cx.local.led_rgb.set_color(r, g, b);
                    } else {
                        cx.local.led_rgb.set_color(0, 0, 0);
                    }
                }
            }
        }

        // Flicker the built-in LED
        // _ = cx.local.led.toggle();

        counter += 1;
    }
}
