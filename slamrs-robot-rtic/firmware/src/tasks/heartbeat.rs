use crate::{app::heartbeat, Mono};
use rp_pico::hal::fugit::ExtU64;
use rtic::Mutex;
use rtic_monotonics::Monotonic;

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

#[derive(defmt::Format, Copy, Clone, Default)]
pub enum LedStatus {
    #[default]
    Off,
    On(Color),
    Blinking(Color, Speed),
}

pub async fn heartbeat(mut cx: heartbeat::Context<'_>) {
    cx.local.led_rgb.set_color(0, 0, 0);

    let mut counter = 0;
    let mut was_on = false;
    const SCALE: u8 = 8;

    // 10hz loop
    let mut next_iteration_instant = Mono::now();
    loop {
        next_iteration_instant += 100.millis();
        Mono::delay_until(next_iteration_instant).await;

        let state = cx.shared.led_status.lock(|s| *s);

        match state {
            LedStatus::Off => {
                cx.local.led_rgb.set_color(0, 0, 0);
                was_on = false;
            }
            LedStatus::On(color) => {
                let (r, g, b) = color.rgb();
                cx.local.led_rgb.set_color(r / SCALE, g / SCALE, b / SCALE);
                was_on = true;
            }
            LedStatus::Blinking(color, speed) => {
                let (r, g, b) = color.rgb();
                let iterations = speed.iterations_at_10hz();
                if counter % iterations == 0 {
                    was_on = !was_on;
                    if was_on {
                        cx.local.led_rgb.set_color(r / SCALE, g / SCALE, b / SCALE);
                    } else {
                        cx.local.led_rgb.set_color(0, 0, 0);
                    }
                }
            }
        }

        counter += 1;
    }
}
