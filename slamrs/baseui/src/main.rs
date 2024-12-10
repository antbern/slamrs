#![warn(clippy::all, rust_2018_idioms)]
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

use eframe::egui;
use egui::{Style, Visuals};
fn set_style(ctx: &egui::Context) {
    let style = Style {
        visuals: Visuals::light(),
        ..Style::default()
    };
    ctx.set_style(style);
}

// When compiling natively:
#[cfg(not(target_arch = "wasm32"))]
fn main() -> Result<(), eframe::Error> {
    // Log to stdout (if you run with `RUST_LOG=debug`).
    tracing_subscriber::fmt::fmt()
        .with_span_events(FmtSpan::CLOSE)
        //.with_target(false)
        //.with_level(false)
        .init();

    use baseui::config::Config;
    use tracing_subscriber::fmt::format::FmtSpan;

    // load configuration file
    let mut args = std::env::args();
    let config = if args.len() >= 2 {
        Config::from_file(&args.nth(1).unwrap()).expect("Could not load config file")
    } else {
        Config::default()
    };

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 720.])
            .with_resizable(true),
        //multisampling: 8, // does not seem to work on my laptop
        renderer: eframe::Renderer::Glow,
        ..Default::default()
    };

    eframe::run_native(
        "Base UI",
        native_options,
        Box::new(|cc| {
            set_style(&cc.egui_ctx);
            Ok(Box::new(baseui::App::new(cc, config)))
        }),
    )
}

// when compiling to web using trunk.
#[cfg(target_arch = "wasm32")]
fn main() {
    use baseui::config::Config;
    use eframe::wasm_bindgen::JsCast as _;
    use eframe::web_sys;

    // Redirect `log` message to `console.log` and friends:
    eframe::WebLogger::init(log::LevelFilter::Debug).ok();

    // Redirect tracing to console.log and friends:
    tracing_wasm::set_as_global_default();

    let config = Config::default();

    let web_options = eframe::WebOptions::default();

    wasm_bindgen_futures::spawn_local(async {
        let document = web_sys::window()
            .expect("No window")
            .document()
            .expect("No document");

        let canvas = document
            .get_element_by_id("the_canvas_id")
            .expect("Failed to find the_canvas_id")
            .dyn_into::<web_sys::HtmlCanvasElement>()
            .expect("the_canvas_id was not a HtmlCanvasElement");

        let start_result = eframe::WebRunner::new()
            .start(
                canvas,
                web_options,
                Box::new(|cc| {
                    set_style(&cc.egui_ctx);
                    Ok(Box::new(baseui::App::new(cc, config)))
                }),
            )
            .await;

        // Remove the loading text and spinner:
        if let Some(loading_text) = document.get_element_by_id("loading_text") {
            match start_result {
                Ok(_) => {
                    loading_text.remove();
                }
                Err(e) => {
                    loading_text.set_inner_html(
                        "<p> The app has crashed. See the developer console for details. </p>",
                    );
                    panic!("Failed to start eframe: {e:?}");
                }
            }
        }
    });
}
