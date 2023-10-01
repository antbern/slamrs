#![warn(clippy::all, rust_2018_idioms)]
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

// When compiling natively:
#[cfg(not(target_arch = "wasm32"))]
fn main() -> Result<(), eframe::Error> {
    // Log to stdout (if you run with `RUST_LOG=debug`).
    tracing_subscriber::fmt::init();

    use baseui::config::Config;
    use egui::{Style, Visuals};

    // load configuration file
    let mut args = std::env::args();
    let config = if args.len() >= 2 {
        Config::from_file(&args.nth(1).unwrap()).expect("Could not load config file")
    } else {
        Config::default()
    };

    

    let native_options = eframe::NativeOptions {
        initial_window_size: Some(egui::vec2(1280.0, 720.0)),
        resizable: true,
        //multisampling: 8, // does not seem to work on my laptop
        renderer: eframe::Renderer::Glow,
        ..Default::default()
    };

    eframe::run_native(
        "Base UI",
        native_options,
        Box::new(|cc| {
            let style = Style {
                visuals: Visuals::light(),
                ..Style::default()
            };
            cc.egui_ctx.set_style(style);
            Box::new(baseui::App::new(cc, config))
        }),
    )
}

// when compiling to web using trunk.
#[cfg(target_arch = "wasm32")]
fn main() {
    use baseui::config::Config;

    // Redirect `log` message to `console.log` and friends:
    eframe::WebLogger::init(log::LevelFilter::Debug).ok();

    // Redirect tracing to console.log and friends:
    tracing_wasm::set_as_global_default();

    let config = Config::default();

    let web_options = eframe::WebOptions::default();

    wasm_bindgen_futures::spawn_local(async {
        eframe::WebRunner::new()
            .start(
                "the_canvas_id", // hardcode it
                web_options,
                Box::new(|cc| Box::new(baseui::App::new(cc, config))),
            )
            .await
            .expect("failed to start eframe");
    });
}
