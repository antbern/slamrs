use std::sync::Arc;

use crate::graphics::{
    camera::Camera,
    primitiverenderer::{Color, PrimitiveType},
    shaperenderer::ShapeRenderer,
};
use eframe::egui_glow;
use egui::{mutex::Mutex, Vec2};
use nalgebra::Matrix4;
pub struct App {
    // Example stuff:
    label: String,

    value: f32,

    /// Behind an `Arc<Mutex<…>>` so we can pass it to [`egui::PaintCallback`] and paint later.
    world_renderer: Arc<Mutex<WorldRenderer>>,
}

impl App {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // This is also where you can customize the look and feel of egui using
        // `cc.egui_ctx.set_visuals` and `cc.egui_ctx.set_fonts`.

        let gl = cc
            .gl
            .as_ref()
            .expect("You need to run eframe with the glow backend");

        Self {
            label: "Hello World!".to_owned(),
            value: 2.7,
            world_renderer: Arc::new(Mutex::new(WorldRenderer::new(gl))),
        }
    }
}

impl eframe::App for App {
    /// Called each time the UI needs repainting, which may be many times per second.
    /// Put your widgets into a `SidePanel`, `TopPanel`, `CentralPanel`, `Window` or `Area`.
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let Self { label, value, .. } = self;

        // Examples of how to create different panels and windows.
        // Pick whichever suits you.
        // Tip: a good default choice is to just keep the `CentralPanel`.
        // For inspiration and more examples, go to https://emilk.github.io/egui

        #[cfg(not(target_arch = "wasm32"))] // no File->Quit on web pages!
        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            // The top panel is often a good place for a menu bar:
            egui::menu::bar(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("Quit").clicked() {
                        _frame.close();
                    }
                });
            });
        });

        egui::Window::new("Window").show(ctx, |ui| {
            ui.label("Windows can be moved by dragging them.");
            ui.label("They are automatically sized based on contents.");
            ui.label("You can turn on resizing and scrolling if you like.");
            ui.label("You would normally choose either panels OR windows.");

            ui.heading("Side Panel");

            ui.horizontal(|ui| {
                ui.label("Write something: ");
                ui.text_edit_singleline(label);
            });

            ui.add(egui::Slider::new(value, 0.0..=10.0).text("value"));
            if ui.button("Increment").clicked() {
                *value += 1.0;
            }

            ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 0.0;
                    ui.label("powered by ");
                    ui.hyperlink_to("egui", "https://github.com/emilk/egui");
                    ui.label(" and ");
                    ui.hyperlink_to(
                        "eframe",
                        "https://github.com/emilk/egui/tree/master/crates/eframe",
                    );
                    ui.label(".");
                });
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            // The central panel the region left after adding TopPanel's and SidePanel's
            self.custom_painting(ui);
        });
    }
    fn on_exit(&mut self, gl: Option<&glow::Context>) {
        if let Some(gl) = gl {
            self.world_renderer.lock().destroy(gl);
        }
    }
}

impl App {
    fn custom_painting(&mut self, ui: &mut egui::Ui) {
        let (rect, response) = ui.allocate_exact_size(
            ui.available_size(), //egui::Vec2::splat(300.0)
            egui::Sense::drag(),
        );

        let scroll = if ui.rect_contains_pointer(rect) {
            ui.ctx().input().scroll_delta.y / 50.0
        } else {
            0.0
        };

        // Clone locals so we can move them into the paint callback:

        let mut drag_delta = response.drag_delta();
        drag_delta.y *= -1.0;

        let size = rect.size();
        let world_renderer = self.world_renderer.clone();

        let callback = egui::PaintCallback {
            rect,
            callback: std::sync::Arc::new(egui_glow::CallbackFn::new(move |_info, painter| {
                world_renderer
                    .lock()
                    .paint(painter.gl(), size, drag_delta, scroll);
            })),
        };
        ui.painter().add(callback);
    }
}

struct WorldRenderer {
    sr: ShapeRenderer,
    camera: Camera,
}

impl WorldRenderer {
    fn new(gl: &glow::Context) -> Self {
        // use glow::HasContext as _;

        Self {
            sr: ShapeRenderer::new(gl),
            camera: Camera::new(),
        }
    }

    fn destroy(&mut self, gl: &glow::Context) {
        self.sr.destroy(gl);
    }

    fn paint(&mut self, gl: &glow::Context, size: Vec2, pan: Vec2, scroll: f32) {
        // first update the camera with any zoom and resize change
        self.camera.resize(size);
        self.camera.pan(pan);
        self.camera.zoom(scroll);
        self.camera.update();
        let mvp: Matrix4<f32> = self.camera.get_mvp();

        self.sr.set_mvp(mvp);

        // draw!
        let c1 = Color::rgba(1.0, 0.0, 0.0, 1.0);
        let c2 = Color::rgba(0.0, 1.0, 0.0, 1.0);
        let c3 = Color::rgba(0.0, 0.0, 1.0, 1.0);

        self.sr.begin(PrimitiveType::Filled);
        for x in 0..255 {
            for y in 0..255 {
                let c = Color::rgba_u8(x, y, 128, 0xff);
                self.sr.rect(
                    x as f32 / 255.0,
                    y as f32 / 255.0,
                    1.0 / 255.0,
                    1.0 / 255.0,
                    c,
                );
            }
        }

        self.sr.end();

        // self.pr.begin(PrimitiveType::Filled);

        // self.pr.xyc(0.0, 1.0, c1);
        // self.pr.xyc(-1.0, -1.0, c2);
        // self.pr.xyc(1.0, -1.0, c3);

        // self.pr.end();

        // self.pr.begin(PrimitiveType::Line);

        // self.pr.xyc(0.0, 1.0 + 0.1, c1);
        // self.pr.xyc(-1.0 - 0.1, -1.0 - 0.1, c2);

        // self.pr.xyc(-1.0 - 0.1, -1.0 - 0.1, c2);
        // self.pr.xyc(1.0 + 0.1, -1.0 - 0.1, c3);

        // self.pr.xyc(1.0 + 0.1, -1.0 - 0.1, c3);
        // self.pr.xyc(0.0, 1.0 + 0.1, c1);

        // self.pr.end();

        self.sr.flush(gl);
    }
}
