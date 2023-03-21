use std::{sync::Arc, time::Instant};

use crate::{
    config::Config,
    node::{
        controls::ControlsNode, frame_viz::FrameVizualizer, mouse_position::MousePosition,
        shape_rendering::ShapeRendering,
    },
};
use common::{node::Node, world::WorldObj, PerfStats};
use eframe::egui_glow;
use egui::{mutex::Mutex, Label, Pos2, RichText, Sense, Vec2};
use graphics::{camera::Camera, shaperenderer::ShapeRenderer};
use nalgebra::{Matrix4, Point2};
use neato::{FileLoader, SerialConnection};
use pubsub::{PubSub, PubSubThreadHandle};
use simulator::SimulatorNode;
use slam::SlamNode;

pub struct App {
    _pubsub: PubSubThreadHandle,
    nodes: Vec<Box<dyn Node>>,

    /// Behind an `Arc<Mutex<…>>` so we can pass it to [`egui::PaintCallback`] and paint later.
    world_renderer: Arc<Mutex<WorldRenderer>>,

    stats: PerfStats,
}

impl App {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>, config: Config) -> Self {
        // This is also where you can customize the look and feel of egui using
        // `cc.egui_ctx.set_visuals` and `cc.egui_ctx.set_fonts`.

        let gl = cc
            .gl
            .as_ref()
            .expect("You need to run eframe with the glow backend");

        let mut pubsub = PubSub::new();

        // instantiate based on the config
        let nodes: Vec<Box<dyn Node>> = config.instantiate_nodes(&mut pubsub);

        // TODO: do stuff with the config.settings object

        // TODO: remove this once we have processing that is not dependent on UI updates...
        let ctx = cc.egui_ctx.clone();

        Self {
            nodes,
            _pubsub: pubsub.start_background_thread(move || ctx.request_repaint()),
            world_renderer: Arc::new(Mutex::new(WorldRenderer::new(gl))),
            stats: PerfStats::new(),
        }
    }
}

impl eframe::App for App {
    /// Called each time the UI needs repainting, which may be many times per second.
    /// Put your widgets into a `SidePanel`, `TopPanel`, `CentralPanel`, `Window` or `Area`.
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let start_time = Instant::now();
        // Examples of how to create different panels and windows.
        // Pick whichever suits you.
        // Tip: a good default choice is to just keep the `CentralPanel`.
        // For inspiration and more examples, go to https://emilk.github.io/egui

        #[cfg(not(target_arch = "wasm32"))] // no File->Quit on web pages!
        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            // The top panel is often a good place for a menu bar:
            egui::menu::bar(ui, |ui| {
                // ui.menu_button("File", |ui| {
                //     if ui.button("Quit").clicked() {
                //         _frame.close();
                //     }
                // });

                ui.label(
                    RichText::new(format!(
                        "Render: {:>5} fps",
                        self.stats.latest_fps() as usize
                    ))
                    .monospace(),
                );

                if ui
                    .add(
                        Label::new(RichText::new(self.stats.to_string()).monospace())
                            .sense(Sense::click()),
                    )
                    .clicked()
                {
                    self.stats.reset();
                }
            });
        });

        for n in self.nodes.iter_mut() {
            n.update();
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            // The central panel the region left after adding TopPanel's and SidePanel's

            // Let all nodes do their drawing. Explicit scope for MutexGuard lifetime.
            {
                let mut world = self.world_renderer.lock();

                let mut world_obj = world.as_world_object();

                for n in self.nodes.iter_mut() {
                    n.draw(ui, &mut world_obj);
                }
            }

            self.custom_painting(ui);
        });

        self.stats.update(start_time.elapsed());
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

        let pos = if ui.rect_contains_pointer(rect) {
            let mut pos = ui.ctx().pointer_hover_pos().unwrap_or(Pos2::default());
            // adjust for the position of the allocated space
            pos.x -= rect.left();
            pos.y -= rect.top();
            Some(pos)
        } else {
            None
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
                    .paint(painter.gl(), pos, size, drag_delta, scroll);
            })),
        };
        ui.painter().add(callback);
    }
}

pub struct WorldRenderer {
    pub sr: ShapeRenderer,
    camera: Camera,
    pub last_mouse_pos: Point2<f32>,
}

impl WorldRenderer {
    fn new(gl: &glow::Context) -> Self {
        // use glow::HasContext as _;

        Self {
            sr: ShapeRenderer::new(gl),
            camera: Camera::new(),
            last_mouse_pos: Point2::new(0.0, 0.0),
        }
    }

    fn destroy(&mut self, gl: &glow::Context) {
        self.sr.destroy(gl);
    }

    fn as_world_object(&mut self) -> WorldObj<'_> {
        WorldObj {
            sr: &mut self.sr,
            last_mouse_pos: self.last_mouse_pos,
        }
    }

    fn paint(&mut self, gl: &glow::Context, pos: Option<Pos2>, size: Vec2, pan: Vec2, scroll: f32) {
        // first update the camera with any zoom and resize change
        self.camera.resize(size);
        self.camera.pan(pan);
        self.camera.zoom(scroll);
        self.camera.update();

        // set the correct MVP matrix for the shape renderer
        let mvp: Matrix4<f32> = self.camera.get_mvp();
        self.sr.set_mvp(mvp);

        // unproject mouse position to
        if let Some(pos) = pos {
            self.last_mouse_pos = self.camera.unproject(pos);
        }

        // do the actual drawing of already cached vertices
        self.sr.flush(gl);
    }
}
