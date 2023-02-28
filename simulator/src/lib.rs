use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use common::node::Node;
use egui::mutex::{Mutex, RwLock};
use graphics::primitiverenderer::{Color, PrimitiveType};
use nalgebra::{Point2, Vector2};

use scene::ray::{Draw, LineSegment, Scene};
use sim::{SimParameters, Simulator};

mod scene;
mod sensor;
mod sim;
pub struct SimulatorNode {
    scene: Arc<RwLock<Scene>>,
    parameters: SimParameters,
    // parameter_channel: Sender<SimParameters>,
    sim: Arc<Mutex<Simulator>>,
    handle: Option<SimulatorThreadHandle>,
    running: bool,
}

impl Node for SimulatorNode {
    fn new(pubsub: &mut pubsub::PubSub) -> Self
    where
        Self: Sized,
    {
        let mut scene = Scene::new();

        scene
            .add_rect(Point2::new(-1.0, -1.0), Vector2::new(2.0, 2.0))
            .add_rect(Point2::new(-0.1, -0.4), Vector2::new(0.5, 0.1))
            .add_rect(Point2::new(-0.6, 0.4), Vector2::new(0.2, 0.5))
            .add(Box::new(LineSegment::new(-0.4, -0.4, 0.4, 0.4)));

        let scene = Arc::new(RwLock::new(scene));

        Self {
            scene: scene.clone(),
            sim: Arc::new(Mutex::new(Simulator::new(
                pubsub.publish("robot/observation"),
                pubsub.publish("robot/pose"),
                pubsub.subscribe("robot/command"),
                scene,
                SimParameters::default(),
            ))),
            parameters: SimParameters::default(),
            handle: None,
            running: false,
        }
    }

    fn draw(&mut self, ui: &egui::Ui, world: &mut common::world::WorldObj<'_>) {
        egui::Window::new("Simulator").show(ui.ctx(), |ui| {
            ui.label("Used to simulate different LIDAR sensors and environment shapes.");

            // TODO: add controls for all the parameters and simulator controls here
            ui.checkbox(&mut self.running, "Running");

            if self.running && self.handle.is_none() {
                self.handle = Some(SimulatorThreadHandle::new(self.sim.clone()))
            }

            if !self.running {
                if let Some(h) = self.handle.take() {
                    h.stop();
                }
            }
        });

        // draw the scene itself
        world.sr.begin(PrimitiveType::Line);
        self.scene.read().draw(world.sr, Color::BLACK);
        world.sr.end();

        // TODO: draw the robot position (if enabled)
    }

    fn terminate(&mut self) {}
}

struct SimulatorThreadHandle {
    handle: JoinHandle<()>,
    running: Arc<AtomicBool>,
}
impl SimulatorThreadHandle {
    pub fn new(sim: Arc<Mutex<Simulator>>) -> Self {
        let running = Arc::new(AtomicBool::new(true));

        let handle = thread::spawn({
            let running = running.clone();
            move || Self::thread(running, sim)
        });

        SimulatorThreadHandle { handle, running }
    }

    fn thread(running: Arc<AtomicBool>, sim: Arc<Mutex<Simulator>>) {
        println!("[SimulatorThread] Started");

        // loop taken from : https://www.gafferongames.com/post/fix_your_timestep/
        let dt = 1.0 / 30.0;

        let mut current_time = Instant::now();
        let mut accumulator = 0.0;

        while running.load(Ordering::Relaxed) {
            let new_time = Instant::now();
            let frame_time = new_time - current_time;
            current_time = new_time;

            accumulator += frame_time.as_secs_f64();

            while accumulator >= dt {
                sim.lock().tick(dt as f32);
                accumulator -= dt;
            }

            thread::sleep(Duration::from_secs_f64(dt));
        }

        println!("[SimulatorThread] Ended");
    }

    pub fn stop(self) {
        self.running.store(false, Ordering::Relaxed);
        self.handle.join().unwrap();
    }
}
