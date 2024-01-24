use common::node::{Node, NodeConfig};
use egui::{
    mutex::{Mutex, RwLock},
    Slider,
};

use graphics::primitiverenderer::{Color, PrimitiveType};
use nalgebra::{Point2, Vector2};
use simulator_loop::SimulatorLoop;
use std::sync::Arc;

use scene::{
    landmark::Landmark,
    ray::{Draw, LineSegment, Scene},
};
use serde::Deserialize;
use sim::{SimParameters, Simulator};

mod scene;
mod sim;
pub struct SimulatorNode {
    scene: Arc<RwLock<Scene>>,
    simulator: Arc<Mutex<Simulator>>,
    simulator_loop: simulator_loop::SimulatorLoop,
    running: bool,
    draw_scene: bool,
    draw_pose: bool,
}

#[derive(Clone, Deserialize)]
pub struct SimulatorNodeConfig {
    topic_observation_scanner: Option<String>,
    topic_observation_landmarks: Option<String>,
    topic_command: String,
    running: bool,

    #[serde(default)]
    scene: Vec<SceneObject>,

    #[serde(default)]
    landmarks: Vec<Landmark>,

    #[serde(default = "_default_true")]
    draw_scene: bool,
    #[serde(default = "_default_true")]
    draw_pose: bool,

    parameters: SimParameters,
}

const fn _default_true() -> bool {
    true
}

#[derive(Clone, Deserialize)]
enum SceneObject {
    Line {
        x1: f32,
        y1: f32,
        x2: f32,
        y2: f32,
    },
    Rectangle {
        x: f32,
        y: f32,
        width: f32,
        height: f32,
    },
}

impl NodeConfig for SimulatorNodeConfig {
    fn instantiate(&self, pubsub: &mut pubsub::PubSub) -> Box<dyn Node> {
        let mut scene = Scene::new();

        scene.add_landmarks(&self.landmarks);

        for o in &self.scene {
            match *o {
                SceneObject::Line { x1, y1, x2, y2 } => {
                    scene.add(Box::new(LineSegment::new(x1, y1, x2, y2)));
                }
                SceneObject::Rectangle {
                    x,
                    y,
                    width,
                    height,
                } => {
                    scene.add_rect(Point2::new(x, y), Vector2::new(width, height));
                }
            }
        }

        let scene = Arc::new(RwLock::new(scene));
        let simulator = Arc::new(Mutex::new(Simulator::new(
            self.topic_observation_scanner
                .as_ref()
                .map(|topic| pubsub.publish(topic)),
            self.topic_observation_landmarks
                .as_ref()
                .map(|topic| pubsub.publish(topic)),
            pubsub.subscribe(&self.topic_command),
            scene.clone(),
            self.parameters,
        )));

        Box::new(SimulatorNode {
            scene,
            running: self.running,
            simulator: simulator.clone(),
            simulator_loop: SimulatorLoop::new(simulator),
            draw_scene: self.draw_scene,
            draw_pose: self.draw_pose,
        })
    }
}

impl Node for SimulatorNode {
    fn draw(&mut self, ui: &egui::Ui, world: &mut common::world::WorldObj<'_>) {
        self.simulator_loop.tick(self.running);

        egui::Window::new("Simulator").show(ui.ctx(), |ui| {
            ui.label("Used to simulate different LIDAR sensors and environment shapes.");

            ui.checkbox(&mut self.running, "Running");

            ui.checkbox(&mut self.draw_scene, "Draw Scene");
            ui.checkbox(&mut self.draw_pose, "Draw Pose");

            // lock the scene to make UI controls for some of the parameters
            {
                let mut simulator = self.simulator.lock();
                let params = simulator.parameters_mut();
                ui.add(Slider::new(&mut params.wheel_base, 0.05..=0.4).text("Wheel Base (m)"));
                ui.add(Slider::new(&mut params.update_period, 0.1..=2.0).text("Update Period (s)"));
                ui.add(Slider::new(&mut params.scanner_range, 0.1..=10.0).text("Scanner Range(m)"));
            }
        });
        if self.draw_scene {
            world.sr.begin(PrimitiveType::Line);
            self.scene.read().draw(world.sr, Color::BLACK);
            world.sr.end();
        }

        if self.draw_pose {
            world.sr.begin(PrimitiveType::Filled);
            let pose = self.simulator_loop.lock().get_pose();
            world.sr.arrow(pose.x, pose.y, pose.theta, 0.1, Color::BLUE);
            world.sr.end()
        }
    }

    fn terminate(&mut self) {
        self.simulator_loop.tick(false);
    }
}

#[cfg(target_arch = "wasm32")]
mod simulator_loop {
    // For now: Run the simulator directly on the main thread on wasm targets
    // since threading is a bit complicated...
    // https://rustwasm.github.io/wasm-bindgen/examples/raytrace.html

    use std::sync::Arc;

    use egui::mutex::{Mutex, MutexGuard};
    use web_time::Instant;

    use crate::sim::Simulator;

    pub struct SimulatorLoop {
        simulator: Arc<Mutex<Simulator>>,
        accumulator: f64,
        current_time: Instant,
    }

    impl SimulatorLoop {
        pub fn new(simulator: Arc<Mutex<Simulator>>) -> Self {
            Self {
                simulator,
                accumulator: 0.0,
                current_time: Instant::now(),
            }
        }

        pub fn tick(&mut self, running: bool) {
            if running {
                let dt = 1.0 / 30.0;

                let new_time = Instant::now();
                let frame_time = new_time - self.current_time;
                self.current_time = new_time;

                self.accumulator += frame_time.as_secs_f64();

                while self.accumulator >= dt {
                    self.simulator.lock().tick(dt as f32);
                    self.accumulator -= dt;
                }
            }
        }

        pub fn lock(&mut self) -> MutexGuard<'_, Simulator> {
            self.simulator.lock()
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
mod simulator_loop {
    // On desktop targets we run the simulator in a separate background thread,
    // while the actual game loop is the same.
    use crate::Simulator;
    use egui::mutex::{Mutex, MutexGuard};
    use std::{
        sync::{
            atomic::{AtomicBool, Ordering},
            Arc,
        },
        thread::{self, JoinHandle},
        time::Duration,
    };
    use tracing::info;
    use web_time::Instant;

    pub struct SimulatorLoop {
        simulator: Arc<Mutex<Simulator>>,
        handle: Option<SimulatorThreadHandle>,
    }

    impl SimulatorLoop {
        pub fn new(simulator: Arc<Mutex<Simulator>>) -> Self {
            Self {
                simulator,
                handle: None,
            }
        }
        pub fn tick(&mut self, running: bool) {
            if running && self.handle.is_none() {
                self.handle = Some(SimulatorThreadHandle::new(self.simulator.clone()))
            }

            if !running {
                if let Some(h) = self.handle.take() {
                    h.stop();
                }
            }
        }
        pub fn lock(&mut self) -> MutexGuard<'_, Simulator> {
            self.simulator.lock()
        }
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
            info!("Simulator Thread Started");

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

            info!("Simulator Thread Ended");
        }

        pub fn stop(self) {
            self.running.store(false, Ordering::Relaxed);
            self.handle.join().unwrap();
        }
    }
}
