use common::node::{Node, NodeConfig};
use egui::mutex::{Mutex, RwLock};
use graphics::primitiverenderer::{Color, PrimitiveType};
use nalgebra::{Point2, Vector2};
use simulator_loop::SimulatorLoop;
use std::sync::Arc;

use scene::ray::{Draw, LineSegment, Scene};
use serde::Deserialize;
use sim::{SimParameters, Simulator};

mod scene;
mod sensor;
mod sim;
pub struct SimulatorNode {
    scene: Arc<RwLock<Scene>>,
    parameters: SimParameters,
    // parameter_channel: Sender<SimParameters>,
    simulator_loop: simulator_loop::SimulatorLoop,
    running: bool,
}

#[derive(Clone, Deserialize)]
pub struct SimulatorNodeConfig {
    topic_observation: String,
    topic_observation_odometry: String,
    topic_command: String,
    running: bool,

    #[serde(default)]
    scene: Vec<SceneObject>,

    parameters: SimParameters,
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
            pubsub.publish(&self.topic_observation),
            pubsub.publish(&self.topic_observation_odometry),
            pubsub.subscribe(&self.topic_command),
            scene.clone(),
            self.parameters,
        )));

        Box::new(SimulatorNode {
            scene,
            parameters: self.parameters,
            running: self.running,
            simulator_loop: SimulatorLoop::new(simulator),
        })
    }
}

impl Node for SimulatorNode {
    fn draw(&mut self, ui: &egui::Ui, world: &mut common::world::WorldObj<'_>) {
        self.simulator_loop.tick(self.running);

        egui::Window::new("Simulator").show(ui.ctx(), |ui| {
            ui.label("Used to simulate different LIDAR sensors and environment shapes.");

            // TODO: add controls for all the parameters and simulator controls here
            ui.checkbox(&mut self.running, "Running");
        });

        // draw the scene itself
        world.sr.begin(PrimitiveType::Line);
        self.scene.read().draw(world.sr, Color::BLACK);
        world.sr.end();

        world.sr.begin(PrimitiveType::Filled);
        let pose = self.simulator_loop.lock().get_pose();
        world.sr.arrow(pose.x, pose.y, pose.theta, 0.1, Color::BLUE);
        world.sr.end()

        // TODO: draw the robot position (if enabled)
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
}
