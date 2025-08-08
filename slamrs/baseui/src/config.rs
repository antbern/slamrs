use std::fs;

use anyhow::anyhow;
use common::node::{Node, NodeConfig};
use pubsub::PubSub;
use serde::Deserialize;
use simulator::SimulatorNodeConfig;
use slam::{EKFLandmarkSlamNodeConfig, GridMapSlamNodeConfig, IcpPointMapNodeConfig};

use crate::node::{
    controls::ControlsNodeConfig, frame_viz::FrameVizualizerNodeConfig, gamepad::GamepadNodeConfig,
    gaussian::GaussianNodeConfig, mouse_position::MousePositionNodeConfig,
    shape_rendering::ShapeRenderingNodeConfig, splitter::SplitterNodeConfig,
};

#[cfg(not(target_arch = "wasm32"))]
use neato::{FileLoaderNodeConfig, RobotConnectionNodeConfig};

#[derive(Clone, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub settings: Settings,

    pub nodes: Vec<NodeEnum>,
}

#[derive(Clone, Deserialize, Default)]
pub struct Settings {
    // headless: bool,
}

#[derive(Clone, Deserialize)]
pub enum NodeEnum {
    Simulator(SimulatorNodeConfig),
    Controls(ControlsNodeConfig),
    Gamepad(GamepadNodeConfig),
    MousePosition(MousePositionNodeConfig),
    ShapeTest(ShapeRenderingNodeConfig),
    #[cfg(not(target_arch = "wasm32"))]
    FileLoader(FileLoaderNodeConfig),
    #[cfg(not(target_arch = "wasm32"))]
    RobotConnection(RobotConnectionNodeConfig),
    IcpPointMapper(IcpPointMapNodeConfig),
    Visualizer(FrameVizualizerNodeConfig),
    GridMapSlam(GridMapSlamNodeConfig),
    GaussianTest(GaussianNodeConfig),
    Splitter(SplitterNodeConfig),
    EKFLandmarkSlam(EKFLandmarkSlamNodeConfig),
}

impl NodeEnum {
    fn instantiate(&self, pubsub: &mut PubSub) -> Box<dyn Node> {
        use NodeEnum::*;
        match self {
            Simulator(c) => c.instantiate(pubsub),
            Controls(c) => c.instantiate(pubsub),
            Gamepad(c) => c.instantiate(pubsub),
            MousePosition(c) => c.instantiate(pubsub),
            ShapeTest(c) => c.instantiate(pubsub),
            #[cfg(not(target_arch = "wasm32"))]
            FileLoader(c) => c.instantiate(pubsub),
            #[cfg(not(target_arch = "wasm32"))]
            RobotConnection(c) => c.instantiate(pubsub),
            IcpPointMapper(c) => c.instantiate(pubsub),
            Visualizer(c) => c.instantiate(pubsub),
            GridMapSlam(c) => c.instantiate(pubsub),
            GaussianTest(c) => c.instantiate(pubsub),
            Splitter(c) => c.instantiate(pubsub),
            EKFLandmarkSlam(c) => c.instantiate(pubsub),
        }
    }
}

impl Config {
    pub fn from_file(path: &String) -> anyhow::Result<Self> {
        // read file contents
        let contents = fs::read_to_string(path)?;

        Self::from_contents(&contents)
    }

    pub fn from_contents(contents: &str) -> anyhow::Result<Self> {
        serde_yaml::from_str(contents).map_err(|e| anyhow!(e))
    }

    pub fn instantiate_nodes(&self, pubsub: &mut PubSub) -> Vec<Box<dyn Node>> {
        self.nodes
            .iter()
            .map(|config| config.instantiate(pubsub))
            .collect()
    }
}
