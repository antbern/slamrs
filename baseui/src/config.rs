use std::fs;

use anyhow::anyhow;
use common::node::Node;
use pubsub::PubSub;
use serde::Deserialize;
use simulator::SimulatorNodeConfig;

#[derive(Deserialize, Default)]
pub struct Config {
    pub settings: Settings,

    pub nodes: Vec<NodeConfig>,
}

#[derive(Deserialize, Default)]
pub struct Settings {
    headless: bool,
}

#[derive(Deserialize)]
pub enum NodeConfig {
    Simulator(SimulatorNodeConfig),
    // IcpPointCould(IcpPointCouldConfig),
}

impl NodeConfig {
    fn instantiate(&self, pubsub: &mut PubSub) -> Box<dyn Node> {
        use NodeConfig::*;
        match self {
            Simulator(c) => c.instantiate(pubsub),
            // IcpPointCould(c) => c.instantiate(pubsub),
        }
    }
}

impl Config {
    pub fn from_file(path: &String) -> anyhow::Result<Self> {
        // read file contents
        let contents = fs::read_to_string(path)?;

        serde_yaml::from_str(&contents).map_err(|e| anyhow!(e))
    }

    pub fn instantiate_nodes(&self, pubsub: &mut PubSub) -> Vec<Box<dyn Node>> {
        self.nodes
            .iter()
            .map(|config| config.instantiate(pubsub))
            .collect()
    }
}
