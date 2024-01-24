use std::sync::Arc;

use common::{
    node::{Node, NodeConfig},
    robot::{LandmarkObservations, Observation, Odometry},
};
use pubsub::{Publisher, Subscription};
use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct SplitterNodeConfig {
    splits: Vec<Split>,
}

trait Splitter {
    fn update(&mut self);
}

/// A splitter that splits one topic into to two
struct OneToTwoSplitter<S: Send + Sync + 'static + Clone, T: Send + Sync + 'static + Clone> {
    input: Subscription<(S, T)>,
    out1: Publisher<S>,
    out2: Publisher<T>,
}

impl<S: Send + Sync + 'static + Clone, T: Send + Sync + 'static + Clone> Splitter
    for OneToTwoSplitter<S, T>
{
    fn update(&mut self) {
        // simply receive and publish the parts separately
        while let Some(data) = self.input.try_recv() {
            self.out1.publish(Arc::new(data.0.clone()));
            self.out2.publish(Arc::new(data.1.clone()));
        }
    }
}
#[derive(Debug, Clone, Deserialize)]
enum Split {
    ScannerOdometry {
        input: String,
        scanner: String,
        odometry: String,
    },
    LandmarkOdometry {
        input: String,
        landmark: String,
        odometry: String,
    },
}

impl Split {
    fn instantiate(&self, pubsub: &mut pubsub::PubSub) -> Box<dyn Splitter> {
        match self {
            Split::ScannerOdometry {
                input,
                scanner,
                odometry,
            } => Box::new(OneToTwoSplitter {
                input: pubsub.subscribe::<(Observation, Odometry)>(&input),
                out1: pubsub.publish(&scanner),
                out2: pubsub.publish(&odometry),
            }),
            Split::LandmarkOdometry {
                input,
                landmark,
                odometry,
            } => Box::new(OneToTwoSplitter {
                input: pubsub.subscribe::<(LandmarkObservations, Odometry)>(&input),
                out1: pubsub.publish(&landmark),
                out2: pubsub.publish(&odometry),
            }),
        }
    }
}

impl NodeConfig for SplitterNodeConfig {
    fn instantiate(&self, pubsub: &mut pubsub::PubSub) -> Box<dyn common::node::Node> {
        Box::new(SplitterNode {
            splitters: self.splits.iter().map(|s| s.instantiate(pubsub)).collect(),
        })
    }
}

pub struct SplitterNode {
    splitters: Vec<Box<dyn Splitter>>,
}

impl Node for SplitterNode {
    fn update(&mut self) {
        for s in &mut self.splitters {
            s.update();
        }
    }
}
