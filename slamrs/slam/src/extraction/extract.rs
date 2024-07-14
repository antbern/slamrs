use common::robot::{LandmarkObservation, Observation};

#[derive(Clone, Debug, serde::Deserialize)]
pub struct Config {}

impl Default for Config {
    fn default() -> Self {
        Self {}
    }
}

pub fn extract_landmarks(config: &Config, observation: &Observation) -> Vec<LandmarkObservation> {
    // TODO: implement here!
    vec![LandmarkObservation {
        angle: 45.0_f32.to_radians(),
        distance: 1.0,
        association: None,
    }]
}
