use common::{
    node::{Node, NodeConfig},
    world::WorldObj,
};
use pubsub::PubSub;
use serde::Deserialize;
pub struct MousePosition {}
#[derive(Deserialize)]
pub struct MousePositionNodeConfig {}

impl NodeConfig for MousePositionNodeConfig {
    fn instantiate(&self, _pubsub: &mut PubSub) -> Box<dyn Node> {
        Box::new(MousePosition {})
    }
}

impl Node for MousePosition {
    fn draw(&mut self, ui: &egui::Ui, world: &mut WorldObj<'_>) {
        egui::Window::new("World").show(ui.ctx(), |ui| {
            ui.label(format!(
                "Mouse Position: [{:.2},{:.2}]",
                world.last_mouse_pos.x, world.last_mouse_pos.y
            ));
        });
    }
}
