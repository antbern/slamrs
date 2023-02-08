use common::{node::Node, world::WorldObj};
use pubsub::PubSub;
pub struct MousePosition {}

impl Node for MousePosition {
    fn new(_pubsub: &mut PubSub) -> Self {
        MousePosition {}
    }

    fn draw(&mut self, ui: &egui::Ui, world: &mut WorldObj<'_>) {
        egui::Window::new("World").show(ui.ctx(), |ui| {
            ui.label(format!(
                "Mouse Position: [{:.2},{:.2}]",
                world.last_mouse_pos.x, world.last_mouse_pos.y
            ));
        });
    }
}
