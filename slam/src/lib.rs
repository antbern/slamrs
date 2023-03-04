use common::node::Node;

mod scan_matching;

pub struct SlamNode {}

impl Node for SlamNode {
    fn new(_pubsub: &mut pubsub::PubSub) -> Self
    where
        Self: Sized,
    {
        SlamNode {}
    }

    fn draw(&mut self, ui: &egui::Ui, _world: &mut common::world::WorldObj<'_>) {
        egui::Window::new("Slam").show(ui.ctx(), |ui| {
            ui.label("Slam Stuff");
        });
    }
}
