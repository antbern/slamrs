use common::node::Node;

pub struct Simulator {}

impl Node for Simulator {
    fn new(_pubsub: &mut pubsub::PubSub) -> Self
    where
        Self: Sized,
    {
        Self {}
    }

    fn draw(&mut self, ui: &egui::Ui, _world: &mut common::world::WorldObj<'_>) {
        egui::Window::new("Simulator").show(ui.ctx(), |ui| {
            ui.label("Used to simulate different LIDAR sensors and environment shapes.");
        });
    }

    fn terminate(&mut self) {}
}
