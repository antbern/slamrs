use crate::world::WorldObj;
use eframe::egui;
use pubsub::PubSub;

/// A Node is an entity that can publish and react to subscibed messages as well as draw itself.
///
/// It can perform processing in the background (using `threads`) or perhaps utilize an `async` runtime
/// to do IO-bound interactions such as via the Network or over Serial.
pub trait Node {
    /// Allows the Node to update itself and perform logic. Note that this is still called
    /// on the rendering thread and as such should be kept brief.
    fn update(&mut self) {}

    /// Draws the UI of the Node as well as any geometries that go into the `World`
    /// (and later also the `Scene`).
    /// Note: No logic update should happen here since it might not be called if running in headless state.
    fn draw(&mut self, _ui: &egui::Ui, _world: &mut WorldObj<'_>) {}

    /// Called when the Node should terminate. Terminate background threads etc. here.
    fn terminate(&mut self) {}
}

pub trait NodeConfig {
    /// Constructs a new Node object. This should also subscribe or request permission to
    /// publish via the Publish/Subscribe mechanism.
    fn instantiate(&self, pubsub: &mut PubSub) -> Box<dyn Node>;
}
