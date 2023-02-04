use crate::{app::WorldRenderer, pubsub::PubSub};

pub mod mouse_position;
pub mod shape_rendering;

/// A Node is an entity that can publish and react to subscibed messages as well as draw itself.
///
/// It can perform processing in the background (using `threads`) or perhaps utilize an `async` runtime
/// to do IO-bound interactions such as via the Network or over Serial.
pub trait Node {
    /// Constructs a new Node object. This should also subscribe or request permission to
    /// publish via the Publish/Subscribe mechanism.
    fn new(pubsub: &mut PubSub) -> Self
    where
        Self: Sized;

    /// Draws the UI of the Node as well as any geometries that go into the `World`
    /// (and later also the `Scene`).
    fn draw(&mut self, _ui: &egui::Ui, _world: &mut WorldRenderer) {}

    /// Called when the Node should terminate. Terminate background threads etc. here.
    fn terminate(&mut self) {}
}
