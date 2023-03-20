use common::robot::Pose;
use egui::Slider;
use graphics::{
    primitiverenderer::{Color, PrimitiveType},
    shaperenderer::ShapeRenderer,
};

pub trait Visualize {
    type Parameters;
    fn visualize(&self, sr: &mut ShapeRenderer, config: &Self::Parameters);
}

pub trait VisualizeParametersUi {
    fn ui(&mut self, ui: &mut egui::Ui);
}

//////////////// Implementation for Pose /////////////////
pub struct PoseVisualizeConfig {
    color: [f32; 3],
    radius: f32,
}

impl Default for PoseVisualizeConfig {
    fn default() -> Self {
        Self {
            color: [0.0, 1.0, 0.0],
            radius: 0.1,
        }
    }
}

impl VisualizeParametersUi for PoseVisualizeConfig {
    fn ui(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Color: ");
            ui.color_edit_button_rgb(&mut self.color);
        });

        ui.horizontal(|ui| {
            ui.label("Radius: ");
            ui.add(
                Slider::new(&mut self.radius, 0.01..=0.2)
                    .step_by(0.01)
                    .fixed_decimals(2),
            );
        });
    }
}

impl Visualize for Pose {
    type Parameters = PoseVisualizeConfig;

    fn visualize(&self, sr: &mut ShapeRenderer, c: &Self::Parameters) {
        sr.begin(PrimitiveType::Filled);
        sr.arrow(self.x, self.y, self.theta, c.radius, Color::from(c.color));
        sr.end()
    }
}
