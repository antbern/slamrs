use common::robot::{Observation, Pose};
use egui::Slider;
use graphics::{
    primitiverenderer::{Color, PrimitiveType},
    shaperenderer::ShapeRenderer,
};

pub trait Visualize {
    type Parameters;
    type Secondary;
    fn visualize(
        &self,
        sr: &mut ShapeRenderer,
        config: &Self::Parameters,
        secondary: &Option<Self::Secondary>,
    );
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
    type Secondary = ();

    fn visualize(&self, sr: &mut ShapeRenderer, c: &Self::Parameters, _: &Option<Self::Secondary>) {
        sr.begin(PrimitiveType::Filled);
        sr.arrow(self.x, self.y, self.theta, c.radius, Color::from(c.color));
        sr.end()
    }
}

//////////////// Implementation for Observation /////////////////
pub struct ObservationVisualizeConfig {
    draw_lines: bool,
    size: f32,
    point_color: [f32; 3],
}

impl Default for ObservationVisualizeConfig {
    fn default() -> Self {
        Self {
            draw_lines: true,
            size: 0.01,
            point_color: [0.0, 0.0, 0.0],
        }
    }
}

impl VisualizeParametersUi for ObservationVisualizeConfig {
    fn ui(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Lines: ");
            ui.checkbox(&mut self.draw_lines, "");
        });

        ui.horizontal(|ui| {
            ui.label("Point Size: ");
            ui.add(
                Slider::new(&mut self.size, 0.001..=0.02)
                    .step_by(0.001)
                    .fixed_decimals(3),
            );
        });

        ui.horizontal(|ui| {
            ui.label("Point Color: ");
            ui.color_edit_button_rgb(&mut self.point_color);
        });
    }
}

impl Visualize for Observation {
    type Parameters = ObservationVisualizeConfig;
    type Secondary = Pose;

    fn visualize(
        &self,
        sr: &mut ShapeRenderer,
        c: &Self::Parameters,
        pose: &Option<Self::Secondary>,
    ) {
        let (ox, oy, otheta) = if let Some(p) = pose {
            (p.x, p.y, p.theta)
        } else {
            (0.0, 0.0, 0.0)
        };

        if c.draw_lines {
            sr.begin(PrimitiveType::Line);

            for m in self.measurements.iter() {
                let (s, c) = (m.angle as f32 + otheta).sin_cos();
                let d = m.distance as f32;
                let x = c * d;
                let y = s * d;

                let color = if m.valid { Color::BLACK } else { Color::RED };
                // let color = Color::BLACK;

                sr.line(ox, oy, ox + x, oy + y, color);
            }

            sr.end();
        }

        sr.begin(PrimitiveType::Filled);

        let map_point_size = c.size;
        let color = Color::from(c.point_color);
        for m in self.measurements.iter() {
            let (s, c) = (m.angle as f32 + otheta).sin_cos();
            let d = m.distance as f32;
            let x = c * d;
            let y = s * d;

            // let color = Color::rgb(m.strength as f32 / 2000.0, 0.0, 0.0);
            sr.rect(
                ox + x - map_point_size / 2.0,
                oy + y - map_point_size / 2.0,
                map_point_size,
                map_point_size,
                color,
            )
        }
        sr.end()
    }
}
