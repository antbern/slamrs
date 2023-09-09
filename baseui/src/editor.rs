use crate::config::Config;

pub struct ConfigEditor {
    selected: u32,
    source: String,
    confirm_open: bool,
    presets: Vec<(&'static str, &'static str)>,

    parsed_config: Option<serde_yaml::Result<Config>>,
}

impl ConfigEditor {
    pub fn new() -> Self {
        let mut s = Self {
            selected: 0,
            source: String::from(
                r#"
settings:
  headless: false

nodes:

- !MousePosition
- !ShapeTest
"#,
            ),
            confirm_open: false,
            presets: vec![("grid_slam", include_str!("../../config/grid_slam.yaml"))],
            parsed_config: None,
        };
        s.parse_source();
        s
    }

    fn parse_source(&mut self) {
        self.parsed_config = Some(serde_yaml::from_str::<Config>(&self.source));
    }

    pub fn draw(&mut self, ui: &mut egui::Ui) -> Option<Config> {
        // draw the config editor window
        //
        let mut result: Option<Config> = None;

        // egui::Window::new("Config Editor")
        //     .open(&mut self.open)
        //     .constrain(true)
        //     .show(ui.ctx(), |ui| {
        ui.set_enabled(!self.confirm_open);

        ui.horizontal(|ui| {
            egui::ComboBox::from_label("Preset")
                .selected_text(format!("{}", self.presets[self.selected as usize].0))
                .show_ui(ui, |ui| {
                    for (i, (name, _)) in self.presets.iter().enumerate() {
                        ui.selectable_value(&mut self.selected, i as u32, *name);
                    }
                });
            if ui.button("Load").clicked() {
                // TODO: replace content with any of the presets
                self.confirm_open = true;
            }
        });

        // show the result of trying to convert it to a valid config object
        if let Some(parsed_config) = &self.parsed_config {
            match parsed_config {
                Ok(c) => {
                    ui.label(format!("OK: {} nodes", c.nodes.len()));
                }
                Err(e) => {
                    ui.label(format!("ERR:\n{}", e));
                }
            }
        } else {
            ui.label("Start Typing");
        }

        if let Some(Ok(c)) = &self.parsed_config {
            if ui.button("Apply").clicked() {
                result = Some(c.to_owned());
            }
        }

        if self.confirm_open {
            egui::Window::new("Are you sure?").show(ui.ctx(), |ui| {
                ui.label(
                    "Loading a preset will replace your current config. \nDo you want to continue?",
                );
                ui.horizontal(|ui| {
                    if ui.button("Load").clicked() {
                        self.source = self.presets[self.selected as usize].1.into();
                        self.confirm_open = false;

                        // also parse the loaded config
                        self.parse_source();
                    }

                    if ui.button("Cancel").clicked() {
                        self.confirm_open = false;
                    }
                });
            });
        }
        let response = egui::ScrollArea::vertical()
            .id_source("source")
            // .max_height(ui.available_height())
            .show(ui, |ui| {
                ui.add(
                    egui::TextEdit::multiline(&mut self.source)
                        .desired_width(f32::INFINITY)
                        .code_editor(),
                )
            })
            .inner;

        if response.changed() {
            self.parse_source();
        }

        result
    }
}
