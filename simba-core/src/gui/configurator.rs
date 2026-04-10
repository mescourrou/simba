use std::{collections::BTreeMap, path::Path};

use crate::{context::Context, error, simulator::SimulatorConfig};

use super::{UIComponent, utils::path_finder};

pub struct Configurator {
    current_config: SimulatorConfig,
    save_path: String,
    buffer: BTreeMap<String, String>,
}

impl Configurator {
    /// Initialize a configurator from an existing simulator config file path.
    ///
    /// If loading fails, a default configuration is used and the error is logged.
    ///
    /// ## Arguments
    /// * `config_path` - Path to the config file to preload.
    /// * `context` - Shared simulation context used for error logging.
    pub fn init(config_path: &String, context: &Context) -> Self {
        let mut save_path = config_path.clone();
        let current_config = match SimulatorConfig::load_from_path(Path::new(&config_path)) {
            Ok(config) => config,
            Err(e) => {
                error!(context,
                    "Impossible to load config at path {}: {}",
                    config_path,
                    e.detailed_error()
                );
                save_path = String::new();
                SimulatorConfig::default()
            }
        };
        Configurator {
            current_config,
            save_path,
            buffer: BTreeMap::new(),
        }
    }

    /// Show the interactive configurator window.
    ///
    /// Returns `true` when the window requests closing.
    pub fn show(&mut self, _ui: &mut egui::Ui, ctx: &egui::Context) -> bool {
        let mut closing = false;
        egui::Window::new("Configurator").show(ctx, |ui| {
            if ui.button("Close").clicked() {
                closing = true;
                return;
            }
            ui.horizontal(|ui| {
                ui.label("Save to: ");
                path_finder(ui, &mut self.save_path, Path::new("."));
                if ui.button("Save").clicked() {
                    confy::store_path(&self.save_path, &self.current_config).unwrap();
                }
                if ui.button("Reset to default").clicked() {
                    self.current_config = SimulatorConfig::default();
                }
            });
            self.current_config.base_path = Box::from(Path::new(&self.save_path));

            egui::ScrollArea::vertical().show(ui, |ui| {
                let unique_id = String::new();
                self.current_config.show_mut(
                    ui,
                    ctx,
                    &mut self.buffer,
                    &self.current_config.clone(),
                    None,
                    &unique_id,
                );
            });
        });
        closing
    }
}
