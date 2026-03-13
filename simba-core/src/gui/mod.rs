//! Graphical user interface components and utilities.
//! 
//! This module contains the implementation of the GUI for Simba, built using the `egui` library. It provides a 
//! main application struct [`SimbaApp`] that manages the state and rendering of the GUI, as well as various 
//! components and utilities to display and edit simulator configurations and records.

mod app;

use std::{collections::BTreeMap, path::Path, sync::Arc};

pub use app::SimbaApp;
mod configurator;
mod drawables;
pub use drawables::Drawable;
mod panels;
pub mod utils;

use crate::{
    plugin_api::PluginAPI,
    simulator::{Simulator, SimulatorConfig},
};

/// Function to run the GUI of Simba. Can be called in the main function.
/// 
/// # Arguments
/// * `default_config_path` - The default path to the configuration file. If some is given, the GUI will start with the configuration loaded from this path. 
/// * `plugin_api` - The plugin API to load external modules. Required to load external modules of the simulator.
/// * `load_results` - Whether to load simulation results at startup. The `default_config_path` must be given to load the results, as the results path is loaded from the configuration file. If `default_config_path` is not given, this argument is ignored.
pub fn run_gui(
    default_config_path: Option<&'static Path>,
    plugin_api: Option<Arc<dyn PluginAPI>>,
    load_results: bool,
) {
    // Initialize the environment, essentially the logging part
    Simulator::init_environment();

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([400.0, 300.0])
            .with_min_inner_size([300.0, 220.0]),
        ..Default::default()
    };

    eframe::run_native(
        "SiMBA",
        native_options,
        Box::new(|cc| {
            Ok(Box::new(SimbaApp::new(
                cc,
                default_config_path,
                plugin_api,
                load_results,
            )))
        }),
    )
    .expect("Error during GUI execution");
}

/// Trait to allow a struct to be displayed in the GUI. Is is essentially used to display Configuration and records.
pub trait UIComponent {
    /// Function to edit the component in the GUI.
    #[allow(unused_variables)]
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        unimplemented!("Mutable UIComponent not implemented.");
    }

    /// Function to only display the component in the GUI, without editing it.
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str);
}
