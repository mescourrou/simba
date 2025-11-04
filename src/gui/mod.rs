#![warn(clippy::all)]

mod app;

use std::{collections::BTreeMap, path::Path};

pub use app::SimbaApp;
mod configurator;
mod drawables;
pub mod utils;

use crate::{
    plugin_api::PluginAPI,
    simulator::{Simulator, SimulatorConfig},
};

pub fn run_gui(
    default_config_path: Option<Box<&'static Path>>,
    plugin_api: Option<Box<&dyn PluginAPI>>,
) {
    // Initialize the environment, essentially the logging part
    Simulator::init_environment();

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([400.0, 300.0])
            .with_min_inner_size([300.0, 220.0]),
        ..Default::default()
    };

    // UNSAFE !! But relatively safe as run_native does not exit while GUI is running.
    // When GUI is leaving, the simulator which uses plugin_api is expected to be closed.
    let static_plugin_api: Option<Box<&'static dyn PluginAPI>> =
        unsafe { std::mem::transmute(plugin_api) };
    eframe::run_native(
        "SiMBA",
        native_options,
        Box::new(|cc| {
            Ok(Box::new(SimbaApp::new(
                cc,
                default_config_path,
                static_plugin_api,
            )))
        }),
    )
    .expect("Error during GUI execution");
}
pub trait UIComponent {
    fn show_mut(
        &mut self,
        _ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _buffer_stack: &mut BTreeMap<String, String>,
        _global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        _unique_id: &String,
    ) {
        unimplemented!("Mutable UIComponent not implemented.");
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &String);
    //  {
}
