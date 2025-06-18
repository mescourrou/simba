#![warn(clippy::all)]

mod app;

use std::collections::BTreeMap;

pub use app::SimbaApp;
mod configurator;
mod drawables;
pub mod utils;


use crate::{
    plugin_api::PluginAPI,
    simulator::{Simulator, SimulatorConfig},
};

pub fn run_gui(plugin_api: Option<Box<&'static dyn PluginAPI>>) {
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
        Box::new(|cc| Box::new(SimbaApp::new(cc, plugin_api))),
    )
    .expect("Error during GUI execution");
}

pub trait UIComponent {
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    );
}
