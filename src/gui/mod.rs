#![warn(clippy::all)]

mod app;
pub use app::SimbaApp;

use crate::simulator::Simulator;

pub fn run_gui(log_level: log::LevelFilter) {
    // Initialize the environment, essentially the logging part
    Simulator::init_environment(log_level);
        
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([400.0, 300.0])
            .with_min_inner_size([300.0, 220.0]),
        ..Default::default()
    };
    eframe::run_native(
        "SiMBA",
        native_options,
        Box::new(|cc| Box::new(SimbaApp::new(cc))),
    ).expect("Error during GUI execution");
}
