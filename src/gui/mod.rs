#![warn(clippy::all)]

mod app;
pub use app::TurtlebotSimulatorApp;

pub fn run_gui() {
    env_logger::init(); // Log to stderr (if you run with `RUST_LOG=debug`).

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([400.0, 300.0])
            .with_min_inner_size([300.0, 220.0]),
        ..Default::default()
    };
    let _ = eframe::run_native(
        "Turtlebot Simulator",
        native_options,
        Box::new(|cc| Box::new(TurtlebotSimulatorApp::new(cc))),
    );
}
