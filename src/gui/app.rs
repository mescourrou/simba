use std::{sync::{Arc, Mutex}, thread::{self, JoinHandle, Thread}};

use crate::simulator::Simulator;

struct PrivateParams {
    simulator: Arc<Mutex<Simulator>>,
    config_loaded: bool,
    need_reset: bool,
    simulator_thread: Option<std::thread::JoinHandle<()>>,
}

/// We derive Deserialize/Serialize so we can persist app state on shutdown.
#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)] // if we add new fields, give them default values when deserializing old state
pub struct SimbaApp {
    // Example stuff:
    config_path: String,
    duration: f32,
    #[serde(skip)]
    p: PrivateParams,
}

impl Default for SimbaApp {
    fn default() -> Self {
        Self {
            // Example stuff:
            config_path: "".to_owned(),
            duration: 60.,
            p: PrivateParams {
                simulator: Arc::new(Mutex::new(Simulator::new())),
                config_loaded: false,
                need_reset: false,
                simulator_thread: None,
            },
        }
    }
}

impl SimbaApp {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // This is also where you can customize the look and feel of egui using
        // `cc.egui_ctx.set_visuals` and `cc.egui_ctx.set_fonts`.

        // Load previous app state (if any).
        // Note that you must enable the `persistence` feature for this to work.
        if let Some(storage) = cc.storage {
            return eframe::get_value(storage, eframe::APP_KEY).unwrap_or_default();
        }

        Default::default()
    }

    fn run_simulation_threaded(simulator: Arc<Mutex<Simulator>>, duration: f32) {
        simulator.lock().unwrap().run(duration);
    }

    fn run_simulation(&mut self) {
        log::info!("Run simulation for {} seconds", self.duration);
        let simu = Arc::clone(&self.p.simulator);
        let duration = self.duration;
        self.p.simulator_thread = Some(thread::spawn(move || {
            Self::run_simulation_threaded(simu, duration);
        }));
    }
}

impl eframe::App for SimbaApp {
    /// Called by the frame work to save state before shutdown.
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, self);
    }

    /// Called each time the UI needs repainting, which may be many times per second.
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Put your widgets into a `SidePanel`, `TopPanel`, `CentralPanel`, `Window` or `Area`.
        // For inspiration and more examples, go to https://emilk.github.io/egui

        if let Some(join_handle) = &self.p.simulator_thread {
            if join_handle.is_finished() {
                self.p.simulator_thread = None;
            }
        }

        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            // The top panel is often a good place for a menu bar:

            egui::menu::bar(ui, |ui| {
                // NOTE: no File->Quit on web pages!
                let is_web = cfg!(target_arch = "wasm32");
                if !is_web {
                    ui.menu_button("File", |ui| {
                        if ui.button("Quit").clicked() {
                            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                        }
                    });
                    ui.add_space(16.0);
                }

                egui::widgets::global_dark_light_mode_buttons(ui);
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            // The central panel the region left after adding TopPanel's and SidePanel's
            ui.heading("SiMBA: Simulator for Multi-Robot Backend Algorithms");

            ui.horizontal(|ui| {
                ui.label("Config path: ");
                ui.text_edit_singleline(&mut self.config_path);
            });

            ui.horizontal(|ui| {
                ui.label("Duration: ");
                ui.add(egui::DragValue::new(&mut self.duration).speed(0.1));
            });

            if ui.button("Load").clicked() {
                log::info!("Load configuration");
                self.p.simulator = Arc::new(Mutex::new(Simulator::from_config_path(
                    std::path::Path::new(&self.config_path),
                    None,
                )));
                self.p.config_loaded = true;
            }

            ui.horizontal(|ui|{
                if ui.add_enabled(self.p.config_loaded && self.p.simulator_thread.is_none(), egui::Button::new("Run")).clicked() {
                    if self.p.need_reset {
                        log::info!("Load configuration");
                        self.p.simulator = Arc::new(Mutex::new(Simulator::from_config_path(
                            std::path::Path::new(&self.config_path),
                            None,
                        )));
                    }
                    self.run_simulation();
                    self.p.need_reset = true;
                }
                if self.p.simulator_thread.is_some() {
                    ui.add(egui::Spinner::new());
                }
            });

            ui.separator();

            ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
                powered_by_egui_and_eframe(ui);
                egui::warn_if_debug_build(ui);
            });
        });
    }
}

fn powered_by_egui_and_eframe(ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 0.0;
        ui.label("Powered by ");
        ui.hyperlink_to("egui", "https://github.com/emilk/egui");
        ui.label(" and ");
        ui.hyperlink_to(
            "eframe",
            "https://github.com/emilk/egui/tree/master/crates/eframe",
        );
        ui.label(".");
    });
}
