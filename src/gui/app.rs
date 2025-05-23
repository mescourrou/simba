use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use crate::{
    api::async_api::{AsyncApi, AsyncApiRunner},
    plugin_api::PluginAPI,
};

struct PrivateParams {
    server: Arc<Mutex<AsyncApiRunner>>,
    api: AsyncApi,
    config_loaded: bool,
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
        let server = Arc::new(Mutex::new(AsyncApiRunner::new()));
        let api = server.lock().unwrap().get_api();
        server.lock().unwrap().run(None);
        Self {
            config_path: "".to_owned(),
            duration: 60.,
            p: PrivateParams {
                server,
                api,
                config_loaded: false,
            },
        }
    }
}

impl SimbaApp {
    /// Called once before the first frame.
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        plugin_api: Option<Box<&'static dyn PluginAPI>>,
    ) -> Self {
        // This is also where you can customize the look and feel of egui using
        // `cc.egui_ctx.set_visuals` and `cc.egui_ctx.set_fonts`.

        // Load previous app state (if any).
        // Note that you must enable the `persistence` feature for this to work.
        if let Some(storage) = cc.storage {
            return eframe::get_value(storage, eframe::APP_KEY)
                .unwrap_or_else(|| Self::new_full(plugin_api));
        }

        Self::new_full(plugin_api)
    }

    fn new_full(plugin_api: Option<Box<&'static dyn PluginAPI>>) -> Self {
        let server = Arc::new(Mutex::new(AsyncApiRunner::new()));
        let api = server.lock().unwrap().get_api();
        server.lock().unwrap().run(plugin_api);
        Self {
            config_path: "".to_owned(),
            duration: 60.,
            p: PrivateParams {
                server,
                api,
                config_loaded: false,
            },
        }
    }

    fn quit(&mut self) {
        self.p.server.lock().unwrap().stop();
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

        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            // The top panel is often a good place for a menu bar:

            egui::menu::bar(ui, |ui| {
                // NOTE: no File->Quit on web pages!
                let is_web = cfg!(target_arch = "wasm32");
                if !is_web {
                    ui.menu_button("File", |ui| {
                        if ui.button("Quit").clicked() {
                            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                            self.quit();
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
                self.p
                    .api
                    .load_config
                    .async_call(self.config_path.clone());
                self.p.config_loaded = true;
            }

            ui.horizontal(|ui| {
                if ui
                    .add_enabled(self.p.config_loaded, egui::Button::new("Run"))
                    .clicked()
                {
                    log::info!("Run simulation");
                    self.p.api.run.async_call(Some(self.duration));
                }
                ui.vertical(|ui| {
                    for (robot, time) in
                        self.p.api.simulator_api.current_time.lock().unwrap().iter()
                    {
                        ui.label(format!("Running: {robot}: {time}",));
                    }
                })
            });
            if ui
                .add_enabled(self.p.config_loaded, egui::Button::new("Results"))
                .clicked()
            {
                log::info!("Analysing results");
                self.p.api.compute_results.async_call(());
            }

            ui.separator();

            ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
                powered_by_egui_and_eframe(ui);
                egui::warn_if_debug_build(ui);
            });
        });

        ctx.request_repaint_after(Duration::from_secs_f32(1.0 / 60.0));
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
