use std::{
    collections::BTreeMap,
    path::Path,
    sync::{Arc, Mutex},
    time::{self, Duration},
};

use egui::{Align2, Color32, Id, Pos2, Rect, Response, Sense, Shape, Vec2};
use serde::{Deserialize, Serialize};

use crate::{
    api::async_api::{AsyncApi, AsyncApiRunner},
    constants::TIME_ROUND_DECIMALS,
    errors::SimbaError,
    gui::{drawables::popup::Popup, UIComponent},
    node_factory::NodeRecord,
    plugin_api::PluginAPI,
    simulator::{Record, SimulatorConfig},
    AUTHORS, VERSION,
};

use super::{
    configurator::Configurator,
    drawables::{self},
};

pub struct PainterInfo {
    pub top_left: Pos2,
    pub bottom_right: Pos2,
    pub shift: Pos2,
}

impl Default for PainterInfo {
    fn default() -> Self {
        Self {
            top_left: Pos2 { x: -10., y: -10. },
            bottom_right: Pos2 { x: 10., y: 10. },
            shift: Pos2 { x: 0., y: 0. },
        }
    }
}

impl PainterInfo {
    pub fn zero(&self, scale: f32) -> Pos2 {
        Pos2 {
            x: -self.top_left.x * scale + self.shift.x,
            y: -self.top_left.y * scale + self.shift.y,
        }
    }

    pub fn include_point(&mut self, pt: Vec2) {
        if pt.x < self.top_left.x {
            // Left of paint area
            self.top_left.x = pt.x * 1.1;
        } else if pt.x > self.bottom_right.x {
            // Right of paint area
            self.bottom_right.x = pt.x * 1.1;
        }

        if pt.y < self.top_left.y {
            // Above paint area
            self.top_left.y = pt.y * 1.1;
        } else if pt.y > self.bottom_right.y {
            // Bottom of paint area
            self.bottom_right.y = pt.y * 1.1;
        }
    }

    pub fn size(&self) -> Vec2 {
        Vec2 {
            x: self.bottom_right.x - self.top_left.x,
            y: self.bottom_right.y - self.top_left.y,
        }
    }

    pub fn is_inside(&self, pt: &Vec2) -> bool {
        if pt.x < self.top_left.x || pt.x > self.bottom_right.x {
            return false;
        }

        if pt.y < self.top_left.y || pt.y > self.bottom_right.y {
            return false;
        }

        true
    }

    pub fn set_shift(&mut self, shift: Pos2) {
        self.shift = shift;
    }

    pub fn rect_painter(&self, scale: f32) -> Rect {
        let size = self.size();
        Rect {
            min: self.shift,
            max: self.shift
                + Vec2 {
                    x: size.x * scale,
                    y: size.y * scale,
                },
        }
    }

    pub fn is_position_clicked(
        &self,
        response_click: Option<Pos2>,
        scale: f32,
        position: Vec2,
    ) -> bool {
        if let Some(click_pos) = response_click {
            let position = self.zero(scale) + position * scale;
            let dist = (click_pos - position).length();
            if dist < 10. {
                return true;
            }
        }
        return false;
    }
}

struct PrivateParams {
    server: Arc<Mutex<AsyncApiRunner>>,
    api: AsyncApi,
    config: Option<SimulatorConfig>,
    current_draw_time: f32,
    robots: BTreeMap<String, drawables::robot::Robot>,
    playing: Option<(f32, std::time::Instant)>,
    simulation_run: bool,
    configurator: Option<Configurator>,
    error_buffer: Vec<(time::Instant, SimbaError)>,
    painter_info: PainterInfo,
    popups: Vec<Popup>,
}

impl Default for PrivateParams {
    fn default() -> Self {
        let server = Arc::new(Mutex::new(AsyncApiRunner::new()));
        let api = server.lock().unwrap().get_api();
        server.lock().unwrap().run(None);
        Self {
            server,
            api,
            config: None,
            current_draw_time: 0.,
            robots: BTreeMap::new(),
            playing: None,
            simulation_run: false,
            configurator: None,
            error_buffer: Vec::new(),
            painter_info: PainterInfo::default(),
            popups: Vec::new(),
        }
    }
}

/// We derive Deserialize/Serialize so we can persist app state on shutdown.
#[derive(Deserialize, Serialize)]
#[serde(default)] // if we add new fields, give them default values when deserializing old state
pub struct SimbaApp {
    // Example stuff:
    config_path: String,
    duration: f32,
    #[serde(skip_serializing, skip_deserializing)]
    p: PrivateParams,
    drawing_scale: f32,
    follow_sim_time: bool,
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
                ..Default::default()
            },
            drawing_scale: 100.,
            follow_sim_time: true,
        }
    }
}

impl SimbaApp {
    /// Called once before the first frame.
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        default_config_path: Option<Box<&'static Path>>,
        plugin_api: Option<Box<&'static dyn PluginAPI>>,
    ) -> Self {
        // This is also where you can customize the look and feel of egui using
        // `cc.egui_ctx.set_visuals` and `cc.egui_ctx.set_fonts`.

        // // Load previous app state (if any).
        // // Note that you must enable the `persistence` feature for this to work.
        if let Some(storage) = cc.storage {
            if let Some(app) = eframe::get_value::<SimbaApp>(storage, eframe::APP_KEY) {
                app.update_api(default_config_path, plugin_api)
            } else {
                Self::new_full(default_config_path, plugin_api)
            }
        } else {
            Self::new_full(default_config_path, plugin_api)
        }
    }

    fn new_full(
        default_config_path: Option<Box<&'static Path>>,
        plugin_api: Option<Box<&'static dyn PluginAPI>>,
    ) -> Self {
        let server = Arc::new(Mutex::new(AsyncApiRunner::new()));
        let api = server.lock().unwrap().get_api();
        server.lock().unwrap().run(plugin_api);
        let mut n = Self {
            p: PrivateParams {
                server,
                api,
                ..Default::default()
            },
            ..Default::default()
        };
        if let Some(config) = default_config_path {
            n.config_path = config.to_str().unwrap().to_string();
            n.p.config = None;
            n.p.api.load_config.async_call(n.config_path.clone());
        }
        n
    }

    fn update_api(
        mut self,
        default_config_path: Option<Box<&'static Path>>,
        plugin_api: Option<Box<&'static dyn PluginAPI>>,
    ) -> Self {
        let server = Arc::new(Mutex::new(AsyncApiRunner::new()));
        let api = server.lock().unwrap().get_api();
        server.lock().unwrap().run(plugin_api);
        self.p.server = server;
        self.p.api = api;
        if let Some(config) = default_config_path {
            self.config_path = config.to_str().unwrap().to_string();
            self.p.config = None;
            self.p.api.load_config.async_call(self.config_path.clone());
        }
        self
    }

    fn quit(&mut self) {
        self.p.server.lock().unwrap().stop();
    }

    fn init_drawables(&mut self) {
        if self.p.config.is_none() {
            return;
        }
        let config = self.p.config.as_ref().unwrap();
        for robot in &config.robots {
            self.p.robots.insert(
                robot.name.clone(),
                drawables::robot::Robot::init(robot, config),
            );
        }
    }

    fn draw(&mut self, ui: &mut egui::Ui, viewport: Rect) -> Result<Vec<Shape>, Vec2> {
        let mut shapes = Vec::new();
        for (_, robot) in &self.p.robots {
            shapes.extend(robot.draw(
                ui,
                &viewport,
                &self.p.painter_info,
                self.drawing_scale,
                self.p.current_draw_time,
            )?);
        }
        Ok(shapes)
    }

    fn react(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        _viewport: Rect,
        response: &Response,
    ) {
        for (_, robot) in &mut self.p.robots {
            robot.react(
                ui,
                ctx,
                response,
                &self.p.painter_info,
                self.drawing_scale,
                self.p.current_draw_time,
            );
        }
    }
}

impl eframe::App for SimbaApp {
    /// Called by the frame work to save state before shutdown.
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, self);
    }

    /// Called each time the UI needs repainting, which may be many times per second.
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        for Record { time, node } in self.p.api.simulator_api.records.lock().unwrap().try_iter() {
            match &node {
                NodeRecord::ComputationUnit(_) => {}
                NodeRecord::Robot(n) => {
                    if let Some(r) = self.p.robots.get_mut(&n.name) {
                        r.add_record(time, n.clone());
                    }
                }
            }
        }

        let mut to_close = Vec::new();
        for (i, pup) in self.p.popups.iter_mut().enumerate() {
            pup.draw(ctx);
            if pup.is_clicked() {
                to_close.push(i);
            }
        }
        for i in to_close.iter().rev() {
            self.p.popups.remove(*i);
        }

        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            // The top panel is often a good place for a menu bar:

            egui::MenuBar::new().ui(ui, |ui| {
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
                    ui.menu_button("Help", |ui| {
                        if ui.button("Version").clicked() {
                            self.p.popups.push(Popup::new_ok(
                                "Version".to_string(),
                                format!("SiMBA\nVersion {}\nGPLv3\n{}", VERSION, AUTHORS),
                                |_| {},
                            ));
                        }
                    });
                    ui.add_space(16.0);
                }

                egui::widgets::global_theme_preference_switch(ui);
            });
            ui.heading("SiMBA: Simulator for Multi-Robot Backend Algorithms");

            ui.horizontal(|ui| {
                ui.label("Config path: ");
                ui.text_edit_singleline(&mut self.config_path);

                if ui.button("Load").clicked() {
                    log::info!("Load configuration");
                    self.p.config = None;
                    self.p.api.load_config.async_call(self.config_path.clone());
                }
                if self.p.config.is_none() {
                    if let Some(res) = self.p.api.load_config.try_get_result() {
                        match res {
                            Err(e) => {
                                let now = time::Instant::now();
                                self.p.error_buffer.push((now, e));
                            }
                            Ok(c) => {
                                self.p.config = Some(c);
                                self.init_drawables();
                            }
                        }
                    }
                }
                if ui.button("Configurator").clicked() {
                    self.p.configurator = Some(Configurator::init(&self.config_path));
                }
                if let Some(configurator) = &mut self.p.configurator {
                    if configurator.show(ui, ctx) {
                        //Closing
                        self.p.configurator = None;
                    }
                }
            });

            ui.horizontal(|ui| {
                ui.label("Duration: ");
                ui.add(egui::DragValue::new(&mut self.duration).speed(0.1));
            });
        });

        egui::TopBottomPanel::bottom("bottom_panel").show(ctx, |ui| {
            ui.vertical(|ui| {
                ui.add(
                    egui::Slider::new(&mut self.drawing_scale, 1.0..=1000.)
                        .text("Zoom")
                        .show_value(false),
                );
                // Mouse wheel for zoom
                ctx.input(|i| {
                    if i.modifiers.ctrl {
                        self.drawing_scale *= (i.zoom_delta() * i.zoom_delta() - 1.) / 50. + 1.;
                    }
                });

                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(self.p.config.is_some(), egui::Button::new("Run"))
                        .clicked()
                    {
                        log::info!("Run simulation");
                        self.p.api.run.async_call(Some(self.duration));
                        self.p.simulation_run = true;
                    }
                    if let Some(r) = self.p.api.run.try_get_result() {
                        if let Err(e) = r {
                            self.p.error_buffer.push((time::Instant::now(), e));
                        }
                    }
                    let max_simulated_time = *self.p.api.simulator_api.current_time.read().unwrap();
                    let play_pause_btn = if self.p.playing.is_none() {
                        egui::Button::new("Play ")
                    } else {
                        egui::Button::new("Pause")
                    };
                    if ui
                        .add_enabled(self.p.simulation_run, play_pause_btn)
                        .clicked()
                    {
                        if self.p.playing.is_some() {
                            // Press Pause
                            self.p.playing = None;
                        } else {
                            self.p.playing =
                                Some((self.p.current_draw_time, std::time::Instant::now()));
                            self.follow_sim_time = false;
                        }
                    }
                    if let Some((begin_sim_time, begin_sys_time)) = self.p.playing {
                        self.p.current_draw_time = begin_sim_time
                            + (std::time::Instant::now() - begin_sys_time).as_secs_f32();
                    }
                    // Set ALL slider size
                    ui.style_mut().spacing.slider_width = ui.available_width() - 180.;
                    if self.p.current_draw_time > max_simulated_time || self.follow_sim_time {
                        self.p.current_draw_time = max_simulated_time;
                    }
                    ui.add(
                        egui::Slider::new(&mut self.p.current_draw_time, 0.0..=self.duration)
                            .fixed_decimals(TIME_ROUND_DECIMALS),
                    );
                    ui.add(egui::Checkbox::new(&mut self.follow_sim_time, "Follow"));
                    if ui
                        .add_enabled(self.p.config.is_some(), egui::Button::new("Results"))
                        .clicked()
                    {
                        log::info!("Analysing results");
                        self.p.api.compute_results.async_call(());
                    }
                    if let Some(r) = self.p.api.compute_results.try_get_result() {
                        if let Err(e) = r {
                            self.p.error_buffer.push((time::Instant::now(), e));
                        }
                    }
                });

                ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
                    egui::warn_if_debug_build(ui);
                });
            });
        });

        egui::SidePanel::right("right-panel").show(ctx, |ui| {
            egui::CollapsingHeader::new("Configuration").show(ui, |ui| {
                egui::ScrollArea::both().show(ui, |ui| {
                    if let Some(cfg) = &self.p.config {
                        let unique_id = String::new();
                        cfg.show(ui, ctx, &unique_id);
                    }
                });
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            egui::ScrollArea::both()
                .auto_shrink([false; 2])
                .scroll_source(egui::scroll_area::ScrollSource {
                    scroll_bar: true,
                    drag: true,
                    mouse_wheel: true,
                })
                .scroll_bar_visibility(egui::scroll_area::ScrollBarVisibility::AlwaysVisible)
                .show_viewport(ui, |ui, viewport| {
                    let mut shapes = Vec::new();
                    loop {
                        let rect = self.p.painter_info.rect_painter(self.drawing_scale);

                        // Draw grid
                        shapes.push(Shape::rect_stroke(
                            rect,
                            0.0,
                            (1.0, Color32::LIGHT_GRAY),
                            egui::StrokeKind::Middle,
                        ));
                        let x_min = rect.left(); //.max(viewport.left());
                        let x_max = rect.right(); //.min(viewport.right());
                        let y_min = rect.top(); //.max(viewport.top());
                        let y_max = rect.bottom(); //.min(viewport.bottom());
                        let zero = self.p.painter_info.zero(self.drawing_scale);
                        let mut x = zero.x;
                        while x > x_min {
                            shapes.push(Shape::vline(
                                x,
                                rect.y_range(),
                                (1.0, Color32::LIGHT_GRAY),
                            ));
                            x -= 1. * self.drawing_scale;
                        }
                        x = zero.x;
                        while x < x_max {
                            shapes.push(Shape::vline(
                                x,
                                rect.y_range(),
                                (1.0, Color32::LIGHT_GRAY),
                            ));
                            x += 1. * self.drawing_scale;
                        }
                        let mut y = zero.y;
                        while y > y_min {
                            shapes.push(Shape::hline(
                                rect.x_range(),
                                y,
                                (1.0, Color32::LIGHT_GRAY),
                            ));
                            y -= 1. * self.drawing_scale;
                        }
                        y = zero.y;
                        while y < y_max {
                            shapes.push(Shape::hline(
                                rect.x_range(),
                                y,
                                (1.0, Color32::LIGHT_GRAY),
                            ));
                            y += 1. * self.drawing_scale;
                        }

                        shapes.push(Shape::vline(zero.x, rect.y_range(), (1.0, Color32::GREEN)));
                        shapes.push(Shape::hline(rect.x_range(), zero.y, (1.0, Color32::RED)));

                        match self.draw(ui, viewport) {
                            Err(request_pos) => {
                                self.p.painter_info.include_point(request_pos);
                            }
                            Ok(s) => {
                                shapes.extend(s);
                                break;
                            }
                        }
                        shapes.clear();
                    }

                    let size = self.p.painter_info.size() * self.drawing_scale;
                    let (response, painter) = ui.allocate_painter(size, Sense::click());
                    painter.extend(shapes);

                    self.p.painter_info.set_shift(response.rect.left_top());

                    self.react(ui, ctx, viewport, &response);
                });
        });

        let mut to_remove = Vec::new();
        let mut curr_y_shift = 0.;
        for (i, (inst, e)) in self.p.error_buffer.iter_mut().enumerate().rev() {
            if egui::Window::new("Error")
                .id(Id::new(format!("error {i}")))
                .anchor(Align2::RIGHT_TOP, [-50., curr_y_shift])
                .fixed_size([500., 50.])
                .hscroll(true)
                .interactable(true)
                .title_bar(false)
                .show(ctx, |ui| {
                    ui.colored_label(Color32::RED, e.detailed_error());
                })
                .unwrap()
                .response
                .contains_pointer()
            {
                *inst = time::Instant::now();
            }
            curr_y_shift += 60.;
            if inst.elapsed().as_secs_f32() > 5. {
                to_remove.push(i);
            }
        }
        for i in to_remove.iter() {
            self.p.error_buffer.remove(*i);
        }

        ctx.request_repaint_after(Duration::from_secs_f32(1.0 / 30.0));
    }
}
