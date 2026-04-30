use std::sync::{Arc, RwLock};

use egui::{
    Align2, Color32, FontId, NumExt, Rect, Resize, ScrollArea, Sense, TextFormat, TextStyle, pos2,
    text::LayoutJob,
};

use crate::{
    context::Context,
    logger::{InternalLog, LogLevel, Logger, LoggerType, StringLogger},
    utils::SharedRwLock,
};

#[derive(Debug)]
struct LogMessage {
    loglevel: LogLevel,
    time: Option<f32>,
    node_name: String,
    callstack: Vec<String>,
    internal_category: Option<InternalLog>,
    message: String,
}

fn egui_format_message(
    message: &LogMessage,
    font_id: &FontId,
    default_color: &Color32,
) -> LayoutJob {
    let mut job = LayoutJob::default();
    job.append(
        "[",
        0.,
        TextFormat::simple(font_id.clone(), default_color.clone()),
    );
    job.append(
        &message.loglevel.to_log_string(false),
        0.,
        TextFormat::simple(
            font_id.clone(),
            match message.loglevel {
                LogLevel::Off => Color32::BLACK,
                LogLevel::Internal(_) => Color32::MAGENTA,
                LogLevel::Error => Color32::RED,
                LogLevel::Warn => Color32::ORANGE,
                LogLevel::Info => Color32::GREEN,
                LogLevel::Debug => Color32::BLUE,
            },
        ),
    );
    job.append(
        "]",
        0.,
        TextFormat::simple(font_id.clone(), default_color.clone()),
    );
    if let Some(time) = message.time {
        job.append(
            &format!("[{:.3}]", time),
            0.,
            TextFormat::simple(font_id.clone(), default_color.clone()),
        );
    }
    job.append(
        "[",
        0.,
        TextFormat::simple(font_id.clone(), default_color.clone()),
    );
    job.append(
        &message.node_name,
        0.,
        TextFormat::simple(font_id.clone(), Color32::CYAN),
    );
    job.append(
        "]",
        0.,
        TextFormat::simple(font_id.clone(), default_color.clone()),
    );
    if let LogLevel::Internal(_) = message.loglevel
        && !message.callstack.is_empty()
    {
        job.append(
            "[",
            0.,
            TextFormat::simple(font_id.clone(), default_color.clone()),
        );
        job.append(
            &message.callstack.join("/"),
            0.,
            TextFormat::simple(font_id.clone(), Color32::MAGENTA),
        );
        job.append(
            "]",
            0.,
            TextFormat::simple(font_id.clone(), default_color.clone()),
        );
    }
    if let Some(internal) = &message.internal_category {
        job.append(
            "[",
            0.,
            TextFormat::simple(font_id.clone(), default_color.clone()),
        );
        job.append(
            &internal.to_string(),
            0.,
            TextFormat::simple(font_id.clone(), Color32::MAGENTA),
        );
        job.append(
            "]",
            0.,
            TextFormat::simple(font_id.clone(), default_color.clone()),
        );
    }
    job.append(
        &format!(" {}", message.message),
        0.,
        TextFormat::simple(font_id.clone(), default_color.clone()),
    );
    job
}

#[derive(Debug)]
struct GUILogger {
    logs: Arc<RwLock<Vec<LogMessage>>>,
}

impl GUILogger {
    fn new(logs: Arc<RwLock<Vec<LogMessage>>>) -> Self {
        Self { logs }
    }
}

impl Logger for GUILogger {
    fn log(
        &mut self,
        loglevel: &crate::logger::LogLevel,
        time: Option<f32>,
        node_name: &String,
        callstack: &Vec<String>,
        internal_category: Option<&crate::logger::InternalLog>,
        message: &str,
    ) {
        let message = LogMessage {
            loglevel: loglevel.clone(),
            time,
            node_name: node_name.clone(),
            callstack: callstack.clone(),
            internal_category: internal_category.cloned(),
            message: message.to_string(),
        };
        self.logs.write().unwrap().push(message);
    }
}

pub struct LogsPanel {
    logs: Arc<RwLock<Vec<LogMessage>>>,
}

impl LogsPanel {
    pub fn new(context: &Context) -> Self {
        let logs = Arc::new(RwLock::new(Vec::new()));
        context.add_write_target(LoggerType::Custom(Box::new(GUILogger::new(logs.clone()))));
        Self { logs }
    }

    pub fn draw(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str, _time: f32) {
        let logs = self.logs.read().unwrap();
        egui::CollapsingHeader::new("Logs").show(ui, |ui| {
            let font_id = TextStyle::Monospace.resolve(ui.style());
            let width = ui.available_width();

            egui::Resize::default()
                .id("log_resize".into())
                .resizable([false, true])
                .min_width(width)
                .max_width(width)
                .min_height(100.0)
                .show(ui, |ui| {
                    ScrollArea::vertical()
                        .auto_shrink(false)
                        .stick_to_bottom(true)
                        .show(ui, |ui| {
                            ui.set_min_width(ui.available_width());
                            for log in logs.iter() {
                                let job =
                                    egui_format_message(log, &font_id, &ui.visuals().text_color());
                                ui.add(egui::Label::new(job).wrap());
                            }
                        });
                });
        });
    }
}
