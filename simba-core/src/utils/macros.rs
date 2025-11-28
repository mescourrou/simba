macro_rules! python_class_config {
    (
        $(#[$meta:meta])*  // Capture attributes including doc comments
        $struct_name:ident,
        $title:expr,
        $unique_key:expr
    ) => {
$(#[$meta])*  // Re-emit all attributes, including doc
#[derive(serde::Serialize, serde::Deserialize, Debug, Clone, config_checker::macros::Check)]
#[serde(default)]
pub struct $struct_name {
    file: String,
    class_name: String,
    /// Config serialized.
    #[serde(flatten)]
    pub config: serde_json::Value,
}

impl Default for $struct_name {
    fn default() -> Self {
        Self {
            file: String::new(),
            class_name: String::new(),
            config: serde_json::Value::Null,
        }
    }
}

impl crate::utils::python::PythonClassConfig for $struct_name {
    fn file(&self) -> &String {
        &self.file
    }

    fn class_name(&self) -> &String {
        &self.class_name
    }
}

#[cfg(feature = "gui")]
impl crate::gui::UIComponent for $struct_name {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &crate::simulator::SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new($title).show(ui, |ui| {
            ui.vertical(|ui| {
                use crate::gui::utils::json_config;

                ui.horizontal(|ui| {
                    ui.label("Script path: ");
                    ui.text_edit_singleline(&mut self.file);
                });

                ui.horizontal(|ui| {
                    ui.label("Class name: ");
                    ui.text_edit_singleline(&mut self.class_name);
                });

                ui.label("Config (JSON):");
                json_config(
                    ui,
                    &format!("{}-key-{}", $unique_key, &unique_id),
                    &format!("{}-error-key-{}", $unique_key, &unique_id),
                    buffer_stack,
                    &mut self.config,
                );
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        egui::CollapsingHeader::new($title).show(ui, |ui| {
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    ui.label("Script path: ");
                    ui.label(&self.file);
                });
                ui.horizontal(|ui| {
                    ui.label("Class name: ");
                    ui.label(&self.class_name);
                });
                ui.label("Config (JSON):");
                ui.label(self.config.to_string());
            });
        });
    }
}
    };
}

pub(crate) use python_class_config;

macro_rules! python_fn_config {
    (
        $(#[$meta:meta])*  // Capture attributes including doc comments
        $struct_name:ident,
        $title:expr,
        $unique_key:expr
    ) => {
$(#[$meta])*  // Re-emit all attributes, including doc
#[derive(serde::Serialize, serde::Deserialize, Debug, Clone, config_checker::macros::Check)]
#[serde(default)]
pub struct $struct_name {
    file: String,
    function_name: String,
}

impl Default for $struct_name {
    fn default() -> Self {
        Self {
            file: String::new(),
            function_name: String::new(),
        }
    }
}

impl crate::utils::python::PythonFunctionConfig for $struct_name {
    fn file(&self) -> &String {
        &self.file
    }

    fn function_name(&self) -> &String {
        &self.function_name
    }
}

#[cfg(feature = "gui")]
impl crate::gui::UIComponent for $struct_name {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &crate::simulator::SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new($title).show(ui, |ui| {
            ui.vertical(|ui| {
                use crate::gui::utils::json_config;

                ui.horizontal(|ui| {
                    ui.label("Script path: ");
                    ui.text_edit_singleline(&mut self.file);
                });

                ui.horizontal(|ui| {
                    ui.label("Function name: ");
                    ui.text_edit_singleline(&mut self.function_name);
                });
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        egui::CollapsingHeader::new($title).show(ui, |ui| {
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    ui.label("Script path: ");
                    ui.label(&self.file);
                });
                ui.horizontal(|ui| {
                    ui.label("Function name: ");
                    ui.label(&self.function_name);
                });
            });
        });
    }
}
    };
}

pub(crate) use python_fn_config;

macro_rules! external_record {
    (
        $(#[$meta:meta])*  // Capture attributes including doc comments
        $struct_name:ident,
    ) => {
$(#[$meta])*
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct $struct_name {
    /// Record serialized.
    #[serde(flatten)]
    pub record: Value,
}

impl Default for $struct_name {
    fn default() -> Self {
        Self {
            record: Value::Null,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for $struct_name {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.label(self.record.to_string());
    }
}
    };
}

pub(crate) use external_record;

macro_rules! external_record_python_methods {
    (
        $(#[$meta:meta])*  // Capture attributes including doc comments
        $struct_name:ident,
    ) => {
crate::utils::macros::external_record!(
    $(#[$meta])*
    #[pyclass]
    $struct_name,
);

#[pymethods]
impl $struct_name {
    #[getter]
    fn record(&self) -> String {
        self.record.to_string()
    }
}

    };
}

pub(crate) use external_record_python_methods;
