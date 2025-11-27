use std::ffi::CStr;

use log::debug;
use pyo3::prelude::*;
use pyo3::{PyResult, Python};

use crate::logger::is_enabled;

/// Ensure that the Python virtual environment's site-packages are included in sys.path.
/// This is useful when the Rust application embeds Python and needs to access packages
/// installed in a virtual environment.
///
/// WARNING: Does not support Windows paths yet.
/// 
/// Arguments:
/// * `py` - The Python GIL token.
/// 
/// Returns:
/// * `PyResult<()>` - Ok if successful, or an error if something went wrong
pub fn ensure_venv_pyo3(py: Python<'_>) -> PyResult<()> {
    // Ensure Python can find installed modules (like simba)
    // This augments sys.path to include virtual environment site-packages across
    // common managers: venv/virtualenv/poetry/pipenv/hatch (VIRTUAL_ENV) and conda (CONDA_PREFIX).
    use std::env;
    use std::path::Path;

    let sys = py.import("sys")?;

    let major: i32 = sys.getattr("version_info")?.getattr("major")?.extract()?;
    let minor: i32 = sys.getattr("version_info")?.getattr("minor")?.extract()?;

    // Gather candidate prefixes to look for site-packages
    let mut prefixes: Vec<String> = Vec::new();
    if let Ok(p) = env::var("VIRTUAL_ENV") {
        prefixes.push(p);
    }
    if let Ok(p) = env::var("CONDA_PREFIX") {
        prefixes.push(p);
    }
    // Heuristic: local .venv or .env in CWD (useful when not "activated")
    if let Ok(cwd) = env::current_dir() {
        let venv = cwd.join(".venv");
        if venv.exists() {
            if let Some(s) = venv.to_str() {
                prefixes.push(s.to_string());
            }
        }
        let envdir = cwd.join(".env");
        if envdir.exists() {
            if let Some(s) = envdir.to_str() {
                prefixes.push(s.to_string());
            }
        }
    }

    // Build candidate site-packages directories (Linux/macOS layout)
    let mut candidates: Vec<String> = prefixes
        .into_iter()
        .map(|prefix| format!("{}/lib/python{}.{}/site-packages", prefix, major, minor))
        .collect();

    // Deduplicate while preserving order
    use std::collections::HashSet;
    let mut seen: HashSet<String> = HashSet::new();
    candidates.retain(|p| seen.insert(p.clone()));

    // Add any existing candidate via site.addsitedir (honors .pth files)
    let site = py.import("site")?;
    let sys_path_vec: Vec<String> = sys.getattr("path")?.extract()?;
    for cand in candidates {
        if Path::new(&cand).exists() && !sys_path_vec.iter().any(|p| p == &cand) {
            // Use addsitedir for proper processing of .pth files
            site.call_method1("addsitedir", (cand.as_str(),))?;
        }
    }

    if is_enabled(crate::logger::InternalLog::API) {
        let path: Vec<String> = sys.getattr("path")?.extract()?;
        debug!("Python sys.path after env injection: {:?}", path);
    }
    Ok(())
}


pub const CONVERT_TO_DICT: &CStr = cr#"
import json
class NoneDict(dict):
    """ dict subclass that returns a value of None for missing keys instead
        of raising a KeyError. Note: doesn't add item to dictionary.
    """
    def __missing__(self, key):
        return None


def converter(decoded_dict):
    """ Convert any None values in decoded dict into empty NoneDict's. """
    return {k: NoneDict() if v is None else v for k,v in decoded_dict.items()}

def convert(records):
    return json.loads(records, object_hook=converter)
"#;

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