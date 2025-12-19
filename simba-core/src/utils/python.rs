use std::ffi::{CStr, CString};
use std::fmt::Debug;
use std::fs;

use log::debug;
use pyo3::call::PyCallArgs;
use pyo3::ffi::c_str;
use pyo3::{PyClass, prelude::*};
use pyo3::{PyResult, Python};
use serde::{Deserialize, Serialize};

use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};
use crate::logger::is_enabled;
use crate::simulator::SimulatorConfig;

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
        if venv.exists()
            && let Some(s) = venv.to_str()
        {
            prefixes.push(s.to_string());
        }
        let envdir = cwd.join(".env");
        if envdir.exists()
            && let Some(s) = envdir.to_str()
        {
            prefixes.push(s.to_string());
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

pub trait PythonClassConfig: Serialize + for<'a> Deserialize<'a> {
    fn file(&self) -> &String;
    fn class_name(&self) -> &String;
}

pub trait PythonFunctionConfig: Serialize + for<'a> Deserialize<'a> {
    fn file(&self) -> &String;
    fn function_name(&self) -> &String;
}

pub struct PythonScriptConfig(CString);

impl PythonScriptConfig {
    pub fn new(script: String) -> Self {
        PythonScriptConfig(CString::new(script).unwrap())
    }

    pub fn script(&self) -> &CString {
        &self.0
    }

    pub fn call<ReturnType, Args>(&mut self, args: Args) -> SimbaResult<ReturnType>
    where
        ReturnType: PyClass + for<'a, 'py> FromPyObject<'a, 'py> + Debug,
        Args: for<'a> PyCallArgs<'a>,
    {
        let res = Python::attach(|py| -> PyResult<ReturnType> {
            ensure_venv_pyo3(py)?;

            let script = PyModule::from_code(py, &self.0, c_str!(""), c_str!(""))?;
            let function: Py<PyAny> = script.getattr("main")?.into();

            let ret = function.call(py, args, None)?;
            ret.extract(py).map_err(|_| {
                PyErr::new::<pyo3::exceptions::PyException, _>(
                    "Error during the call of Python script",
                )
            })
        });

        res.map_err(|err| SimbaError::new(SimbaErrorTypes::PythonError, err.to_string()))
    }
}

pub fn load_class_from_python_script<T: PythonClassConfig>(
    config: &T,
    global_config: &SimulatorConfig,
    initial_time: f32,
    log_info: &str,
) -> SimbaResult<Py<PyAny>> {
    let json_config = serde_json::to_string(&config)
        .unwrap_or_else(|_| format!("Error during converting Python {} config to json", log_info));

    let script_path = global_config.base_path.as_ref().join(config.file());
    let python_script = match fs::read_to_string(script_path.clone()) {
        Err(e) => {
            return Err(SimbaError::new(
                SimbaErrorTypes::ConfigError,
                format!(
                    "Python {log_info} script not found ({}): {}",
                    script_path.to_str().unwrap(),
                    e
                ),
            ));
        }
        Ok(s) => CString::new(s).unwrap(),
    };
    let res = Python::attach(|py| -> PyResult<Py<PyAny>> {
        ensure_venv_pyo3(py)?;

        let script = PyModule::from_code(py, CONVERT_TO_DICT, c_str!(""), c_str!(""))?;
        let convert_fn: Py<PyAny> = script.getattr("convert")?.into();
        let config_dict = convert_fn.call(py, (json_config,), None)?;

        let script = PyModule::from_code(py, &python_script, c_str!(""), c_str!(""))?;
        let class: Py<PyAny> = script.getattr(config.class_name().as_str())?.into();
        log::info!("Load {log_info} class {} ...", config.class_name());

        let res = class.call(py, (config_dict, initial_time), None);
        let instance = match res {
            Err(err) => {
                err.display(py);
                return Err(err);
            }
            Ok(instance) => instance,
        };
        Ok(instance)
    });

    res.map_err(|err| SimbaError::new(SimbaErrorTypes::PythonError, err.to_string()))
}

macro_rules! call_py_method {
    (
        $instance:expr,
        $method_name:expr,
        $result_type:ty,
        $( $args:expr ),*
    ) => {
    Python::attach(|py| -> $result_type {
        match $instance.bind(py).call_method(
            $method_name,
            ( $( $args ),* ),
            None,
        ) {
            Err(e) => {
                e.display(py);
                panic!("Error while calling '{}' method of PythonController.", $method_name);
            }
            Ok(r) => r
                .extract()
                .expect(&format!("Error during the call of Python implementation of '{}'", $method_name)),
        }
    })
    }
}
pub(crate) use call_py_method;

macro_rules! call_py_method_void {
    (
        $instance:expr,
        $method_name:expr,
        $( $args:expr ),*
    ) => {
    Python::attach(|py| {
        if let Err(res) = $instance.bind(py).call_method(
            $method_name,
            ( $( $args ),* ),
            None,
        ) {
            res.display(py);
            panic!("Error while calling '{}' method of PythonController.", $method_name);
        }
    })
    }
}

pub(crate) use call_py_method_void;
