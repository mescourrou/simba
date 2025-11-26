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
