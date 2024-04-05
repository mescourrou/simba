use std::path::Path;

use pyo3::prelude::*;
use pyo3::prepare_freethreaded_python;

pub fn execute_python_analyser(result_filename: &Path, show_figures: bool, figure_path: &Path, figure_format: String) -> PyResult<()> {
    prepare_freethreaded_python();
    let result_analyser_py = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/python_scripts/analyse_results.py"
    ));
    Python::with_gil(|py| -> PyResult<()> {
        let result_analyser: Py<PyAny> = PyModule::from_code_bound(py, result_analyser_py, "", "")?
        .getattr("analyse")?
        .into();
        result_analyser.call_bound(py, (result_filename.to_str().unwrap(), show_figures, figure_path, figure_format), None)?;
        Ok(())
    })
}