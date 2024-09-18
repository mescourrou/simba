/*!
Provide function to analyse the results.
*/

use std::path::Path;

use pyo3::prelude::*;
use pyo3::prepare_freethreaded_python;

/// Run the python script `python_scripts/analyse_results.py`.
///
/// It calls the function `analyse` in the python file.
///
/// ## Arguments
/// * `result_filename` -- Filename of the result log (produced by [`Simulator`](crate::simulator::Simulator)).
/// * `show_figures` -- Python argument to show matplotlib figures (need a GUI).
/// * `figure_path` -- Directory where to save the figures.
/// * `figure_format` -- Figure extension, e.g. ".pdf".
///
/// ## Return
/// Return the Result of the python execution (Ok or PyErr).
pub fn execute_python_analyser(
    result_filename: &Path,
    show_figures: bool,
    figure_path: &Path,
    figure_format: String,
) -> PyResult<()> {
    prepare_freethreaded_python();
    let result_analyser_py = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/python_scripts/analyse_results.py"
    ));
    Python::with_gil(|py| -> PyResult<()> {
        let result_analyser: Py<PyAny> = PyModule::from_code_bound(py, result_analyser_py, "", "")?
            .getattr("analyse")?
            .into();
        result_analyser.call_bound(
            py,
            (
                result_filename.to_str().unwrap(),
                show_figures,
                figure_path,
                figure_format,
            ),
            None,
        )?;
        Ok(())
    })
}

pub fn run_python(python_code: &str, py: &Python, config_record: &Py<PyAny>, show_figures: bool, figure_path: &Path, figure_format: &str) -> PyResult<()> {
    let result_analyser: Py<PyAny> = PyModule::from_code_bound(*py, python_code, "", "")?
    .getattr("analyse")?
    .into();
    result_analyser.call_bound(
        *py,
        (
            config_record.as_any(),
            show_figures,
            figure_path,
            figure_format,
        ),
        None,
    )?;
    Ok(())
}

pub trait Analysable {
    
}