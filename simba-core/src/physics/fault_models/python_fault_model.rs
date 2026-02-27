use pyo3::prelude::*;

use crate::{
    errors::SimbaResult,
    physics::fault_models::fault_model::PhysicsFaultModel,
    pywrappers::{NodeWrapper, StateWrapper},
    simulator::SimulatorConfig,
    state_estimators::State,
    utils::{
        macros::python_class_config,
        python::{call_py_method, call_py_method_void, load_class_from_python_script},
    },
};

python_class_config!(
    PythonPhysicsFaultModelConfig,
    "Python Physics Fault Model",
    "python-physics-fault-model"
);

#[derive(Debug)]
pub struct PythonPhysicsFaultModel {
    instance: Py<PyAny>,
}

impl PythonPhysicsFaultModel {
    pub fn from_config(
        config: &PythonPhysicsFaultModelConfig,
        global_config: &SimulatorConfig,
        initial_time: f32,
    ) -> SimbaResult<Self> {
        let instance = load_class_from_python_script(
            config,
            global_config,
            initial_time,
            "Physics Fault Model",
        )?;
        Ok(Self { instance })
    }
}

impl PhysicsFaultModel for PythonPhysicsFaultModel {
    fn post_init(&mut self, node: &mut crate::node::Node) -> SimbaResult<()> {
        call_py_method_void!(self.instance, "post_init", (NodeWrapper::from_rust(node),));
        Ok(())
    }

    fn add_faults(&self, time: f32, state: &mut State) {
        let py_state = StateWrapper::from_rust(state);
        let new_state = call_py_method!(self.instance, "add_faults", StateWrapper, time, py_state);
        *state = new_state.to_rust();
    }
}
