/*!
Provide the implementation of the [`Physics`] trait.
*/

use std::sync::{Arc, Mutex};

#[cfg(feature = "gui")]
use crate::{gui::UIComponent, simulator::SimulatorConfig};

use crate::{
    networking::service::HasService,
    physics::{
        fault_models::fault_model::{
            PhysicsFaultModel, PhysicsFaultModelConfig, make_physics_fault_model_from_config,
        },
        robot_models::{
            Command, RobotModel, RobotModelConfig, make_model_from_config, unicycle::UnicycleConfig,
        },
    },
    recordable::Recordable,
    state_estimators::{State, StateConfig, StateRecord},
    utils::determinist_random_variable::DeterministRandomVariableFactory,
};
use log::debug;
use nalgebra::Matrix3;
use serde_derive::{Deserialize, Serialize};
use simba_macros::config_derives;

/// Config for the [`InternalPhysics`].
#[config_derives]
pub struct InternalPhysicConfig {
    #[check]
    pub model: RobotModelConfig,
    /// Starting state.
    #[check]
    pub initial_state: StateConfig,
    #[check]
    pub faults: Vec<PhysicsFaultModelConfig>,
}

#[cfg(feature = "gui")]
impl UIComponent for InternalPhysicConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Internal Physics")
            .id_salt(format!("internal-physics-{}", unique_id))
            .show(ui, |ui| {
                self.model.show_mut(
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );

                ui.horizontal(|ui| {
                    ui.label("Initial state:");
                    self.initial_state.show_mut(
                        ui,
                        ctx,
                        buffer_stack,
                        global_config,
                        current_node_name,
                        unique_id,
                    );
                });

                PhysicsFaultModelConfig::show_faults_mut(
                    &mut self.faults,
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );
            });
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new("Internal Physics")
            .id_salt(format!("internal-physics-{}", unique_id))
            .show(ui, |ui| {
                self.model.show(ui, ctx, unique_id);

                ui.horizontal(|ui| {
                    ui.label("Initial state:");
                    self.initial_state.show(ui, ctx, unique_id);
                });

                PhysicsFaultModelConfig::show_faults(&self.faults, ui, ctx, unique_id);
            });
    }
}

impl Default for InternalPhysicConfig {
    fn default() -> Self {
        Self {
            model: RobotModelConfig::Unicycle(UnicycleConfig::default()),
            initial_state: StateConfig::default(),
            faults: Vec::new(),
        }
    }
}

/// Record for the [`InternalPhysics`].
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct InternalPhysicsRecord {
    /// State at the time `last_time_update`
    pub state: StateRecord,
    /// Time of the state
    pub last_time_update: f32,
    /// Current command applied.
    pub current_command: Command,
}

#[cfg(feature = "gui")]
impl UIComponent for InternalPhysicsRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| {
            egui::CollapsingHeader::new("State").show(ui, |ui| {
                self.state.show(ui, ctx, unique_id);
            });

            egui::CollapsingHeader::new("Current command").show(ui, |ui| {
                self.current_command.show(ui, ctx, unique_id);
            });
        });
    }
}

#[derive(Debug)]
pub struct InternalPhysics {
    model: Box<dyn RobotModel>,
    /// Current state
    state: State,
    /// Time of the current state.
    last_time_update: f32,
    /// Current command applied.
    current_command: Command,
    cum_lie_action: Matrix3<f32>,
    faults: Arc<Mutex<Vec<Box<dyn PhysicsFaultModel>>>>,
}

impl InternalPhysics {
    /// Makes a new [`InternalPhysics`] with the given configurations.
    ///
    /// ## Arguments
    /// * `config` - Configuration of [`InternalPhysics`].
    /// * `plugin_api` - [`PluginAPI`] not used there.
    /// * `global_config` - Configuration of the simulator.
    pub fn from_config(
        config: &InternalPhysicConfig,
        robot_name: &String,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> Self {
        let model = make_model_from_config(&config.model);
        let current_command = model.default_command();
        InternalPhysics {
            model,
            state: State::from_config(&config.initial_state, va_factory),
            last_time_update: initial_time,
            current_command,
            faults: Arc::new(Mutex::new(
                config
                    .faults
                    .iter()
                    .map(|f| {
                        make_physics_fault_model_from_config(
                            f,
                            config.model.clone(),
                            robot_name,
                            va_factory,
                            initial_time,
                        )
                    })
                    .collect(),
            )),
            cum_lie_action: Matrix3::zeros(),
        }
    }

    /// Compute the state to the given `time`, using `self.command`.
    fn compute_state_until(&mut self, time: f32) {
        let dt = time - self.last_time_update;
        assert!(
            dt >= 0.,
            "Physics delta time should be positive: {} - {} = {} >= 0",
            time,
            self.last_time_update,
            dt
        );
        if dt == 0. {
            return;
        }

        debug!("InternalPhysics: Computing state until time {}, dt={}", time, dt);
        debug!("  Current state: {:?}", self.state);
        debug!("  Current command: {:?}", self.current_command);

        self.model.update_state(
            &mut self.state,
            &self.current_command,
            &mut self.cum_lie_action,
            dt,
        );

        debug!("  New state: {:?}", self.state);

        self.last_time_update = time;

        for fault in self.faults.lock().unwrap().iter() {
            fault.add_faults(time, &mut self.state);
        }

        debug!("  State after faults: {:?}", self.state);
    }
}

use super::{GetRealStateReq, GetRealStateResp};
use super::{Physics, PhysicsRecord};

impl Physics for InternalPhysics {
    /// Apply the given `command` perfectly.
    fn apply_command(&mut self, command: &Command, _time: f32) {
        self.current_command = command.clone();
    }

    /// Compute the state at the given `time`.
    fn update_state(&mut self, time: f32) {
        self.compute_state_until(time);
    }

    /// Return the current state. Do not compute the state again.
    fn state(&self, time: f32) -> State {
        assert!(time == self.last_time_update);
        self.state.clone()
    }

    fn cummulative_lie_action(&self) -> Option<Matrix3<f32>> {
        Some(self.cum_lie_action)
    }
}

impl HasService<GetRealStateReq, GetRealStateResp> for InternalPhysics {
    fn handle_service_requests(
        &mut self,
        _req: GetRealStateReq,
        time: f32,
    ) -> Result<GetRealStateResp, String> {
        Ok(GetRealStateResp {
            state: self.state(time).clone(),
        })
    }
}

impl Recordable<PhysicsRecord> for InternalPhysics {
    fn record(&self) -> PhysicsRecord {
        PhysicsRecord::Internal(InternalPhysicsRecord {
            state: self.state.record(),
            last_time_update: self.last_time_update,
            current_command: self.current_command.clone(),
        })
    }
}
