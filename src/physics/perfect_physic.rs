/*!
Provide the implementation of the [`Physic`] trait without any noise added to the [`Command`].
*/

use crate::gui::utils::state_widget;
use crate::gui::UIComponent;
use crate::networking::service::HasService;
use crate::plugin_api::PluginAPI;
use crate::simulator::SimulatorConfig;
use crate::state_estimators::state_estimator::{State, StateConfig, StateRecord};
use crate::stateful::Stateful;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use config_checker::macros::Check;
use log::error;
use nalgebra::SMatrix;
use serde_derive::{Deserialize, Serialize};

/// Config for the [`PerfectPhysic`].
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct PerfectPhysicConfig {
    /// Distance between the two wheels, to compute the angular velocity from the wheel speeds.
    #[check(ge(0.))]
    pub wheel_distance: f32,
    /// Starting state.
    #[check]
    pub initial_state: StateConfig,
}

#[cfg(feature = "gui")]
impl UIComponent for PerfectPhysicConfig {
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::HashMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        egui::CollapsingHeader::new("Perfect Physics")
            .id_source(format!("perfect-physics-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Wheel distance:");
                    if self.wheel_distance < 0. {
                        self.wheel_distance = 0.;
                    }
                    ui.add(egui::DragValue::new(&mut self.wheel_distance));
                });

                ui.horizontal(|ui| {
                    ui.label("Initial state:");
                    self.initial_state.show(
                        ui,
                        ctx,
                        buffer_stack,
                        global_config,
                        current_node_name,
                        unique_id,
                    );
                });
            });
    }
}

impl Default for PerfectPhysicConfig {
    fn default() -> Self {
        Self {
            wheel_distance: 0.25,
            initial_state: StateConfig::default(),
        }
    }
}

/// Record for the [`PerfectPhysic`].
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PerfectPhysicRecord {
    /// State at the time `last_time_update`
    pub state: StateRecord,
    /// Time of the state
    pub last_time_update: f32,
    /// Current command applied.
    pub current_command: Command,
}

// Services

// /// Request to get the real state of the robot.
// /// (not used yet)
// struct RealStateHandler {
//     pub physics: Box<dyn Physic>,
// }

// impl fmt::Debug for RealStateHandler {
//     fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
//         write!(f, "RealStateHandler {{ physics: ... }}")
//     }
// }

// impl ServiceHandler<GetRealStateReq, GetRealStateResp> for RealStateHandler {
//     fn treat_request(&self, _req: GetRealStateReq, time: f32) -> Result<GetRealStateResp, String> {
//         debug!("Treating request...");
//         let state = self.physics.state(time).clone();
//         debug!("Treating request...OK");
//         Ok(GetRealStateResp { state })
//     }
// }

/// Implementation of [`Physic`] with the command perfectly applied.
#[derive(Debug, Clone)]
pub struct PerfectPhysic {
    /// Distance between the wheels
    wheel_distance: f32,
    /// Current state
    state: State,
    /// Time of the current state.
    last_time_update: f32,
    /// Current command applied.
    current_command: Command,
}

impl PerfectPhysic {
    /// Makes a new [`PerfectPhysic`] situated at (0,0,0) and 25cm between wheels.
    pub fn new() -> Self {
        Self::from_config(
            &PerfectPhysicConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Makes a new [`PerfectPhysic`] with the given configurations.
    ///
    /// ## Arguments
    /// * `config` - Configuration of [`PerfectPhysic`].
    /// * `plugin_api` - [`PluginAPI`] not used there.
    /// * `global_config` - Configuration of the simulator.
    pub fn from_config(
        config: &PerfectPhysicConfig,
        _plugin_api: &Option<Box<&dyn PluginAPI>>,
        _global_config: &SimulatorConfig,
        _va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        PerfectPhysic {
            wheel_distance: config.wheel_distance,
            state: State::from_config(&config.initial_state),
            last_time_update: 0.,
            current_command: Command {
                left_wheel_speed: 0.,
                right_wheel_speed: 0.,
            },
        }
    }

    /// Compute the state to the given `time`, using `self.command`.
    fn compute_state_until(&mut self, time: f32) {
        let dt = time - self.last_time_update;
        assert!(
            dt > 0.,
            "PID delta time should be positive: {} - {} = {} > 0",
            time,
            self.last_time_update,
            dt
        );

        let theta = self.state.pose.z;

        let displacement_wheel_left = self.current_command.left_wheel_speed * dt;
        let displacement_wheel_right = self.current_command.right_wheel_speed * dt;

        let translation = (displacement_wheel_left + displacement_wheel_right) / 2.;
        let rotation = (displacement_wheel_right - displacement_wheel_left) / self.wheel_distance;

        self.last_time_update = time;

        // Using Lie theory
        // Reference: Sola, J., Deray, J., & Atchuthan, D. (2018). A micro lie theory for state estimation in robotics. arXiv preprint arXiv:1812.01537.

        let lie_action =
            SMatrix::<f32, 3, 3>::new(0., -rotation, translation, rotation, 0., 0., 0., 0., 0.);

        let rot_mat = nalgebra::Rotation2::new(theta).matrix().clone();

        let mut se2_mat = SMatrix::<f32, 3, 3>::new(
            rot_mat[(0, 0)],
            rot_mat[(0, 1)],
            self.state.pose.x,
            rot_mat[(1, 0)],
            rot_mat[(1, 1)],
            self.state.pose.y,
            0.,
            0.,
            1.,
        );

        se2_mat = se2_mat * lie_action.exp();

        self.state.pose.z =
            nalgebra::Rotation2::from_matrix(&se2_mat.fixed_view::<2, 2>(0, 0).into()).angle();

        self.state.pose.x = se2_mat[(0, 2)];
        self.state.pose.y = se2_mat[(1, 2)];

        self.state.velocity = translation / dt;
    }
}

use super::physic::{Command, GetRealStateReq, GetRealStateResp};
use super::physic::{Physic, PhysicRecord};

impl Physic for PerfectPhysic {
    /// Apply the given `command` perfectly.
    fn apply_command(&mut self, command: &Command, _time: f32) {
        self.current_command = command.clone();
    }

    /// Compute the state at the given `time`.
    fn update_state(&mut self, time: f32) {
        self.compute_state_until(time);
    }

    /// Return the current state. Do not compute the state again.
    fn state(&self, time: f32) -> &State {
        assert!(time == self.last_time_update);
        &self.state
    }
}

impl HasService<GetRealStateReq, GetRealStateResp> for PerfectPhysic {
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

impl Stateful<PhysicRecord> for PerfectPhysic {
    fn record(&self) -> PhysicRecord {
        PhysicRecord::Perfect(PerfectPhysicRecord {
            state: self.state.record(),
            last_time_update: self.last_time_update,
            current_command: self.current_command.clone(),
        })
    }

    fn from_record(&mut self, record: PhysicRecord) {
        if let PhysicRecord::Perfect(perfect_record) = record {
            self.state.from_record(perfect_record.state);
            self.last_time_update = perfect_record.last_time_update;
            self.current_command = perfect_record.current_command.clone();
        } else {
            error!(
                "Using a PhysicRecord type which does not match the used Physic (PerfectPhysic)"
            );
        }
    }
}
