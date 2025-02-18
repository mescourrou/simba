/*!
Provide the implementation of the [`Physic`] trait without any noise added to the [`Command`].
*/

use std::fmt;
use std::sync::{Arc, Condvar, Mutex, RwLock};

use crate::networking::service::{HasService, Service, ServiceClient, ServiceHandler};
use crate::plugin_api::PluginAPI;
use crate::robot::Robot;
use crate::simulator::SimulatorConfig;
use crate::state_estimators::state_estimator::{State, StateConfig, StateRecord};
use crate::stateful::Stateful;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use config_checker::macros::Check;
use log::{debug, error};
use pyo3::pyclass;
use serde_derive::{Deserialize, Serialize};

/// Config for the [`PerfectPhysic`].
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
pub struct PerfectPhysicConfig {
    /// Distance between the two wheels, to compute the angular velocity from the wheel speeds.
    #[check(ge(0.))]
    pub wheel_distance: f32,
    /// Starting state.
    #[check]
    pub initial_state: StateConfig,
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
#[pyclass(get_all)]
pub struct PerfectPhysicRecord {
    /// State at the time `last_time_update`
    pub state: StateRecord,
    /// Time of the state
    pub last_time_update: f32,
    /// Current command applied.
    pub current_command: Command,
}

// Services

/// Request to get the real state of the robot.
/// (not used yet)
struct RealStateHandler {
    pub physics: Box<dyn Physic>,
}

impl fmt::Debug for RealStateHandler {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "RealStateHandler {{ physics: ... }}")
    }
}

impl ServiceHandler<GetRealStateReq, GetRealStateResp> for RealStateHandler {
    fn treat_request(&self, _req: GetRealStateReq, time: f32) -> Result<GetRealStateResp, String> {
        debug!("Treating request...");
        let state = self.physics.state(time).clone();
        debug!("Treating request...OK");
        Ok(GetRealStateResp { state })
    }
}

/// Implementation of [`Physic`] with the command perfectly applied.
#[derive(Debug)]
pub struct PerfectPhysic {
    /// Distance between the wheels
    wheel_distance: f32,
    /// Current state
    state: State,
    /// Time of the current state.
    last_time_update: f32,
    /// Current command applied.
    current_command: Command,
    /// Service to get the real state of the robot.
    real_state_service: Service<GetRealStateReq, GetRealStateResp>,
    /// [`ServiceHandler`] for the real state service (not used yet)
    real_state_handler: Option<Box<dyn ServiceHandler<GetRealStateReq, GetRealStateResp>>>,
}

impl PerfectPhysic {
    /// Makes a new [`PerfectPhysic`] situated at (0,0,0) and 25cm between wheels.
    pub fn new(time_cv: Arc<(Mutex<usize>, Condvar)>) -> Self {
        Self::from_config(
            &PerfectPhysicConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &DeterministRandomVariableFactory::default(),
            time_cv,
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
        time_cv: Arc<(Mutex<usize>, Condvar)>,
    ) -> Self {
        PerfectPhysic {
            wheel_distance: config.wheel_distance,
            state: State::from_config(&config.initial_state),
            last_time_update: 0.,
            current_command: Command {
                left_wheel_speed: 0.,
                right_wheel_speed: 0.,
            },
            real_state_service: Service::new(time_cv),
            real_state_handler: None,
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

        self.state.pose.x += translation * (theta + rotation / 2.).cos();
        self.state.pose.y += translation * (theta + rotation / 2.).sin();
        self.state.pose.z += rotation;

        self.state.velocity = translation / dt;
        self.state = self.state.clone().theta_modulo();
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
    fn state(&self, _time: f32) -> &State {
        &self.state
    }
}

impl HasService<GetRealStateReq, GetRealStateResp> for PerfectPhysic {
    fn make_service(&mut self, _robot: Arc<RwLock<Robot>>) {
        debug!("Making service");
        debug!("Service made");
    }

    fn new_client(
        &mut self,
        client_name: &str,
    ) -> ServiceClient<GetRealStateReq, GetRealStateResp> {
        self.real_state_service.new_client(client_name)
    }

    fn process_service_requests(&self) -> usize {
        self.real_state_service.process_requests()
    }

    fn handle_service_requests(&mut self, time: f32) {
        self.real_state_service
            .handle_service_requests(time, &|msg, t| {
                Ok(GetRealStateResp {
                    state: self.state(t).clone(),
                })
            });
    }

    fn service_next_time(&self) -> (f32, bool) {
        self.real_state_service.next_time()
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
