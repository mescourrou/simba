use std::{fmt::Debug, sync::{Arc, Condvar, Mutex, RwLock}};


use crate::{physics::physic::{GetRealStateReq, GetRealStateResp, Physic}, robot::Robot};

use super::service::{Service, ServiceClient, ServiceInterface};


#[derive(Debug, Clone)]
pub struct ServiceManager {
    get_real_state: Arc<RwLock<Service<GetRealStateReq, GetRealStateResp, dyn Physic>>>,
}

impl ServiceManager {
    pub fn initialize(robot: Arc<RwLock<Robot>>, time_cv: Arc<(Mutex<usize>, Condvar)>) -> Self {
        let open_robot= robot.read().unwrap();
        Self {
            get_real_state: Arc::new(RwLock::new(Service::new(time_cv.clone(), open_robot.physics()))),
        }
    }

    pub fn get_real_state_client(&self, robot_name: &str) -> ServiceClient<GetRealStateReq, GetRealStateResp> {
        self.get_real_state.write().unwrap().new_client(robot_name)
    }

    pub fn handle_requests(&self, time: f32) {
        self.get_real_state.read().unwrap().handle_requests(time);
    }

    pub fn process_requests(&self) -> usize {
        let mut s = 0usize;
        s += self.get_real_state.read().unwrap().process_requests();
        s
    }

    pub fn next_time(&self) -> (f32, bool) {
        let mut min_time = (f32::INFINITY, false);
        let mt = self.get_real_state.read().unwrap().next_time();
        if mt.0 < min_time.0 {
            min_time = mt;
        }
        min_time
    }
}
