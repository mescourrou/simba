use std::{
    collections::HashMap,
    fmt::Debug,
    sync::{Arc, Condvar, Mutex, RwLock},
};

use crate::{
    physics::physic::{GetRealStateReq, GetRealStateResp, Physic},
    robot::Robot,
    simulator::TimeCvData,
    state_estimators::state_estimator::State,
};

use super::{
    network::MessageFlag,
    service::{Service, ServiceClient, ServiceInterface},
};

#[derive(Debug, Clone)]
pub struct ServiceManager {
    get_real_state: Arc<RwLock<Service<GetRealStateReq, GetRealStateResp, dyn Physic>>>,
    get_real_state_clients: HashMap<String, ServiceClient<GetRealStateReq, GetRealStateResp>>,
}

impl ServiceManager {
    pub fn initialize(robot: &Robot, time_cv: Arc<(Mutex<TimeCvData>, Condvar)>) -> Self {
        Self {
            get_real_state: Arc::new(RwLock::new(Service::new(time_cv.clone(), robot.physics()))),
            get_real_state_clients: HashMap::new(),
        }
    }

    fn get_real_state_client(
        &self,
        robot_name: &str,
    ) -> ServiceClient<GetRealStateReq, GetRealStateResp> {
        self.get_real_state.write().unwrap().new_client(robot_name)
    }

    pub fn get_real_state(&self, robot_name: String, robot: &Robot, time: f32) -> State {
        let client = self.get_real_state_clients.get(&robot_name).unwrap();
        client
            .send_request(
                robot.name(),
                GetRealStateReq {},
                time,
                vec![MessageFlag::God],
            )
            .unwrap();

        let resp;
        loop {
            if let Ok(r) = client.try_recv() {
                resp = r;
                break;
            }
            if self.process_requests() > 0 {
                self.handle_requests(time);
            }
        }

        resp.state
    }

    pub fn make_links(
        &mut self,
        service_managers: &HashMap<String, Arc<RwLock<ServiceManager>>>,
        robot: &Robot,
    ) {
        let my_name = robot.name();
        for (name, sm) in service_managers {
            if name == &my_name {
                continue;
            }
            self.get_real_state_clients.insert(
                name.clone(),
                sm.read().unwrap().get_real_state_client(&my_name),
            );
        }
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
