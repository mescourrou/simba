use std::{
    collections::BTreeMap,
    fmt::Debug,
    sync::{Arc, RwLock},
};


use crate::{
    node::Node,
    physics::physics::{GetRealStateReq, GetRealStateResp, Physics},
    simulator::TimeCv,
    state_estimators::state_estimator::State,
};

use super::{
    network::MessageFlag,
    service::{Service, ServiceClient, ServiceInterface},
};

#[derive(Debug, Clone)]
pub struct ServiceManager {
    get_real_state: Option<Arc<RwLock<Service<GetRealStateReq, GetRealStateResp, dyn Physics>>>>,
    get_real_state_clients: BTreeMap<String, ServiceClient<GetRealStateReq, GetRealStateResp>>,
}

impl ServiceManager {
    pub fn initialize(node: &Node, time_cv: Arc<TimeCv>) -> Self {
        Self {
            get_real_state: match node.node_type.has_physics() {
                true => Some(Arc::new(RwLock::new(Service::new(
                    time_cv.clone(),
                    node.physics().unwrap(),
                )))),
                false => None,
            },
            get_real_state_clients: BTreeMap::new(),
        }
    }

    fn get_real_state_client(
        &self,
        client_node_name: &str,
    ) -> Option<ServiceClient<GetRealStateReq, GetRealStateResp>> {
        if let Some(get_real_state) = &self.get_real_state {
            Some(get_real_state.write().unwrap().new_client(client_node_name))
        } else {
            None
        }
    }

    pub fn get_real_state(&self, node_name: &String, node: &Node, time: f32) -> Option<State> {
        let client = self.get_real_state_clients.get(node_name);
        if client.is_none() {
            return None;
        }
        let client = client.unwrap();
        client
            .send_request(
                node.name(),
                GetRealStateReq {},
                time,
                vec![MessageFlag::God],
            )
            .unwrap();

        // Next loop to avoid deadlock between services
        let resp;
        loop {
            if let Ok(r) = client.try_recv() {
                // Time_cv circulating_messages already decreased in client
                resp = r;
                break;
            }
            if self.process_requests() > 0 {
                self.handle_requests(time);
            }
        }

        Some(resp.state)
    }

    pub fn make_links(
        &mut self,
        service_managers: &BTreeMap<String, Arc<RwLock<ServiceManager>>>,
        node: &Node,
    ) {
        let my_name = node.name();
        for (name, sm) in service_managers {
            if name == &my_name {
                continue;
            }
            if let Some(client) = sm.read().unwrap().get_real_state_client(&my_name) {
                self.get_real_state_clients.insert(name.clone(), client);
            }
        }
    }

    pub fn handle_requests(&self, time: f32) {
        if let Some(get_real_state) = &self.get_real_state {
            get_real_state.read().unwrap().handle_requests(time);
        }
    }

    pub fn process_requests(&self) -> usize {
        let mut s = 0usize;
        if let Some(get_real_state) = &self.get_real_state {
            s += get_real_state.read().unwrap().process_requests();
        }
        s
    }

    pub fn next_time(&self) -> (f32, bool) {
        let mut min_time = (f32::INFINITY, false);
        if let Some(get_real_state) = &self.get_real_state {
            let mt = get_real_state.read().unwrap().next_time();
            if mt.0 < min_time.0 {
                min_time = mt;
            }
        }
        // Place for new services
        min_time
    }
}
