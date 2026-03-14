//! Service orchestration layer for node-to-node request/response communication.
//!
//! This module centralizes service registration, link creation between nodes, and service
//! progression during simulation time. It currently manages the internal `get_real_state`
//! service, which allows one node to request another node's real physical state.

use std::{
    collections::BTreeMap,
    fmt::Debug,
    sync::{Arc, RwLock},
};

use crate::{
    errors::{SimbaError, SimbaErrorTypes, SimbaResult},
    node::Node,
    physics::{GetRealStateReq, GetRealStateResp, Physics},
    simulator::TimeCv,
    state_estimators::State,
    utils::SharedRwLock,
};

use super::service::{Service, ServiceClient, ServiceInterface};

/// Errors raised by the service management layer.
#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub enum ServiceError {
    /// The service channel was closed.
    Closed,
    /// The requested service is not available on the target node.
    Unavailable,
    /// An unknown service error represented by a free-form message.
    Other(String),
    /// The error happened on the client side of a service exchange.
    ClientSide,
}

/// Owns services exposed by a node and service clients connected to other nodes.
///
/// A [`ServiceManager`] is attached to a node and is responsible for:
/// - exposing local services,
/// - creating clients toward remote services,
/// - processing pending requests and replies,
/// - providing timing hints for the next service-related event.
#[derive(Debug, Clone)]
pub struct ServiceManager {
    get_real_state: Option<SharedRwLock<Service<GetRealStateReq, GetRealStateResp, dyn Physics>>>,
    get_real_state_clients: BTreeMap<String, ServiceClient<GetRealStateReq, GetRealStateResp>>,
}

impl ServiceManager {
    /// Creates a new [`ServiceManager`] for a node.
    ///
    /// If the node supports physics, this registers the local `get_real_state` service,
    /// otherwise no local service is exposed.
    pub(crate) fn initialize(node: &Node, time_cv: Arc<TimeCv>) -> Self {
        Self {
            get_real_state: match node.node_type().has_physics() {
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
    ) -> SimbaResult<ServiceClient<GetRealStateReq, GetRealStateResp>> {
        if let Some(get_real_state) = &self.get_real_state {
            Ok(get_real_state.write().unwrap().new_client(client_node_name))
        } else {
            Err(SimbaError::new(
                SimbaErrorTypes::ServiceError(ServiceError::Unavailable),
                "No service `get_real_state` available.".to_string(),
            ))
        }
    }

    /// Requests the real state of `node_name` at simulation `time`.
    ///
    /// This method sends a [`GetRealStateReq`] through the corresponding
    /// [`ServiceClient`], then waits for a [`GetRealStateResp`]. While waiting, it may process
    /// local pending requests to avoid service deadlocks.
    pub fn get_real_state(&self, node_name: &String, node: &Node, time: f32) -> SimbaResult<State> {
        let client = self.get_real_state_clients.get(node_name);
        if client.is_none() {
            return Err(SimbaError::new(
                SimbaErrorTypes::ServiceError(ServiceError::Unavailable),
                format!("No service `get_real_state` found for node {node_name}."),
            ));
        }
        let client = client.unwrap();
        client.send_request(node.name(), GetRealStateReq {}, time)?;

        // Next loop to avoid deadlock between services
        let resp;
        loop {
            match client.try_recv() {
                Ok(r) => {
                    // Time_cv circulating_messages already decreased in client
                    resp = r;
                    break;
                }
                Err(e) => {
                    if let SimbaErrorTypes::ServiceError(ServiceError::Closed) = e.error_type() {
                        return Err(e);
                    }
                }
            }
            if self.process_requests() > 0 {
                self.handle_requests(time);
            }
        }

        Ok(resp.state)
    }

    /// Builds service-client links to all other nodes' exposed services.
    ///
    /// For each entry in `service_managers` except the current node, this attempts to create a
    /// client for the remote `get_real_state` service and stores it in this manager.
    pub fn make_links(
        &mut self,
        service_managers: &BTreeMap<String, SharedRwLock<ServiceManager>>,
        node: &Node,
    ) {
        let my_name = node.name();
        for (name, sm) in service_managers {
            if name == &my_name {
                continue;
            }
            if let Ok(client) = sm.read().unwrap().get_real_state_client(&my_name) {
                self.get_real_state_clients.insert(name.clone(), client);
            }
        }
    }

    /// Handles all currently queued incoming service requests at `time`.
    pub fn handle_requests(&self, time: f32) {
        if let Some(get_real_state) = &self.get_real_state {
            get_real_state.read().unwrap().handle_requests(time);
        }
    }

    /// Processes completed requests and returns how many were handled.
    pub fn process_requests(&self) -> usize {
        let mut s = 0usize;
        if let Some(get_real_state) = &self.get_real_state {
            s += get_real_state.read().unwrap().process_requests();
        }
        s
    }

    /// Returns the next simulation time at which a local service needs processing.
    ///
    /// Returns [`f32::INFINITY`] when no local service is registered.
    pub fn next_time(&self) -> f32 {
        let mut min_time = f32::INFINITY;
        if let Some(get_real_state) = &self.get_real_state {
            let mt = get_real_state.read().unwrap().next_time();
            if mt < min_time {
                min_time = mt;
            }
        }
        // Place for new services
        min_time
    }

    /// Unsubscribes and deletes all local service channels for this node.
    pub fn unsubscribe_node(&self) {
        if let Some(get_real_state) = &self.get_real_state {
            get_real_state.write().unwrap().delete();
        }
    }
}
