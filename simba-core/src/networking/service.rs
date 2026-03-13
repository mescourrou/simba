//! Request/response service primitives for node-to-node communication.
//!
//! This module implements two-way communication where a client node sends a request and later
//! receives a response from a server node. A server exposes a [`Service`], while clients use a
//! [`ServiceClient`] created by that service.
//!
//! This design keeps message ordering consistent with simulation time.
//!
//! A service is parameterized by two payload types:
//! - a request type sent by the client;
//! - a response type produced by the server.

use core::f32;
use std::{
    collections::BTreeMap,
    fmt::Debug,
    sync::{Arc, Mutex, RwLock, mpsc},
};

use log::debug;
use simba_com::time_ordered_data::TimeOrderedData;

use crate::{
    constants::TIME_ROUND,
    errors::{SimbaError, SimbaErrorTypes},
    logger::is_enabled,
    networking::service_manager::ServiceError,
    simulator::TimeCv,
    utils::{SharedMutex, SharedRwLock},
};

use super::network::MessageFlag;

/// Common runtime interface for service implementations.
pub trait ServiceInterface: Debug + Send + Sync {
    /// Pulls newly received requests from channels into the time-ordered request buffer.
    ///
    /// Returns the number of buffered requests remaining after ingestion.
    fn process_requests(&self) -> usize;
    /// Handles all requests that are due at `time`.
    fn handle_requests(&self, time: f32);
    /// Returns the next timestamp at which a request is waiting.
    ///
    /// Returns [`f32::INFINITY`] if no request is currently buffered.
    fn next_time(&self) -> f32;
}

/// Client to make requests to a service.
///
/// The client is linked to a server, and can make requests to it. The client is blocked until the
/// server sends a response.
#[derive(Debug, Clone)]
pub struct ServiceClient<RequestMsg: Debug + Clone, ResponseMsg: Debug + Clone> {
    /// Channel to send the request to the server (given by the server).
    response_channel: SharedMutex<mpsc::Receiver<ServiceResponse<ResponseMsg>>>,
    /// Channel to receive the response from the server (given by the server).
    request_channel: SharedMutex<mpsc::Sender<(String, RequestMsg, f32)>>,
    /// Simulator condition variable needed so that all nodes wait the end of other,
    /// and continue to treat messages.
    time_cv: Arc<TimeCv>,
    /// Flag to indicate if the service is still active. If false, the client should not send new requests.
    living: SharedRwLock<bool>,
}

impl<RequestMsg: Debug + Clone, ResponseMsg: Debug + Clone> ServiceClient<RequestMsg, ResponseMsg> {
    /// Make a request to the server.
    ///
    /// The client is blocked until the server sends a response. The client is linked to a server,
    /// and is made by it through the [`Service::new_client`](crate::networking::service::Service::new_client)
    /// method.
    ///
    /// ## Arguments
    /// * `node_name` - Name of the [`Node`](crate::node::Node) making the request.
    /// * `req` - Request message to send to the server.
    /// * `time` - Time at which the request is made.
    ///
    /// ## Returns
    /// The response from the server, or an error message if the request failed.
    pub fn send_request(
        &self,
        node_name: String,
        req: RequestMsg,
        time: f32,
    ) -> Result<(), SimbaError> {
        if !*self.living.read().unwrap() {
            return Err(SimbaError::new(
                SimbaErrorTypes::ServiceError(ServiceError::Closed),
                "Service server closed".to_string(),
            ));
        }
        let _lk = self.time_cv.waiting.lock().unwrap();
        if is_enabled(crate::logger::InternalLog::ServiceHandling) {
            debug!("Sending a request");
        }
        let mut circulating_messages = self.time_cv.circulating_messages.lock().unwrap();
        *circulating_messages += 1;
        if let Err(e) = self
            .request_channel
            .lock()
            .unwrap()
            .send((node_name, req, time))
        {
            *circulating_messages -= 1;
            if e.to_string() == "sending on a closed channel" {
                *self.living.write().unwrap() = false;
                return Err(SimbaError::new(
                    SimbaErrorTypes::ServiceError(ServiceError::Closed),
                    "Channel closed".to_string(),
                ));
            } else {
                return Err(SimbaError::new(
                    SimbaErrorTypes::ServiceError(ServiceError::Other(e.to_string())),
                    format!("Error while sending the request: {}", e),
                ));
            }
        }
        if is_enabled(crate::logger::InternalLog::ServiceHandling) {
            debug!("Sending a request: OK");
        }
        if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
            debug!("Notify CV  (service line {})", line!());
        }
        // Needed to unlock the other node if it has finished and is waiting for messages.
        self.time_cv.condvar.notify_all();
        Ok(())
    }

        /// Tries to receive a response from the server without blocking.
        ///
        /// On success, this decrements the global count of circulating service messages.
        /// If an unsubscribe flag is received, the client is marked as closed and a
        /// [`ServiceError::Closed`] error is returned.
    pub fn try_recv(&self) -> Result<ResponseMsg, SimbaError> {
        let result = match self.response_channel.lock().unwrap().try_recv() {
            Ok(result) => result,
            Err(e) => {
                return Err(SimbaError::new(
                    SimbaErrorTypes::ServiceError(ServiceError::Other(e.to_string())),
                    format!("{:?}", e),
                ));
            }
        };

        let mut circulating_messages = self.time_cv.circulating_messages.lock().unwrap();
        *circulating_messages -= 1;
        for flag in result.flags {
            if let MessageFlag::Unsubscribe = flag {
                *self.living.write().unwrap() = false;
                return Err(SimbaError::new(
                    SimbaErrorTypes::ServiceError(ServiceError::Closed),
                    "Closing service server: Unsubscribe".to_string(),
                ));
            }
        }
        if is_enabled(crate::logger::InternalLog::ServiceHandling) {
            debug!("Result received");
        }
        result.response
    }
}

/// Service to handle requests from clients.
///
/// Handles requests from clients, and send responses to them.
#[derive(Debug)]
pub struct Service<
    RequestMsg: Debug + Clone,
    ResponseMsg: Debug + Clone,
    T: HasService<RequestMsg, ResponseMsg> + ?Sized,
> {
    /// Channel to receive requests from clients.
    request_channel: SharedMutex<mpsc::Receiver<(String, RequestMsg, f32)>>,
    /// Channel to send requests to the server, which is cloned to the clients.
    request_channel_give: SharedMutex<mpsc::Sender<(String, RequestMsg, f32)>>,
    /// Map of the clients and their sender channel, to send responses.
    clients: BTreeMap<String, SharedMutex<mpsc::Sender<ServiceResponse<ResponseMsg>>>>,
    /// Buffer to store the requests until it is time to treat them.
    request_buffer: SharedRwLock<TimeOrderedData<(String, RequestMsg)>>,
    /// Simulator condition variable needed so that all nodes wait the end of others,
    /// and continue to treat messages.
    time_cv: Arc<TimeCv>,
    /// Target struct which handles the service requests.
    target: SharedRwLock<Box<T>>,
    /// Flag to indicate if the service is still active. If false, the server should not process new requests.
    living: SharedRwLock<bool>,
}

impl<
    RequestMsg: Debug + Clone,
    ResponseMsg: Debug + Clone,
    T: HasService<RequestMsg, ResponseMsg> + ?Sized,
> Service<RequestMsg, ResponseMsg, T>
{
    /// Create a new service.
    ///
    /// ## Arguments
    /// * `time_cv` - Condition variable of the simulator, to wait the end of the nodes.
    /// * `target` - Target struct which handles the service requests.
    pub(crate) fn new(time_cv: Arc<TimeCv>, target: SharedRwLock<Box<T>>) -> Self {
        let (tx, rx) = mpsc::channel::<(String, RequestMsg, f32)>();
        Self {
            request_channel_give: Arc::new(Mutex::new(tx)),
            request_channel: Arc::new(Mutex::new(rx)),
            clients: BTreeMap::new(),
            request_buffer: Arc::new(RwLock::new(TimeOrderedData::new(TIME_ROUND))),
            time_cv,
            target,
            living: Arc::new(RwLock::new(true)),
        }
    }

    /// Makes a new client to the service, with the channels already setup.
    ///
    /// ## Arguments
    /// * `node_name` - Name of the client node, to be able to send responses to it.
    pub fn new_client(&mut self, node_name: &str) -> ServiceClient<RequestMsg, ResponseMsg> {
        let (tx, rx) = mpsc::channel::<ServiceResponse<ResponseMsg>>();
        self.clients
            .insert(node_name.to_string(), Arc::new(Mutex::new(tx)));
        ServiceClient {
            request_channel: self.request_channel_give.clone(),
            response_channel: Arc::new(Mutex::new(rx)),
            time_cv: self.time_cv.clone(),
            living: Arc::new(RwLock::new(true)),
        }
    }

    /// Closes the service and notifies all registered clients to unsubscribe.
    pub fn delete(&mut self) {
        *self.living.write().unwrap() = false;
        for client in self.clients.values() {
            // If the client is closed, ignore the error
            let _ = client.lock().unwrap().send(ServiceResponse {
                response: Err(SimbaError::new(
                    SimbaErrorTypes::ServiceError(ServiceError::Closed),
                    "Closing channel".to_string(),
                )),
                flags: vec![MessageFlag::Unsubscribe],
            });
        }
    }
}

impl<
    RequestMsg: Clone + Debug + Send + Sync,
    ResponseMsg: Clone + Debug + Send + Sync,
    T: HasService<RequestMsg, ResponseMsg> + ?Sized,
> ServiceInterface for Service<RequestMsg, ResponseMsg, T>
{
    /// Process the requests received from the clients.
    ///
    /// The requests are added to the buffer, to be treated later by
    /// [`handle_requests`](Self::handle_requests).
    ///
    /// ## Returns
    /// The number of requests remaining in the buffer.
    fn process_requests(&self) -> usize {
        for (from, message, time) in self.request_channel.lock().unwrap().try_iter() {
            let mut circulating_messages = self.time_cv.circulating_messages.lock().unwrap();
            *circulating_messages -= 1;
            std::mem::drop(circulating_messages);
            if is_enabled(crate::logger::InternalLog::ServiceHandling) {
                debug!("Insert request from {from} at time {time}");
            }
            self.request_buffer
                .write()
                .unwrap()
                .insert(time, (from, message), false);
        }
        self.request_buffer.read().unwrap().len()
    }

    /// Handle the requests received from the clients at the given `time`.
    ///
    /// The `closure` is called for each request matching the given `time`, and should
    /// return the response to send to the client. The request is then removed from the
    /// buffer. If no request is matching the given `time`, the method does nothing.
    fn handle_requests(&self, time: f32) {
        while let Some((_msg_time, (from, message))) =
            self.request_buffer.write().unwrap().remove(time)
        {
            if is_enabled(crate::logger::InternalLog::ServiceHandling) {
                debug!("Handling message from {from} at time {time}...");
            }
            let result = self
                .target
                .write()
                .unwrap()
                .handle_service_requests(message, time);
            if is_enabled(crate::logger::InternalLog::ServiceHandling) {
                debug!("Got result");
            }
            {
                let mut circulating_messages = self.time_cv.circulating_messages.lock().unwrap();
                *circulating_messages += 1;
            }
            self.clients
                .get(&from)
                .unwrap_or_else(|| {
                    panic!("Given sender ({from}) is not known, use 'get_client' accordingly")
                })
                .lock()
                .unwrap()
                .send(ServiceResponse {
                    response: result.map_err(|e| {
                        SimbaError::new(SimbaErrorTypes::ServiceError(ServiceError::ClientSide), e)
                    }),
                    flags: Vec::new(),
                })
                .expect("Fail to send response");
            if is_enabled(crate::logger::InternalLog::ServiceHandling) {
                debug!("Handling message from {from} at time {time}... Response sent");
            }
        }
    }

    /// Get the minimal time among all waiting requests.
    fn next_time(&self) -> f32 {
        self.request_buffer
            .read()
            .unwrap()
            .min_time()
            .map(|tpl| tpl.0)
            .unwrap_or(f32::INFINITY)
    }
}

/// Response packet sent by a [`Service`] to a [`ServiceClient`].
pub struct ServiceResponse<ResponseMsg> {
    /// Service response payload, or an error raised while handling the request.
    pub response: Result<ResponseMsg, SimbaError>,
    /// Transport-level flags that alter client behavior (for example unsubscribe).
    pub flags: Vec<MessageFlag>,
}

/// Trait for specific service handlers, which can be different from
/// the struct which own the service (e.g. in case of strategy pattern).
pub trait ServiceHandler<RequestMsg, ResponseMsg>: Sync + Send + Debug {
    /// Handles one request received at simulation `time` and returns the response.
    fn treat_request(&self, req: RequestMsg, time: f32) -> Result<ResponseMsg, String>;
}

/// Common interface for all struct which manages a service.
pub trait HasService<RequestMsg, ResponseMsg>: Debug + Sync + Send {
    /// Handle the requests received from the clients at the given `time`.
    fn handle_service_requests(
        &mut self,
        req: RequestMsg,
        time: f32,
    ) -> Result<ResponseMsg, String>;
}
