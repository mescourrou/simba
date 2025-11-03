/*!
Provides a service system to handle two-way communication between nodes, with the
client node beeing blocked until the server node sends a response.

The server node should create a [`Service`] and handle the requests in
[`run_time_step`](crate::node::Node::run_time_step). The client node should get a
[`ServiceClient`] instance to be able to make a request.

To operate a service, two messages types should be defined:
- The request message type, which is sent by the client to the server.
- The response message type, which is sent by the server to the client.
*/

use core::f32;
use std::{
    collections::BTreeMap,
    fmt::Debug,
    sync::{mpsc, Arc, Mutex, RwLock},
};

use log::debug;

use crate::{
    errors::{SimbaError, SimbaErrorTypes},
    logger::is_enabled,
    networking::service_manager::ServiceError,
    simulator::TimeCv,
    utils::time_ordered_data::TimeOrderedData,
};

use super::network::MessageFlag;

pub trait ServiceInterface: Debug + Send + Sync {
    fn process_requests(&self) -> usize;
    fn handle_requests(&self, time: f32);
    fn next_time(&self) -> f32;
}

/// Client to make requests to a service.
///
/// The client is linked to a server, and can make requests to it. The client is blocked until the
/// server sends a response.
#[derive(Debug, Clone)]
pub struct ServiceClient<RequestMsg: Debug + Clone, ResponseMsg: Debug + Clone> {
    // Channel to send the request to the server (given by the server).
    response_channel:
        Arc<Mutex<mpsc::Receiver<(Result<ResponseMsg, SimbaError>, Vec<MessageFlag>)>>>,
    // Channel to receive the response from the server (given by the server).
    request_channel: Arc<Mutex<mpsc::Sender<(String, RequestMsg, f32)>>>,
    // Simulator condition variable needed so that all nodes wait the end of other,
    // and continue to treat messages.
    time_cv: Arc<TimeCv>,
    living: Arc<Mutex<bool>>,
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
        if !*self.living.lock().unwrap() {
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
        match self
            .request_channel
            .lock()
            .unwrap()
            .send((node_name, req, time))
        {
            Err(e) => {
                *circulating_messages -= 1;
                if e.to_string() == "sending on a closed channel".to_string() {
                    *self.living.lock().unwrap() = false;
                    return Err(SimbaError::new(
                        SimbaErrorTypes::ServiceError(ServiceError::Closed),
                        "Channel closed".to_string(),
                    ));
                } else {
                    return Err(SimbaError::new(
                        SimbaErrorTypes::ServiceError(ServiceError::Other(e.to_string())),
                        format!("Error while sending the request: {}", e.to_string()),
                    ));
                }
            }
            _ => (),
        }
        if is_enabled(crate::logger::InternalLog::ServiceHandling) {
            debug!("Sending a request: OK");
        }
        if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
            debug!("Notify CV");
        }
        // Needed to unlock the other node if it has finished and is waiting for messages.
        self.time_cv.condvar.notify_all();
        Ok(())
    }

    pub fn try_recv(&self) -> Result<ResponseMsg, SimbaError> {
        let result = match self.response_channel.lock().unwrap().try_recv() {
            Ok(result) => result,
            Err(e) => {
                return Err(SimbaError::new(
                    SimbaErrorTypes::ServiceError(ServiceError::Other(e.to_string())),
                    format!("{:?}", e),
                ))
            }
        };

        let mut circulating_messages = self.time_cv.circulating_messages.lock().unwrap();
        *circulating_messages -= 1;
        for flag in result.1 {
            match flag {
                MessageFlag::Unsubscribe => {
                    *self.living.lock().unwrap() = false;
                    return Err(SimbaError::new(
                        SimbaErrorTypes::ServiceError(ServiceError::Closed),
                        "Closing service server: Unsubscribe".to_string(),
                    ));
                }
                _ => (),
            }
        }
        if is_enabled(crate::logger::InternalLog::ServiceHandling) {
            debug!("Result received");
        }
        result.0
    }
}

/// Service to handle requests from clients.
///
/// Handles requests from clients, and send responses to them. The server should handle
/// the requests in [`run_time_step`](crate::node::Node::run_time_step).
#[derive(Debug)]
pub struct Service<
    RequestMsg: Debug + Clone,
    ResponseMsg: Debug + Clone,
    T: HasService<RequestMsg, ResponseMsg> + ?Sized,
> {
    /// Channel to receive requests from clients.
    request_channel: Arc<Mutex<mpsc::Receiver<(String, RequestMsg, f32)>>>,
    /// Channel to send requests to the server, which is cloned to the clients.
    request_channel_give: Arc<Mutex<mpsc::Sender<(String, RequestMsg, f32)>>>,
    /// Map of the clients and their sender channel, to send responses.
    clients: BTreeMap<
        String,
        Arc<Mutex<mpsc::Sender<(Result<ResponseMsg, SimbaError>, Vec<MessageFlag>)>>>,
    >,
    /// Buffer to store the requests until it is time to treat them.
    request_buffer: Arc<RwLock<TimeOrderedData<(String, RequestMsg)>>>,
    /// Simulator condition variable needed so that all nodes wait the end of others,
    /// and continue to treat messages.
    time_cv: Arc<TimeCv>,
    target: Arc<RwLock<Box<T>>>,
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
    pub fn new(time_cv: Arc<TimeCv>, target: Arc<RwLock<Box<T>>>) -> Self {
        let (tx, rx) = mpsc::channel::<(String, RequestMsg, f32)>();
        Self {
            request_channel_give: Arc::new(Mutex::new(tx)),
            request_channel: Arc::new(Mutex::new(rx)),
            clients: BTreeMap::new(),
            request_buffer: Arc::new(RwLock::new(TimeOrderedData::new())),
            time_cv,
            target,
        }
    }

    /// Makes a new client to the service, with the channels already setup.
    ///
    /// ## Arguments
    /// * `node_name` - Name of the client node, to be able to send responses to it.
    pub fn new_client(&mut self, node_name: &str) -> ServiceClient<RequestMsg, ResponseMsg> {
        let (tx, rx) = mpsc::channel::<(Result<ResponseMsg, SimbaError>, Vec<MessageFlag>)>();
        self.clients
            .insert(node_name.to_string(), Arc::new(Mutex::new(tx)));
        ServiceClient {
            request_channel: self.request_channel_give.clone(),
            response_channel: Arc::new(Mutex::new(rx)),
            time_cv: self.time_cv.clone(),
            living: Arc::new(Mutex::new(true)),
        }
    }

    pub fn delete(&mut self) {
        for (_, client) in &self.clients {
            client
                .lock()
                .unwrap()
                .send((
                    Err(SimbaError::new(
                        SimbaErrorTypes::ServiceError(ServiceError::Closed),
                        format!("Closing channel"),
                    )),
                    vec![MessageFlag::Unsubscribe],
                ))
                .unwrap();
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
    /// [`handle_requests`](Service::handle_requests).
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
                .expect(
                    format!("Given sender ({from}) is not known, use 'get_client' accordingly")
                        .as_str(),
                )
                .lock()
                .unwrap()
                .send((
                    result.map_err(|e| {
                        SimbaError::new(SimbaErrorTypes::ServiceError(ServiceError::ClientSide), e)
                    }),
                    Vec::new(),
                ))
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

/// Trait for specific service handlers, which can be different from
/// the struct which own the service (e.g. in case of strategy pattern).
pub trait ServiceHandler<RequestMsg, ResponseMsg>: Sync + Send + Debug {
    fn treat_request(&self, req: RequestMsg, time: f32) -> Result<ResponseMsg, String>;
}

/// Common interface for all struct which manages a service.
pub trait HasService<RequestMsg, ResponseMsg>: Debug + Sync + Send {
    // /// Create the service: should be called only once, at the beginning.
    // fn make_service(&mut self, node: Arc<RwLock<Robot>>);
    // /// Create a new client to the service, should be called by client nodes.
    // fn new_client(&mut self, client_name: &str) -> ServiceClient<RequestMsg, ResponseMsg>;
    /// Handle the requests received from the clients at the given `time`.
    fn handle_service_requests(
        &mut self,
        req: RequestMsg,
        time: f32,
    ) -> Result<ResponseMsg, String>;
    // /// Process the requests received from the clients.
    // fn process_service_requests(&self) -> usize;
    // /// Get the minimal time among all waiting requests. BOol is for read only (no change in state, which does not requires a new computation)
    // fn service_next_time(&self) -> (f32, bool);
}
