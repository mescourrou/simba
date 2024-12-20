/*!
Provides a service system to handle two-way communication between robots, with the
client robot beeing blocked until the server robot sends a response.

The server robot should create a [`Service`] and handle the requests in
[`run_time_step`](crate::turtlebot::Turtlebot::run_time_step). The client robot should get a
[`ServiceClient`] instance to be able to make a request.

To operate a service, two messages types should be defined:
- The request message type, which is sent by the client to the server.
- The response message type, which is sent by the server to the client.
*/

use core::f32;
use std::{
    collections::BTreeMap,
    fmt::Debug,
    sync::{mpsc, Arc, Condvar, Mutex, RwLock},
};

use log::debug;

use crate::{turtlebot::Turtlebot, utils::time_ordered_data::TimeOrderedData};

use super::network::MessageMode;

/// Client to make requests to a service.
///
/// The client is linked to a server, and can make requests to it. The client is blocked until the
/// server sends a response.
#[derive(Debug)]
pub struct ServiceClient<RequestMsg, ResponseMsg> {
    // Channel to send the request to the server (given by the server).
    response_channel: Arc<Mutex<mpsc::Receiver<Result<ResponseMsg, String>>>>,
    // Channel to receive the response from the server (given by the server).
    request_channel: Arc<Mutex<mpsc::Sender<(String, RequestMsg, f32, MessageMode)>>>,
    // Simulator condition variable needed so that all robots wait the end of other,
    // and continue to treat messages.
    time_cv: Arc<(Mutex<usize>, Condvar)>,
}

impl<RequestMsg, ResponseMsg> ServiceClient<RequestMsg, ResponseMsg> {
    /// Make a request to the server.
    ///
    /// The client is blocked until the server sends a response. The client is linked to a server,
    /// and is made by it through the [`Service::new_client`](crate::networking::service::Service::new_client)
    /// method.
    ///
    /// ## Arguments
    /// * `turtle` - Reference to the [`Turtlebot`](crate::turtlebot::Turtlebot) making the request.
    /// * `req` - Request message to send to the server.
    /// * `time` - Time at which the request is made.
    ///
    /// ## Returns
    /// The response from the server, or an error message if the request failed.
    pub fn make_request(
        &mut self,
        turtle: &mut Turtlebot,
        req: RequestMsg,
        time: f32,
    ) -> Result<ResponseMsg, String> {
        debug!("[{}] Sending a request...", turtle.name());
        let lk = self.time_cv.0.lock().unwrap();
        match self.request_channel.lock().unwrap().send((
            turtle.name(),
            req,
            time,
            // To be changed when full support of the message mode will be implemented
            MessageMode::Default,
        )) {
            Err(e) => return Err(e.to_string()),
            _ => (),
        }
        debug!("[{}] Sending a request... OK", turtle.name());
        // Needed to unlock the other turtle if it has finished and is waiting for messages.
        self.time_cv.1.notify_all();
        std::mem::drop(lk);
        debug!("[{}] Waiting for result...", turtle.name());
        let result = match self.response_channel.lock().unwrap().recv() {
            Ok(result) => result,
            Err(e) => return Err(e.to_string()),
        };
        debug!("[{}] Result received", turtle.name());
        result
    }
}

/// Service to handle requests from clients.
///
/// Handles requests from clients, and send responses to them. The server should handle
/// the requests in [`run_time_step`](crate::turtlebot::Turtlebot::run_time_step).
#[derive(Debug)]
pub struct Service<RequestMsg, ResponseMsg> {
    /// Channel to receive requests from clients.
    request_channel: Arc<Mutex<mpsc::Receiver<(String, RequestMsg, f32, MessageMode)>>>,
    /// Channel to send requests to the server, which is cloned to the clients.
    request_channel_give: Arc<Mutex<mpsc::Sender<(String, RequestMsg, f32, MessageMode)>>>,
    /// Map of the clients and their sender channel, to send responses.
    clients: BTreeMap<String, Arc<Mutex<mpsc::Sender<Result<ResponseMsg, String>>>>>,
    /// Buffer to store the requests until it is time to treat them.
    request_buffer: Arc<RwLock<TimeOrderedData<(String, RequestMsg, MessageMode)>>>,
    /// Simulator condition variable needed so that all robots wait the end of others,
    /// and continue to treat messages.
    time_cv: Arc<(Mutex<usize>, Condvar)>,
}

impl<RequestMsg, ResponseMsg> Service<RequestMsg, ResponseMsg> {
    /// Create a new service.
    ///
    /// ## Arguments
    /// * `time_cv` - Condition variable of the simulator, to wait the end of the robots.
    pub fn new(time_cv: Arc<(Mutex<usize>, Condvar)>) -> Self {
        let (tx, rx) = mpsc::channel::<(String, RequestMsg, f32, MessageMode)>();
        Self {
            request_channel_give: Arc::new(Mutex::new(tx)),
            request_channel: Arc::new(Mutex::new(rx)),
            clients: BTreeMap::new(),
            request_buffer: Arc::new(RwLock::new(TimeOrderedData::new())),
            time_cv,
        }
    }

    /// Process the requests received from the clients.
    ///
    /// The requests are added to the buffer, to be treated later by
    /// [`handle_service_requests`](Service::handle_service_requests).
    ///
    /// ## Returns
    /// The number of requests remaining in the buffer.
    pub fn process_requests(&self) -> usize {
        debug!("Processing requests...");
        for (from, message, time, message_mode) in self.request_channel.lock().unwrap().try_iter() {
            self.request_buffer
                .write()
                .unwrap()
                .insert(time, (from, message, message_mode), false);
        }
        debug!(
            "Processing requests... {} request in the buffer",
            self.request_buffer.read().unwrap().len()
        );
        self.request_buffer.read().unwrap().len()
    }

    /// Handle the requests received from the clients at the given `time`.
    ///
    /// The `closure` is called for each request matching the given `time`, and should
    /// return the response to send to the client. The request is then removed from the
    /// buffer. If no request is matching the given `time`, the method does nothing.
    pub fn handle_service_requests(
        &self,
        time: f32,
        closure: &dyn Fn(RequestMsg, f32) -> Result<ResponseMsg, String>,
    ) {
        while let Some((_msg_time, (from, message, _message_mode))) =
            self.request_buffer.write().unwrap().remove(time)
        {
            debug!("Handling message from {from} at time {time}...");
            let result = closure(message, time);
            debug!("Handling message from {from} at time {time}... Request treated, sending...");
            self.clients
                .get(&from)
                .expect(
                    format!("Given sender ({from}) is not known, use 'get_client' accordingly")
                        .as_str(),
                )
                .lock()
                .unwrap()
                .send(result)
                .expect("Fail to send response");
            debug!("Handling message from {from} at time {time}... Response sent");
        }
    }

    /// Makes a new client to the service, with the channels already setup.
    ///
    /// ## Arguments
    /// * `turtle_name` - Name of the client turtle, to be able to send responses to it.
    pub fn new_client(&mut self, turtle_name: &str) -> ServiceClient<RequestMsg, ResponseMsg> {
        let (tx, rx) = mpsc::channel::<Result<ResponseMsg, String>>();
        self.clients
            .insert(turtle_name.to_string(), Arc::new(Mutex::new(tx)));
        ServiceClient {
            request_channel: self.request_channel_give.clone(),
            response_channel: Arc::new(Mutex::new(rx)),
            time_cv: self.time_cv.clone(),
        }
    }

    /// Get the minimal time among all waiting requests.
    pub fn next_time(&self) -> f32 {
        self.request_buffer
            .read()
            .unwrap()
            .min_time()
            .unwrap_or(f32::INFINITY)
    }
}

/// Trait for specific service handlers, which can be different from
/// the struct which own the service (e.g. in case of strategy pattern).
pub trait ServiceHandler<RequestMsg, ResponseMsg>: Sync + Send + Debug {
    fn treat_request(&self, req: RequestMsg, time: f32) -> Result<ResponseMsg, String>;
}

/// Common interface for all struct which manages a service.
pub trait HasService<RequestMsg, ResponseMsg> {
    /// Create the service: should be called only once, at the beginning.
    fn make_service(&mut self, turtle: Arc<RwLock<Turtlebot>>);
    /// Create a new client to the service, should be called by client robots.
    fn new_client(&mut self, client_name: &str) -> ServiceClient<RequestMsg, ResponseMsg>;
    /// Handle the requests received from the clients at the given `time`.
    fn handle_service_requests(&mut self, time: f32);
    /// Process the requests received from the clients.
    fn process_service_requests(&self) -> usize;
    /// Get the minimal time among all waiting requests.
    fn service_next_time(&self) -> f32;
}
