use core::f32;
use std::{
    collections::BTreeMap,
    fmt::Debug,
    marker::PhantomData,
    sync::{mpsc, Arc, Condvar, Mutex, RwLock},
};

use log::debug;

use crate::{
    sensors::turtle_sensor, turtlebot::Turtlebot, utils::time_ordered_data::TimeOrderedData,
};

use super::network::MessageMode;

#[derive(Debug)]
pub struct ServiceClient<RequestMsg, ResponseMsg> {
    response_channel: Arc<Mutex<mpsc::Receiver<Result<ResponseMsg, String>>>>,
    request_channel: Arc<Mutex<mpsc::Sender<(String, RequestMsg, f32, MessageMode)>>>,
    time_cv: Arc<(Mutex<usize>, Condvar)>,
}

impl<RequestMsg, ResponseMsg> ServiceClient<RequestMsg, ResponseMsg> {
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
            MessageMode::Default,
        )) {
            Err(e) => return Err(e.to_string()),
            _ => (),
        }
        debug!("[{}] Sending a request... OK", turtle.name());
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

#[derive(Debug)]
pub struct Service<RequestMsg, ResponseMsg> {
    request_channel: Arc<Mutex<mpsc::Receiver<(String, RequestMsg, f32, MessageMode)>>>,
    request_channel_give: Arc<Mutex<mpsc::Sender<(String, RequestMsg, f32, MessageMode)>>>,
    clients: BTreeMap<String, Arc<Mutex<mpsc::Sender<Result<ResponseMsg, String>>>>>,
    request_buffer: Arc<RwLock<TimeOrderedData<(String, RequestMsg, MessageMode)>>>,
    time_cv: Arc<(Mutex<usize>, Condvar)>,
}

impl<RequestMsg, ResponseMsg> Service<RequestMsg, ResponseMsg> {
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

    pub fn next_time(&self) -> f32 {
        self.request_buffer
            .read()
            .unwrap()
            .min_time()
            .unwrap_or(f32::INFINITY)
    }
}

pub trait ServiceHandler<RequestMsg, ResponseMsg>: Sync + Send + Debug {
    fn treat_request(&self, req: RequestMsg, time: f32) -> Result<ResponseMsg, String>;
}

pub trait HasService<RequestMsg, ResponseMsg> {
    fn make_service(&mut self, turtle: Arc<RwLock<Turtlebot>>);
    fn new_client(&mut self, client_name: &str) -> ServiceClient<RequestMsg, ResponseMsg>;
    fn handle_service_requests(&mut self, time: f32);
    fn process_service_requests(&self) -> usize;
    fn service_next_time(&self) -> f32;
}
