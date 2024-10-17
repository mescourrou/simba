use core::f32;
use std::{
    collections::BTreeMap,
    fmt::Debug,
    marker::PhantomData,
    sync::{mpsc, Arc, Mutex, RwLock},
};

use crate::{sensors::turtle_sensor, turtlebot::Turtlebot, utils::time_ordered_data::TimeOrderedData};

use super::network::MessageMode;

#[derive(Debug)]
pub struct ServiceClient<RequestMsg, ResponseMsg> {
    response_channel:
        Arc<Mutex<mpsc::Receiver<Result<ResponseMsg, Box<dyn std::error::Error + Sync + Send>>>>>,
    request_channel: Arc<Mutex<mpsc::Sender<(String, RequestMsg, f32, MessageMode)>>>,
}

impl<RequestMsg, ResponseMsg> ServiceClient<RequestMsg, ResponseMsg> {
    pub fn make_request(
        &mut self,
        turtle: &mut Turtlebot,
        req: RequestMsg,
    ) -> Result<ResponseMsg, Box<dyn std::error::Error + Sync + Send>> {
        todo!()
    }
}

#[derive(Debug)]
pub struct Service<RequestMsg, ResponseMsg> {
    request_channel: Arc<Mutex<mpsc::Receiver<(String, RequestMsg, f32, MessageMode)>>>,
    request_channel_give: Arc<Mutex<mpsc::Sender<(String, RequestMsg, f32, MessageMode)>>>,
    clients: BTreeMap<
        String,
        Arc<Mutex<mpsc::Sender<Result<ResponseMsg, Box<dyn std::error::Error + Sync + Send>>>>>,
    >,
    request_buffer: Arc<RwLock<TimeOrderedData<(String, RequestMsg, MessageMode)>>>,
}

impl<RequestMsg, ResponseMsg> Service<RequestMsg, ResponseMsg> {
    pub fn new() -> Self {
        let (tx, rx) = mpsc::channel::<(String, RequestMsg, f32, MessageMode)>();
        Self {
            request_channel_give: Arc::new(Mutex::new(tx)),
            request_channel: Arc::new(Mutex::new(rx)),
            clients: BTreeMap::new(),
            request_buffer: Arc::new(RwLock::new(TimeOrderedData::new())),
        }
    }

    pub fn process_requests(&self) {
        for (from, message, time, message_mode) in self.request_channel.lock().unwrap().try_iter() {
            self.request_buffer.write().unwrap().insert(time, (from, message, message_mode), false);
        }
        // while let Some(msg) = self.request_channel.lock().unwrap().try_iter() {
        //     let (sender, req, time, mode) = msg;
        //     let result = handler.treat_request(req, time);
        //     self.clients
        //         .get(&sender)
        //         .expect(
        //             format!("Given sender ({sender}) is not known, use 'get_client' accordingly")
        //                 .as_str(),
        //         )
        //         .lock()
        //         .unwrap()
        //         .send(result)
        //         .expect("Fail to send response");
        // }
    }

    pub fn new_client(&mut self, turtle_name: &str) -> ServiceClient<RequestMsg, ResponseMsg> {
        let (tx, rx) =
            mpsc::channel::<Result<ResponseMsg, Box<dyn std::error::Error + Sync + Send>>>();
        self.clients
            .insert(turtle_name.to_string(), Arc::new(Mutex::new(tx)));
        ServiceClient {
            request_channel: self.request_channel_give.clone(),
            response_channel: Arc::new(Mutex::new(rx)),
        }
    }

    pub fn next_time(&mut self) -> f32 {
        self.request_buffer.read().unwrap().min_time().unwrap_or(f32::INFINITY)
    }
}

pub trait ServiceHandler<RequestMsg, ResponseMsg>: Sync + Send + Debug {
    fn treat_request(
        &self,
        req: RequestMsg,
        time: f32,
    ) -> Result<ResponseMsg, Box<dyn std::error::Error + Sync + Send>>;
}

pub trait HasService<RequestMsg, ResponseMsg> {
    fn make_service(&mut self, turtle: Arc<RwLock<Turtlebot>>);
    fn new_client(
        &mut self,
        client_name: &str,
    ) -> ServiceClient<RequestMsg, ResponseMsg>;
    fn start(&self);
    fn service_next_time(&self) -> f32;
}
