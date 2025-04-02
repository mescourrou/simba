use std::sync::mpsc::{self, Receiver, Sender};

use crate::state_estimators::state_estimator::State;


// The server is on Robot side
#[derive(Debug)]
pub struct RobotServer {
    pub state_update: Sender<(f32, State)>,
}

#[derive(Debug)]
pub struct RobotClient {
    pub state_update: Receiver<(f32, State)>,
}

pub fn make_robot_api() -> (RobotServer, RobotClient) {
    let state_update = mpsc::channel();

    (RobotServer {
        state_update: state_update.0
    },
    RobotClient {
        state_update: state_update.1
    })
}