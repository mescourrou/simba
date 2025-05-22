use std::sync::mpsc::{self, Receiver, Sender};

use crate::{node_factory::NodeType, state_estimators::state_estimator::State};

// The server is on Node side
#[derive(Debug)]
pub struct NodeServer {
    pub state_update: Option<Sender<(f32, State)>>,
}

#[derive(Debug)]
pub struct NodeClient {
    pub state_update: Option<Receiver<(f32, State)>>,
}

pub fn make_node_api(node_type: &NodeType) -> (NodeServer, NodeClient) {
    let state_update = if node_type.has_physics() {
        let (tx, rx) = mpsc::channel();
        (Some(tx), Some(rx))
    } else {
        (None, None)
    };

    (
        NodeServer {
            state_update: state_update.0,
        },
        NodeClient {
            state_update: state_update.1,
        },
    )
}
