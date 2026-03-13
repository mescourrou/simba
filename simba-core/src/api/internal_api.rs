
//! Internal API for communication between the [`Node`](crate::node::Node)s and the [`Simulator`](crate::simulator::Simulator). This module contains the definitions of the internal communication channels between the nodes and the simulator, as well as the functions to create them.
use std::sync::mpsc::{self, Receiver, Sender};

use crate::{
    node::{NodeState, node_factory::NodeType},
    state_estimators::State,
};

/// Internal communication between [`Node`](crate::node::Node)s and the [`Simulator`](crate::simulator::Simulator). The [`NodeServer`] is
/// owned by the [`Node`](crate::node::Node) ans is used to send its [`State`] and state-machine [`NodeState`] to the [`Simulator`](crate::simulator::Simulator).
#[derive(Debug)]
pub struct NodeServer {
    /// Send an update of the node state to the simulator, with the simulated time of the update, and the new state and node state.
    pub state_update: Option<Sender<(f32, (State, NodeState))>>,
}

/// Internal communication between [`Node`](crate::node::Node)s and the [`Simulator`](crate::simulator::Simulator). The [`NodeClient`] is
/// owned by the [`Simulator`](crate::simulator::Simulator) and is used to receive the [`State`] and state-machine [`NodeState`] updates from the [`Node`](crate::node::Node)s.
#[derive(Debug)]
pub struct NodeClient {
    /// Receive an update of the node state from the node, with the simulated time of the update, and the new state and node state.
    pub state_update: Option<Receiver<(f32, (State, NodeState))>>,
}

/// Create a new [`NodeServer`] and [`NodeClient`] for a [`Node`](crate::node::Node) with the given [`NodeType`]. If the node type has physics, the server and client will be created with a channel to send and receive state updates. Otherwise, the server and client will be created without a channel.
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
