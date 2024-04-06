/*!
Module which provides communication tools between robots.

Each [`Turtlebot`](crate::turtlebot::Turtlebot) includes a [`Network`](network::Network).
The [`NetworkManager`](network_manager::NetworkManager) makes the link between the
[`Network`](network::Network)s and is owned by the [`Simulator`](crate::simulator::Simulator).
*/

pub mod message_handler;
pub mod network;
pub mod network_manager;
