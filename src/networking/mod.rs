/*!
Module which provides communication tools between robots.

Each [`Turtlebot`](crate::turtlebot::Turtlebot) includes a [`Network`](network::Network).
The [`NetworkManager`](network_manager::NetworkManager) makes the link between the
[`Network`](network::Network)s and is owned by the [`Simulator`](crate::simulator::Simulator).

There are two main ways to communicate between robots.

1) One-way communication: A robot sends a message to another robot. This is done using the
[`Network::send_to`](network::Network::send_to) and
[`Network::broadcast`](network::Network::broadcast) methods. The message is sent to the
receiver [`Network`](network::Network) and is stored in a time ordered buffer. The receiver
unwraps the message when it reaches the time of the message. If the message is sent in the
past, the receiver will go back in time to unwrap the message. The message treatment is done
in [`run_time_step`](crate::turtlebot::Turtlebot::run_time_step), at the end. The message is
then passed from one [`MessageHandler`](message_handler::MessageHandler) to the next until
one of them handles the message.

2) Two-way communication: A robot sends a request to another robot and waits for the response.
This is done using the [`Service`](service::Service) and [`ServiceClient`](service::ServiceClient).
The server robot proposes a service, and then a client robot need to get a
[`ServiceClient`](service::ServiceClient) instance to be able to make a request. The client
sends a request to the server, and is blocked until the server sends a response. The server
robot should handle the requests in [`run_time_step`](crate::turtlebot::Turtlebot::run_time_step).
*/

pub mod message_handler;
pub mod network;
pub mod network_manager;
pub mod service;
