use serde_json::Value;
use crate::turtlebot::Turtlebot;

pub trait MessageHandler: std::marker::Send + std::marker::Sync {
    fn handle_message(&mut self, turtle: &mut Turtlebot, from: &String, message: &Value) -> Result<(),()>;
}