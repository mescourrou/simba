use crate::turtlebot::Turtlebot;
use serde_json::Value;

pub trait MessageHandler: std::marker::Send + std::marker::Sync {
    fn handle_message(
        &mut self,
        turtle: &mut Turtlebot,
        from: &String,
        message: &Value,
        time: f32,
    ) -> Result<(), ()>;
}
