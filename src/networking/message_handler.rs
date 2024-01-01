use serde_json::Value;

pub trait MessageHandler {
    fn handle_message(&mut self, from: &String, message: &Value) -> Result<(),()>;
}