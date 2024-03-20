use serde_json::Value;

pub trait MessageHandler: std::marker::Send + std::marker::Sync {
    fn handle_message(&mut self, from: &String, message: &Value) -> Result<(),()>;
}