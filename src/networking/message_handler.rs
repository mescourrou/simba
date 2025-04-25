/*!
Provide the [`MessageHandler`] trait.
*/

use std::fmt::Debug;

use crate::node::Node;
use serde_json::Value;

/// Trait which add a method to handle a message. If the message cannot be handled,
/// the function returns Err, and it returns Ok if the message was handled.
pub trait MessageHandler: std::marker::Send + std::marker::Sync + Debug {
    /// Handle the given `message` or not.
    ///
    /// ## Arguments
    /// * `robot` - Reference to the [`Robot`], to access the modules.
    /// * `from` - Emitter of the message.
    /// * `message` - Serialized message using [`serde_json::Value`]/
    /// * `time` - Time of the message.
    ///
    /// ## Return
    /// * `Ok` - The message was handled.
    /// * `Err` - The message was not handled and should be sent to the next handler.
    fn handle_message(
        &mut self,
        robot: &mut Node,
        from: &String,
        message: &Value,
        time: f32,
    ) -> Result<(), ()>;
}
