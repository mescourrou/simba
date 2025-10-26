/*!
Provide the [`MessageHandler`] trait.
*/

use std::{fmt::Debug, sync::mpsc::Sender};

use serde_json::Value;

pub trait MessageHandler: std::marker::Send + std::marker::Sync + Debug {
    fn get_letter_box(&self) -> Option<Sender<(String, Value, f32)>> {
        None
    }
}
