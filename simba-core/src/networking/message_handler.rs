/*!
Provide the [`MessageHandler`] trait.
*/

use std::{fmt::Debug, sync::mpsc::Sender};

use crate::networking::network::Envelope;

pub trait MessageHandler: std::marker::Send + std::marker::Sync + Debug {
    fn get_letter_box(&self) -> Option<Sender<Envelope>> {
        None
    }
}
