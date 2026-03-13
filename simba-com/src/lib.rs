#![warn(missing_docs)]
#![deny(rustdoc::broken_intra_doc_links)]

//! Simba communication library for inter-node messaging and data exchange.
//! 
//! This crate provides the core communication primitives for Simba, including a publish-subscribe system and a 
//! service manager for handling inter-node service requests. It is designed to be used by the main simulator 
//! crate (`simba-core`) and can also be used independently for custom communication needs in multi-robot 
//! simulations.

pub mod pub_sub;
pub mod rfc;
pub mod time_ordered_data;
