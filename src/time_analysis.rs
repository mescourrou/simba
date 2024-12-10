#[macro_use]
use lazy_static::lazy_static;
use serde::de;

use std::sync::{Arc, Mutex};
use std::thread::ThreadId;
use std::{thread, time};
use std::collections::HashMap;

#[derive(Debug)]
pub struct TimeAnalysisFactory {
    turtles_names: HashMap<ThreadId, String>,
    turtles_depth: HashMap<ThreadId, usize>,
}

impl TimeAnalysisFactory {
    fn get_instance() -> &'static Mutex<TimeAnalysisFactory> {
        lazy_static! {
            static ref FACTORY: Mutex<TimeAnalysisFactory> = {
                let m = Mutex::new(TimeAnalysisFactory {
                    turtles_names: HashMap::new(),
                    turtles_depth: HashMap::new(),
                });
                m
            };
        }
        &FACTORY
    }

    pub fn set_turtle_name(name: String) {
        let factory = TimeAnalysisFactory::get_instance();
        let mut factory = factory.lock().unwrap();
        factory._set_turtle_name(name);
    }

    fn _set_turtle_name(&mut self, name: String) {
        self.turtles_names.insert(thread::current().id(), name);
        self.turtles_depth.insert(thread::current().id(), 0);
    }

    pub fn time_analysis(name: String) -> TimeAnalysis {
        let factory = TimeAnalysisFactory::get_instance();
        let mut factory = factory.lock().unwrap();
        let ta = factory._time_analysis(name).clone();
        ta
    }

    fn _time_analysis(&mut self, name: String) -> TimeAnalysis {
        let ta = TimeAnalysis {
            begin: time::Instant::now(),
            name,
            depth: *self.turtles_depth.get(&thread::current().id()).unwrap(),
        };
        *self.turtles_depth.get_mut(&thread::current().id()).unwrap() += 1;
        ta
    }

    pub fn finished_time_analysis(ta: TimeAnalysis) {
        let elapsed = ta.begin.elapsed();
        let factory = TimeAnalysisFactory::get_instance();
        let mut factory = factory.lock().unwrap();
        factory._finished_time_analysis(ta, elapsed);
    }

    fn _finished_time_analysis(&mut self, ta: TimeAnalysis, elapsed: time::Duration) {
        *self.turtles_depth.get_mut(&thread::current().id()).unwrap() -= 1;
        // let indent = ta.depth*2;
        println!(
            "{:indent$}{}: {}ms",
            "",
            ta.name,
            elapsed.as_millis(),
            indent = ta.depth * 2
        );
    }
}

#[derive(Clone, Debug)]
pub struct TimeAnalysis {
    begin: time::Instant,
    name: String,
    depth: usize,
}