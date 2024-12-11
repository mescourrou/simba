#[cfg_attr(not(feature = "time-analysis"), allow(dead_code))]

#[cfg(feature = "time-analysis")]
use lazy_static::lazy_static;
#[cfg(feature = "time-analysis")]
use serde::Serialize;

use std::path::Path;
#[cfg(feature = "time-analysis")]
use std::sync::Mutex;
#[cfg(feature = "time-analysis")]
use std::thread::ThreadId;

use std::time;
#[cfg(feature = "time-analysis")]
use std::thread;
#[cfg(feature = "time-analysis")]
use std::collections::HashMap;
#[cfg(feature = "time-analysis")]
use log::info;

#[cfg(feature = "time-analysis")]
#[derive(Debug)]
struct ExecutionProfile {
    name: String,
    begin: time::Instant,
    end: time::Instant,
    depth: usize,
    duration: time::Duration,
}

#[cfg(feature = "time-analysis")]
#[derive(Debug)]
struct TimeAnalysisFactory {
    turtles_names: HashMap<ThreadId, String>,
    turtles_depth: HashMap<ThreadId, usize>,
    execution_profiles: HashMap<String, Vec<ExecutionProfile>>,
    exporter: Box<dyn ProfilerExporter>,
    begin: time::Instant,

}

#[cfg(feature = "time-analysis")]
impl TimeAnalysisFactory {
    fn get_instance() -> &'static Mutex<TimeAnalysisFactory> {
        lazy_static! {
            static ref FACTORY: Mutex<TimeAnalysisFactory> = {
                let m = Mutex::new(TimeAnalysisFactory {
                    begin: time::Instant::now(),
                    turtles_names: HashMap::new(),
                    turtles_depth: HashMap::new(),
                    execution_profiles: HashMap::new(),
                    exporter: Box::new(TraceEventExporter {}),
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
        self.turtles_names.insert(thread::current().id(), name.clone());
        self.turtles_depth.insert(thread::current().id(), 0);
        self.execution_profiles.insert(name, Vec::new());
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
        let turtle_name = self.turtles_names.get(&thread::current().id()).unwrap();
        *self.turtles_depth.get_mut(&thread::current().id()).unwrap() -= 1;
        // let indent = ta.depth*2;
        self.execution_profiles.get_mut(turtle_name).unwrap().push(ExecutionProfile {
            name: ta.name,
            begin: ta.begin,
            end: ta.begin + elapsed,
            depth: ta.depth,
            duration: elapsed,
        });
    }

    pub fn save_results(path: &Path) {
        let factory = TimeAnalysisFactory::get_instance();
        let factory = factory.lock().unwrap();
        factory._save_results(path);
    }

    fn _save_results(&self, path: &Path) {
        info!("Saving Time Analysis results to {:?}", path);
        self.exporter.export(self, path);
    }
}

// Expose the function depending on the compilation feature "time-analysis"
// Feature enabled
#[cfg(feature = "time-analysis")]
pub fn set_turtle_name(name: String) {
    TimeAnalysisFactory::set_turtle_name(name);
}

#[cfg(feature = "time-analysis")]
pub fn time_analysis(name: String) -> TimeAnalysis {
    TimeAnalysisFactory::time_analysis(name)
}

#[cfg(feature = "time-analysis")]
pub fn finished_time_analysis(ta: TimeAnalysis) {
    TimeAnalysisFactory::finished_time_analysis(ta);
}

#[cfg(feature = "time-analysis")]
pub fn save_results(path: &Path) {
    TimeAnalysisFactory::save_results(path);
}

// Feature disabled
#[cfg(not(feature = "time-analysis"))]
pub fn set_turtle_name(_name: String) {}

#[cfg(not(feature = "time-analysis"))]
pub fn time_analysis(name: String) -> TimeAnalysis {
    TimeAnalysis {
        begin: time::Instant::now(),
        name,
        depth: 0,
    }
}

#[cfg(not(feature = "time-analysis"))]
pub fn finished_time_analysis(_ta: TimeAnalysis) {}

#[cfg(not(feature = "time-analysis"))]
pub fn save_results(_path: &Path) {}

//////////////////////////////////////////////////

#[derive(Clone, Debug)]
pub struct TimeAnalysis {
    begin: time::Instant,
    name: String,
    depth: usize,
}

#[cfg(feature = "time-analysis")]
trait ProfilerExporter: std::fmt::Debug + Sync + Send {
    fn export(&self, taf: &TimeAnalysisFactory, path: &Path);
}
#[cfg(feature = "time-analysis")]
#[derive(Debug)]
struct TraceEventExporter {}

#[cfg(feature = "time-analysis")]
#[derive(Serialize, Debug)]
struct TraceEvent {
    name: String,
    cat: String,
    ph: String,
    ts: i64,
    dur: i64,
    tid: i64,
    pid: i64,
    args: Option<serde_json::Value>,
    sf: i64,
}

#[cfg(feature = "time-analysis")]
#[derive(Serialize, Debug)]
struct TraceEventRoot {
    traceEvents: Vec<TraceEvent>,
    displayTimeUnit: String,
    otherData: serde_json::Value,

}

#[cfg(feature = "time-analysis")]
impl ProfilerExporter for TraceEventExporter {

    fn export(&self, taf: &TimeAnalysisFactory, path: &Path) {
        let mut trace_events = Vec::new();
        let mut turtle_names = Vec::new();
        for (turtle_name, profiles) in taf.execution_profiles.iter() {
            turtle_names.push(turtle_name);
            for profile in profiles {
                trace_events.push(TraceEvent {
                    name: profile.name.clone(),
                    cat: "PERF".to_string(),
                    ph: "X".to_string(),
                    ts: profile.begin.duration_since(taf.begin).as_micros() as i64,
                    dur: profile.duration.as_micros() as i64,
                    tid: turtle_names.len() as i64 - 1,
                    pid: 0,
                    args: None,
                    sf: 0,
                });
            }
        }
        let json = serde_json::to_string(&TraceEventRoot{
            traceEvents: trace_events,
            displayTimeUnit: "ns".to_string(),
            otherData: serde_json::Value::Null,
        }
        ).unwrap();
        std::fs::write(path, json).unwrap();
    }
}

