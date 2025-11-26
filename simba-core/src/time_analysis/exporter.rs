use std::path::Path;

use serde::{Deserialize, Serialize};
#[cfg(feature = "gui")]
use simba_macros::{EnumToString, ToVec};

use crate::time_analysis::TimeAnalysisFactory;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[cfg_attr(feature = "gui", derive(ToVec, EnumToString))]
pub enum ProfileExporterConfig {
    TraceEventExporter,
}

pub trait ProfilerExporter: std::fmt::Debug + Sync + Send {
    fn export(&self, taf: &TimeAnalysisFactory, path: &Path);
}

#[derive(Debug)]
pub struct TraceEventExporter {}

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

#[derive(Serialize, Debug)]
struct TraceEventRoot {
    #[serde(rename = "traceEvents")]
    trace_events: Vec<TraceEvent>,
    #[serde(rename = "displayTimeUnit")]
    display_time_unit: String,
    #[serde(rename = "otherData")]
    other_data: serde_json::Value,
}

impl ProfilerExporter for TraceEventExporter {
    fn export(&self, taf: &TimeAnalysisFactory, path: &Path) {
        let mut trace_events = Vec::new();
        let mut node_names = Vec::new();
        for (node_name, profiles) in taf.iter_execution_profiles() {
            node_names.push(node_name.clone());
            trace_events.push(TraceEvent {
                name: "thread_name".to_string(),
                cat: "PERF".to_string(),
                ph: "M".to_string(),
                pid: 0,
                dur: 0,
                sf: 0,
                ts: 0,
                tid: node_names.len() as i64 - 1,
                args: Some(serde_json::json!({
                    "name" : node_name.to_string()
                })),
            });
            for profile in profiles {
                trace_events.push(TraceEvent {
                    name: profile.name.clone(),
                    cat: "PERF".to_string(),
                    ph: "X".to_string(),
                    ts: profile.begin,
                    dur: profile.duration.as_micros() as i64,
                    tid: node_names.len() as i64 - 1,
                    pid: 0,
                    args: None,
                    sf: 0,
                });
            }
        }
        let json = serde_json::to_string(&TraceEventRoot {
            trace_events,
            display_time_unit: "us".to_string(),
            other_data: serde_json::Value::Null,
        })
        .unwrap();
        let json_path = path.with_extension("json");
        std::fs::write(json_path, json).unwrap_or_else(|_| panic!("Failed to write time analysis results to {:?}", path));
    }
}
