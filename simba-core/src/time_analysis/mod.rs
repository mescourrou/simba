mod time_analysis_config;
pub use time_analysis_config::TimeAnalysisConfig;

mod exporter;
pub use exporter::ProfileExporterConfig;
use exporter::{ProfilerExporter, TraceEventExporter};

mod execution;
use execution::{ExecutionProfile, ExecutionTree};

mod statistics;
use statistics::TimeAnalysisStatistics;

use std::{
    path::Path,
    sync::{Arc, Mutex},
    time::Duration,
};

use log::info;
use std::collections::BTreeMap;
use std::time;

use crate::{errors::SimbaResult, utils::SharedMutex};

#[derive(Debug)]
pub struct TimeAnalysisNode {
    name: String,
    depth: usize,
    execution_tree: ExecutionTree,
    current_coordinates: (i64, Vec<usize>),
}

impl TimeAnalysisNode {
    pub fn time_analysis(&mut self, time: f32, name: String) -> TimeAnalysis {
        let time_int = Duration::from_secs_f32(time).as_micros() as i64;
        if self.current_coordinates.0 != time_int {
            self.current_coordinates.0 = time_int;
            self.current_coordinates.1.clear();
        }
        if self.current_coordinates.1.len() < self.depth + 1 {
            self.current_coordinates.1.push(0);
        } else {
            self.current_coordinates.1[self.depth] += 1;
        }

        self.execution_tree
            .add(name.clone(), time_int, self.current_coordinates.1.clone());

        let ta = TimeAnalysis {
            simulated_time: time,
            begin: time::Instant::now(),
            name: self.name.clone() + "_" + &name,
            coordinates: self.current_coordinates.1.clone(),
        };
        self.depth += 1;
        ta
    }

    pub fn finished_time_analysis(&mut self, ta: TimeAnalysis) {
        let elapsed = ta.begin.elapsed();
        self.depth -= 1;
        // let indent = ta.depth*2;
        let time_int = Duration::from_secs_f32(ta.simulated_time).as_micros() as i64;
        // let coordinates = self.current_coordinates.get_mut(&thread::current().id()).unwrap();
        let coordinates = &ta.coordinates;

        let node = self
            .execution_tree
            .get_node(ta.name.clone(), time_int, coordinates.clone());
        if node.is_none() {
            panic!("Node not found: should not happen");
        }
        let node = node.unwrap();
        node.end = node.begin + elapsed.as_micros() as i64;
        node.duration = elapsed;
    }
}

#[derive(Debug)]
pub struct TimeAnalysisFactory {
    nodes: Vec<SharedMutex<TimeAnalysisNode>>,
    exporter: Box<dyn ProfilerExporter>,
    config: TimeAnalysisConfig,
}

impl TimeAnalysisFactory {
    pub fn init_from_config(config: &TimeAnalysisConfig) -> SimbaResult<Self> {
        let s = Self {
            config: config.clone(),
            exporter: match config.exporter {
                ProfileExporterConfig::TraceEventExporter => Box::new(TraceEventExporter {}),
            },
            nodes: Vec::new(),
        };
        Ok(s)
    }

    pub fn new_node(&mut self, name: String) -> SharedMutex<TimeAnalysisNode> {
        let node = TimeAnalysisNode {
            current_coordinates: (0, Vec::new()),
            execution_tree: ExecutionTree::new(),
            depth: 0,
            name,
        };
        let node = Arc::new(Mutex::new(node));
        self.nodes.push(node.clone());
        node
    }

    fn iter_execution_profiles(
        &self,
    ) -> impl Iterator<Item = (String, Vec<ExecutionProfile>)> + use<'_> {
        self.nodes.iter().map(|n| {
            let mut profiles: Vec<ExecutionProfile> = Vec::new();
            let node = n.lock().unwrap();
            for execution in node.execution_tree.iter() {
                profiles.push(ExecutionProfile {
                    name: execution.name.clone(),
                    begin: execution.begin,
                    end: execution.end,
                    depth: execution.depth,
                    duration: execution.duration,
                });
            }
            let name = node.name.clone();
            (name, profiles)
        })
    }

    pub fn save_results(&self) {
        let path = Path::new(self.config.output_path.as_str());
        info!("Saving Time Analysis results to {}", path.to_str().unwrap());
        self.exporter.export(self, path);
        self.real_time_analysis(path);
    }

    fn real_time_analysis(&self, path: &Path) {
        let path = path.with_extension("report").with_extension("csv");

        let mut stats = BTreeMap::<String, Vec<String>>::new();
        let mut node_headers = vec!["Unit:".to_string()];
        let mut track_headers = vec![self.config.analysis_unit.clone()];

        let unit_multiplier = match self.config.analysis_unit.as_str() {
            "s" => 1.,
            "ms" => 1000.,
            "us" | "µs" => 1000000.,
            "ns" => 1000000000.,
            &_ => panic!("Unknown unit!"),
        };

        for (node_name, profiles) in self.iter_execution_profiles() {
            node_headers.push(node_name.clone());
            let mut map: BTreeMap<String, Vec<f32>> = BTreeMap::new();
            for profile in profiles {
                let t = profile.duration.as_secs_f32() * unit_multiplier;
                if let Some(stat) = map.get_mut(&profile.name) {
                    stat.push(t);
                } else {
                    map.insert(profile.name.clone(), vec![t]);
                }
            }
            for (profile_name, samples) in map {
                let ta_stats = TimeAnalysisStatistics::from_array(samples);
                let ta_map = ta_stats.as_map();
                for (key, value) in ta_map {
                    if let Some(v) = stats.get_mut(&key) {
                        v.push(value);
                    } else {
                        stats.insert(key, vec![value]);
                    }
                }
                track_headers.push(profile_name);
                node_headers.push("".to_string());
            }
            node_headers.pop();
        }

        let mut writer =
            csv::Writer::from_path(path).expect("Unknown path for time analysis report");
        writer.write_record(node_headers).unwrap();
        writer.write_record(track_headers).unwrap();
        for (row_name, mut values) in stats {
            values.insert(0, row_name);
            writer.write_record(values).unwrap();
        }
    }
}

/// Save the starting time of a time analysis block and parameters
/// Will be reclaimed at the end of the block to compute elapsed time
#[derive(Clone, Debug)]
pub struct TimeAnalysis {
    simulated_time: f32,
    begin: time::Instant,
    name: String,
    coordinates: Vec<usize>,
}
