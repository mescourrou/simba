use config_checker::macros::Check;
use libm::ceilf;
use serde::{Deserialize, Serialize};
#[cfg(feature = "gui")]
use simba_macros::{EnumToString, ToVec};

use std::{
    path::Path,
    sync::{Arc, Mutex},
    time::Duration,
};

use log::info;
use std::collections::BTreeMap;
use std::time;

use crate::errors::SimbaResult;
#[cfg(feature = "gui")]
use crate::gui::{
    utils::{enum_radio, path_finder, string_combobox},
    UIComponent,
};

#[derive(Debug, Clone)]
#[allow(dead_code)]
struct ExecutionProfile {
    name: String,
    begin: i64, // micro
    end: i64,   // micro
    depth: usize,
    duration: time::Duration,
}

#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct TimeAnalysisConfig {
    pub exporter: ProfileExporterConfig,
    pub keep_last: bool,
    pub output_path: String,
    #[convert(as_str)]
    #[check(inside("s", "ms", "us", "µs", "ns"))]
    pub analysis_unit: String,
}

impl Default for TimeAnalysisConfig {
    fn default() -> Self {
        TimeAnalysisConfig {
            exporter: ProfileExporterConfig::TraceEventExporter,
            keep_last: true,
            output_path: "time_performance".to_string(),
            analysis_unit: "s".to_string(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for TimeAnalysisConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _buffer_stack: &mut BTreeMap<String, String>,
        global_config: &crate::simulator::SimulatorConfig,
        _current_node_name: Option<&String>,
        _unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Time Analysis").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Exporter:");
                enum_radio(ui, &mut self.exporter);
            });

            ui.horizontal(|ui| {
                ui.label("Output path: ");
                path_finder(ui, &mut self.output_path, &global_config.base_path);
            });

            ui.horizontal(|ui| {
                ui.label("Analysis unit:");
                string_combobox(
                    ui,
                    &vec![
                        "s".to_string(),
                        "ms".to_string(),
                        "us".to_string(),
                        "ns".to_string(),
                    ],
                    &mut self.analysis_unit,
                    "time-analysis-unit",
                );
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        egui::CollapsingHeader::new("Time Analysis").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!("Exporter: {}", self.exporter));
            });

            ui.horizontal(|ui| {
                ui.label(format!("Output path: {}", self.output_path));
            });

            ui.horizontal(|ui| {
                ui.label(format!("Analysis unit: {}", self.analysis_unit));
            });
        });
    }
}

#[derive(Debug, Clone)]
struct ExecutionNode {
    name: String,
    begin: i64,
    end: i64,
    depth: usize,
    duration: time::Duration,
    next: Option<Box<ExecutionNode>>,
    subdepth_child: Option<Box<ExecutionNode>>,
}

impl ExecutionNode {
    pub fn from(name: String, begin: i64, depth: usize) -> Self {
        ExecutionNode {
            name,
            begin,
            end: begin,
            depth,
            duration: time::Duration::from_secs(0),
            next: None,
            subdepth_child: None,
        }
    }
}

#[derive(Debug)]
struct ExecutionTree {
    top_time_nodes: Vec<ExecutionNode>,
    keep_last: bool,
}

impl ExecutionTree {
    pub fn new() -> Self {
        ExecutionTree {
            top_time_nodes: Vec::new(),
            keep_last: true,
        }
    }

    pub fn add(&mut self, name: String, time_int: i64, coordinates: Vec<usize>) {
        let depth = coordinates.len();

        // Get the right timed node:
        let current = self
            .top_time_nodes
            .iter_mut()
            .find(|node| node.begin == time_int);

        if let Some(mut current) = current {
            if coordinates[0] == 0 {
                if self.keep_last {
                    *current = ExecutionNode::from(name, time_int, depth);
                    return;
                } else {
                    panic!("Keep_last = false not implemented yet for Execution Tree");
                }
            }
            for coord in coordinates.iter().take(depth - 1) {
                for _j in 0..*coord {
                    current = current.next.as_mut().unwrap();
                }
                if current.subdepth_child.is_none() {
                    let time_int = current.begin;
                    current.subdepth_child =
                        Some(Box::new(ExecutionNode::from(name.clone(), time_int, depth)));
                    return;
                } else {
                    current = current.subdepth_child.as_mut().unwrap();
                }
            }
            for _j in 0..coordinates[depth - 1] - 1 {
                current = current.next.as_mut().unwrap();
            }
            let time_int = current.end;
            current.next = Some(Box::new(ExecutionNode::from(name, time_int, depth)));
        } else {
            self.top_time_nodes
                .push(ExecutionNode::from(name, time_int, depth));
        }
    }

    pub fn get_node(
        &mut self,
        _name: String,
        time_int: i64,
        coordinates: Vec<usize>,
    ) -> Option<&mut ExecutionNode> {
        let depth = coordinates.len();

        // Get the right timed node:
        let current = self
            .top_time_nodes
            .iter_mut()
            .find(|node| node.begin == time_int);

        if let Some(mut current) = current {
            if depth >= 2 {
                for coord in coordinates.iter().take(depth - 1) {
                    for _j in 0..*coord {
                        current = current.next.as_mut().unwrap();
                    }
                    if current.subdepth_child.is_none() {
                        return None;
                    } else {
                        current = current.subdepth_child.as_mut().unwrap();
                    }
                }
            }
            if *coordinates.last().unwrap() == 0 {
                return Some(current);
            }
            for _j in 0..*coordinates.last().unwrap() {
                current = current.next.as_mut().unwrap();
            }
            Some(current)
        } else {
            None
        }
    }

    fn recusive_iter(node: &ExecutionNode) -> Vec<ExecutionNode> {
        let mut nodes = Vec::new();
        let mut current = Some(node.clone());
        while current.is_some() {
            nodes.push(current.clone().unwrap());
            if let Some(child) = current.clone().unwrap().subdepth_child.as_deref() {
                nodes.append(&mut ExecutionTree::recusive_iter(child));
            }
            current = current.unwrap().next.as_deref().cloned();
        }
        nodes
    }

    pub fn iter<'a>(&'a self) -> impl Iterator<Item = ExecutionNode> + 'a {
        self.top_time_nodes
            .iter()
            .flat_map(move |node| ExecutionTree::recusive_iter(node).into_iter())
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    pub fn execution_tree_add() {
        let mut exe_tree = ExecutionTree::new();
        exe_tree.add("test0".to_string(), 1, vec![0]);
        assert!(exe_tree.top_time_nodes.len() == 1);
        assert!(exe_tree.top_time_nodes[0].name == "test0");
        assert!(exe_tree.top_time_nodes[0].next.is_none());
        assert!(exe_tree.top_time_nodes[0].subdepth_child.is_none());
        println!("{:?}", exe_tree);
        exe_tree.add("test1".to_string(), 1, vec![1]);
        assert!(exe_tree.top_time_nodes.len() == 1);
        assert!(exe_tree.top_time_nodes[0].next.as_ref().unwrap().name == "test1");
        assert!(exe_tree.top_time_nodes[0]
            .next
            .as_ref()
            .unwrap()
            .next
            .is_none());
        assert!(exe_tree.top_time_nodes[0]
            .next
            .as_ref()
            .unwrap()
            .subdepth_child
            .is_none());
        println!("{:?}", exe_tree);
        exe_tree.add("test2".to_string(), 1, vec![1, 0]);
        assert!(exe_tree.top_time_nodes.len() == 1);
        assert!(exe_tree.top_time_nodes[0].next.as_ref().unwrap().name == "test1");
        assert!(
            exe_tree.top_time_nodes[0]
                .next
                .as_ref()
                .unwrap()
                .subdepth_child
                .as_ref()
                .unwrap()
                .name
                == "test2"
        );
        assert!(exe_tree.top_time_nodes[0]
            .next
            .as_ref()
            .unwrap()
            .subdepth_child
            .as_ref()
            .unwrap()
            .next
            .is_none());
        assert!(exe_tree.top_time_nodes[0]
            .next
            .as_ref()
            .unwrap()
            .subdepth_child
            .as_ref()
            .unwrap()
            .subdepth_child
            .is_none());
        println!("{:?}", exe_tree);
        exe_tree.add("test3".to_string(), 1, vec![1, 1]);
        assert!(exe_tree.top_time_nodes.len() == 1);
        assert!(exe_tree.top_time_nodes[0].next.as_ref().unwrap().name == "test1");
        assert!(
            exe_tree.top_time_nodes[0]
                .next
                .as_ref()
                .unwrap()
                .subdepth_child
                .as_ref()
                .unwrap()
                .name
                == "test2"
        );
        assert!(
            exe_tree.top_time_nodes[0]
                .next
                .as_ref()
                .unwrap()
                .subdepth_child
                .as_ref()
                .unwrap()
                .next
                .as_ref()
                .unwrap()
                .name
                == "test3"
        );
        assert!(exe_tree.top_time_nodes[0]
            .next
            .as_ref()
            .unwrap()
            .subdepth_child
            .as_ref()
            .unwrap()
            .next
            .as_ref()
            .unwrap()
            .next
            .is_none());
        assert!(exe_tree.top_time_nodes[0]
            .next
            .as_ref()
            .unwrap()
            .subdepth_child
            .as_ref()
            .unwrap()
            .next
            .as_ref()
            .unwrap()
            .subdepth_child
            .is_none());
        println!("{:?}", exe_tree);
    }

    #[test]
    pub fn execution_tree_get() {
        let mut exe_tree = ExecutionTree::new();
        exe_tree.add("test0".to_string(), 1, vec![0]);
        exe_tree.add("test1".to_string(), 1, vec![1]);
        exe_tree.add("test2".to_string(), 1, vec![1, 0]);
        exe_tree.add("test3".to_string(), 1, vec![1, 1]);
        let node = exe_tree.get_node("test0".to_string(), 1, vec![0]);
        assert!(node.is_some());
        let name = &node.unwrap().name;
        assert!(name == "test0", "Name: {:?} instead of test0", name);
        let node = exe_tree.get_node("test1".to_string(), 1, vec![1]);
        assert!(node.is_some());
        let name = &node.unwrap().name;
        assert!(name == "test1", "Name: {:?} instead of test1", name);
        let node = exe_tree.get_node("test2".to_string(), 1, vec![1, 0]);
        assert!(node.is_some());
        let name = &node.unwrap().name;
        assert!(name == "test2", "Name: {:?} instead of test2", name);
        let node = exe_tree.get_node("test3".to_string(), 1, vec![1, 1]);
        assert!(node.is_some());
        let name = &node.unwrap().name;
        assert!(name == "test3", "Name: {:?} instead of test3", name);
    }
}

#[derive(Debug)]
struct TimeAnalysisStatistics {
    pub mean: f32,
    pub median: f32,
    pub min: f32,
    pub max: f32,
    pub n: u32,
    pub q1: f32,
    pub q3: f32,
    pub q99: f32,
    pub q01: f32,
}

impl TimeAnalysisStatistics {
    pub fn from_array(mut v: Vec<f32>) -> Self {
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let sum: f32 = v.iter().sum();
        let n = v.len();
        let nf32 = n as f32;
        TimeAnalysisStatistics {
            mean: sum / nf32,
            median: if n.is_multiple_of(2) {
                (v[n / 2] + v[n / 2 - 1]) / 2.
            } else {
                v[(n - 1) / 2]
            },
            max: *v
                .iter()
                .max_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap_or(&f32::INFINITY),
            min: *v
                .iter()
                .min_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap_or(&0.),
            n: (n as u32),
            q1: v[(ceilf(nf32 / 4.) as usize).min(n - 1)],
            q3: v[(ceilf(nf32 * 0.75) as usize).min(n - 1)],
            q01: v[(ceilf(nf32 * 0.01) as usize).min(n - 1)],
            q99: v[(ceilf(nf32 * 0.99) as usize).min(n - 1)],
        }
    }

    pub fn as_map(&self) -> BTreeMap<String, String> {
        let mut map = BTreeMap::<String, String>::new();
        map.insert("mean".to_string(), self.mean.to_string());
        map.insert("median".to_string(), self.median.to_string());
        map.insert("min".to_string(), self.min.to_string());
        map.insert("max".to_string(), self.max.to_string());
        map.insert("n".to_string(), self.n.to_string());
        map.insert("q1".to_string(), self.q1.to_string());
        map.insert("q3".to_string(), self.q3.to_string());
        map.insert("q99".to_string(), self.q99.to_string());
        map.insert("q01".to_string(), self.q01.to_string());
        map
    }
}

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
    nodes: Vec<Arc<Mutex<TimeAnalysisNode>>>,
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

    pub fn new_node(&mut self, name: String) -> Arc<Mutex<TimeAnalysisNode>> {
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

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[cfg_attr(feature = "gui", derive(ToVec, EnumToString))]
pub enum ProfileExporterConfig {
    TraceEventExporter,
}

#[derive(Clone, Debug)]
pub struct TimeAnalysis {
    simulated_time: f32,
    begin: time::Instant,
    name: String,
    coordinates: Vec<usize>,
}

trait ProfilerExporter: std::fmt::Debug + Sync + Send {
    fn export(&self, taf: &TimeAnalysisFactory, path: &Path);
}

#[derive(Debug)]
struct TraceEventExporter {}

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
        std::fs::write(json_path, json).unwrap();
    }
}
