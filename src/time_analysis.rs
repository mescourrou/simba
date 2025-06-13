use config_checker::macros::Check;
#[cfg(feature = "time-analysis")]
use lazy_static::lazy_static;
#[cfg(feature = "time-analysis")]
use libm::ceilf;
use serde::{Deserialize, Serialize};
use simba_macros::ToVec;

#[cfg(feature = "time-analysis")]
use std::sync::Mutex;
#[cfg(feature = "time-analysis")]
use std::thread::ThreadId;

#[cfg(feature = "time-analysis")]
use log::info;
use std::collections::HashMap;
#[cfg(feature = "time-analysis")]
use std::thread;
use std::time;
#[cfg(feature = "time-analysis")]
use std::{path::Path, time::Duration};

#[cfg(feature = "gui")]
use crate::gui::{
    utils::{enum_radio, path_finder, string_combobox},
    UIComponent,
};

#[cfg(feature = "time-analysis")]
#[allow(dead_code)]
#[derive(Debug, Clone)]
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
#[cfg_attr(not(feature = "time-analysis"), allow(dead_code, unused_variables))]
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
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut HashMap<String, String>,
        global_config: &crate::simulator::SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
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
}

#[cfg(feature = "time-analysis")]
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

#[cfg(feature = "time-analysis")]
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

#[cfg(feature = "time-analysis")]
#[derive(Debug)]
struct ExecutionTree {
    top_time_nodes: Vec<Box<ExecutionNode>>,
    keep_last: bool,
}

#[cfg(feature = "time-analysis")]
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
                    *current = Box::new(ExecutionNode::from(name, time_int, depth));
                    return;
                } else {
                    panic!("Keep_last = false not implemented yet for Execution Tree");
                }
            }
            for i in 0..depth - 1 {
                for _j in 0..coordinates[i] {
                    current = current.next.as_mut().unwrap();
                }
                if current.as_mut().subdepth_child.is_none() {
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
                .push(Box::new(ExecutionNode::from(name, time_int, depth)));
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
                for i in 0..depth - 1 {
                    for _j in 0..coordinates[i] {
                        current = current.next.as_mut().unwrap();
                    }
                    if current.as_mut().subdepth_child.is_none() {
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
            return Some(current);
        } else {
            return None;
        }
    }

    fn recusive_iter(&self, node: &Box<ExecutionNode>) -> Vec<Box<ExecutionNode>> {
        let mut nodes = Vec::new();
        let mut current = Some(node.clone());
        while current.is_some() {
            nodes.push(current.clone().unwrap());
            if let Some(child) = current.clone().unwrap().subdepth_child.as_ref() {
                nodes.append(&mut self.recusive_iter(child));
            }
            current = current.unwrap().next.clone();
        }
        nodes
    }

    pub fn iter<'a>(&'a self) -> impl Iterator<Item = Box<ExecutionNode>> + 'a {
        self.top_time_nodes
            .iter()
            .flat_map(move |node| self.recusive_iter(node).into_iter())
    }
}

#[cfg(feature = "time-analysis")]
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

#[cfg(feature = "time-analysis")]
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

#[cfg(feature = "time-analysis")]
impl TimeAnalysisStatistics {
    pub fn from_array(mut v: Vec<f32>) -> Self {
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let sum: f32 = v.iter().sum();
        let n = v.len();
        let nf32 = n as f32;
        TimeAnalysisStatistics {
            mean: sum / nf32,
            median: if n % 2 == 0 {
                (v[n / 2] + v[n / 2 - 1]) / 2.
            } else {
                v[(n - 1) / 2]
            },
            max: v
                .iter()
                .max_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap_or(&f32::INFINITY)
                .clone(),
            min: v
                .iter()
                .min_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap_or(&0.)
                .clone(),
            n: (n as u32),
            q1: v[(ceilf(nf32 / 4.) as usize).min(n - 1)],
            q3: v[(ceilf(nf32 * 0.75) as usize).min(n - 1)],
            q01: v[(ceilf(nf32 * 0.01) as usize).min(n - 1)],
            q99: v[(ceilf(nf32 * 0.99) as usize).min(n - 1)],
        }
    }

    pub fn as_map(&self) -> HashMap<String, String> {
        let mut map = HashMap::<String, String>::new();
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

#[cfg(feature = "time-analysis")]
#[derive(Debug)]
struct TimeAnalysisFactory {
    nodes_names: HashMap<ThreadId, String>,
    nodes_depth: HashMap<ThreadId, usize>,
    execution_tree: HashMap<String, ExecutionTree>,
    current_coordinates: HashMap<ThreadId, (i64, Vec<usize>)>,
    exporter: Box<dyn ProfilerExporter>,
    config: TimeAnalysisConfig,
}

#[cfg(feature = "time-analysis")]
impl TimeAnalysisFactory {
    fn get_instance() -> &'static Mutex<TimeAnalysisFactory> {
        lazy_static! {
            static ref FACTORY: Mutex<TimeAnalysisFactory> = {
                let m = Mutex::new(TimeAnalysisFactory {
                    nodes_names: HashMap::new(),
                    nodes_depth: HashMap::new(),
                    current_coordinates: HashMap::new(),
                    execution_tree: HashMap::new(),
                    exporter: Box::new(TraceEventExporter {}),
                    config: TimeAnalysisConfig::default(),
                });
                m
            };
        }
        &FACTORY
    }

    pub fn init_from_config(config: &TimeAnalysisConfig) {
        let factory = TimeAnalysisFactory::get_instance();
        let mut factory = factory.lock().unwrap();
        factory._init_from_config(config);
    }

    fn _init_from_config(&mut self, config: &TimeAnalysisConfig) {
        match config.exporter {
            ProfileExporterConfig::TraceEventExporter => {
                self.exporter = Box::new(TraceEventExporter {});
            }
        }
        self.config = config.clone();
    }

    pub fn set_node_name(name: String) {
        let factory = TimeAnalysisFactory::get_instance();
        let mut factory = factory.lock().unwrap();
        factory._set_node_name(name);
    }

    fn _set_node_name(&mut self, name: String) {
        self.nodes_names
            .insert(thread::current().id(), name.clone());
        self.nodes_depth.insert(thread::current().id(), 0);
        self.current_coordinates
            .insert(thread::current().id(), (0, Vec::new()));
        self.execution_tree.insert(name, ExecutionTree::new());
    }

    pub fn time_analysis(time: f32, name: String) -> TimeAnalysis {
        let factory = TimeAnalysisFactory::get_instance();
        let mut factory = factory.lock().unwrap();
        let ta = factory._time_analysis(time, name).clone();
        ta
    }

    fn _time_analysis(&mut self, time: f32, name: String) -> TimeAnalysis {
        self._time_analysis_node_name(
            time,
            name,
            self.nodes_names
                .get(&thread::current().id())
                .unwrap()
                .clone(),
        )
    }

    pub fn time_analysis_node_name(time: f32, name: String, node_name: String) -> TimeAnalysis {
        let factory = TimeAnalysisFactory::get_instance();
        let mut factory = factory.lock().unwrap();
        let ta = factory
            ._time_analysis_node_name(time, name, node_name)
            .clone();
        ta
    }

    fn _time_analysis_node_name(
        &mut self,
        time: f32,
        name: String,
        node_name: String,
    ) -> TimeAnalysis {
        let node_id = self
            .nodes_names
            .iter()
            .find_map(|(key, &ref val)| {
                if val == &node_name {
                    Some(key.clone())
                } else {
                    None
                }
            })
            .expect("Robot name not found");

        let time_int = Duration::from_secs_f32(time).as_micros() as i64;
        let depth = *self.nodes_depth.get(&thread::current().id()).unwrap();
        let current_coordinates = self.current_coordinates.get_mut(&node_id).unwrap();
        if current_coordinates.0 != time_int {
            current_coordinates.0 = time_int;
            current_coordinates.1.clear();
        }
        if current_coordinates.1.len() < depth + 1 {
            current_coordinates.1.push(0);
        } else {
            current_coordinates.1[depth] += 1;
        }

        self.execution_tree.get_mut(&node_name).unwrap().add(
            name.clone(),
            time_int,
            current_coordinates.1.clone(),
        );

        let ta = TimeAnalysis {
            simulated_time: time,
            begin: time::Instant::now(),
            name: node_name.clone() + "_" + &name,
            coordinates: current_coordinates.1.clone(),
        };
        *self.nodes_depth.get_mut(&thread::current().id()).unwrap() += 1;
        ta
    }

    pub fn finished_time_analysis(ta: TimeAnalysis) {
        let elapsed = ta.begin.elapsed();
        let factory = TimeAnalysisFactory::get_instance();
        let mut factory = factory.lock().unwrap();
        factory._finished_time_analysis(ta, elapsed);
    }

    fn _finished_time_analysis(&mut self, ta: TimeAnalysis, elapsed: time::Duration) {
        let node_name = self.nodes_names.get(&thread::current().id()).unwrap();
        *self.nodes_depth.get_mut(&thread::current().id()).unwrap() -= 1;
        // let indent = ta.depth*2;
        let time_int = Duration::from_secs_f32(ta.simulated_time).as_micros() as i64;
        // let coordinates = self.current_coordinates.get_mut(&thread::current().id()).unwrap();
        let coordinates = &ta.coordinates;

        let node = self
            .execution_tree
            .get_mut(&node_name.clone())
            .unwrap()
            .get_node(ta.name.clone(), time_int, coordinates.clone());
        if node.is_none() {
            panic!("Node not found: should not happen");
        }
        let node = node.unwrap();
        node.end = node.begin + elapsed.as_micros() as i64;
        node.duration = elapsed;
    }

    pub fn save_results() {
        let factory = TimeAnalysisFactory::get_instance();
        let factory = factory.lock().unwrap();
        factory._save_results();
    }

    pub fn iter_execution_profiles(
        &self,
    ) -> impl Iterator<Item = (&String, Vec<ExecutionProfile>)> {
        self.execution_tree.iter().map(|(k, v)| {
            let mut profiles = Vec::new();
            for node in v.iter() {
                profiles.push(ExecutionProfile {
                    name: node.name.clone(),
                    begin: node.begin,
                    end: node.end,
                    depth: node.depth,
                    duration: node.duration,
                });
            }
            (k, profiles)
        })
    }

    fn _save_results(&self) {
        let path = Path::new(self.config.output_path.as_str());
        info!("Saving Time Analysis results to {}", path.to_str().unwrap());
        self.exporter.export(self, path);
        self.real_time_analysis(path);
    }

    fn real_time_analysis(&self, path: &Path) {
        let path = path.with_extension("report").with_extension("csv");

        let mut stats = HashMap::<String, Vec<String>>::new();
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
            let mut map: HashMap<String, Vec<f32>> = HashMap::new();
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

// Expose the function depending on the compilation feature "time-analysis"
// Feature enabled
#[cfg(feature = "time-analysis")]
pub fn init_from_config(config: &TimeAnalysisConfig) {
    TimeAnalysisFactory::init_from_config(config);
}

#[cfg(feature = "time-analysis")]
pub fn set_node_name(name: String) {
    TimeAnalysisFactory::set_node_name(name);
}

#[cfg(feature = "time-analysis")]
pub fn time_analysis(time: f32, name: String) -> TimeAnalysis {
    TimeAnalysisFactory::time_analysis(time, name)
}

#[cfg(feature = "time-analysis")]
pub fn time_analysis_node_name(time: f32, name: String, node_name: String) -> TimeAnalysis {
    TimeAnalysisFactory::time_analysis_node_name(time, name, node_name)
}

#[cfg(feature = "time-analysis")]
pub fn finished_time_analysis(ta: TimeAnalysis) {
    TimeAnalysisFactory::finished_time_analysis(ta);
}

#[cfg(feature = "time-analysis")]
pub fn save_results() {
    TimeAnalysisFactory::save_results();
}

// Feature disabled
#[cfg(not(feature = "time-analysis"))]
pub fn init_from_config(_config: &TimeAnalysisConfig) {}

#[cfg(not(feature = "time-analysis"))]
pub fn set_node_name(_name: String) {}

#[cfg(not(feature = "time-analysis"))]
pub fn time_analysis(time: f32, name: String) -> TimeAnalysis {
    TimeAnalysis {
        simulated_time: time,
        begin: time::Instant::now(),
        name,
        coordinates: Vec::new(),
    }
}

#[cfg(not(feature = "time-analysis"))]
pub fn time_analysis_node_name(time: f32, name: String, _node_name: String) -> TimeAnalysis {
    TimeAnalysis {
        simulated_time: time,
        begin: time::Instant::now(),
        name,
        coordinates: Vec::new(),
    }
}

#[cfg(not(feature = "time-analysis"))]
pub fn finished_time_analysis(_ta: TimeAnalysis) {}

#[cfg(not(feature = "time-analysis"))]
pub fn save_results() {}

//////////////////////////////////////////////////

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[cfg_attr(feature = "gui", derive(ToVec))]
pub enum ProfileExporterConfig {
    TraceEventExporter,
}

#[derive(Clone, Debug)]
#[cfg_attr(not(feature = "time-analysis"), allow(dead_code, unused_variables))]
pub struct TimeAnalysis {
    simulated_time: f32,
    begin: time::Instant,
    name: String,
    coordinates: Vec<usize>,
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
    #[serde(rename = "traceEvents")]
    trace_events: Vec<TraceEvent>,
    #[serde(rename = "displayTimeUnit")]
    display_time_unit: String,
    #[serde(rename = "otherData")]
    other_data: serde_json::Value,
}

#[cfg(feature = "time-analysis")]
impl ProfilerExporter for TraceEventExporter {
    fn export(&self, taf: &TimeAnalysisFactory, path: &Path) {
        let mut trace_events = Vec::new();
        let mut node_names = Vec::new();
        for (node_name, profiles) in taf.iter_execution_profiles() {
            node_names.push(node_name);
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
            trace_events: trace_events,
            display_time_unit: "us".to_string(),
            other_data: serde_json::Value::Null,
        })
        .unwrap();
        let json_path = path.with_extension("json");
        std::fs::write(json_path, json).unwrap();
    }
}
