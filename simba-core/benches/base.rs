use criterion::{Criterion, criterion_group, criterion_main};
use simba::{
    logger::LogLevel,
    simulator::{Simulator, SimulatorConfig},
};
use std::path::Path;
extern crate simba;

fn run_benchmark(c: &mut Criterion, name: &str, config: SimulatorConfig) {
    let mut simulator = Simulator::from_config(&config, None)
        .inspect_err(|e| println!("Error while loading config: {}", e.detailed_error()))
        .unwrap();

    c.bench_function(name, |b| {
        b.iter(|| {
            simulator.reset(None).unwrap();
            simulator.run().expect("Error while running simulation");
        })
    });
}

fn config_monothread(c: &mut Criterion) {
    let mut config = SimulatorConfig::load_from_path(Path::new("test_config/config.yaml")).unwrap();
    config.max_time = 60.0;
    config.log.log_level = LogLevel::Off;
    config.results = None;
    config.optimization.threads = 1;
    run_benchmark(c, "run_config_monothread", config);
}

fn config_multithread(c: &mut Criterion) {
    let mut config = SimulatorConfig::load_from_path(Path::new("test_config/config.yaml")).unwrap();
    config.max_time = 60.0;
    config.log.log_level = LogLevel::Off;
    config.results = None;
    config.optimization.threads = 0;
    run_benchmark(c, "run_config_multithread", config);
}

fn config_pooled(c: &mut Criterion) {
    let mut config = SimulatorConfig::load_from_path(Path::new("test_config/config.yaml")).unwrap();
    config.max_time = 60.0;
    config.log.log_level = LogLevel::Off;
    config.results = None;
    config.optimization.threads = 2;
    run_benchmark(c, "run_config_pooled", config);
}

criterion_group!(
    benches_config,
    config_monothread,
    config_multithread,
    config_pooled
);
criterion_main!(benches_config);
