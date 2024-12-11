use std::path::Path;
use turtlebot_simulator::simulator::Simulator;

use turtlebot_simulator::time_analysis;

pub fn test_time_analysis() {
    println!("Doing some stuff...");
    // sleep 1 sec
    let one_sec = std::time::Duration::from_secs(1);
    std::thread::sleep(one_sec);
    let ta = time_analysis::time_analysis("test".to_string());
    println!("Still doing some stuff...");
    std::thread::sleep(one_sec);
    time_analysis::finished_time_analysis(ta);
    println!("One last thing...");
    std::thread::sleep(one_sec);
}

fn main() {
    time_analysis::set_turtle_name("main".to_string());
    let ta = time_analysis::time_analysis("main1".to_string());
    test_time_analysis();
    time_analysis::finished_time_analysis(ta);

    let ta = time_analysis::time_analysis("main2".to_string());
    std::thread::sleep(std::time::Duration::from_secs(1));
    time_analysis::finished_time_analysis(ta);

    // // Initialize the environment, essentially the logging part
    // Simulator::init_environment(log::LevelFilter::Debug);
    // info!("Load configuration...");
    // // Load the configuration
    // let config_path = Path::new("config_example/config.yaml");
    // let mut simulator = Simulator::from_config_path(
    //     config_path,                               //<- configuration path
    //     None,                                      //<- plugin API, to load external modules
    //     Some(Box::from(Path::new("result.json"))), //<- path to save the results (None to not save)
    //     true,                                      //<- Analyse the results
    //     false,                                     //<- Show the figures after analyse
    //     None,
    // );

    // // Show the simulator loaded configuration
    // simulator.show();

    // // Run the simulation for 60 seconds.
    // // It also save the results to "result.json",
    // // compute the results and show the figures.
    // simulator.run(60.);

    time_analysis::save_results(Path::new("time_performance.json"));
}
