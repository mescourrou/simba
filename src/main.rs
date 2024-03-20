use turtlebot_simulator::simulator::Simulator;
use turtlebot_simulator::gui;
use std::path::Path;
use turtlebot_simulator::test_config;


use std::sync::mpsc;
use std::thread;
use std::time::Duration;

fn main() {
    println!("Hello, world!");


    let config_path = Path::new("configs/config.yaml");
    let mut simulator = Simulator::from_config_path(config_path, None);

    simulator.show();

    simulator.run(1.);

    // gui::run_gui();
    

    // test_config::test();

    // let (tx, rx) = mpsc::channel();

    // let tx1 = tx.clone();
    // thread::spawn(move || {
    //     let vals = vec![
    //         String::from("hi"),
    //         String::from("from"),
    //         String::from("the"),
    //         String::from("thread"),
    //     ];

    //     for val in vals {
    //         tx1.send(val).unwrap();
    //         thread::sleep(Duration::from_secs(1));
    //     }
    // });

    // thread::spawn(move || {
    //     let vals = vec![
    //         String::from("more"),
    //         String::from("messages"),
    //         String::from("for"),
    //         String::from("you"),
    //     ];

    //     for val in vals {
    //         tx.send(val).unwrap();
    //         thread::sleep(Duration::from_secs(1));
    //     }
    // });

    // thread::spawn(move || {
    //     for received in rx {
    //         println!("Got: {}", received);
    //     }
    // });

    // thread::sleep(Duration::from_secs(4));

}
