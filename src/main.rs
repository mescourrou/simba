pub mod turtlebot;
mod configurable;

use std::path::Path;

use turtlebot::Turtlebot;

use crate::configurable::Configurable;

fn main() {
    println!("Hello, world!");

    let turtlebot = Turtlebot::new(String::from("Test"));

    println!("{}", turtlebot.name());

    let mut turtlebot = turtlebot;
    turtlebot.set_name(String::from("Test2"));

    println!("{}", turtlebot.name());

    let config_path = Path::new("./config.yaml");
    match turtlebot.load_config(config_path) {
        Err(e) => eprintln!("{}", e), // An Error Occurred, Please Try Again!
        _ => println!("No error"),
    }

}
