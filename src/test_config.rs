extern crate confy;

extern crate serde;
extern crate serde_json;

use std::collections::BTreeMap as Map;
use serde_json::Value;

use serde_derive::{Serialize, Deserialize};

use std::path::Path;

#[derive(Serialize, Deserialize, Debug)]
struct TestConfig {
    #[serde(flatten)]
    other: Map<String, Value>
}

impl Default for TestConfig {
    fn default() -> Self {
        Self {
            other: Map::new()
        }
    }
}

pub fn test() {
    let _config_path = Path::new("./test_config.yaml");
    let config: TestConfig = confy::load_path(_config_path).expect("Error during config parsing");

    println!("{:?}", config);
    
}