extern crate confy;

use serde_derive::{Serialize, Deserialize};
extern crate nalgebra as na;

use super::navigators::trajectory::{TrajectoryConfig};

#[derive(Serialize, Deserialize, Debug)]
struct ConfigGenerale {
    nom: String,
    class: ClassTypes,
    positions: TrajectoryConfig
}

impl Default for ConfigGenerale {
    fn default() -> Self {
        Self {
            nom: String::from("Bouh"),
            class: ClassTypes::Mage(ConfigMage::default()),
            positions: TrajectoryConfig::default()
        }
    }
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
struct ConfigMage {
    baton: String,
    mana: i32,
    sort_principal: String
}

impl Default for ConfigMage {
    fn default() -> Self {
        Self {
            baton: String::from("Pam"),
            mana: 12,
            sort_principal: String::from("Eclair")
        }
    }
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
struct ConfigGuerrier {
    epee: String,
    bouclier: String,
    rage: i8,
    force: f32
}

impl Default for ConfigGuerrier {
    fn default() -> Self {
        Self {
            epee: String::from("FeuEpee"),
            bouclier: String::from("Rempart"),
            rage: 3,
            force: 125.4
        }
    }
}


#[derive(Serialize, Deserialize, Debug, PartialEq)]
enum ClassTypes {
    Mage(ConfigMage),
    Guerrier(ConfigGuerrier)
}



use std::path::Path;

pub fn test() {
    let config_path = Path::new("./test_config.yaml");
    let confs:Vec<ConfigGenerale> = match confy::load_path(&config_path) {
        Ok(config) => config,
        Err(error) => {
            panic!("Error from Confy while loading the config file : {}", error)
        }
    };

    // let mut confs: Vec<ConfigGenerale> = Vec::new();
    // confs.push(ConfigGenerale {
    //     nom: String::from("Test 1"),
    //     class: ClassTypes::Guerrier(ConfigGuerrier {
    //         epee:String::from("Truc"),
    //         bouclier: String::from("Machin"),
    //         rage: 10,
    //         force: 4.4
    //     }),
    //     positions: TrajectoryConfig {
    //         point_list: vec![vec![0.,1.], vec![1., 0.]]
    //     }
    // });

    // confs.push(ConfigGenerale {
    //     nom: String::from("Test 2"),
    //     class: ClassTypes::Mage(ConfigMage {
    //         baton:String::from("Blanc"),
    //         mana: 150,
    //         sort_principal: String::from("Pluie")
    //     })
    // });

    println!("Config:");
    for conf in &confs {
        println!("{:?}", conf.positions);
        
        
    }

    let config_save_path = Path::new("./test_config2.yaml");
    let _ = confy::store_path(config_save_path, &confs);
}