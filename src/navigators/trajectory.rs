use ndarray::{Array1, arr1};

#[macro_use]
use serde_derive::{Serialize, Deserialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct TrajectoryConfig {
    pub point_list: Vec<Vec<f64>>
}

impl Default for TrajectoryConfig {
    fn default() -> Self {
        Self {
            point_list: Vec::new()
        }
    }
}

pub struct Trajectory {
    point_list: Vec<Array1<f64>>
}

impl Trajectory {
    pub fn new() -> Trajectory {
        return Trajectory { point_list: Vec::new() };
    }

    pub fn from_config(config: &TrajectoryConfig) -> Trajectory {
        let mut trajectory = Self::new();
        for point in &config.point_list {
            let mut point_arr = arr1(&point);
            
            trajectory.point_list.push(point_arr);
        }
        return trajectory;
    }
}

impl std::fmt::Debug for Trajectory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Trajectory [");
        let mut first = true;
        for point in &self.point_list {
            if first {
                first = false;
            } else {
                write!(f, ", ");
            }
            write!(f, "({}, {})", point[0], point[1]);
        }
        write!(f, "]")
    }
}
