use super::trajectory::{Trajectory, TrajectoryConfig};
use super::navigator::{Navigator, NavigatorRecord};

extern crate nalgebra as na;
use na::Vector3;
use libm::{atan2};

// Configuration for TrajectoryFollower
use serde_derive::{Serialize, Deserialize};

use std::path::Path;

#[derive(Serialize, Deserialize, Debug)]
#[serde(default)]
pub struct TrajectoryFollowerConfig {
    pub trajectory_path: String,
    forward_distance: f32,
    target_speed: f32
}

impl Default for TrajectoryFollowerConfig {
    fn default() -> Self {
        Self {
            trajectory_path: String::from(""),
            forward_distance: 1.0,
            target_speed: 0.5
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TrajectoryFollowerRecord {
    error: ControllerError
}

impl Default for TrajectoryFollowerRecord {
    fn default() -> Self {
        Self {
            error: ControllerError {
                lateral: 0.,
                theta: 0.,
                velocity: 0.
            }
        }
    }
}


#[derive(Debug)]
pub struct TrajectoryFollower {
    trajectory: Trajectory,
    forward_distance: f32,
    target_speed: f32,
    current_record: TrajectoryFollowerRecord
}

impl TrajectoryFollower {
    pub fn new() -> Self {
        Self {
            trajectory: Trajectory::new(),
            forward_distance: 1.0,
            target_speed: 0.5,
            current_record: TrajectoryFollowerRecord::default()
        }
    }

    pub fn from_config(config: &TrajectoryFollowerConfig) -> Self {
        let mut path = Path::new(&config.trajectory_path);
        if config.trajectory_path == "" {
            return Self::new();
        }
        let joined_path = Path::new("./configs").join(&config.trajectory_path);
        if path.is_relative() {
            path = joined_path.as_path();
        }
        TrajectoryFollower{
            trajectory: Self::load_trajectory_from_path(&path),
            forward_distance: config.forward_distance,
            target_speed: config.target_speed,
            current_record: TrajectoryFollowerRecord::default()
        }

    }

    fn load_trajectory_from_path(path: &Path) -> Trajectory {
        let trajectory: TrajectoryConfig =  match confy::load_path(&path) {
            Ok(config) => config,
            Err(error) => {
                println!("Error from Confy while loading the trajectory file {} : {}", path.display(), error);
                return Trajectory::new();
            }
        };
        Trajectory::from_config(&trajectory)
    }

    
}

use crate::state_estimators::state_estimator::State;
use crate::controllers::controller::ControllerError;

impl Navigator for TrajectoryFollower {

    fn compute_error(&mut self, state: &State) -> ControllerError {
        let forward_pose = state.pose + self.forward_distance * Vector3::new(
            state.pose.z.cos(),
            state.pose.z.sin(),
            0.
        );
        let (segment, projected_point) = self.trajectory.map_matching(forward_pose.fixed_view::<2, 1>(0, 0).into());
        println!("segment: {:?}", segment);
        let segment_angle: f32 = atan2((segment.1.y - segment.0.y).into(), (segment.1.x - segment.0.x).into()) as f32;
        let projected_point = Vector3::new(
            projected_point.x,
            projected_point.y,
            segment_angle
        );

        let forward_pose_with_segment: f32 = segment_angle - atan2((forward_pose.y - segment.0.y).into(), (forward_pose.x - segment.0.x).into()) as f32;
        
        let error = ControllerError {
            lateral: forward_pose_with_segment / forward_pose_with_segment.abs() * ((forward_pose.x - projected_point.x).powf(2.) + (forward_pose.y - projected_point.y).powf(2.)).sqrt(),
            theta: projected_point.z - forward_pose.z,
            velocity: self.target_speed - state.velocity
        };
        self.current_record = TrajectoryFollowerRecord {
            error: error.clone()
        };
        error
    }

    fn record(&self) ->  NavigatorRecord {
        NavigatorRecord::TrajectoryFollower(self.current_record.clone())
    }
}



#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use super::*;
    use super::super::trajectory;
    #[test]
    fn compute_error() {
        // let navigator = TrajectoryFollower {
        //     trajectory: trajectory::tests::default_trajectory(),
        //     forward_distance: 1.,
        //     target_speed: 1.
        // };

        // // 1st segment
        // let pose = SVector::<f32, 3>::new(0., -0.5, 0.);
        // let error = navigator.compute_error(pose);
        // assert_eq!(error[0], -0.5, "lateral error should be 0.5, but is {}", error[0]);
        // assert_eq!(error[1], 0., "angle error should be 0., but is {}", error[1]);

        // // 1st segment, with angle
        // let pose = SVector::<f32, 3>::new(1., 0.5, PI / 4.);
        // let error = navigator.compute_error(pose);
        // assert_eq!(error[0], 0.5 + navigator.forward_distance * (PI / 4.).sin(), "lateral error should be {}, but is {}", 0.5 + navigator.forward_distance * (PI / 4.).sin(), error[0]);
        // assert_eq!(error[1], PI / 4., "angle error should be {}., but is {}", PI / 4., error[1]);

        // // 2nd segment
        // let pose = SVector::<f32, 3>::new(6., 1., PI / 2.);
        // let error = navigator.compute_error(pose);
        // assert_eq!(error[0], -1., "lateral error should be -1., but is {}", error[0]);
        // assert_eq!(error[1], 0., "angle error should be 0., but is {}", error[1]);

        // // Last segment (with looping)
        // let pose = SVector::<f32, 3>::new(-1., 3., -PI / 2.);
        // let error = navigator.compute_error(pose);
        // assert_eq!(error[0], -1., "lateral error should be -1., but is {}", error[0]);
        // assert_eq!(error[1], 0., "angle error should be 0., but is {}", error[1]);

    }
}