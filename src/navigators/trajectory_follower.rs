use super::trajectory::{Trajectory, TrajectoryConfig};
use super::navigator::Navigator;

extern crate nalgebra as na;
use na::{SVector, Vector3};
use libm::{atan2};

// Configuration for TrajectoryFollower
use serde_derive::{Serialize, Deserialize};

use std::path::Path;

#[derive(Serialize, Deserialize, Debug)]
#[serde(default)]
pub struct TrajectoryFollowerConfig {
    pub trajectory_path: String,
    forward_distance: f32
}

impl Default for TrajectoryFollowerConfig {
    fn default() -> Self {
        Self {
            trajectory_path: String::from(""),
            forward_distance: 1.0
        }
    }
}




#[derive(Debug)]
pub struct TrajectoryFollower {
    trajectory: Trajectory,
    forward_distance: f32
}

impl TrajectoryFollower {
    pub fn new() -> Self {
        Self {
            trajectory: Trajectory::new(),
            forward_distance: 1.0
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
            forward_distance: config.forward_distance
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

impl Navigator for TrajectoryFollower {
    fn compute_error(&self, pose: SVector<f32, 3>) -> SVector<f32, 2> {
        let forward_pose = pose + self.forward_distance * Vector3::new(
            pose.z.cos(),
            pose.z.sin(),
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

        let forward_pose_with_segment: f32 = atan2((forward_pose.y - segment.0.y).into(), (forward_pose.x - segment.0.x).into()) as f32 - segment_angle;
        SVector::<f32,2>::new(
            forward_pose_with_segment / forward_pose_with_segment.abs() * ((forward_pose.x - projected_point.x).powf(2.) + (forward_pose.y - projected_point.y).powf(2.)).sqrt(),
            forward_pose.z - projected_point.z
        )
    }
}



#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use super::*;
    use super::super::trajectory;
    #[test]
    fn compute_error() {
        let navigator = TrajectoryFollower {
            trajectory: trajectory::tests::default_trajectory(),
            forward_distance: 1.
        };

        // 1st segment
        let pose = SVector::<f32, 3>::new(0., -0.5, 0.);
        let error = navigator.compute_error(pose);
        assert_eq!(error[0], -0.5, "lateral error should be 0.5, but is {}", error[0]);
        assert_eq!(error[1], 0., "angle error should be 0., but is {}", error[1]);

        // 1st segment, with angle
        let pose = SVector::<f32, 3>::new(1., 0.5, PI / 4.);
        let error = navigator.compute_error(pose);
        assert_eq!(error[0], 0.5 + navigator.forward_distance * (PI / 4.).sin(), "lateral error should be {}, but is {}", 0.5 + navigator.forward_distance * (PI / 4.).sin(), error[0]);
        assert_eq!(error[1], PI / 4., "angle error should be {}., but is {}", PI / 4., error[1]);

        // 2nd segment
        let pose = SVector::<f32, 3>::new(6., 1., PI / 2.);
        let error = navigator.compute_error(pose);
        assert_eq!(error[0], -1., "lateral error should be -1., but is {}", error[0]);
        assert_eq!(error[1], 0., "angle error should be 0., but is {}", error[1]);

        // Last segment (with looping)
        let pose = SVector::<f32, 3>::new(-1., 3., -PI / 2.);
        let error = navigator.compute_error(pose);
        assert_eq!(error[0], -1., "lateral error should be -1., but is {}", error[0]);
        assert_eq!(error[1], 0., "angle error should be 0., but is {}", error[1]);

    }
}