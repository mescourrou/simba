/*!
Trajectory tool.
*/

extern crate nalgebra as na;
use config_checker::macros::Check;
use libm::atan2;
use log::debug;
use na::{DMatrix, SVector};
use nalgebra::Vector2;

use crate::stateful::Stateful;

use crate::utils::geometry::*;

use serde_derive::{Deserialize, Serialize};

/// Config of the [`Trajectory`].
#[derive(Serialize, Deserialize, Debug, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct TrajectoryConfig {
    /// Ordered list of the points to follow.
    pub point_list: Vec<Vec<f32>>,
    /// Closing the loop or not.
    pub do_loop: bool,
}

impl Default for TrajectoryConfig {
    fn default() -> Self {
        Self {
            point_list: Vec::new(),
            do_loop: true,
        }
    }
}

/// Record for [`Stateful`] trait. The only dynamic element
/// is the current segment.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TrajectoryRecord {
    pub current_segment: usize,
}

impl TrajectoryRecord {
    pub fn default() -> Self {
        Self { current_segment: 0 }
    }
}

/// Tool to manage a polyline trajectory.
///
/// The matching starts at the segment 0, and continues in the order.
pub struct Trajectory {
    point_list: DMatrix<f32>,
    do_loop: bool,
    current_segment: usize,
}

impl Trajectory {
    /// Creates a new empty trajectory
    pub fn new() -> Self {
        Self {
            point_list: DMatrix::<f32>::from_vec(0, 0, Vec::<f32>::new()),
            do_loop: true,
            current_segment: 0,
        }
    }

    /// Makes a new trajectory from the given config.
    pub fn from_config(config: &TrajectoryConfig) -> Self {
        let mut trajectory = Self::new();
        trajectory.point_list = trajectory.point_list.resize(config.point_list.len(), 2, 0.);
        let mut i: usize = 0;
        for point in &config.point_list {
            let mut j: usize = 0;
            for &coord in point {
                if j >= 2 {
                    continue;
                }
                trajectory.point_list[(i, j)] = coord;
                j += 1;
            }
            i += 1;
        }
        trajectory.do_loop = config.do_loop;
        return trajectory;
    }

    // Map matching anywhere on trajectory, but not what is expected
    // pub fn map_matching(&self, point: SVector<f32, 2>) -> ((SVector<f32, 2>, SVector<f32, 2>), SVector<f32, 2>) {
    //     println!("point: {:?}", point);
    //     let mut pt1: SVector<f32, 2> = self.point_list.fixed_view::<1, 2>(0, 0).transpose();
    //     let mut best_distance = 0.;
    //     let mut best_segment = (pt1, pt1);
    //     let mut best_projected = pt1;
    //     let mut first = true;
    //     let first_point = pt1;
    //     for row in self.point_list.row_iter() {
    //         if first {
    //             first = false;
    //             continue;
    //         }
    //         let pt2: SVector<f32, 2> = row.fixed_view::<1, 2>(0, 0).transpose();
    //         let projected_point = project_point(point, pt1, pt2);
    //         let d = ((point.x - projected_point.x).powf(2.) + (point.y - projected_point.y).powf(2.)).sqrt();
    //         println!("pt1: {:?}, pt2: {:?}, projected: {:?}, d: {:?}", pt1, pt2, projected_point, d);
    //         if best_distance == 0. || d < best_distance {
    //             best_distance = d;
    //             best_segment.0 = pt1;
    //             best_segment.1 = pt2;
    //             best_projected = projected_point;
    //         }
    //         pt1 = pt2;
    //     }
    //     if self.do_loop {
    //         let projected_point = project_point(point, pt1, first_point);
    //         let d = ((point.x - projected_point.x).powf(2.) + (point.y - projected_point.y).powf(2.)).sqrt();
    //         println!("pt1: {:?}, pt2: {:?}, projected: {:?}, d: {:?}", pt1, first_point, projected_point, d);
    //         if d < best_distance {
    //             best_segment.0 = pt1;
    //             best_segment.1 = first_point;
    //             best_projected = projected_point;
    //         }
    //     }
    //     return (best_segment, best_projected);
    // }

    /// Match a point on the polyline, and returns the projected point and the segment.
    ///
    /// The segment privilieged is the previous matched segment. If the next segment is
    /// closer, it is taken, but there is no loop to go through all segments.
    ///
    /// ## Arguments
    /// * `point` - Point to match
    /// * `forward_distance` - Curvilinear distance to add
    ///
    /// ## Return
    /// * Matched segment, as a tuple of two points.
    /// * Projected point.
    /// * If the point is on the last segment.
    pub fn map_matching(
        &mut self,
        point: SVector<f32, 2>,
        forward_distance: f32,
    ) -> ((SVector<f32, 2>, SVector<f32, 2>), SVector<f32, 2>, bool) {
        let mut forward_distance = forward_distance;
        let (mut pt1, mut pt2, mut projected_point) = self.project(&point);
        while (projected_point - pt2).norm() < 1e-6 {
            if self.current_segment + 1 == self.point_list.nrows() {
                if !self.do_loop {
                    debug!("No loop so give last point");
                    return ((pt1, pt2), pt2, true);
                } else {
                    self.current_segment = 0;
                    (pt1, pt2, projected_point) = self.project(&point);
                }
            } else {
                self.current_segment += 1;
                (pt1, pt2, projected_point) = self.project(&point);
            }
        }

        let mut d =
            ((pt2.x - projected_point.x).powf(2.) + (pt2.y - projected_point.y).powf(2.)).sqrt();
        let mut start_point = projected_point;
        let mut segment = self.current_segment;
        while d < forward_distance {
            forward_distance -= d;
            segment += 1;
            if segment >= self.point_list.nrows() && self.do_loop {
                segment = 0;
            }
            if segment + 1 == self.point_list.nrows() && !self.do_loop {
                return ((pt1, pt2), pt2, true);
            }
            pt1 = pt2;
            start_point = pt1;
            pt2 = if segment + 1 >= self.point_list.nrows() {
                self.point_list.fixed_view::<1, 2>(0, 0).transpose()
            } else {
                self.point_list
                    .fixed_view::<1, 2>(segment + 1, 0)
                    .transpose()
            };
            d = ((pt2.x - pt1.x).powf(2.) + (pt2.y - pt1.y).powf(2.)).sqrt();
        }
        let segment_direction = atan2((pt2.y - pt1.y).into(), (pt2.x - pt1.x).into()) as f32;
        let projected_point = start_point
            + forward_distance * Vector2::new(segment_direction.cos(), segment_direction.sin());
        return ((pt1, pt2), projected_point, false);
    }

    /// Handle the projection of a point on the current segment. Get to the next segment if needed.
    ///
    /// ## Arguments
    /// * `point` - Point to project.
    ///
    /// ## Return
    /// * First point of the segment.
    /// * Second point of the segment.
    /// * Projected point.
    fn project(
        &self,
        point: &SVector<f32, 2>,
    ) -> (SVector<f32, 2>, SVector<f32, 2>, SVector<f32, 2>) {
        let pt1 = self
            .point_list
            .fixed_view::<1, 2>(self.current_segment, 0)
            .transpose();
        let pt2 = if self.current_segment + 1 >= self.point_list.nrows() {
            self.point_list.fixed_view::<1, 2>(0, 0).transpose()
        } else {
            self.point_list
                .fixed_view::<1, 2>(self.current_segment + 1, 0)
                .transpose()
        };
        let projected_point = project_point(point.clone(), pt1, pt2);
        (pt1, pt2, projected_point)
    }
}

impl Stateful<TrajectoryRecord> for Trajectory {
    fn record(&self) -> TrajectoryRecord {
        TrajectoryRecord {
            current_segment: self.current_segment,
        }
    }

    fn from_record(&mut self, record: TrajectoryRecord) {
        self.current_segment = record.current_segment;
    }
}

impl std::fmt::Debug for Trajectory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Trajectory {{ point_list: [").unwrap();
        let mut first = true;
        for point in self.point_list.row_iter() {
            if first {
                first = false;
            } else {
                write!(f, ", ").unwrap();
            }
            write!(f, "({}, {})", point[0], point[1]).unwrap();
        }
        write!(f, "], do_loop: {} }}", self.do_loop)
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;

    #[test]
    fn loading_from_config() {
        let config = TrajectoryConfig {
            point_list: vec![vec![0.0, 1.], vec![0., 0.], vec![1., 0.], vec![0., 1.]],
            do_loop: false,
        };

        let trajectory = Trajectory::from_config(&config);
        assert_eq!(trajectory.do_loop, config.do_loop);
        let mut i: usize = 0;
        for row in trajectory.point_list.row_iter() {
            assert_eq!(row[0], config.point_list[i][0]);
            assert_eq!(row[1], config.point_list[i][1]);
            i += 1;
        }
    }

    pub fn default_trajectory() -> Trajectory {
        let config = TrajectoryConfig {
            point_list: vec![vec![0., 0.], vec![5., 0.], vec![5., 5.], vec![0., 5.]],
            do_loop: true,
        };
        Trajectory::from_config(&config)
    }
}
