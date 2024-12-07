/*!
Provide geometry tools.
*/

extern crate nalgebra as na;
use std::f32::consts::PI;

use na::SVector;

/// Computes the projection of a point on a segment.
///
/// If the projected point is out of the segment, the closest segment point is selected.
///
/// ## Arguments
/// * `point` -- Point to project.
/// * `p1` -- Point 1 of the segment.
/// * `p2` -- Point 2 of the segment.
///
/// ## Return
/// Projected point.
pub fn project_point(
    point: SVector<f32, 2>,
    p1: SVector<f32, 2>,
    p2: SVector<f32, 2>,
) -> SVector<f32, 2> {
    let x_1 = p1.x;
    let y_1 = p1.y;

    let x_n = p2.x - x_1;
    let y_n = p2.y - y_1;
    let d_n = (x_n * x_n + y_n * y_n).sqrt();
    let x_n = x_n / d_n;
    let y_n = y_n / d_n;

    let projected_point_distance = (point.x - x_1) * x_n + (point.y - y_1) * y_n;
    let projected_point_distance = (0.0f32).max(d_n.min(projected_point_distance));

    SVector::<f32, 2>::new(
        x_1 + projected_point_distance * x_n,
        y_1 + projected_point_distance * y_n,
    )
}

pub fn mod2pi(f: f32) -> f32 {
    let mut f = f;
    while f > PI {
        f -= 2. * PI;
    }
    while f <= -PI {
        f += 2. * PI;
    }
    f
}

/// Computes the smallest difference between two angles,
/// i.e. the difference between `a` and `b` in the range `[-PI/2, PI/2]`.
pub fn smallest_theta_diff(a: f32, b: f32) -> f32 {
    let mut diff = a - b;
    if diff > PI {
        diff = a - (b + 2. * PI);
    } else if diff <= -PI {
        diff = a + 2. * PI - b;
    }
    diff
}

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    #[test]
    pub fn test_smalles_theta_diff() {
        let a = 0.1;
        let b = 0.2;
        let diff = super::smallest_theta_diff(a, b);
        assert_eq!(diff, -0.1);
        let a = PI - 0.1;
        let b = -PI + 0.2;
        let diff = super::smallest_theta_diff(a, b);
        assert!((diff - 0.3).abs() < 1e-6);
        let a = -PI + 0.1;
        let b = PI - 0.2;
        let diff = super::smallest_theta_diff(a, b);
        assert!((diff - (-0.3)).abs() < 1e-6);
    }
}
