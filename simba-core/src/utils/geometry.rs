/*!
Provide geometry tools.
*/

extern crate nalgebra as na;
use std::f32::consts::PI;

use na::SVector;
use nalgebra::Matrix3;

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
/// i.e. the difference between `a` and `b` in the range `[-PI/2, PI/2]` (a-b).
pub fn smallest_theta_diff(a: f32, b: f32) -> f32 {
    let mut diff = a - b;
    if diff > PI {
        diff = a - (b + 2. * PI);
    } else if diff <= -PI {
        diff = a + 2. * PI - b;
    }
    diff
}

pub fn segment_circle_intersection(
    p1: &SVector<f32, 2>,
    p2: &SVector<f32, 2>,
    center: &SVector<f32, 2>,
    radius: f32,
) -> Option<(SVector<f32, 2>, SVector<f32, 2>)> {
    // For wide landmarks, check if a part of the landmark is in range
    // Source: https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm

    let d = p2 - p1;
    let f = p1 - center;

    let a = d.dot(&d);
    let b = 2. * f.dot(&d);
    let c = f.dot(&f) - radius * radius;

    let discriminant = b * b - 4. * a * c;

    if discriminant < 0. {
        // no intersection
        return None;
    }
    // ray didn't totally miss sphere,
    // so there is a solution to
    // the equation.

    let discriminant = discriminant.sqrt();

    if a == 0. {
        // segment is a point
        let dist_sq = (p1 - center).dot(&(p1 - center));
        if dist_sq <= radius * radius {
            return Some((*p1, *p1));
        } else {
            return None;
        }
    }

    // either solution may be on or off the ray so need to test both
    // t1 is always the smaller value, because BOTH discriminant and
    // a are nonnegative.
    let t1 = (-b - discriminant) / (2. * a);
    let t2 = (-b + discriminant) / (2. * a);

    // 3x HIT cases:
    //          -o->             --|-->  |            |  --|->
    // Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit),

    // 3x MISS cases:
    //       ->  o                     o ->              | -> |
    // FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

    // t1 inside [0..1] => hit
    // t1 is the intersection, and it's closer than t2
    // (since t1 uses -b - discriminant)
    // Impale, Poke
    // or  t1 didn't intersect so we are either started
    // inside the sphere or completely past it
    if !(0. ..=1.).contains(&t1) && !(0. ..=1.).contains(&t2) {
        // no intn: FallShort, Past, CompletelyInside
        return None;
    }

    // If we are here, we have an intersection
    let mut intersect1 = p1 + t1 * d;
    if t1 < 0. {
        intersect1 = *p1;
    }
    let mut intersect2 = p1 + t2 * d;
    if t2 > 1. {
        intersect2 = *p2;
    }
    Some((intersect1, intersect2))
}

pub fn segments_intersection(
    a1: &SVector<f32, 2>,
    a2: &SVector<f32, 2>,
    b1: &SVector<f32, 2>,
    b2: &SVector<f32, 2>,
) -> Option<SVector<f32, 2>> {
    // Source: https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect // response https://stackoverflow.com/a/28390934
    let ax = a2.x - a1.x;
    let ay = a2.y - a1.y;
    let bx = b2.x - b1.x;
    let by = b2.y - b1.y;
    let d = ax * by - ay * bx;

    // parallel lines
    if d == 0.0 {
        return None;
    }

    let pos = d > 0.0;

    let ua = bx * (a1.y - b1.y) - by * (a1.x - b1.x);
    let ub = ax * (a1.y - b1.y) - ay * (a1.x - b1.x);

    if ((ua < 0.) == pos && ua != 0.) || ((ub < 0.) == pos && ub != 0.) {
        // no intersection
        return None;
    }

    if ((ua > d) == pos && ua != d) || ((ub > d) == pos && ub != d) {
        // no intersection
        return None;
    }

    // Get the intersection point\
    let ua = ua / d;
    Some(SVector::<f32, 2>::new(a1.x + ua * ax, a1.y + ua * ay))
}

pub fn segment_to_line_intersection(
    a1: &SVector<f32, 2>,
    a2: &SVector<f32, 2>,
    l1: &SVector<f32, 2>,
    l2: &SVector<f32, 2>,
) -> Option<SVector<f32, 2>> {
    // Source: https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect // response https://stackoverflow.com/a/28390934

    let ax = a2.x - a1.x;
    let ay = a2.y - a1.y;
    let bx = l2.x - l1.x;
    let by = l2.y - l1.y;
    let d = ax * by - ay * bx;

    // parallel lines
    if d == 0.0 {
        return None;
    }

    let ua = bx * (a1.y - l1.y) - by * (a1.x - l1.x);
    // let ub = ax * (a1.y - l1.y) - ay * (a1.x - l1.x);

    // Get the intersection point
    let ua = ua / d;
    Some(SVector::<f32, 2>::new(a1.x + ua * ax, a1.y + ua * ay))
}

pub fn segment_triangle_intersection(
    p1: &SVector<f32, 2>,
    p2: &SVector<f32, 2>,
    triangle_top: &SVector<f32, 2>,
    triangle_a: &SVector<f32, 2>,
    triangle_b: &SVector<f32, 2>,
) -> Option<(SVector<f32, 2>, SVector<f32, 2>)> {
    // Check if collinear triangle
    if aligned_points(triangle_top, triangle_a, triangle_b, 1e-10) {
        return None;
    }

    let mut p1h = p1.to_homogeneous();
    p1h.z = 1.;
    let mut p2h = p2.to_homogeneous();
    p2h.z = 1.;
    let mut t1h = triangle_top.to_homogeneous();
    t1h.z = 1.;
    let mut t2h = triangle_a.to_homogeneous();
    t2h.z = 1.;
    let mut t3h = triangle_b.to_homogeneous();
    t3h.z = 1.;

    // https://math.stackexchange.com/a/2385307
    // Barycentric coordinates
    let to_bary = Matrix3::<f32>::from_rows(&[
        t2h.cross(&t3h).transpose(),
        t3h.cross(&t1h).transpose(),
        t1h.cross(&t2h).transpose(),
    ]) / Matrix3::<f32>::from_columns(&[t1h, t2h, t3h]).determinant();
    let lambda1 = to_bary * p1h;
    let lambda2 = to_bary * p2h;

    // Check if both points are inside the triangle
    let p1inside = lambda1.fold(
        true,
        |acc, x| if !(0. ..=1.).contains(&x) { false } else { acc },
    );
    let p2inside = lambda2.fold(
        true,
        |acc, x| if !(0. ..=1.).contains(&x) { false } else { acc },
    );

    if p1inside && p2inside {
        return Some((*p1, *p2));
    }
    let edges = vec![
        (triangle_top, triangle_a),
        (triangle_a, triangle_b),
        (triangle_b, triangle_top),
    ];
    if p1inside || p2inside {
        // One point is inside the triangle
        let inside_point = if p1inside { p1 } else { p2 };
        let outside_point = if p1inside { p2 } else { p1 };

        // Find intersection with triangle edges
        for (e1, e2) in &edges {
            if let Some(intersection) = segments_intersection(inside_point, outside_point, e1, e2) {
                if p1inside {
                    return Some((*inside_point, intersection));
                } else {
                    return Some((intersection, *inside_point));
                }
            }
        }
    }

    // If both outside, check for edge intersections
    let mut intersections = Vec::new();
    for (e1, e2) in &edges {
        if let Some(intersection) = segments_intersection(p1, p2, e1, e2) {
            intersections.push(intersection);
        }
    }

    if intersections.len() == 3 {
        if intersections[0] == intersections[1] {
            intersections = vec![intersections[0], intersections[2]];
        } else if intersections[0] == intersections[2] {
            intersections = vec![intersections[0], intersections[1]];
        } else if intersections[1] == intersections[2] {
            intersections = vec![intersections[1], intersections[0]];
        }
    }
    if intersections.len() >= 2 {
        if (intersections[0] - p1).norm() > (intersections[1] - p1).norm() {
            intersections.reverse();
        }
        return Some((intersections[0], intersections[1]));
    }
    assert!(
        intersections.is_empty(),
        "Bug found! intersections.len() = {}",
        intersections.len()
    );
    None
}

pub fn aligned_points(
    p1: &SVector<f32, 2>,
    p2: &SVector<f32, 2>,
    p3: &SVector<f32, 2>,
    tolerance: f32,
) -> bool {
    let area = (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y)).abs() / 2.0;
    area < tolerance
}

#[cfg(test)]
mod tests {
    use std::{f32::consts::PI, iter::zip};

    use nalgebra::Vector2;

    #[test]
    pub fn test_smallest_theta_diff() {
        let a = 0.1;
        let b = 0.2;
        let diff = super::smallest_theta_diff(a, b);
        assert!((diff - (-0.1)).abs() < 1e-6);
        let a = PI - 0.1;
        let b = -PI + 0.2;
        let diff = super::smallest_theta_diff(a, b);
        assert!((diff - (-0.3)).abs() < 1e-6, "Diff = {diff}");
        let a = -PI + 0.1;
        let b = PI - 0.2;
        let diff = super::smallest_theta_diff(a, b);
        assert!((diff - 0.3).abs() < 1e-6, "Diff = {diff}");

        let a = -PI + 0.01;
        let b = PI;
        let diff = super::smallest_theta_diff(a, b);
        assert!((diff - 0.01).abs() < 1e-6, "Diff = {diff}");

        let a = -PI;
        let b = PI;
        let diff = super::smallest_theta_diff(a, b);
        assert!(diff.abs() < 1e-6, "Diff = {diff}");
    }

    #[test]
    pub fn segment_circle_intersection() {
        let segments = vec![
            (Vector2::new(-7., 6.), Vector2::new(10., 6.)),
            (Vector2::new(-10., 2.), Vector2::new(8., -6.)),
            (Vector2::new(6., 4.), Vector2::new(-8., -8.)),
            (Vector2::new(-8., 6.), Vector2::new(-2., 2.)),
            (Vector2::new(-2., -4.), Vector2::new(-2., 10.)),
            (Vector2::new(12., -8.), Vector2::new(-2., 10.)),
            (Vector2::new(0., 0.), Vector2::new(4., 0.)),
            (Vector2::new(2., 0.), Vector2::new(2., 0.)),
            (Vector2::new(-2., 2.), Vector2::new(2., -2.)),
        ];
        let centers_radius = vec![
            (Vector2::new(0., 0.), 7.),
            (Vector2::new(0., 0.), 7.),
            (Vector2::new(0., 0.), 4.),
            (Vector2::new(-2., 0.), 6.),
            (Vector2::new(-2., 0.), 6.),
            (Vector2::new(-2., 0.), 6.),
            (Vector2::new(-2., 0.), 6.),
            (Vector2::new(0., 0.), 4.),
            (Vector2::new(0., 0.), 0.),
        ];
        let expected_results = vec![
            Some((Vector2::new(-3.60555, 6.), Vector2::new(3.60555, 6.))),
            Some((Vector2::new(-6.9695, 0.6531), Vector2::new(5.1550, -4.7356))),
            Some((Vector2::new(3.5294, 1.8824), Vector2::new(-2.4, -3.2))),
            Some((Vector2::new(-5.8734, 4.5822), Vector2::new(-2., 2.))),
            Some((Vector2::new(-2., -4.), Vector2::new(-2., 6.))),
            None,
            Some((Vector2::new(0., 0.), Vector2::new(4., 0.))),
            Some((Vector2::new(2., 0.), Vector2::new(2., 0.))),
            Some((Vector2::new(0., 0.), Vector2::new(0., 0.))),
        ];

        for (((p1, p2), (center, radius)), expected) in
            zip(zip(segments, centers_radius), expected_results)
        {
            let result = super::segment_circle_intersection(&p1, &p2, &center, radius);
            match (result, expected) {
                (Some((res1, res2)), Some((exp1, exp2))) => {
                    assert!(
                        (res1 - exp1).norm() < 1e-3,
                        "res1: {res1:?}, exp1: {exp1:?}"
                    );
                    assert!(
                        (res2 - exp2).norm() < 1e-3,
                        "res2: {res2:?}, exp2: {exp2:?}"
                    );
                }
                (None, None) => {}
                _ => panic!(
                    "Result and expected do not match: result={result:?}, expected={expected:?}"
                ),
            }
        }
    }

    #[test]
    pub fn segments_intersection() {
        let segments_a = vec![
            (Vector2::new(-2., -2.), Vector2::new(6., 4.)), // 1
            (Vector2::new(-2., 2.), Vector2::new(6., 4.)),  // 2
            (Vector2::new(4., -4.), Vector2::new(6., 4.)),  // 3
            (Vector2::new(2., -4.), Vector2::new(2., 4.)),  // 4
            (Vector2::new(-2., 4.), Vector2::new(6., 4.)),  // 5
            (Vector2::new(-2., 4.), Vector2::new(6., 4.)),  // 6
            (Vector2::new(-2., 4.), Vector2::new(2., 0.)),  // 7
            (Vector2::new(-2., 4.), Vector2::new(2., 0.)),  // 8
            (Vector2::new(-2., 4.), Vector2::new(4., 4.)),  // 9
            (Vector2::new(-2., 4.), Vector2::new(4., 4.)),  // 10
            (Vector2::new(-2., 4.), Vector2::new(4., 4.)),  // 11
        ];
        let segments_b = vec![
            (Vector2::new(-2., 2.), Vector2::new(4., -2.)),   // 1
            (Vector2::new(-2., 2.), Vector2::new(4., -2.)),   // 2
            (Vector2::new(-2., 2.), Vector2::new(4., -2.)),   // 3
            (Vector2::new(-2., 2.), Vector2::new(4., 2.)),    // 4
            (Vector2::new(4., 2.), Vector2::new(4., -2.)),    // 5
            (Vector2::new(6., 4.), Vector2::new(4., -2.)),    // 6
            (Vector2::new(6., 4.), Vector2::new(0., -2.)),    // 7
            (Vector2::new(10., 10.), Vector2::new(-4., -8.)), // 8
            (Vector2::new(4., 0.), Vector2::new(-2., 0.)),    // 9
            (Vector2::new(4., 0.), Vector2::new(-2., 4.)),    // 10
            (Vector2::new(4., 0.), Vector2::new(4., 4.)),     // 11
        ];
        let expected_results = vec![
            Some(Vector2::new(0.8235, 0.1176)), // 1
            Some(Vector2::new(-2., 2.)),        // 2
            None,                               // 3
            Some(Vector2::new(2., 2.)),         // 4
            None,                               // 5
            Some(Vector2::new(6., 4.)),         // 6
            Some(Vector2::new(2., 0.)),         // 7
            None,                               // 8
            None,                               // 9
            Some(Vector2::new(-2., 4.)),        // 10
            Some(Vector2::new(4., 4.)),         // 11
        ];

        for (((a1, a2), (b1, b2)), expected) in zip(zip(segments_a, segments_b), expected_results) {
            let result = super::segments_intersection(&a1, &a2, &b1, &b2);
            match (result, expected) {
                (Some(res), Some(exp)) => {
                    assert!((res - exp).norm() < 1e-3, "res: {res:?}, exp: {exp:?}");
                }
                (None, None) => {}
                _ => panic!(
                    "Result and expected do not match: result={result:?}, expected={expected:?}"
                ),
            }
        }
    }

    #[test]
    pub fn segment_triangle_intersection() {
        let segments = vec![
            (Vector2::new(4., 0.), Vector2::new(6., 4.)),     // 1
            (Vector2::new(4., 0.), Vector2::new(6., 6.)),     // 2
            (Vector2::new(4., 0.), Vector2::new(10., 6.)),    // 3
            (Vector2::new(0., 6.), Vector2::new(12., 6.)),    // 4
            (Vector2::new(0., 6.), Vector2::new(12., 0.)),    // 5
            (Vector2::new(0., 6.), Vector2::new(14., -8.)),   // 6
            (Vector2::new(4., 2.), Vector2::new(14., -8.)),   // 7
            (Vector2::new(6., -4.), Vector2::new(-2., 2.)),   // 8
            (Vector2::new(6., -4.), Vector2::new(4., 2.)),    // 9
            (Vector2::new(12., -6.), Vector2::new(4., 2.)),   // 10
            (Vector2::new(20., 2.), Vector2::new(2., 8.)),    // 11
            (Vector2::new(10., 10.), Vector2::new(-10., 4.)), // 12
            (Vector2::new(4., 0.), Vector2::new(-10., 0.)),   // 13
        ];
        let triangles = vec![
            (
                Vector2::new(-2., 0.),
                Vector2::new(6., 6.),
                Vector2::new(10., -2.),
            ), // 1
            (
                Vector2::new(-2., 0.),
                Vector2::new(6., 6.),
                Vector2::new(10., -2.),
            ), // 2
            (
                Vector2::new(-2., 0.),
                Vector2::new(6., 6.),
                Vector2::new(10., -2.),
            ), // 3
            (
                Vector2::new(-2., 0.),
                Vector2::new(6., 6.),
                Vector2::new(10., -2.),
            ), // 4
            (
                Vector2::new(-2., 0.),
                Vector2::new(6., 6.),
                Vector2::new(10., -2.),
            ), // 5
            (
                Vector2::new(-2., 0.),
                Vector2::new(6., 6.),
                Vector2::new(8., -2.),
            ), // 6
            (
                Vector2::new(-2., 0.),
                Vector2::new(6., 6.),
                Vector2::new(8., -2.),
            ), // 7
            (
                Vector2::new(-2., 0.),
                Vector2::new(6., 6.),
                Vector2::new(8., -2.),
            ), // 8
            (
                Vector2::new(-2., 0.),
                Vector2::new(6., 6.),
                Vector2::new(8., -2.),
            ), // 9
            (
                Vector2::new(-2., 0.),
                Vector2::new(6., 6.),
                Vector2::new(8., -2.),
            ), // 10
            (
                Vector2::new(-2., 0.),
                Vector2::new(6., 6.),
                Vector2::new(8., -2.),
            ), // 11
            (
                Vector2::new(-2., 0.),
                Vector2::new(6., 6.),
                Vector2::new(8., -2.),
            ), // 12
            (
                Vector2::new(-2., 0.),
                Vector2::new(6., 6.),
                Vector2::new(8., -2.),
            ), // 13
        ];
        let expected_results = vec![
            Some((Vector2::new(4., 0.), Vector2::new(6., 4.))), // 1
            Some((Vector2::new(4., 0.), Vector2::new(6., 6.))), // 2
            Some((Vector2::new(4., 0.), Vector2::new(7.3333, 3.3333))), // 3
            Some((Vector2::new(6., 6.), Vector2::new(6., 6.))), // 4
            Some((Vector2::new(3.6, 4.2), Vector2::new(8., 2.))), // 5
            Some((Vector2::new(2.5714, 3.4286), Vector2::new(8., -2.))), // 6
            Some((Vector2::new(4., 2.), Vector2::new(8., -2.))), // 7
            Some((Vector2::new(1.6364, -0.7273), Vector2::new(-0.6667, 1.))), // 8
            Some((Vector2::new(5.1429, -1.4286), Vector2::new(4., 2.))), // 9
            Some((Vector2::new(8., -2.), Vector2::new(4., 2.))), // 10
            None,                                               // 11
            None,                                               // 12
            Some((Vector2::new(4., 0.), Vector2::new(-2., 0.))), // 13
        ];

        for (((p1, p2), (t1, t2, t3)), expected) in zip(zip(segments, triangles), expected_results)
        {
            let result = super::segment_triangle_intersection(&p1, &p2, &t1, &t2, &t3);
            match (result, expected) {
                (Some((res1, res2)), Some((exp1, exp2))) => {
                    assert!(
                        (res1 - exp1).norm() < 1e-3 && (res2 - exp2).norm() < 1e-3,
                        "res: ({:?}, {:?}), exp: ({:?}, {:?})",
                        res1,
                        res2,
                        exp1,
                        exp2
                    );
                }
                (None, None) => {}
                _ => panic!(
                    "Result and expected do not match: result={result:?}, expected={expected:?}"
                ),
            }
        }
    }
}
