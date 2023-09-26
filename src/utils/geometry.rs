extern crate nalgebra as na;
use na::SVector;

pub fn project_point(point: SVector<f32, 2>, p1: SVector<f32, 2>, p2: SVector<f32, 2>) -> SVector::<f32, 2> {
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
        y_1 + projected_point_distance * y_n
    )
}