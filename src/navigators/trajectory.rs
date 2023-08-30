use ndarray::Array1;


pub struct Trajectory {
    point_list: Vec<Array1<f64>>
}

impl Trajectory {
    pub fn new() -> Trajectory {
        return Trajectory { point_list: Vec::new() };
    }
}