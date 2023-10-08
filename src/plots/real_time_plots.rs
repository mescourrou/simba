use plotters::prelude::*;
use plotters_canvas::CanvasBackend;

pub fn test() {
    let root_drawing_area = CanvasBackend::new("one").expect("Error")
        .into_drawing_area();

    root_drawing_area.fill(&WHITE).unwrap();

    let mut chart = ChartBuilder::on(&root_drawing_area)
        .build_cartesian_2d(-3.14..3.14, -1.2..1.2)
        .unwrap();

    chart.draw_series(LineSeries::new(
        (-314..314).map(|x| x as f64 / 100.0).map(|x| (x, x.sin())),
        &RED
    )).unwrap();

}