/*!
Module providing different function tools and data structures.
*/

pub mod barrier;
pub mod confy;
pub mod determinist_random_variable;
pub mod distributions;
pub mod enum_tools;
pub mod geometry;
pub mod maths;
pub mod occupancy_grid;
pub mod python;
pub mod rfc;
pub mod time_ordered_data;

use serde::Serializer;
pub fn format_f32<S>(val: &f32, serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    let s = format!("{:.40e}", val);
    let number: f64 = s.parse().unwrap();
    serializer.serialize_f64(number)
}

pub fn format_option_f32<S>(val: &Option<f32>, serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    match val {
        Some(inner) => format_f32(inner, serializer),
        None => serializer.serialize_none(),
    }
}
