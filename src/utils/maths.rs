use crate::errors::{SimbaError, SimbaErrorTypes};

pub fn closest_uint_modulo(numerator: f32, denumerator: f32) -> Result<u32, SimbaError> {
    let float_mod = numerator / denumerator;
    if float_mod < 0. {
        return Err(SimbaError::new(
            SimbaErrorTypes::MathError,
            "closest_uint_modulo with negative division!",
        ));
    }
    let floor = float_mod.floor();
    let ceil = float_mod.ceil();

    if float_mod - floor < ceil - float_mod {
        return Ok(floor as u32);
    }
    Ok(ceil as u32)
}

pub fn round_precision(number: f32, precision: f32) -> Result<f32, SimbaError> {
    if precision <= 0. {
        return Err(SimbaError::new(
            SimbaErrorTypes::MathError,
            "`precision` given to `round_precision` should be strictly positive",
        ));
    }
    Ok((number / precision).round_ties_even() * precision)
}
