use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};

pub fn closest_uint_modulo(numerator: f32, denumerator: f32) -> SimbaResult<u32> {
    let float_mod = numerator / denumerator;
    if float_mod < 0. {
        return Err(SimbaError::new(
            SimbaErrorTypes::MathError,
            "closest_uint_modulo with negative division!".to_string(),
        ));
    }
    let floor = float_mod.floor();
    let ceil = float_mod.ceil();

    if float_mod - floor < ceil - float_mod {
        return Ok(floor as u32);
    }
    Ok(ceil as u32)
}

pub fn round_precision(number: f32, precision: f32) -> SimbaResult<f32> {
    if precision <= 0. {
        return Err(SimbaError::new(
            SimbaErrorTypes::MathError,
            "`precision` given to `round_precision` should be strictly positive".to_string(),
        ));
    }
    Ok((number / precision).round_ties_even() * precision)
}

#[derive(Debug, Clone)]
pub struct Integrator {
    cum_integral: f32,
}

impl Integrator {
    pub fn new() -> Self {
        Self { cum_integral: 0. }
    }

    pub fn integral_value(&self) -> f32 {
        self.cum_integral
    }

    pub fn integrate(&mut self, value: f32, dt: f32) {
        self.cum_integral += value * dt;
    }
}

impl Default for Integrator {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone)]
pub struct Derivator {
    previous_value: f32,
}

impl Derivator {
    pub fn new() -> Self {
        Self { previous_value: 0. }
    }

    pub fn init(initial_value: f32) -> Self {
        Self {
            previous_value: initial_value,
        }
    }

    pub fn derivate(&mut self, next_value: f32, dt: f32) -> f32 {
        let deriv = (next_value - self.previous_value) / dt;
        self.previous_value = next_value;
        deriv
    }
}

impl Default for Derivator {
    fn default() -> Self {
        Self::new()
    }
}
