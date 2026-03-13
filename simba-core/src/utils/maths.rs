//! Mathematical helpers used across the simulator.

use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};

/// Return the closest unsigned integer to `numerator / denumerator`.
///
/// The result is rounded to the nearest integer with ties rounded up.
/// Returns an error if the quotient is negative.
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

/// Round `number` to the nearest multiple of `precision`.
/// 
/// `precision` is a float such as `0.1` to round to the nearest tenth, `0.01` for the nearest hundredth, etc.
///
/// Returns an error if `precision` is not strictly positive.
pub fn round_precision(number: f32, precision: f32) -> SimbaResult<f32> {
    if precision <= 0. {
        return Err(SimbaError::new(
            SimbaErrorTypes::MathError,
            "`precision` given to `round_precision` should be strictly positive".to_string(),
        ));
    }
    Ok((number / precision).round_ties_even() * precision)
}

/// Stateful numerical integrator accumulating `value * dt`.
#[derive(Debug, Clone)]
pub struct Integrator {
    cum_integral: f32,
}

impl Integrator {
    /// Create a new integrator initialized at zero.
    pub fn new() -> Self {
        Self { cum_integral: 0. }
    }

    /// Return the current accumulated integral value.
    pub fn integral_value(&self) -> f32 {
        self.cum_integral
    }

    /// Accumulate one integration step using `value * dt`.
    pub fn integrate(&mut self, value: f32, dt: f32) {
        self.cum_integral += value * dt;
    }
}

impl Default for Integrator {
    fn default() -> Self {
        Self::new()
    }
}

/// Stateful first-order finite-difference derivator.
#[derive(Debug, Clone)]
pub struct Derivator {
    previous_value: f32,
}

impl Derivator {
    /// Create a new derivator with previous value set to zero.
    pub fn new() -> Self {
        Self { previous_value: 0. }
    }

    /// Create a derivator initialized with a custom previous value.
    pub fn init(initial_value: f32) -> Self {
        Self {
            previous_value: initial_value,
        }
    }

    /// Compute derivative `(next_value - previous_value) / dt` and update state.
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
