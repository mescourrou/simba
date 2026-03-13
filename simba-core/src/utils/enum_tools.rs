//! Utilities for working with enums in Simba, including traits and macros

/// A trait for converting an enum into a vector of its variants or string representations.
pub trait ToVec<T> {
    /// Convert the enum into a vector of its variants or string representations.
    fn to_vec() -> Vec<T>;
}

/// A trait for creating an enum from a string representation.
pub trait FromString
where
    Self: Sized,
{
    /// Create an enum variant from a string representation.
    /// Returns `None` if the string does not correspond to any variant.
    fn from_string(str: &str) -> Option<Self>;
}

use std::{
    collections::HashMap,
    fmt::{Debug, Display},
    hash::Hash,
    str::FromStr,
};

/// A trait for enums that represent variables names in the simulator.
/// This trait provides a method to create a mapping from enum variants to any type `T` (e.g. f32) using a provided mapping function.
pub trait EnumVariables:
    Clone + Eq + Debug + ToString + ToVec<&'static str> + ToVec<Self> + Display + FromStr + Hash
{
    /// Maps each enum variant to a value of type `T` using the provided mapping function, and returns a `HashMap` of the results.
    fn mapped_values<T>(mut map_fn: impl FnMut(&Self) -> T) -> HashMap<Self, T> {
        let mut map = HashMap::new();
        for variant in Self::to_vec() {
            let value = map_fn(&variant);
            map.insert(variant, value);
        }
        map
    }
}

/// Macro to define an enum with variants and their string representations, and automatically implement the `EnumVariables` trait for it.
/// 
/// To be used by the procedural macro `enum_variables!` to generate enums from a concise syntax.
macro_rules! __enum_variables_emit_subenum {
    (   
        $(#[$meta:meta])*
        $name:ident;
        $($($documentation:literal)? $variant:ident, $value:literal $(, $add_value:literal)*);+ $(;)?
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq, std::hash::Hash)]
        #[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
        pub enum $name {
            $(
                $(#[doc = concat!($documentation, "\n\n")])?
                #[doc = concat!("Serialization values(s) for this variable: ", "`", $value, "`" $(, ", `", $add_value, "`")*)]
                #[serde(rename = $value$(, alias = $add_value)*)]
                $variant,
            )+
        }

        impl std::str::FromStr for $name {
            type Err = $crate::errors::SimbaError;
            fn from_str(s: &str) -> $crate::errors::SimbaResult<Self> {
                match s.to_lowercase().as_str() {
                    $(
                        $value $(| $add_value)* => Ok(Self::$variant),
                    )+
                    _ => Err($crate::errors::SimbaError::new(
                        $crate::errors::SimbaErrorTypes::ConfigError,
                        format!("Unknown variable name: '{}'", s),
                    )),
                }
            }
        }

        impl $crate::utils::enum_tools::ToVec<&'static str> for $name {
            fn to_vec() -> Vec<&'static str> {
                vec![$(
                    $value,
                )+
                ]
            }
        }

        impl $crate::utils::enum_tools::ToVec<$name> for $name {
            fn to_vec() -> Vec<$name> {
                vec![
                    $(
                        $name::$variant,
                    )+
                ]
            }
        }

        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(
                    f,
                    "{}",
                    match self {
                        $(
                             Self::$variant => $value,
                        )+
                    }
                )
            }
        }

        impl $crate::utils::enum_tools::EnumVariables for $name {}
    };
}

pub(crate) use __enum_variables_emit_subenum;
