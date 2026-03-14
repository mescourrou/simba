use std::{collections::HashMap, fmt::{Debug, Display}, hash::Hash, str::FromStr};

use crate::utils::enum_tools::ToVec;


pub trait EnumVariables: Clone + Eq + Debug + ToString + ToVec<&'static str> + ToVec<Self> + Display + FromStr + Hash
 {
    fn mapped_values(mut map_fn: impl FnMut(&Self) -> f32) -> HashMap::<Self, f32> {
        let mut map = HashMap::new();
        for variant in Self::to_vec() {
            let value = map_fn(&variant);
            map.insert(variant, value);
        }
        map
    }
}


#[macro_export]
macro_rules! enum_variables {
    ($name:ident;
        $($variant:ident, $value:expr $(, $add_value:expr)*);* $(;)?
    ) => {
        #[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq, std::hash::Hash)]
        #[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
        pub enum $name {
            $(
                #[serde(rename = $value$(, alias = $add_value)*)]
                $variant
            ),*
        }

        impl std::str::FromStr for $name {
            type Err = crate::errors::SimbaError;
            fn from_str(s: &str) -> crate::errors::SimbaResult<Self> {
                match s.to_lowercase().as_str() {
                    $(
                        $value $(| $add_value)* => Ok(Self::$variant),
                    )*
                    _ => Err(crate::errors::SimbaError::new(
                        crate::errors::SimbaErrorTypes::ConfigError,
                        format!("Unknown variable name: '{}'", s),
                    )),
                }
            }
        }

        impl crate::utils::enum_tools::ToVec<&'static str> for $name {
            fn to_vec() -> Vec<&'static str> {
                vec![$($value),*]
            }
        }

        impl crate::utils::enum_tools::ToVec<$name> for $name {
            fn to_vec() -> Vec<$name> {
                vec![
                    $(
                        $name::$variant
                    ),*
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
                        )*
                    }
                )
            }   
        }

        impl crate::sensors::sensor_variables::EnumVariables for $name {}
    };
}

pub use enum_variables;

