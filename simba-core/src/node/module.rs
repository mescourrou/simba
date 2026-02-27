use std::fmt::Debug;

use config_checker::ConfigCheckable;
use serde::{Deserialize, Serialize, de::DeserializeOwned};
use simba_macros::config_derives;

// #[config_derives]
#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "schema", schemars(default))]
pub struct ModuleConfig<ModuleConfigType: Default> {
    pub input_source: IOSource,
    pub output_source: IOSource,
    pub config: ModuleConfigType,

}

impl<T: Default> Default for ModuleConfig<T> {
    fn default() -> Self {
        Self {
            input_source: IOSource::Loop,
            output_source: IOSource::Loop,
            config: T::default(),
        }
    }
}

#[config_derives]
pub enum IOSource {
    Loop,
    Channel(String),
}

pub struct Module<InputType, ModuleType, OutputType> {
    module: ModuleType,
    input_source: IOSource,
    output_source: IOSource,
    _input_type: std::marker::PhantomData<InputType>,
    _output_type: std::marker::PhantomData<OutputType>,
}

impl<InputType, ModuleType, OutputType> Module<InputType, ModuleType, OutputType> 
where
    InputType: Clone + Send + 'static,
    OutputType: Clone + Send + 'static,
    ModuleType: ModuleTrait<InputType, OutputType> + Debug,
    ModuleType::Config: Debug + Default,
{
    pub fn from_config(config: ModuleConfig<ModuleType::Config>) -> Self {
        Self {
            module: ModuleType::from_config(config.config),
            input_source: config.input_source,
            output_source: config.output_source,
            _input_type: std::marker::PhantomData,
            _output_type: std::marker::PhantomData,
        }
    }
}

pub trait ModuleTrait<InputType, OutputType>: Send + Sync + Debug + Clone
where
    InputType: Clone + Send + 'static,
    OutputType: Clone + Send + 'static,
{
    type Config;

    fn from_config(config: Self::Config) -> Self;
    fn process(&mut self, input: InputType) -> OutputType;
}