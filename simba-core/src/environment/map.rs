use std::{path::Path, sync::Arc};

use serde::{Deserialize, Serialize};
use simba_macros::{config_derives, enum_variables};

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::{
    config::NumberConfig,
    environment::oriented_landmark::{
        OrientedLandmark, OrientedLandmarkConfig, OrientedLandmarkRecord,
    },
    errors::{SimbaError, SimbaErrorTypes, SimbaResult},
    recordable::Recordable,
    utils::determinist_random_variable::{
        DeterministRandomVariableFactory, RandomVariableTypeConfig,
    },
};

/// Map containing multiple [`OrientedLandmark`] and possibility for random generation.
///
/// # File example
/// ```yaml
/// landmarks:
///   - id: 1
///     x: 3
///     y: 2
///     theta: 0.7854
///     width: 2.8284
///     height: 0
///   - id: 2
///     x: 5.5
///     y: 7
///     theta: 1.5708
///     width: 3
///     height: 1
/// random:
///   - number:
///       type: Num
///       value: 5
///     distributions:
///       - type: Uniform  
///         min: [0, 0, -3.1416]
///         max: [10, 10, 3.1416]
///     variable_order: [x, y, theta]
/// ```
#[config_derives]
pub struct MapConfig {
    /// Landmarks contained in the map.
    pub landmarks: Vec<OrientedLandmarkConfig>,
    /// Random generation parameters.
    pub random: Vec<RandomMap>,
}

impl Default for MapConfig {
    fn default() -> Self {
        Self {
            landmarks: Vec::new(),
            random: Vec::new(),
        }
    }
}

enum_variables!(
    "Variables of the [`MapRandom`]."
    MapVariables;
    "X position of the landmark"
    X, "x";
    "Y position of the landmark"
    Y, "y";
    "Orientation of the landmark"
    Theta, "theta", "orientation";
    "Height of the landmark"
    Height, "height", "h";
    "Width of the landmark"
    Width, "width", "w";
);

#[config_derives]
pub struct RandomMap {
    /// Number of landmarks to generate randomly.
    pub number: NumberConfig,
    /// List of distributions to generate the landmarks. The order should be consistent
    /// with `variable_order`.
    #[check]
    pub distributions: Vec<RandomVariableTypeConfig>,
    /// Ordered list of target variables receiving sampled perturbations.
    pub variable_order: Vec<MapVariables>,
}

impl Default for RandomMap {
    fn default() -> Self {
        Self {
            number: NumberConfig::default(),
            distributions: Vec::new(),
            variable_order: Vec::new(),
        }
    }
}

impl Check for RandomMap {
    fn do_check(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if !self.variable_order.is_empty()
            && self.distributions.iter().map(|d| d.dim()).sum::<usize>()
                != self.variable_order.len()
        {
            errors.push(format!("If variable order is given, its length should match the total distribution dimension. Got total distribution dimension {} and variable order length {}.",
                self.distributions.iter().map(|d| d.dim()).sum::<usize>(),
                self.variable_order.len()
            ));
        }
        if let NumberConfig::Num(n) = self.number
            && n < 0.
        {
            errors.push(format!(
                "Number of random landmarks should be non-negative, got {}",
                n
            ));
        } else if let NumberConfig::Rand(cfg) = &self.number {
            if cfg.dim() != 1 {
                errors.push(format!("Number of random landmarks should be a univariate random variable, got dimension {}", cfg.dim()));
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

impl RandomMap {
    pub fn generate_landmarks(
        &self,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        first_id: usize,
    ) -> Vec<OrientedLandmark> {
        let nb = match &self.number {
            NumberConfig::Num(n) => n.round() as usize,
            NumberConfig::Rand(cfg) => {
                let va = va_factory.make_variable(cfg.clone());
                va.generate(0.).first().unwrap().max(0.).round() as usize
            }
        };
        let mut landmarks = Vec::new();
        for i in 0..nb {
            let mut landmark = OrientedLandmark::from_config(&OrientedLandmarkConfig::default());
            landmark.id = (first_id + i) as i32;
            let mut random_sample = Vec::new();
            for d_config in &self.distributions {
                let va = va_factory.make_variable(d_config.clone());
                random_sample.extend_from_slice(&va.generate(0.));
            }

            for (i, var) in self.variable_order.iter().enumerate() {
                match var {
                    MapVariables::X => landmark.pose.x = random_sample[i],
                    MapVariables::Y => landmark.pose.y = random_sample[i],
                    MapVariables::Theta => landmark.pose.z = random_sample[i],
                    MapVariables::Height => landmark.height = random_sample[i],
                    MapVariables::Width => landmark.width = random_sample[i],
                }
            }
            landmarks.push(landmark);
        }

        landmarks
    }
}

#[derive(Debug, Clone, Default)]
pub struct Map {
    /// Landmarks contained in the map.
    landmarks: Vec<OrientedLandmark>,
}

impl Map {
    /// Creates an empty map.
    pub fn new() -> Self {
        Self {
            landmarks: Vec::new(),
        }
    }

    /// Load the map from the given `path`.
    pub fn load_from_path(
        path: &Path,
        va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> SimbaResult<Map> {
        let config: MapConfig = match confy::load_path(path) {
            Ok(config) => config,
            Err(error) => {
                return Err(SimbaError::new(
                    SimbaErrorTypes::ConfigError,
                    format!(
                        "Error from Confy while loading the map file {} : {}",
                        path.display(),
                        error
                    ),
                ));
            }
        };
        let mut landmarks = config
            .landmarks
            .iter()
            .map(OrientedLandmark::from_config)
            .collect::<Vec<_>>();
        let mut max_id = landmarks.iter().map(|l| l.id).max().unwrap_or(0) as usize + 1;
        for random_config in config.random {
            let new_landmarks = random_config.generate_landmarks(va_factory, max_id);
            max_id += new_landmarks.len();
            landmarks.extend(new_landmarks);
        }
        Ok(Map { landmarks })
    }

    pub fn landmarks(&self) -> &Vec<OrientedLandmark> {
        &self.landmarks
    }
}

impl Recordable<MapRecord> for Map {
    fn record(&self, context: &crate::context::Context) -> MapRecord {
        MapRecord {
            landmarks: self
                .landmarks
                .iter()
                .map(|l| l.record(context))
                .collect::<Vec<_>>(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapRecord {
    pub landmarks: Vec<OrientedLandmarkRecord>,
}

#[cfg(feature = "gui")]
impl UIComponent for MapRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new("Landmarks:").show(ui, |ui| {
            ui.vertical(|ui| {
                for l in &self.landmarks {
                    l.show(ui, ctx, format!("{}-{}", unique_id, l.id).as_str());
                }
            })
        });
    }
}