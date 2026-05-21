//! Oriented landmark model used by map loading and geometric visibility checks.
//!
//! This module defines [`OrientedLandmark`], a serializable landmark carrying position,
//! orientation, and optional geometric extent (`height`, `width`) used for obstruction
//! computations.

use nalgebra::{Rotation2, Vector2, Vector3};
use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::recordable::Recordable;

/// Landmark entry loaded from map data.
///
#[config_derives]
pub struct OrientedLandmarkConfig {
    /// Unique landmark identifier.
    pub id: i32,
    /// Optional semantic labels associated with this landmark.
    pub labels: Vec<String>,
    /// Landmark x position.
    pub x: f32,
    /// Landmark y position.
    pub y: f32,
    /// Landmark orientation in radians.
    pub theta: f32,
    /// Height of the landmark used for obstruction checks.
    ///
    /// Use `0.0` for fully transparent landmarks. Default is `1.0` for non-transparent landmarks.
    pub height: f32,
    /// Landmark width, in meters.
    ///
    /// Can be `0.0` for point-like landmarks.
    pub width: f32,
}

impl Default for OrientedLandmarkConfig {
    fn default() -> Self {
        Self {
            id: 0,
            labels: Vec::new(),
            x: 0.,
            y: 0.,
            theta: 0.,
            height: 1.,
            width: 0.,
        }
    }
}

/// Landmark entry loaded from map data.
///
/// The pose is represented as `[x, y, theta]` in a [`Vector3`] where `theta` is in radians.
#[derive(Debug, Clone)]
pub struct OrientedLandmark {
    /// Unique landmark identifier.
    pub id: i32,
    /// Optional semantic labels associated with this landmark.
    pub labels: Vec<String>,
    /// Landmark pose encoded as `(x, y, theta)`.
    pub pose: Vector3<f32>,
    /// Height of the landmark used for obstruction checks.
    ///
    /// Use `0.0` for fully transparent landmarks.
    pub height: f32,
    /// Landmark width, in meters.
    ///
    /// Can be `0.0` for point-like landmarks.
    pub width: f32,
}

impl OrientedLandmark {
    pub fn from_config(config: &OrientedLandmarkConfig) -> Self {
        Self {
            id: config.id,
            labels: config.labels.clone(),
            pose: Vector3::new(config.x, config.y, config.theta),
            height: config.height,
            width: config.width,
        }
    }
    /// Returns the two segment extremities representing this oriented landmark.
    ///
    /// For point-like landmarks (`width <= 0.0`), both returned points are equal to [`Self::pose`].
    pub fn extremities(&self) -> (Vector3<f32>, Vector3<f32>) {
        if self.width <= 0. {
            return (self.pose, self.pose);
        }
        let half_width = self.width / 2.;
        let rotation_matrix = Rotation2::new(self.pose.z);
        let width_vector = rotation_matrix * Vector2::new(0., half_width);
        let extremity1 = self.pose + Vector3::new(width_vector.x, width_vector.y, 0.);
        let extremity2 = self.pose - Vector3::new(width_vector.x, width_vector.y, 0.);
        (extremity1, extremity2)
    }
}

impl Recordable<OrientedLandmarkRecord> for OrientedLandmark {
    fn record(&self, _context: &crate::context::Context) -> OrientedLandmarkRecord {
        OrientedLandmarkConfig {
            id: self.id,
            labels: self.labels.clone(),
            x: self.pose.x,
            y: self.pose.y,
            theta: self.pose.z,
            height: self.height,
            width: self.width,
        }
    }
}

pub type OrientedLandmarkRecord = OrientedLandmarkConfig;

#[cfg(feature = "gui")]
impl UIComponent for OrientedLandmarkRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        egui::CollapsingHeader::new(format!("Landmark Id {}", self.id)).show(ui, |ui| {
            ui.vertical(|ui| {
                ui.label(format!("Labels: {}", self.labels.join(", ")));
                ui.label(format!("Pose: {}, {}, {}", self.x, self.y, self.theta));
                ui.label(format!("Height: {}", self.height));
                ui.label(format!("Width: {}", self.width));
            });
        });
    }
}