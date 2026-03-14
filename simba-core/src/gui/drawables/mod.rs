use egui::{Rect, Response, Shape, Vec2};

use crate::{gui::app::PainterInfo, node::node_factory::NodeRecord};

pub mod map;
pub mod observations;
pub mod popup;
pub mod robot;

/// Trait for a drawable element in the GUI. It is used to draw the elements of the simulation in the GUI, such as the robot, the landmarks, the trajectory, etc., to react to the user interaction, and to draw additionnal windows.
pub trait Drawable {
    /// Function called when a new record is received. The time of the record can be different from the time of the last drawing.
    ///
    /// You can use [`TimeOrderedData`](simba_com::time_ordered_data::TimeOrderedData) to manage the records.
    ///
    /// # Arguments
    /// * `time` - The time of the record. It can be different from the time of the last drawing, and it can be in the past or in the future.
    /// * `record` - The record to add. It contains the information to draw the drawable for the given time.
    ///
    fn add_record(&mut self, time: f32, record: NodeRecord);

    /// Draws the drawable for the given time.
    ///
    /// # Arguments
    /// * `ui` - The egui UI to draw the drawable on.
    /// * `viewport` - The viewport to draw the drawable in. It is used to determine if the drawable is visible or not, and to get the position of the drawable in the viewport.
    /// * `painter_info` - The painter info to draw the drawable
    /// * `scale` - The scale to draw the drawable. It is used to scale the drawable according to the zoom level of the viewport.
    /// * `time` - The time to draw the drawable for. It is used to get the record of the drawable for the given time, and to draw the drawable according to the record. If there is no record for the given time, the drawable should be drawn according to the closest record in a given limit, or not drawn at all if there is no record at all.
    ///
    /// ## Returns
    /// * `Ok(Vec<Shape>)` if the drawable could be drawn for the given time.
    /// * `Err(Vec2)` if the drawable could not be drawn for the given time because it is outside of the viewport. The `Vec2` is the position of the drawable that is outside of the viewport. This will trigger an extension of the viewport in the GUI, allowing you to see the drawable.
    fn draw(
        &self,
        ui: &mut egui::Ui,
        viewport: &Rect,
        painter_info: &PainterInfo,
        scale: f32,
        time: f32,
    ) -> Result<Vec<Shape>, Vec2>;

    /// Reacts to the user interaction with the drawable. This can be used to implement clickable drawables.
    ///
    /// The default implementation does nothing, but it can be overridden to implement custom behavior.
    ///
    /// # Arguments
    /// * `ui` - The egui UI to react to the user interaction.
    /// * `ctx` - The egui context to react to the user interaction.
    /// * `response` - The response of the user interaction with the drawable. It contains information about the type of interaction (click, hover, etc.) and the position of the interaction.
    /// * `painter_info` - The painter info to react to the user interaction such as the position of the click in the viewport.
    /// * `scale` - The scale to react to the user interaction. It is used to scale the position of the interaction according to the zoom level of the viewport.
    /// * `time` - The time to react to the user interaction for.
    ///
    #[allow(unused_variables)]
    fn react(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        response: &Response,
        painter_info: &PainterInfo,
        scale: f32,
        time: f32,
    ) {
    }
}
