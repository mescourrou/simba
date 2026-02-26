use egui::{Rect, Response, Shape, Vec2};

use crate::{gui::app::PainterInfo, node::node_factory::NodeRecord};

pub mod map;
pub mod observations;
pub mod popup;
pub mod robot;

pub trait Drawable {
    fn add_record(&mut self, time: f32, record: NodeRecord);

    /// Draws the drawable for the given time.
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

    fn react(
        &mut self,
        _ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _response: &Response,
        _painter_info: &PainterInfo,
        _scale: f32,
        _time: f32,
    ) {
    }
}
