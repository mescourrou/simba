use nalgebra::{Rotation2, Vector2, Vector3};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OccupancyGrid {
    grid: Vec<f32>,
    cell_height: f32,
    cell_width: f32,
    center: Vector3<f32>,
    nb_rows: usize,
    nb_cols: usize,
}

impl Default for OccupancyGrid {
    fn default() -> Self {
        Self::new(Vector3::zeros(), 0.5, 0.5, 10, 10)
    }
}

impl OccupancyGrid {
    pub fn new(
        center: Vector3<f32>,
        cell_height: f32,
        cell_width: f32,
        nb_rows: usize,
        nb_cols: usize,
    ) -> Self {
        let mut grid = Vec::with_capacity(nb_cols * nb_rows);
        grid.resize(nb_cols * nb_rows, 0.);
        Self {
            grid,
            cell_height,
            cell_width,
            center,
            nb_rows,
            nb_cols,
        }
    }

    pub fn get_idx(&self, row: usize, col: usize) -> Option<&f32> {
        if row >= self.nb_rows || col >= self.nb_cols {
            return None;
        }
        self.grid.get(row * self.nb_cols + col)
    }

    pub fn get_idx_mut(&mut self, row: usize, col: usize) -> Option<&mut f32> {
        if row >= self.nb_rows || col >= self.nb_cols {
            return None;
        }
        self.grid.get_mut(row * self.nb_cols + col)
    }

    /// Converts a world position to grid (row, col) indices, considering grid orientation.
    pub fn pos_to_idx(&self, position: Vector2<f32>) -> Option<(usize, usize)> {
        let translated = position - self.center.xy();

        let orientation = self.center.z;
        let rot = Rotation2::new(-orientation);
        let local = rot * translated;

        let col_f = (local.x + (self.nb_cols as f32 * self.cell_width) / 2.0) / self.cell_width;
        let row_f = (local.y + (self.nb_rows as f32 * self.cell_height) / 2.0) / self.cell_height;

        let col = col_f.floor() as isize;
        let row = row_f.floor() as isize;

        if row >= 0 && row < self.nb_rows as isize && col >= 0 && col < self.nb_cols as isize {
            Some((row as usize, col as usize))
        } else {
            None
        }
    }

    pub fn get_pos(&self, position: Vector2<f32>) -> Option<&f32> {
        if let Some((row, col)) = self.pos_to_idx(position) {
            self.get_idx(row, col)
        } else {
            None
        }
    }

    pub fn get_pos_mut(&mut self, position: Vector2<f32>) -> Option<&mut f32> {
        if let Some((row, col)) = self.pos_to_idx(position) {
            self.get_idx_mut(row, col)
        } else {
            None
        }
    }
}
