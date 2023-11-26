use nalgebra::Vector2;

use super::map::Cell;

pub struct GridRayIterator {
    /// Number of rows and columns in the grid to generate `Cell`s for.
    size: Vector2<usize>,

    delta: Vector2<f32>,
    increment: Vector2<isize>,
    error: f32,
    x: isize,
    y: isize,
    remaining_cells: usize,
}

impl GridRayIterator {
    /// Create an iterator to iterate over the line specified. The start and end points are specified in grid coordinates,
    /// with the origin at the bottom left. The iterator values correspond to the center of each cell that is visited
    /// along the ray from start to end point.

    pub fn new(
        x0: f32,
        y0: f32,
        x1: f32,
        y1: f32,
        size: Vector2<usize>,
        additional_steps: usize,
    ) -> Self {
        let delta = Vector2::new(x1 - x0, y1 - y0).abs();

        // set up stuff

        let x = x0.floor() as isize;
        let y = y0.floor() as isize;

        // start with at least one
        let mut n = 1 + additional_steps as isize;
        let mut x_inc = 0;
        let mut y_inc = 0;
        let mut error = 0.0;

        // decide based on case
        if delta.x == 0.0 {
            x_inc = 0;
            error = f32::INFINITY;
        } else if x1 > x0 {
            x_inc = 1;
            n += (x1.floor() - x as f32) as isize;
            error = ((x0.floor() + 1.0 - x0) * delta.y);
        } else {
            x_inc = -1;
            n += x - x1.floor() as isize;
            error = ((x0 - x0.floor()) * delta.y);
        }

        if delta.y == 0.0 {
            y_inc = 0;
            error -= f32::INFINITY;
        } else if y1 > y0 {
            y_inc = 1;
            n += y1.floor() as isize - y;
            error -= (y0.floor() + 1.0 - y0) * delta.x;
        } else {
            y_inc = -1;
            n += y - y1.floor() as isize;
            error -= (y0 - y0.floor()) * delta.x;
        }

        Self {
            size,
            delta,
            increment: Vector2::new(x_inc, y_inc),
            error,
            x,
            y,
            remaining_cells: n as usize,
        }
    }
}

impl Iterator for GridRayIterator {
    type Item = (Cell, Vector2<f32>);

    fn next(&mut self) -> Option<Self::Item> {
        // make sure we do not leave the allowed range of x,y values
        let one_more = self.remaining_cells > 0
            && !(self.x < 0
                || self.x >= self.size.x as isize
                || self.y < 0
                || self.y >= self.size.y as isize);

        if one_more {
            let cell = Cell::new(self.x as usize, self.y as usize);
            let cell_center = Vector2::new(self.x as f32 + 0.5, self.y as f32 + 0.5);

            // move to next position
            if self.error > 0.0 {
                self.y += self.increment.y;
                self.error -= self.delta.x;
            } else {
                self.x += self.increment.x;
                self.error += self.delta.y;
            }

            // decrease number of cells
            self.remaining_cells -= 1;
            Some((cell, cell_center))
        } else {
            None
        }
    }
}
