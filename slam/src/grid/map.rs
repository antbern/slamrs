use common::robot::Observation;
use nalgebra::{DMatrix, Matrix2, Vector2};

use super::math::{LogOdds, Probability};

#[derive(Clone)]
pub struct Map {
    /** the position of this map in the world (lower left corner) */
    position: Vector2<f32>,

    /** the size of the map in world coordinates */
    world_size: Vector2<f32>,

    /** the size of the map in cells */
    grid_size: Vector2<usize>,

    /** array for storing the probability data */
    // double[] probData;

    /** the resolution of this GridMap, given in meters per cell */
    resolution: f32,

    // Data vectors
    odds: GridData<LogOdds>,
}

impl Map {
    pub fn new(position: Vector2<f32>, width: f32, height: f32, resolution: f32) -> Self {
        // calculate the required size in cells to fill the desired area based on the resolution
        let grid_size = Vector2::new(
            (width / resolution).ceil() as usize,
            (height / resolution).ceil() as usize,
        );

        // calculate the "real" size of this grid map (potentially caused by ceil() above)
        let world_size = Vector2::new(
            grid_size.x as f32 * resolution,
            grid_size.y as f32 * resolution,
        );

        let vec_len = grid_size.x * grid_size.y;

        Self {
            position,
            world_size,
            grid_size,
            resolution,
            odds: GridData::new_fill(grid_size, Probability::new(0.8).log_odds()),
        }
    }

    pub fn likelihood(&self) -> GridData<Probability> {
        self.odds.transform()
    }

    pub fn position(&self) -> Vector2<f32> {
        self.position
    }

    pub fn integrate(&mut self, observation: &Observation) {
        *self.odds.get_mut(Cell::new(10, 20)) += Probability::new(0.01).log_odds();
    }
}

#[derive(Clone)]
struct MapData {
    odds: Vec<LogOdds>,
    likelihood: Vec<Probability>,
}

#[derive(Clone)]
pub struct GridData<T> {
    /** the size of the grid in cells */
    size: Vector2<usize>,

    /// Vector containing all the data values
    data: Vec<T>,
}

pub struct Cell {
    pub row: usize,
    pub column: usize,
}

impl Cell {
    pub fn new(column: usize, row: usize) -> Self {
        Cell { column, row }
    }
}

impl<T> GridData<T> {
    fn index(&self, cell: Cell) -> usize {
        // Row-major order
        cell.row * self.size.y + cell.column
    }

    fn cell(&self, index: usize) -> Cell {
        // Row-major order
        assert!(index < self.size.x * self.size.y);

        Cell {
            row: index / self.size.y,
            column: index % self.size.y,
        }
    }

    pub fn get(&self, cell: Cell) -> &T {
        &self.data[self.index(cell)]
    }

    pub fn get_mut(&mut self, cell: Cell) -> &mut T {
        let index = self.index(cell);
        &mut self.data[index]
    }

    /// Returns a copy of this GridData with each element converted to `S` using the provided function.
    pub fn transform_map<S>(&self, f: impl Fn(&T) -> S) -> GridData<S> {
        GridData {
            size: self.size,
            data: self.data.iter().map(f).collect(),
        }
    }

    /// Returns a copy of this GridData with each element converted to `S` using `into()`.
    pub fn transform<S>(&self) -> GridData<S>
    where
        T: Into<S> + Copy,
    {
        GridData {
            size: self.size,
            data: self.data.iter().map(|&t| Into::<S>::into(t)).collect(),
        }
    }

    pub fn iter_cells(&self) -> impl Iterator<Item = (Cell, &T)> {
        self.data.iter().enumerate().map(|(i, v)| (self.cell(i), v))
    }

    pub fn size(&self) -> Vector2<usize> {
        self.size
    }
}

impl<T: Clone> GridData<T> {
    pub fn new_fill(size: Vector2<usize>, initial_value: T) -> Self {
        Self {
            size,
            data: vec![initial_value; size.x * size.y],
        }
    }
}
