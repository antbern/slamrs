use common::robot::{Observation, Pose};
use nalgebra::{DMatrix, EuclideanNorm, Matrix2, Vector2};

use super::ray::GridRayIterator;
use common::math::{LogOdds, LogProbability, Probability};

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
            odds: GridData::new_fill(grid_size, Probability::new(0.5).log_odds()),
        }
    }

    pub fn likelihood(&self) -> GridData<Probability> {
        self.odds.transform()
    }

    pub fn position(&self) -> Vector2<f32> {
        self.position
    }

    /// Converts a position in the world into a grid-relative position. Note that the returned
    /// value is not guaranteed to lie _within_ the bounds of this Map.
    pub fn world_to_grid(&self, world: Vector2<f32>) -> Vector2<f32> {
        (world - self.position) / self.resolution
    }

    pub fn is_valid(&self, grid: Vector2<f32>) -> bool {
        !((grid.x < 0.0)
            || (grid.y < 0.0)
            || (grid.x as usize >= self.grid_size.x)
            || (grid.y as usize >= self.grid_size.y))
    }

    pub fn integrate(&mut self, observation: &Observation, pose: Pose) {
        let start = self.world_to_grid(pose.xy());

        for m in &observation.measurements {
            let end = Vector2::new(
                pose.x + (pose.theta + m.angle as f32).cos() * m.distance as f32,
                pose.y + (pose.theta + m.angle as f32).sin() * m.distance as f32,
            );

            let end = self.world_to_grid(end);

            // println!("{} -> {}", start, end);

            self.apply_measurement(start, end, m.distance as f32 / self.resolution, m.valid);
        }
    }

    fn apply_measurement(
        &mut self,
        start: Vector2<f32>,
        end: Vector2<f32>,
        measured_distance: f32,
        was_hit: bool,
    ) {
        // TODO: additional_steps below need to coincide with the threshold in the inverse sensor model (so that we correctly take the model into account)
        for (cell, center) in
            GridRayIterator::new(start.x, start.y, end.x, end.y, self.grid_size, 2)
        {
            // calculate the distance from the start to the center of this visited cell
            let distance = start.apply_metric_distance(&center, &EuclideanNorm);

            // update the log odds based on the inverse sensor model
            *self.odds.get_mut(cell) +=
                inverse_sensor_model(distance, measured_distance, was_hit, 2.0).log_odds();
        }
    }
    /// Probability to assign when hit, random is the complement (1-Z_HIT)
    const Z_HIT: f64 = 0.9;
    const SENSOR_MAXDIST: f64 = 1.0; // Meters

    /// Computes the probability of the observation given the map and the pose: p(z | m, x)
    /// TODO: include the smoothed binarized version of the map here instead
    pub(crate) fn probability_of(&self, z: &Observation, pose: Pose) -> LogProbability {
        let mut product = LogProbability::new(1.0);

        for m in &z.measurements {
            if !m.valid {
                continue;
            }
            let end = Vector2::new(
                pose.x + (pose.theta + m.angle as f32).cos() * m.distance as f32,
                pose.y + (pose.theta + m.angle as f32).sin() * m.distance as f32,
            );

            let end = self.world_to_grid(end);

            if self.is_valid(end) {
                let gridx = end.x as usize;
                let gridy = end.y as usize;
                let cell = Cell::new(gridx, gridy);

                let odds = self.odds.get(cell);

                // if the probability neither points to free or occupied, just treat as uniform
                if odds.probability().value() == 0.5 {
                    product *= (1.0 / Self::SENSOR_MAXDIST);
                } else {
                    product *= (Self::Z_HIT * odds.probability().value()
                        + (1.0 - Self::Z_HIT) * 1.0 / Self::SENSOR_MAXDIST);
                }
            }
        }

        product
    }
}

fn inverse_sensor_model(
    distance: f32,
    measured_distance: f32,
    was_hit: bool,
    tolerance: f32,
) -> Probability {
    const P_FREE: Probability = Probability::new_unchecked(0.30);
    const P_OCCUPPIED: Probability = Probability::new_unchecked(0.9);
    const P_PRIOR: Probability = Probability::new_unchecked(0.5);

    if !was_hit {
        if distance < measured_distance {
            return P_FREE;
        } else {
            return P_PRIOR;
        }
    }
    if distance < measured_distance - tolerance / 2.0 {
        P_FREE
    } else if distance > measured_distance + tolerance / 2.0 {
        P_PRIOR
    } else {
        P_OCCUPPIED
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

        // SAFETY: we assume the cell is created to be within bounds
        // unsafe { self.data.get_unchecked(self.index(cell)) }
    }

    pub fn get_mut(&mut self, cell: Cell) -> &mut T {
        let index = self.index(cell);
        &mut self.data[index]
        // unsafe { self.data.get_unchecked_mut(index) }
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
