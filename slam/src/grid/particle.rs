use core::num;

#[derive(Clone)]
struct Particle<T: Clone> {
    weight: f64,
    value: T,
}

pub struct ParticleFilter<T: Clone> {
    particles: Vec<Particle<T>>,
    max_particle: usize,
}

impl<T: Clone> ParticleFilter<T> {
    pub fn new(number_of_particles: usize, initial_value: T) -> Self {
        assert!(number_of_particles > 0, "Must have at least one particle");

        Self {
            particles: vec![
                Particle {
                    weight: 1.0 / number_of_particles as f64,
                    value: initial_value
                };
                number_of_particles
            ],
            max_particle: 0,
        }
    }

    /// Allows performing any updates to the value of each Particle and then normalizes the weights.
    pub fn update(&mut self, mut f: impl FnMut(&mut T) -> f64) {
        self.particles
            .iter_mut()
            .map(|p| p.weight = f(&mut p.value))
            .collect::<()>();

        self.normalize_weights();

        // store the particle with the maximum weight
        self.max_particle = self
            .particles
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.weight.total_cmp(&b.weight))
            .map(|(index, _)| index)
            .expect("No maximum found!");
    }

    fn normalize_weights(&mut self) {
        let sum: f64 = self.particles.iter().map(|p| p.weight).sum();

        self.particles
            .iter_mut()
            .map(|p| p.weight /= sum)
            .collect::<()>();
    }

    /// Computes the number of effective particles, a measure on X
    pub fn number_of_effective_particles(&self) -> f64 {
        1.0 / self
            .particles
            .iter()
            .map(|p| p.weight * p.weight)
            .sum::<f64>()
    }

    pub fn particle_value(&self, index: usize) -> &T {
        &self.particles[index].value
    }
    pub fn particle_value_mut(&mut self, index: usize) -> &mut T {
        &mut self.particles[index].value
    }

    pub fn strongest_particle_idx(&self) -> usize {
        self.max_particle
    }

    pub fn resample(&mut self) {
        let num_particles = self.particles.len();
        let mut new_particles: Vec<Particle<T>> = Vec::with_capacity(num_particles);

        // Assumes weights are normalized, which they are since the only way to modify them is through the call to `update`

        let r: f64 = rand::random::<f64>() * 1.0 / num_particles as f64;
        let mut c = self.particles[0].weight;
        let mut i = 0;

        for m in 1..=num_particles {
            let u = r + (m as f64 - 1.0) * 1.0 / num_particles as f64;

            while (u > c) {
                i += 1;
                c += self.particles[i].weight;
            }

            // add the i:th particle to the new generation (note that this a copying operation)
            new_particles.push(Particle {
                weight: 1.0 / num_particles as f64,
                value: self.particles[i].value.clone(),
            });
        }

        // make the new generation the current one
        self.particles = new_particles;
    }
}
