pub mod node;
pub mod robot;
pub mod world;

/// Keeps track of performance metrics and incremental updates the values. In milliseconds.
pub struct PerfStats {
    // exp_mean: f64,
    mean: f64,
    var_sum: f64,
    std: f64,
    sample_count: usize,
    min: f64,
    max: f64,
    latest: f64,
}

impl Default for PerfStats {
    fn default() -> Self {
        Self {
            mean: 0.0,
            var_sum: 0.0,
            std: 0.0,
            sample_count: 0,
            min: f64::INFINITY,
            max: f64::NEG_INFINITY,
            latest: 0.0,
        }
    }
}

impl PerfStats {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset(&mut self) {
        *self = Self::default()
    }

    pub fn update(&mut self, duration: Duration) {
        let msecs = duration.as_secs_f64() * 1000.0;

        if self.sample_count > 0 {
            // mean
            let old_mean = self.mean;
            self.mean += (msecs - self.mean) / self.sample_count as f64;

            // variance / std
            self.var_sum += (msecs - old_mean) * (msecs - self.mean);
            self.std = (self.var_sum / self.sample_count as f64).sqrt();
        } else {
            self.mean = msecs;
        }
        // min and max values
        self.min = self.min.min(msecs);
        self.max = self.max.max(msecs);
        self.latest = msecs;

        self.sample_count += 1;
    }

    pub fn latest(&self) -> f64 {
        self.latest
    }

    pub fn latest_fps(&self) -> f64 {
        1000.0 / self.latest
    }
}

impl Display for PerfStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{:>5.2}ms ({:.2}±{:.2} , [{:.2}, {:.2}], {})",
            self.latest, self.mean, self.std, self.min, self.max, self.sample_count
        )
    }
}
