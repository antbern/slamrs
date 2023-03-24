use std::ops::{Add, AddAssign, Mul, Sub, SubAssign};

/// A probability in the range 0-1
#[derive(Clone, Copy, PartialEq)]
pub struct Probability(f32);

impl Mul<Probability> for Probability {
    type Output = Probability;

    fn mul(self, rhs: Probability) -> Self::Output {
        Probability(self.0 * rhs.0)
    }
}

impl Probability {
    pub fn new(value: f32) -> Probability {
        assert!(
            (0.0..=1.0).contains(&value),
            "A probability needs to be in the interval [0.0, 1.0]"
        );
        Probability(value)
    }
    pub fn log_odds(&self) -> LogOdds {
        LogOdds(1.0 - 1.0 / (1.0 + self.0.exp()))
    }

    pub fn value(&self) -> f32 {
        self.0
    }
}

impl From<LogOdds> for Probability {
    fn from(value: LogOdds) -> Self {
        value.probability()
    }
}

/// A probability in log-odds representation. Range +/- infinity.
#[derive(Clone, Copy)]
pub struct LogOdds(f32);

impl Add<LogOdds> for LogOdds {
    type Output = LogOdds;

    fn add(self, rhs: LogOdds) -> Self::Output {
        LogOdds(self.0 + rhs.0)
    }
}

impl AddAssign<LogOdds> for LogOdds {
    fn add_assign(&mut self, rhs: LogOdds) {
        self.0 += rhs.0
    }
}

impl Sub<LogOdds> for LogOdds {
    type Output = LogOdds;

    fn sub(self, rhs: LogOdds) -> Self::Output {
        LogOdds(self.0 - rhs.0)
    }
}

impl SubAssign<LogOdds> for LogOdds {
    fn sub_assign(&mut self, rhs: LogOdds) {
        self.0 -= rhs.0
    }
}

impl LogOdds {
    pub fn probability(&self) -> Probability {
        Probability((self.0 / (1.0 - self.0)).ln())
    }
}

impl From<Probability> for LogOdds {
    fn from(value: Probability) -> Self {
        value.log_odds()
    }
}

#[cfg(test)]
mod test {

    use approx::assert_relative_eq;

    use crate::grid::math::Probability;

    #[test]
    fn inverse() {
        for v in 0..100 {
            let value = v as f32 / 100.0;

            assert_relative_eq!(
                Probability(value).log_odds().probability().0,
                Probability(value).0,
                epsilon = 1e-6
            );
        }
    }
}
