use std::{
    f64::consts::PI,
    ops::{Add, AddAssign, Mul, MulAssign, Sub, SubAssign},
};

/// A probability in the range 0-1
#[derive(Clone, Copy, PartialEq)]
pub struct Probability(f64);

impl Mul<Probability> for Probability {
    type Output = Probability;

    fn mul(self, rhs: Probability) -> Self::Output {
        Probability(self.0 * rhs.0)
    }
}

impl Probability {
    pub const fn new_unchecked(value: f64) -> Probability {
        Probability(value)
    }

    pub fn new(value: f64) -> Probability {
        assert!(
            (0.0..=1.0).contains(&value),
            "A probability needs to be in the interval [0.0, 1.0]"
        );
        Probability(value)
    }
    pub fn log_odds(&self) -> LogOdds {
        LogOdds((self.0 / (1.0 - self.0)).ln())
    }

    pub fn log(&self) -> LogProbability {
        LogProbability::new_unchecked(self.0)
    }

    pub fn value(&self) -> f64 {
        self.0
    }
}

impl From<LogOdds> for Probability {
    fn from(value: LogOdds) -> Self {
        value.probability()
    }
}

/// A probability represented in the log space. When many probabilities are multiplied together, this improves performance and numerical stability.
pub struct LogProbability(f64);

impl Mul<LogProbability> for LogProbability {
    type Output = LogProbability;

    fn mul(self, rhs: LogProbability) -> Self::Output {
        LogProbability(self.0 + rhs.0)
    }
}

impl Add<LogProbability> for LogProbability {
    type Output = LogProbability;

    fn add(self, rhs: LogProbability) -> Self::Output {
        // More involved, see https://en.wikipedia.org/wiki/Log_probability

        let (x, y) = if self.0 > rhs.0 {
            (self.0, rhs.0)
        } else {
            (rhs.0, self.0)
        };

        LogProbability(x + (y - x).exp().ln_1p())
    }
}

impl LogProbability {
    pub fn new_unchecked(value: f64) -> LogProbability {
        LogProbability(value.ln())
    }

    pub fn new(value: f64) -> LogProbability {
        assert!(
            (0.0..=1.0).contains(&value),
            "A probability needs to be in the interval [0.0, 1.0], got: {value}"
        );
        Self::new_unchecked(value)
    }
    pub fn prob(&self) -> Probability {
        Probability(self.0.exp())
    }
}

impl MulAssign<f64> for LogProbability {
    fn mul_assign(&mut self, rhs: f64) {
        self.0 += rhs.ln();
    }
}

/// A probability in log-odds representation. Range +/- infinity.
#[derive(Clone, Copy)]
pub struct LogOdds(f64);

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
        Probability(1.0 - 1.0 / (1.0 + self.0.exp()))
    }
}

impl From<Probability> for LogOdds {
    fn from(value: Probability) -> Self {
        value.log_odds()
    }
}

/// Computes the shortest distance between two angles in radians and returns the result in the
/// range [-PI,PI)
///
/// Source: https://stackoverflow.com/a/28037434
pub fn angle_diff(alpha: f64, beta: f64) -> f64 {
    let diff = (beta - alpha + PI) % (PI * 2.0) - PI;
    if diff < -PI {
        diff + 2.0 * PI
    } else {
        diff
    }
}

#[cfg(test)]
mod test {

    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn inverse() {
        for v in 0..100 {
            let value = v as f64 / 100.0;

            assert_relative_eq!(
                Probability(value).log_odds().probability().0,
                Probability(value).0,
                epsilon = 1e-6
            );
        }
    }

    #[test]
    fn zero_is_half() {
        assert_relative_eq!(Probability(0.5).log_odds().0, 0.0);
    }

    #[test]
    fn test_angle_diff() {
        assert_relative_eq!(angle_diff(PI, PI), 0.0);
        assert_relative_eq!(angle_diff(-PI, PI), 0.0);
        assert_relative_eq!(angle_diff(0.0, PI), -PI);
        assert_relative_eq!(angle_diff(PI, 0.0), -PI);
        assert_relative_eq!(angle_diff(0.0, PI / 2.0), PI / 2.0);
        assert_relative_eq!(angle_diff(PI / 2.0, 0.0), -PI / 2.0);
        assert_relative_eq!(angle_diff(PI, PI / 2.0), -PI / 2.0);
        assert_relative_eq!(angle_diff(PI / 2.0, PI), PI / 2.0);
    }
}
