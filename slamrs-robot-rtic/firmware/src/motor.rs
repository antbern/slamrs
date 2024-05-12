//! Module for controling the motors through the Afafruit Featherwing Motor Driver
//!

// embedded_hal::blocking::i2c:

use core::marker::PhantomData;
use embedded_hal::blocking::i2c;
use pwm_pca9685::{Channel, Pca9685};

/// All possible errors
#[derive(Debug)]
pub enum Error<E> {
    /// Pca9685 error
    Pca9685(pwm_pca9685::Error<E>),
    /// Motor already taken
    MotorAlreadyTaken,
}

impl<E> From<pwm_pca9685::Error<E>> for Error<E> {
    fn from(e: pwm_pca9685::Error<E>) -> Self {
        Error::Pca9685(e)
    }
}

/// Represents the motor driver and can give out [`Motor`] instances
pub struct MotorDriver<I2C> {
    pwm: Pca9685<I2C>,
    taken: [bool; 4],
}

#[allow(unused)]
pub enum MotorDirection {
    Forward,
    Backward,
    Brake,
    Free,
}

#[allow(unused)]
pub enum MotorId {
    M0,
    M1,
    M2,
    M3,
}

impl MotorId {
    fn as_u8(&self) -> u8 {
        match self {
            MotorId::M0 => 0,
            MotorId::M1 => 1,
            MotorId::M2 => 2,
            MotorId::M3 => 3,
        }
    }
}

pub struct Motor<I2C> {
    _i2c: PhantomData<I2C>,
    in1: Channel,
    in2: Channel,
    pwm: Channel,
}

impl<I2C, E> MotorDriver<I2C>
where
    I2C: i2c::Write<Error = E> + i2c::WriteRead<Error = E>,
{
    /// Create and initialize a new motor driver instance
    pub fn new(i2c: I2C, address: u8, mut frequency_hz: f32) -> Result<Self, Error<E>> {
        let mut pwm = Pca9685::new(i2c, address)?;

        // initialize the driver

        frequency_hz *= 0.9; // Correct for overshoot in the frequency setting

        let mut prescaleval = 25000000.0;
        prescaleval /= 4096.0;
        prescaleval /= frequency_hz;
        prescaleval -= 1.0;
        let prescale = (prescaleval + 0.5) as u8;

        pwm.enable()?;
        pwm.set_prescale(prescale)?;

        Ok(Self {
            pwm,
            taken: [false; 4],
        })
    }

    /// Get a motor instance
    pub fn motor(&mut self, motor: MotorId) -> Result<Motor<I2C>, Error<E>> {
        // make sure the motor is not already taken
        if self.taken[motor.as_u8() as usize] {
            return Err(Error::MotorAlreadyTaken);
        }
        self.taken[motor.as_u8() as usize] = true;

        let (in1, in2, pwm) = match motor {
            MotorId::M0 => (Channel::C10, Channel::C9, Channel::C8),
            MotorId::M1 => (Channel::C13, Channel::C12, Channel::C11),
            MotorId::M2 => (Channel::C4, Channel::C3, Channel::C2),
            MotorId::M3 => (Channel::C5, Channel::C6, Channel::C7),
        };

        let motor = Motor {
            _i2c: PhantomData,
            in1,
            in2,
            pwm,
        };

        Ok(motor)
    }
}

impl<I2C, E> Motor<I2C>
where
    I2C: i2c::Write<Error = E> + i2c::WriteRead<Error = E>,
{
    /// Set the speed of the motor
    pub fn set_speed(&mut self, mc: &mut MotorDriver<I2C>, speed: u16) -> Result<(), Error<E>> {
        mc.pwm.set_channel_on_off(self.pwm, 0, speed)?;
        Ok(())
    }

    /// Set the direction of the motor
    pub fn set_direction(
        &mut self,
        mc: &mut MotorDriver<I2C>,
        direction: MotorDirection,
    ) -> Result<(), Error<E>> {
        match direction {
            MotorDirection::Forward => {
                mc.pwm.set_channel_on_off(self.in2, 0, 0)?; // take low first to avoid brake'
                mc.pwm.set_channel_on_off(self.in1, 0, 4095)?;
            }
            MotorDirection::Backward => {
                mc.pwm.set_channel_on_off(self.in1, 0, 0)?; // take low first to avoid brake'
                mc.pwm.set_channel_on_off(self.in2, 0, 4095)?;
            }
            MotorDirection::Free => {
                mc.pwm.set_channel_on_off(self.in1, 0, 0)?;
                mc.pwm.set_channel_on_off(self.in2, 0, 0)?;
            }
            MotorDirection::Brake => {
                mc.pwm.set_channel_on_off(self.in1, 0, 4095)?;
                mc.pwm.set_channel_on_off(self.in2, 0, 4095)?;
            }
        }
        Ok(())
    }
}
