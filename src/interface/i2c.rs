//! I2C Interface
use super::Interface;
use crate::sensor::Sensor;
use embedded_hal_async::i2c::I2c;

/// Errors in this crate
#[derive(Debug)]
pub enum Error<CommE> {
    /// Communication error
    Comm(CommE),
}

/// Accelerometer/Gyro sensor address for I2C communication
pub enum AgAddress {
    _1 = 0x6A,
    _2 = 0x6B,
}

impl AgAddress {
    pub const fn addr(self) -> u8 {
        self as u8
    }
}

/// Magnetometer sensor address for I2C communication
pub enum MagAddress {
    _1 = 0x1C,
    _2 = 0x1E,
}

impl MagAddress {
    pub const fn addr(self) -> u8 {
        self as u8
    }
}

/// This holds `I2C` and AG and Mag addresses
pub struct I2cInterface<I2C> {
    i2c: I2C,
    ag_addr: u8,
    mag_addr: u8,
}

impl<I2C> I2cInterface<I2C> {
    /// Initializes an Interface with `I2C` instance and AG and Mag addresses
    /// # Arguments
    /// * `i2C` - I2C instance
    /// * `ag_addr` - `AgAddress`: register address for Accelerometer/Gyroscope
    /// * `mag_addr` - `MagAddress`: register address for Magnetometer
    pub fn init(i2c: I2C, ag_addr: AgAddress, mag_addr: MagAddress) -> Self {
        Self {
            i2c,
            ag_addr: ag_addr.addr(),
            mag_addr: mag_addr.addr(),
        }
    }
}

/// Implementation of `Interface`
impl<I2C, CommE> Interface for I2cInterface<I2C>
where
    I2C: I2c<Error = CommE>,
{
    type Error = Error<CommE>;

    async fn write(&mut self, sensor: Sensor, addr: u8, value: u8) -> Result<(), Self::Error> {
        let sensor_addr = match sensor {
            Sensor::Accelerometer | Sensor::Gyro | Sensor::Temperature => self.ag_addr,
            Sensor::Magnetometer => self.mag_addr,
        };
        self.i2c
            .write(sensor_addr, &[addr, value])
            .await
            .map_err(Error::Comm)
    }

    async fn read(
        &mut self,
        sensor: Sensor,
        addr: u8,
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        let sensor_addr = match sensor {
            Sensor::Accelerometer | Sensor::Gyro | Sensor::Temperature => self.ag_addr,
            Sensor::Magnetometer => self.mag_addr,
        };
        self.i2c
            .write_read(sensor_addr, &[addr], buffer)
            .await
            .map_err(Error::Comm)
    }
}
