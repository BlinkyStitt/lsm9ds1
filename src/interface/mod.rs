//! Interface trait
pub mod i2c;
pub mod spi;

pub use self::i2c::I2cInterface;
pub use self::spi::SpiInterface;
use crate::sensor::Sensor;

/// Interface Trait. `SpiInterface` and `I2cInterface` implement this.
pub trait Interface {
    type Error;

    /// Writes a byte to a sensor's specified register address.
    /// # Arguments
    /// * `sensor` - `Sensor` to talk to
    /// * `addr` - register address
    /// * `value` - value to write
    async fn write(&mut self, sensor: Sensor, addr: u8, value: u8) -> Result<(), Self::Error>;
    /// Reads multiple bytes from a sensor's specified register address.
    /// # Arguments
    /// * `sensor` - `Sensor` to talk to
    /// * `addr` - register address
    /// * `buffer` - buffer to store read data
    async fn read(
        &mut self,
        sensor: Sensor,
        addr: u8,
        buffer: &mut [u8],
    ) -> Result<(), Self::Error>;
}
