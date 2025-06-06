//! SPI Interface
use super::Interface;
use crate::sensor::Sensor;
use embedded_hal_async::spi::{Operation, SpiDevice};

/// R/W bit should be high for SPI Read operation
const SPI_READ: u8 = 0x80;
/// Magnetometer MS bit. When 0, does not increment the address; when 1, increments the address in multiple reads. (Refer to page 34)
const MS_BIT: u8 = 0x40;

/// This combines the SPI Interface and chip select pins
pub struct SpiInterface<AGSpi, MSpi>
where
    AGSpi: SpiDevice,
    MSpi: SpiDevice,
{
    ag_device: AGSpi,
    m_device: MSpi,
}

impl<AGSpi, MSpi> SpiInterface<AGSpi, MSpi>
where
    AGSpi: SpiDevice,
    MSpi: SpiDevice,
{
    /// Initializes an Interface with `SPI` instance and AG and M chip select `OutputPin`s
    /// # Arguments
    /// * `spi` - SPI instance
    /// * `ag_cs` - Chip Select pin for Accelerometer/Gyroscope
    /// * `m_cs` - Chip Select pin for Magnetometer
    ///
    /// TODO: take the chip select pins and the bus and create the device here? will need to use embedded-hal-bus probably
    pub fn init(ag_device: AGSpi, m_device: MSpi) -> Self {
        Self {
            ag_device,
            m_device,
        }
    }
}

/// TODO: this doesn't seem right. these are probably the same error type with a normal setup, but you could have them using different buses
pub enum Errors<AgE, ME> {
    AG(AgE),
    M(ME),
}

/// Implementation of `Interface`
impl<AGSpi, MSpi> Interface for SpiInterface<AGSpi, MSpi>
where
    AGSpi: SpiDevice,
    MSpi: SpiDevice,
{
    type Error = Errors<AGSpi::Error, MSpi::Error>;

    async fn write(&mut self, sensor: Sensor, addr: u8, value: u8) -> Result<(), Self::Error> {
        let bytes = [addr, value];
        match sensor {
            Sensor::Accelerometer | Sensor::Gyro | Sensor::Temperature => {
                self.ag_device.write(&bytes).await.map_err(Errors::AG)
            }
            Sensor::Magnetometer => self.m_device.write(&bytes).await.map_err(Errors::M),
        }
    }

    async fn read(
        &mut self,
        sensor: Sensor,
        addr: u8,
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        match sensor {
            Sensor::Accelerometer | Sensor::Gyro | Sensor::Temperature => self
                .ag_device
                .transaction(&mut [
                    Operation::Write(&[SPI_READ | addr]),
                    Operation::Read(buffer),
                ])
                .await
                .map_err(Errors::AG),
            Sensor::Magnetometer => self
                .m_device
                .transaction(&mut [
                    Operation::Write(&[SPI_READ | MS_BIT | addr]),
                    Operation::Read(buffer),
                ])
                .await
                .map_err(Errors::M),
        }
    }
}
