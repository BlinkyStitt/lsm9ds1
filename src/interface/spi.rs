//! SPI Interface
use super::Interface;
use crate::sensor::Sensor::{self, Accelerometer, Gyro, Magnetometer, Temperature};
use embedded_hal::spi::SpiDevice;

/// R/W bit should be high for SPI Read operation
const SPI_READ: u8 = 0x80;
/// Magnetometer MS bit. When 0, does not increment the address; when 1, increments the address in multiple reads. (Refer to page 34)
const MS_BIT: u8 = 0x40;

/// This combines the SPI Interface and chip select pins
pub struct SpiInterface<SPI>
where
    SPI: SpiDevice,
{
    ag_device: SPI,
    m_device: SPI,
}

impl<SPI, CommE> SpiInterface<SPI>
where
    SPI: SpiDevice<u8, Error = CommE>,
{
    /// Initializes an Interface with `SPI` instance and AG and M chip select `OutputPin`s
    /// # Arguments
    /// * `spi` - SPI instance
    /// * `ag_cs` - Chip Select pin for Accelerometer/Gyroscope
    /// * `m_cs` - Chip Select pin for Magnetometer
    pub fn init(ag_device: SPI, m_device: SPI) -> Self {
        Self {
            ag_device,
            m_device,
        }
    }
}

/// Implementation of `Interface`
impl<SPI, CommE> Interface for SpiInterface<SPI>
where
    SPI: SpiDevice<u8, Error = CommE>,
{
    type Error = CommE;

    fn write(&mut self, sensor: Sensor, addr: u8, value: u8) -> Result<(), Self::Error> {
        let bytes = [addr, value];
        match sensor {
            Accelerometer | Gyro | Temperature => self.ag_device.write(&bytes),
            Magnetometer => self.m_device.write(&bytes),
        }
    }

    fn read(&mut self, sensor: Sensor, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        match sensor {
            Accelerometer | Gyro | Temperature => {
                self.ag_device.transfer(buffer, &[SPI_READ | addr])
            }
            Magnetometer => self.m_device.transfer(buffer, &[SPI_READ | MS_BIT | addr]),
        }
    }
}
