//! A platform agnostic driver to interface with LSM9DS1 3D accelerometer, 3D gyroscope, 3D magnetometer sensor module.
//!
//! ### Datasheets
//! - [LSM9DS1](https://www.st.com/resource/en/datasheet/lsm9ds1.pdf)
//!
//! TODO: not sure about `async_fn_in_trait`
#![allow(async_fn_in_trait)]
#![no_std]
// TODO: not sure about this
pub mod accel;
pub mod configuration;
pub mod fifo;
pub mod gyro;
pub mod interface;
pub mod interrupts;
pub mod mag;
pub mod register;
pub mod sensor;

use accel::AccelSettings;
use configuration::ConfigToWrite;
use fifo::{Decimate, FIFOBitmasks, FIFOConfig, FIFOStatus};
use gyro::GyroSettings;
use interrupts::accel_int::IntConfigAccel;
use interrupts::gyro_int::IntConfigGyro;
use interrupts::mag_int::IntConfigMag;
use interrupts::pins_config::{self, IntConfigAG1, IntConfigAG2, PinConfig};
use mag::MagSettings;
use pins_config::PinConfigBitmask;
use sensor::Sensor;

use crate::interface::Interface;

/// Accelerometer/Gyroscope's ID
const WHO_AM_I_AG: u8 = 0x68;
/// Magnetometer's ID
const WHO_AM_I_M: u8 = 0x3D;
/// temperature scale
const TEMP_SCALE: f32 = 16.0;
/// The output of the temperature sensor is 0 (typ.) at 25 °C. see page 14: Temperature sensor characteristics
const TEMP_BIAS: f32 = 25.0;

/// LSM9DS1 init struct.
/// Use this struct to configure sensors and init LSM9DS1 with an interface of your choice.
#[derive(Default)]
pub struct LSM9DS1Init {
    pub accel: AccelSettings,
    pub gyro: GyroSettings,
    pub mag: MagSettings,
}

impl LSM9DS1Init {
    /// Constructs a new LSM9DS1 driver instance with a I2C or SPI peripheral.
    ///
    /// # Arguments
    /// * `interface` - `SpiInterface` or `I2cInterface`
    pub fn with_interface<T>(self, interface: T) -> LSM9DS1<T>
    where
        T: Interface,
    {
        LSM9DS1 {
            interface,
            accel: self.accel,
            gyro: self.gyro,
            mag: self.mag,
        }
    }
}

/// LSM9DS1 IMU
pub struct LSM9DS1<T>
where
    T: Interface,
{
    interface: T,
    accel: AccelSettings,
    gyro: GyroSettings,
    mag: MagSettings,
}

/// TODO: is this right? i don't think so
pub struct LSM9DS1AccelGyro<'a, T>
where
    T: Interface,
{
    interface: &'a T,
    accel: AccelSettings,
    gyro: GyroSettings,
}

/// TODO: is this right? i don't think so
pub struct LSM9DS1Mag<'a, T>
where
    T: Interface,
{
    interface: &'a T,
    mag: MagSettings,
}

impl<T> LSM9DS1<T>
where
    T: Interface,
{
    // TODO: `async fn split(self) -> (LSM9DS1AccelGyro<&'static T>)`?

    /// Write a configuration to a register.
    async fn write_register<C: ConfigToWrite>(&mut self, config: C) -> Result<(), T::Error> {
        self.interface
            .write(config.sensor(), config.addr(), config.byte())
            .await?;
        Ok(())
    }
    /// Modify a register with a configuration.
    async fn modify_register<C: ConfigToWrite>(
        &mut self,
        config: C,
        original_value: u8,
        bitmask: u8,
    ) -> Result<(), T::Error> {
        let mut data: u8 = original_value & bitmask;
        data |= config.byte();
        self.interface
            .write(config.sensor(), config.addr(), data)
            .await?;
        Ok(())
    }

    async fn reachable(&mut self, sensor: Sensor) -> Result<bool, T::Error> {
        let (who_am_i, register) = match sensor {
            Sensor::Accelerometer | Sensor::Gyro | Sensor::Temperature => {
                (WHO_AM_I_AG, register::AG::WHO_AM_I.addr())
            }
            Sensor::Magnetometer => (WHO_AM_I_M, register::Mag::WHO_AM_I.addr()),
        };
        Ok(self.read_register(sensor, register).await? == who_am_i)
    }

    /// Verifies communication with WHO_AM_I register
    pub async fn accel_is_reacheable(&mut self) -> Result<bool, T::Error> {
        self.reachable(Sensor::Accelerometer).await
    }
    /// Verifies communication with WHO_AM_I register
    pub async fn mag_is_reacheable(&mut self) -> Result<bool, T::Error> {
        self.reachable(Sensor::Magnetometer).await
    }
    /// Initializes Accelerometer with sensor settings.
    pub async fn begin_accel(&mut self) -> Result<(), T::Error> {
        self.write_register(self.accel.ctrl_reg5_xl_config())
            .await?;
        self.write_register(self.accel.ctrl_reg6_xl_config())
            .await?;
        self.write_register(self.accel.ctrl_reg7_xl_config())
            .await?;
        Ok(())
    }
    /// Initializes Gyro with sensor settings.
    pub async fn begin_gyro(&mut self) -> Result<(), T::Error> {
        self.write_register(self.gyro.ctrl_reg1_g_config()).await?;
        self.write_register(self.gyro.ctrl_reg2_g_config()).await?;
        self.write_register(self.gyro.ctrl_reg3_g_config()).await?;
        self.write_register(self.gyro.ctrl_reg4_config()).await?;
        Ok(())
    }
    /// Initializes Magnetometer with sensor settings.
    pub async fn begin_mag(&mut self) -> Result<(), T::Error> {
        self.write_register(self.mag.ctrl_reg1_m_config()).await?;
        self.write_register(self.mag.ctrl_reg2_m_config()).await?;
        self.write_register(self.mag.ctrl_reg3_m_config()).await?;
        self.write_register(self.mag.ctrl_reg4_m_config()).await?;
        self.write_register(self.mag.ctrl_reg5_m_config()).await?;
        Ok(())
    }

    async fn data_available(&mut self, sensor: Sensor) -> Result<u8, T::Error> {
        let register = match sensor {
            Sensor::Accelerometer | Sensor::Gyro | Sensor::Temperature => {
                register::AG::STATUS_REG_1.addr()
            }
            Sensor::Magnetometer => register::Mag::STATUS_REG_M.addr(),
        };
        self.read_register(sensor, register).await
    }
    /// Sees if new Accelerometer data is available
    pub async fn accel_data_available(&mut self) -> Result<bool, T::Error> {
        match self.data_available(Sensor::Accelerometer).await? {
            x if x & 0x01 > 0 => Ok(true),
            _ => Ok(false),
        }
    }
    /// Sees if new Gyro data is available
    pub async fn gyro_data_available(&mut self) -> Result<bool, T::Error> {
        match self.data_available(Sensor::Gyro).await? {
            x if x & 0x02 > 0 => Ok(true),
            _ => Ok(false),
        }
    }
    /// Sees if new Magnetometer data is available
    pub async fn mag_data_available(&mut self) -> Result<bool, T::Error> {
        match self.data_available(Sensor::Magnetometer).await? {
            x if x & 0x01 > 0 => Ok(true),
            _ => Ok(false),
        }
    }
    /// Sees if new Temperature data is available
    pub async fn temp_data_available(&mut self) -> Result<bool, T::Error> {
        match self.data_available(Sensor::Temperature).await? {
            x if x & 0x04 > 0 => Ok(true),
            _ => Ok(false),
        }
    }
    /// raw sensor reading for x, y, z axis
    async fn read_sensor_raw(
        &mut self,
        sensor: Sensor,
        addr: u8,
    ) -> Result<(i16, i16, i16), T::Error> {
        let mut bytes = [0u8; 6];
        self.interface.read(sensor, addr, &mut bytes).await?;
        let x: i16 = (bytes[1] as i16) << 8 | bytes[0] as i16;
        let y: i16 = (bytes[3] as i16) << 8 | bytes[2] as i16;
        let z: i16 = (bytes[5] as i16) << 8 | bytes[4] as i16;
        Ok((x, y, z))
    }
    /// raw accelerometer readings
    pub async fn read_accel_raw(&mut self) -> Result<(i16, i16, i16), T::Error> {
        self.read_sensor_raw(Sensor::Accelerometer, register::AG::OUT_X_L_XL.addr())
            .await
    }
    /// calculated accelerometer readings (x, y, z)
    pub async fn read_accel(&mut self) -> Result<(f32, f32, f32), T::Error> {
        let (x, y, z) = self.read_accel_raw().await?;
        let sensitivity = self.accel.scale.sensitivity();
        Ok((
            x as f32 * sensitivity,
            y as f32 * sensitivity,
            z as f32 * sensitivity,
        ))
    }
    /// raw gyro readings
    pub async fn read_gyro_raw(&mut self) -> Result<(i16, i16, i16), T::Error> {
        self.read_sensor_raw(Sensor::Gyro, register::AG::OUT_X_L_G.addr())
            .await
    }
    /// calculated gyro readings (x, y, z)
    pub async fn read_gyro(&mut self) -> Result<(f32, f32, f32), T::Error> {
        let (x, y, z) = self.read_gyro_raw().await?;
        let sensitivity = self.gyro.scale.sensitivity();
        Ok((
            x as f32 * sensitivity,
            y as f32 * sensitivity,
            z as f32 * sensitivity,
        ))
    }
    /// raw magnetometer readings
    pub async fn read_mag_raw(&mut self) -> Result<(i16, i16, i16), T::Error> {
        self.read_sensor_raw(Sensor::Magnetometer, register::Mag::OUT_X_L_M.addr())
            .await
    }
    /// calculated magnetometer readings (x, y, z)
    pub async fn read_mag(&mut self) -> Result<(f32, f32, f32), T::Error> {
        let (x, y, z) = self.read_mag_raw().await?;
        let sensitivity = self.mag.scale.sensitivity();
        Ok((
            x as f32 * sensitivity,
            y as f32 * sensitivity,
            z as f32 * sensitivity,
        ))
    }
    /// Reads calculated temperature in Celsius
    pub async fn read_temp(&mut self) -> Result<f32, T::Error> {
        let mut bytes = [0u8; 2];
        self.interface
            .read(
                Sensor::Accelerometer,
                register::AG::OUT_TEMP_L.addr(),
                &mut bytes,
            )
            .await?;
        let result: i16 = (bytes[1] as i16) << 8 | bytes[0] as i16;
        Ok((result as f32) / TEMP_SCALE + TEMP_BIAS)
    }

    /// Enable and configure FIFO
    pub async fn configure_fifo(&mut self, config: FIFOConfig) -> Result<(), T::Error> {
        // write values to the FIFO_CTRL register
        self.write_register(config.f_fifo_ctrl_config()).await?;

        let ctrl_reg9: u8 = self
            .read_register(Sensor::Accelerometer, register::AG::CTRL_REG9.addr())
            .await?;
        // write values to specific bits of the CTRL_REG9 register
        self.modify_register(
            config.f_ctrl_reg9_config(),
            ctrl_reg9,
            !FIFOBitmasks::CTRL_REG9_FIFO,
        )
        .await
    }

    /// Get flags and FIFO level from the FIFO_STATUS register
    pub async fn get_fifo_status(&mut self) -> Result<FIFOStatus, T::Error> {
        Ok(self
            .read_register(Sensor::Accelerometer, register::AG::FIFO_SRC.addr())
            .await?
            .into())
    }

    /// Sets decimation of acceleration data on OUT REG and FIFO
    pub async fn set_decimation(&mut self, decimation: Decimate) -> Result<(), T::Error> {
        let ctrl_reg5 = self
            .read_register(Sensor::Accelerometer, register::AG::CTRL_REG5_XL.addr())
            .await?;
        self.modify_register(decimation, ctrl_reg5, !FIFOBitmasks::DEC)
            .await
    }

    /// Enable interrupts for accelerometer/gyroscope and configure the INT1_A/G interrupt pin
    pub async fn configure_interrupts_ag1(&mut self, config: IntConfigAG1) -> Result<(), T::Error> {
        self.write_register(config).await
    }

    /// Enable interrupts for accelerometer/gyroscope and configure the INT2_A/G interrupt pin
    pub async fn configure_interrupts_ag2(&mut self, config: IntConfigAG2) -> Result<(), T::Error> {
        self.write_register(config).await
    }

    /// Interrupt pins electrical configuration
    pub async fn configure_interrupts_pins(&mut self, config: PinConfig) -> Result<(), T::Error> {
        let ctrl_reg8 = self
            .read_register(Sensor::Accelerometer, register::AG::CTRL_REG8.addr())
            .await?;
        self.modify_register(
            config,
            ctrl_reg8,
            !(PinConfigBitmask::ACTIVE_LEVEL | PinConfigBitmask::PIN_MODE),
        )
        .await
    }

    /// Get the current A/G1 pin configuration
    pub async fn get_ag1_config(&mut self) -> Result<IntConfigAG1, T::Error> {
        Ok(self
            .read_register(Sensor::Accelerometer, register::AG::INT1_CTRL.addr())
            .await?
            .into())
    }

    /// Get the current A/G2 pin configuration
    pub async fn get_ag2_config(&mut self) -> Result<IntConfigAG2, T::Error> {
        Ok(self
            .read_register(Sensor::Accelerometer, register::AG::INT2_CTRL.addr())
            .await?
            .into())
    }

    /// Get the current common pins configuration
    pub async fn get_pins_config(&mut self) -> Result<PinConfig, T::Error> {
        Ok(self
            .read_register(Sensor::Accelerometer, register::AG::CTRL_REG8.addr())
            .await?
            .into())
    }

    /// Get the current Accelerometer interrupt configuration
    pub async fn get_accel_int_config(&mut self) -> Result<IntConfigAccel, T::Error> {
        Ok(self
            .read_register(Sensor::Accelerometer, register::AG::INT_GEN_CFG_XL.addr())
            .await?
            .into())
    }

    /// Get the current Gyro interrupt configuration
    pub async fn get_gyro_int_config(&mut self) -> Result<IntConfigGyro, T::Error> {
        Ok(self
            .read_register(Sensor::Gyro, register::AG::INT_GEN_CFG_G.addr())
            .await?
            .into())
    }

    /// Get the current Magnetometer interrupt configuration
    pub async fn get_mag_int_config(&mut self) -> Result<IntConfigMag, T::Error> {
        Ok(self
            .read_register(Sensor::Magnetometer, register::Mag::INT_CFG_M.addr())
            .await?
            .into())
    }

    /// Configure Accelerometer interrupt
    pub async fn configure_interrupts_accel(
        &mut self,
        config: IntConfigAccel,
    ) -> Result<(), T::Error> {
        self.write_register(config).await
    }

    /// Configure Gyro interrupt
    pub async fn configure_interrupts_gyro(
        &mut self,
        config: IntConfigGyro,
    ) -> Result<(), T::Error> {
        self.write_register(config).await
    }

    /// Configure Magnetometer interrupt
    pub async fn configure_interrupts_mag(&mut self, config: IntConfigMag) -> Result<(), T::Error> {
        self.write_register(config).await
    }

    /// Read a byte from the given register.
    async fn read_register(&mut self, sensor: Sensor, address: u8) -> Result<u8, T::Error> {
        let mut reg_data = [0u8];
        self.interface.read(sensor, address, &mut reg_data).await?;
        Ok(reg_data[0])
    }
}
