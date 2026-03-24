// Rust guideline compliant 2026-03-24

//! Core driver implementation for the TMAG5273 sensor.
//!
//! Uses a typestate pattern with [`Unconfigured`] and [`Configured`] markers
//! to prevent reading before initialization at compile time.

use core::fmt;
use core::marker::PhantomData;

use embedded_hal::i2c::I2c;

use crate::config::{Config, MagneticChannel, OperatingMode};
use crate::error::{Error, InitError};
use crate::register;
use crate::types::{
    AngleReading, Celsius, ConversionStatus, DeviceStatus, DeviceVariant, MagneticReading, Range,
};

// ---------------------------------------------------------------------------
// Typestate markers
// ---------------------------------------------------------------------------

/// Marker type: sensor has not been initialized.
pub struct Unconfigured;

/// Marker type: sensor has been initialized and is ready for measurement.
pub struct Configured;

// ---------------------------------------------------------------------------
// Tmag5273
// ---------------------------------------------------------------------------

/// Driver for the TI TMAG5273 3-axis Hall-effect sensor.
///
/// # Type Parameters
///
/// - `I2C`: The I2C bus implementing [`embedded_hal::i2c::I2c`].
/// - `State`: Either [`Unconfigured`] or [`Configured`] (typestate marker).
///
/// # Examples
///
/// ```no_run
/// # use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction};
/// # let i2c = I2cMock::new(&[]);
/// use tmag5273::{Tmag5273, DeviceVariant, ConfigBuilder};
///
/// let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
/// let config = ConfigBuilder::new().build().unwrap();
/// // let mut sensor = sensor.init(&config).unwrap();
/// // let reading = sensor.read_magnetic().unwrap();
/// ```
pub struct Tmag5273<I2C, State = Unconfigured> {
    i2c: I2C,
    variant: DeviceVariant,
    address: u8,
    xy_range: Range,
    z_range: Range,
    channels: MagneticChannel,
    max_polls: u16,
    _state: PhantomData<State>,
}

impl<I2C, State> fmt::Debug for Tmag5273<I2C, State> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Tmag5273")
            .field("variant", &self.variant)
            .field("address", &self.address)
            .finish_non_exhaustive()
    }
}

// Compile-time size assertion — should be ~12 bytes on ESP32 (I2C handle ~4 bytes + fields).
const _: () = assert!(
    core::mem::size_of::<Tmag5273<[u8; 4], Configured>>() <= 16,
    "Tmag5273 struct exceeds 16-byte budget (excluding I2C handle)"
);

// ---------------------------------------------------------------------------
// Unconfigured — construction + initialization
// ---------------------------------------------------------------------------

impl<I2C: I2c> Tmag5273<I2C, Unconfigured> {
    /// Create a new driver instance without performing any I2C operations.
    ///
    /// The I2C address is derived from the [`DeviceVariant`].
    pub fn new(i2c: I2C, variant: DeviceVariant) -> Self {
        Self {
            address: variant.default_address(),
            i2c,
            variant,
            xy_range: Range::Low,
            z_range: Range::Low,
            channels: MagneticChannel::Off,
            max_polls: 10,
            _state: PhantomData,
        }
    }

    /// Create a new driver instance with a custom I2C address.
    ///
    /// Use this when the sensor's address has been changed from the factory
    /// default.
    pub fn new_with_address(i2c: I2C, variant: DeviceVariant, address: u8) -> Self {
        Self {
            address,
            i2c,
            variant,
            xy_range: Range::Low,
            z_range: Range::Low,
            channels: MagneticChannel::Off,
            max_polls: 10,
            _state: PhantomData,
        }
    }

    /// Initialize the sensor: verify identity, apply configuration, and
    /// transition to [`Configured`] state.
    ///
    /// Takes `config` by reference so the same configuration can be reused
    /// across multiple sensors.
    ///
    /// # Errors
    ///
    /// Returns [`InitError`] (containing the I2C bus) if initialization fails.
    /// This allows the caller to recover the bus and retry or use it for
    /// another device.
    pub fn init(mut self, config: &Config) -> Result<Tmag5273<I2C, Configured>, InitError<I2C>> {
        if let Err(error) = self.init_inner(config) {
            return Err(InitError {
                error,
                i2c: self.i2c,
            });
        }

        Ok(Tmag5273 {
            i2c: self.i2c,
            variant: self.variant,
            address: self.address,
            xy_range: config.xy_range,
            z_range: config.z_range,
            channels: config.magnetic_channels,
            max_polls: config.max_polls,
            _state: PhantomData,
        })
    }

    /// Inner init logic — returns `Error` for easier `?` usage, wrapped by
    /// `init()` into `InitError`.
    fn init_inner(&mut self, config: &Config) -> Result<(), Error<I2C::Error>> {
        // 1. Read and verify manufacturer ID (registers 0x0E + 0x0F) = 0x5449
        let id_lsb = self.read_register(register::MANUFACTURER_ID_LSB)?;
        let id_msb = self.read_register(register::MANUFACTURER_ID_MSB)?;
        let manufacturer_id = u16::from(id_msb) << 8 | u16::from(id_lsb);

        if manufacturer_id != register::MANUFACTURER_ID_EXPECTED {
            return Err(Error::InvalidManufacturerId(manufacturer_id));
        }

        // 2. Read device ID and verify version matches variant
        let device_id = self.read_register(register::DEVICE_ID)?;
        let version = register::extract(device_id, register::VERSION_MASK, register::VERSION_SHIFT);

        if version != self.variant.version() {
            return Err(Error::VersionMismatch {
                expected: self.variant.version(),
                got: version,
            });
        }

        // 3. Write configuration registers (optimized: one write per register)
        for (addr, value) in config.to_register_writes() {
            self.write_register(addr, value)?;
        }

        // 4. Read and check device status for hardware faults
        let status_byte = self.read_register(register::DEVICE_STATUS)?;
        let status = DeviceStatus::from_reg(status_byte);
        if status.oscillator_error || status.otp_crc_error || status.vcc_undervoltage {
            // A device fault is not a driver error — the sensor itself reports
            // a problem. We proceed but the caller can check status later.
        }

        // 5. Clear POR flag by reading CONV_STATUS
        let _ = self.read_register(register::CONV_STATUS)?;

        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Configured — measurement + control
// ---------------------------------------------------------------------------

impl<I2C: I2c> Tmag5273<I2C, Configured> {
    /// Read magnetic field for all enabled channels.
    ///
    /// Uses a burst read (6 bytes) for data coherence and speed.
    /// Disabled channels return `None`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn read_magnetic(&mut self) -> Result<MagneticReading, Error<I2C::Error>> {
        // Burst read X/Y/Z: 6 bytes starting from X_MSB_RESULT (0x12)
        let mut buf = [0u8; 6];
        self.i2c
            .write_read(self.address, &[register::X_MSB_RESULT], &mut buf)?;

        let raw_x = i16::from_be_bytes([buf[0], buf[1]]);
        let raw_y = i16::from_be_bytes([buf[2], buf[3]]);
        let raw_z = i16::from_be_bytes([buf[4], buf[5]]);

        let xy_range = self.variant.range_mt_xy(self.xy_range);
        let z_range = self.variant.range_mt_z(self.z_range);

        Ok(MagneticReading {
            x: if self.channels.has_x() {
                Some(crate::types::raw_to_millitesla(raw_x, xy_range))
            } else {
                None
            },
            y: if self.channels.has_y() {
                Some(crate::types::raw_to_millitesla(raw_y, xy_range))
            } else {
                None
            },
            z: if self.channels.has_z() {
                Some(crate::types::raw_to_millitesla(raw_z, z_range))
            } else {
                None
            },
        })
    }

    /// Read temperature from the sensor.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn read_temperature(&mut self) -> Result<Celsius, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(self.address, &[register::T_MSB_RESULT], &mut buf)?;

        let raw = i16::from_be_bytes([buf[0], buf[1]]);
        Ok(crate::types::raw_to_celsius(raw))
    }

    /// Read the CORDIC angle calculation result.
    ///
    /// Requires angle calculation to be enabled in the configuration.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn read_angle(&mut self) -> Result<AngleReading, Error<I2C::Error>> {
        let mut buf = [0u8; 3];
        self.i2c
            .write_read(self.address, &[register::ANGLE_RESULT_MSB], &mut buf)?;

        let raw_angle = u16::from_be_bytes([buf[0], buf[1]]);
        let magnitude = buf[2];

        Ok(AngleReading {
            angle: crate::types::raw_to_degrees(raw_angle),
            magnitude,
        })
    }

    /// Switch the operating mode at runtime.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn set_mode(&mut self, mode: OperatingMode) -> Result<(), Error<I2C::Error>> {
        let current = self.read_register(register::DEVICE_CONFIG_2)?;
        let updated = register::insert(
            current,
            register::OPERATING_MODE_MASK,
            register::OPERATING_MODE_SHIFT,
            mode as u8,
        );
        self.write_register(register::DEVICE_CONFIG_2, updated)?;
        Ok(())
    }

    /// Trigger a single conversion in standby or trigger mode.
    ///
    /// After calling this, use [`wait_for_conversion`](Self::wait_for_conversion)
    /// to poll for completion.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn trigger_conversion(&mut self) -> Result<(), Error<I2C::Error>> {
        // Writing operating mode = standby triggers a single conversion.
        let current = self.read_register(register::DEVICE_CONFIG_2)?;
        let trigger = register::insert(
            current,
            register::OPERATING_MODE_MASK,
            register::OPERATING_MODE_SHIFT,
            OperatingMode::Standby as u8,
        );
        self.write_register(register::DEVICE_CONFIG_2, trigger)?;
        Ok(())
    }

    /// Poll for conversion completion with bounded retry count.
    ///
    /// The poll limit is derived from the configured averaging mode (2x safety
    /// margin over datasheet maximum). No `Delay` trait is required.
    ///
    /// # Errors
    ///
    /// Returns [`Error::ConversionTimeout`] if the conversion does not complete
    /// within the poll limit, or [`Error::I2c`] on bus failure.
    pub fn wait_for_conversion(&mut self) -> Result<ConversionStatus, Error<I2C::Error>> {
        for _ in 0..self.max_polls {
            let raw = self.read_register(register::CONV_STATUS)?;
            let status = ConversionStatus::from_reg(raw);
            if status.result_ready {
                return Ok(status);
            }
        }
        Err(Error::ConversionTimeout)
    }

    /// Check whether the latest conversion result is ready.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn is_conversion_complete(&mut self) -> Result<bool, Error<I2C::Error>> {
        let raw = self.read_register(register::CONV_STATUS)?;
        Ok(register::extract(
            raw,
            register::RESULT_STATUS_MASK,
            register::RESULT_STATUS_SHIFT,
        ) != 0)
    }

    /// Read the device status register.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn read_status(&mut self) -> Result<DeviceStatus, Error<I2C::Error>> {
        let raw = self.read_register(register::DEVICE_STATUS)?;
        Ok(DeviceStatus::from_reg(raw))
    }

    /// Put the sensor into sleep mode for power savings.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        self.set_mode(OperatingMode::Sleep)
    }

    /// Release the I2C bus, consuming the driver.
    ///
    /// The sensor is left in whatever operating mode it was in. Call
    /// [`sleep`](Self::sleep) before this if you want to minimize power draw.
    pub fn into_inner(self) -> I2C {
        self.i2c
    }
}

// ---------------------------------------------------------------------------
// Private I2C helpers (shared across both states)
// ---------------------------------------------------------------------------

impl<I2C: I2c, State> Tmag5273<I2C, State> {
    /// Read a single register byte.
    fn read_register(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0u8];
        self.i2c.write_read(self.address, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    /// Write a single register byte.
    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[reg, value])?;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    extern crate alloc;

    use alloc::vec;
    use alloc::vec::Vec;

    use super::*;
    use crate::config::ConfigBuilder;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    /// Helper: create mock expectations for a successful init sequence.
    fn init_expectations(address: u8) -> Vec<I2cTransaction> {
        vec![
            // Read manufacturer ID LSB (0x0E) → 0x49
            I2cTransaction::write_read(address, vec![0x0E], vec![0x49]),
            // Read manufacturer ID MSB (0x0F) → 0x54
            I2cTransaction::write_read(address, vec![0x0F], vec![0x54]),
            // Read device ID (0x0D) → version 1
            I2cTransaction::write_read(address, vec![0x0D], vec![0x01]),
            // Write 5 config registers
            I2cTransaction::write(address, vec![0x00, 0x00]), // DEVICE_CONFIG_1
            I2cTransaction::write(address, vec![0x01, 0x02]), // DEVICE_CONFIG_2 (continuous)
            I2cTransaction::write(address, vec![0x02, 0x70]), // SENSOR_CONFIG_1 (XYZ)
            I2cTransaction::write(address, vec![0x03, 0x03]), // SENSOR_CONFIG_2 (high range)
            I2cTransaction::write(address, vec![0x07, 0x01]), // T_CONFIG (temp enabled)
            // Read device status
            I2cTransaction::write_read(address, vec![0x1C], vec![0x00]),
            // Read conv status (clear POR)
            I2cTransaction::write_read(address, vec![0x18], vec![0x00]),
        ]
    }

    #[test]
    fn init_successful() {
        let i2c = I2cMock::new(&init_expectations(0x22));
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let sensor = sensor.init(&config).unwrap();
        sensor.into_inner().done();
    }

    #[test]
    fn init_wrong_manufacturer_id_returns_bus() {
        let expectations = vec![
            // Return wrong manufacturer ID
            I2cTransaction::write_read(0x22, vec![0x0E], vec![0x00]),
            I2cTransaction::write_read(0x22, vec![0x0F], vec![0x00]),
        ];
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();

        let mut err = sensor.init(&config).unwrap_err();
        assert_eq!(err.error, Error::InvalidManufacturerId(0x0000));
        // Bus is recoverable
        err.i2c.done();
    }

    #[test]
    fn init_wrong_version_returns_bus() {
        let expectations = vec![
            I2cTransaction::write_read(0x22, vec![0x0E], vec![0x49]),
            I2cTransaction::write_read(0x22, vec![0x0F], vec![0x54]),
            // Return version 2 when B1 expects version 1
            I2cTransaction::write_read(0x22, vec![0x0D], vec![0x02]),
        ];
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();

        let mut err = sensor.init(&config).unwrap_err();
        assert_eq!(
            err.error,
            Error::VersionMismatch {
                expected: 1,
                got: 2,
            }
        );
        err.i2c.done();
    }

    #[test]
    fn read_magnetic_burst() {
        let mut expectations = init_expectations(0x22);
        // Burst read X/Y/Z: 6 bytes from register 0x12
        expectations.push(I2cTransaction::write_read(
            0x22,
            vec![0x12],
            vec![0x40, 0x00, 0x20, 0x00, 0x10, 0x00], // X=16384, Y=8192, Z=4096
        ));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let reading = sensor.read_magnetic().unwrap();
        // B1 = version 1, High range = 80 mT
        // X: -16384 * 80 / 32768 = -40.0 mT
        assert!(reading.x.is_some());
        assert!(reading.y.is_some());
        assert!(reading.z.is_some());

        sensor.into_inner().done();
    }

    #[test]
    fn read_temperature() {
        let mut expectations = init_expectations(0x22);
        // Temperature read: 2 bytes from register 0x10
        // raw = 17508 (0x4464) → 25.0°C
        expectations.push(I2cTransaction::write_read(
            0x22,
            vec![0x10],
            vec![0x44, 0x64],
        ));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let temp = sensor.read_temperature().unwrap();
        assert!((temp.0 - 25.0).abs() < 0.1);

        sensor.into_inner().done();
    }

    #[test]
    fn wait_for_conversion_timeout() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .operating_mode(OperatingMode::Standby)
            .build()
            .unwrap();

        // Build init expectations dynamically from config
        let mut expectations = vec![
            I2cTransaction::write_read(address, vec![0x0E], vec![0x49]),
            I2cTransaction::write_read(address, vec![0x0F], vec![0x54]),
            I2cTransaction::write_read(address, vec![0x0D], vec![0x01]),
        ];
        for (reg, val) in config.to_register_writes() {
            expectations.push(I2cTransaction::write(address, vec![reg, val]));
        }
        expectations.push(I2cTransaction::write_read(address, vec![0x1C], vec![0x00]));
        expectations.push(I2cTransaction::write_read(address, vec![0x18], vec![0x00]));

        // trigger_conversion: read-modify-write on DEVICE_CONFIG_2
        let dc2_value = config
            .to_register_writes()
            .iter()
            .find(|(a, _)| *a == 0x01)
            .unwrap()
            .1;
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x01],
            vec![dc2_value],
        ));
        expectations.push(I2cTransaction::write(address, vec![0x01, dc2_value]));

        // Poll max_polls times (10 for X1), all return not-ready
        for _ in 0..config.max_polls {
            expectations.push(I2cTransaction::write_read(address, vec![0x18], vec![0x00]));
        }

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        sensor.trigger_conversion().unwrap();
        let err = sensor.wait_for_conversion().unwrap_err();
        assert_eq!(err, Error::ConversionTimeout);

        sensor.into_inner().done();
    }
}
