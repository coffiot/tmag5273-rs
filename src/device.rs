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

    // -----------------------------------------------------------------------
    // Advanced: Thresholds
    // -----------------------------------------------------------------------

    /// Configure per-axis magnetic field and temperature thresholds.
    ///
    /// Threshold crossing can trigger an interrupt when combined with
    /// [`set_interrupt`](Self::set_interrupt). The direction (above/below)
    /// and hysteresis are part of the threshold configuration.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn set_thresholds(
        &mut self,
        config: &crate::config::ThresholdConfig,
    ) -> Result<(), Error<I2C::Error>> {
        // Per-axis thresholds: direct register writes (full-byte fields).
        self.write_register(register::X_THR_CONFIG, config.x as u8)?;
        self.write_register(register::Y_THR_CONFIG, config.y as u8)?;
        self.write_register(register::Z_THR_CONFIG, config.z as u8)?;

        // Temperature threshold: upper 7 bits of T_CONFIG, preserve T_CH_EN.
        let t_reg = self.read_register(register::T_CONFIG)?;
        let t_new = register::insert(
            t_reg,
            register::T_THR_CONFIG_MASK,
            register::T_THR_CONFIG_SHIFT,
            config.temperature as u8,
        );
        self.write_register(register::T_CONFIG, t_new)?;

        // Hysteresis: DEVICE_CONFIG_2[7:5] — read-modify-write.
        let dc2 = self.read_register(register::DEVICE_CONFIG_2)?;
        let dc2 = register::insert(
            dc2,
            register::THR_HYST_MASK,
            register::THR_HYST_SHIFT,
            config.hysteresis & 0x07, // clamp to 3-bit
        );
        self.write_register(register::DEVICE_CONFIG_2, dc2)?;

        // Direction: SENSOR_CONFIG_2[5] — read-modify-write.
        let sc2 = self.read_register(register::SENSOR_CONFIG_2)?;
        let sc2 = register::insert(
            sc2,
            register::MAG_THR_DIR_MASK,
            register::MAG_THR_DIR_SHIFT,
            config.direction as u8,
        );
        self.write_register(register::SENSOR_CONFIG_2, sc2)?;

        Ok(())
    }

    // -----------------------------------------------------------------------
    // Advanced: Interrupts
    // -----------------------------------------------------------------------

    /// Configure the interrupt output behavior.
    ///
    /// The TMAG5273 supports 5 interrupt routing modes plus options for
    /// conversion-complete and threshold-crossing triggers.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn set_interrupt(
        &mut self,
        config: &crate::config::InterruptConfig,
    ) -> Result<(), Error<I2C::Error>> {
        // INT_CONFIG_1 is a standalone register — full write, no read-modify-write.
        let reg = register::insert(
            0,
            register::RSLT_INT_MASK,
            register::RSLT_INT_SHIFT,
            u8::from(config.on_conversion_complete),
        ) | register::insert(
            0,
            register::THRESHOLD_INT_MASK,
            register::THRESHOLD_INT_SHIFT,
            u8::from(config.on_threshold_crossing),
        ) | register::insert(
            0,
            register::INT_STATE_MASK,
            register::INT_STATE_SHIFT,
            config.pin_behavior as u8,
        ) | register::insert(
            0,
            register::INT_MODE_MASK,
            register::INT_MODE_SHIFT,
            config.mode as u8,
        ) | register::insert(
            0,
            register::MASK_INTB_MASK,
            register::MASK_INTB_SHIFT,
            u8::from(config.mask),
        );
        self.write_register(register::INT_CONFIG_1, reg)?;
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Advanced: Gain/Offset Calibration
    // -----------------------------------------------------------------------

    /// Configure magnetic gain and offset calibration.
    ///
    /// Gain and offset corrections are applied by the sensor hardware to
    /// improve measurement accuracy. The gain channel selects which axis
    /// pair receives the adjustment.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn set_calibration(
        &mut self,
        config: &crate::config::CalibrationConfig,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(register::MAG_GAIN_CONFIG, config.gain)?;
        self.write_register(register::MAG_OFFSET_CONFIG_1, config.offset_1 as u8)?;
        // NOTE: This is register 0x0B (MAG_OFFSET_CONFIG_2), NOT 0x0A.
        // The SparkFun Arduino library has a copy-paste bug that writes both
        // offsets to 0x0A. We write to the correct register.
        self.write_register(register::MAG_OFFSET_CONFIG_2, config.offset_2 as u8)?;

        // Gain channel select: SENSOR_CONFIG_2[4] — read-modify-write.
        let sc2 = self.read_register(register::SENSOR_CONFIG_2)?;
        let sc2 = register::insert(
            sc2,
            register::MAG_GAIN_CH_MASK,
            register::MAG_GAIN_CH_SHIFT,
            config.gain_channel as u8,
        );
        self.write_register(register::SENSOR_CONFIG_2, sc2)?;

        Ok(())
    }

    // -----------------------------------------------------------------------
    // Advanced: I2C Address Change
    // -----------------------------------------------------------------------

    /// Change the sensor's I2C address at runtime.
    ///
    /// Writes the new 7-bit address to register `I2C_ADDRESS` (0x0C) with the
    /// update-enable bit set, then verifies the device responds at the new
    /// address by reading the manufacturer ID.
    ///
    /// On success, all subsequent I2C operations use the new address.
    ///
    /// **Note:** The address resets to the factory default on power cycle.
    /// To make the change permanent across power cycles, the firmware must
    /// re-apply it on every boot.
    ///
    /// # Errors
    ///
    /// Returns [`Error::AddressChangeFailed`] if the device does not respond
    /// at the new address after the write. The device may have already changed
    /// its address — the caller must decide whether to power-cycle or retry.
    ///
    /// Returns [`Error::I2c`] if the initial write to register 0x0C fails.
    pub fn change_address(&mut self, new_address: u8) -> Result<(), Error<I2C::Error>> {
        // Write new address with update-enable bit set.
        let reg_val = register::insert(
            0,
            register::I2C_ADDRESS_BITS_MASK,
            register::I2C_ADDRESS_BITS_SHIFT,
            new_address,
        ) | register::insert(
            0,
            register::I2C_ADDRESS_UPDATE_EN_MASK,
            register::I2C_ADDRESS_UPDATE_EN_SHIFT,
            1,
        );
        self.write_register(register::I2C_ADDRESS, reg_val)?;

        // Switch to new address and verify by reading manufacturer ID.
        let old_address = self.address;
        self.address = new_address;

        let id_lsb = match self.read_register(register::MANUFACTURER_ID_LSB) {
            Ok(v) => v,
            Err(_) => {
                self.address = old_address;
                return Err(Error::AddressChangeFailed { new_address });
            }
        };
        let id_msb = match self.read_register(register::MANUFACTURER_ID_MSB) {
            Ok(v) => v,
            Err(_) => {
                // Device responded to LSB but not MSB — address is changed
                // but something else is wrong. Don't revert.
                return Err(Error::AddressChangeFailed { new_address });
            }
        };

        let id = u16::from(id_msb) << 8 | u16::from(id_lsb);
        if id == register::MANUFACTURER_ID_EXPECTED {
            Ok(())
        } else {
            Err(Error::AddressChangeFailed { new_address })
        }
    }

    // -----------------------------------------------------------------------
    // Advanced: Wake-Up-and-Sleep Lifecycle
    // -----------------------------------------------------------------------

    /// Configure the sensor for wake-up-and-sleep mode.
    ///
    /// This is a convenience method that writes the exact register values
    /// documented in the TI datasheet (section 8.2.1.2) and verified in the
    /// SparkFun Arduino `setupWakeUpAndSleep()` function:
    ///
    /// - `INT_CONFIG_1` = `0x64`: threshold interrupt enabled, INT pin pulsed,
    ///   routed through INT pin
    /// - `DEVICE_CONFIG_2` = `0x23`: 7-LSB hysteresis, low active current,
    ///   glitch filter on, I2C trigger, wake-up-and-sleep mode
    ///
    /// After calling this, the sensor periodically wakes, measures, checks
    /// thresholds, and asserts INT if crossed. Configure thresholds with
    /// [`set_thresholds`](Self::set_thresholds) before calling this method.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn configure_wake_and_sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        // Hardcoded values from TI datasheet section 8.2.1.2:
        //   INT_CONFIG_1 = 0x64 = 0b0110_0100
        //     RSLT_INT=0, THRESHOLD_INT=1, INT_STATE=1(pulse),
        //     INT_MODE=001(INT pin), MASK_INTB=0, INT_PINSTATE=0
        const WAKE_SLEEP_INT_CONFIG: u8 = 0x64;

        //   DEVICE_CONFIG_2 = 0x23 = 0b0010_0011
        //     THR_HYST=001(7 LSB), LP_LN=0, GLITCH_FILTER=0(on),
        //     TRIGGER=0(I2C), OPERATING_MODE=11(wake-up-and-sleep)
        const WAKE_SLEEP_DEVICE_CONFIG: u8 = 0x23;

        self.write_register(register::INT_CONFIG_1, WAKE_SLEEP_INT_CONFIG)?;
        self.write_register(register::DEVICE_CONFIG_2, WAKE_SLEEP_DEVICE_CONFIG)?;
        Ok(())
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

    // -- Phase 1: Thresholds ------------------------------------------------

    #[test]
    fn set_thresholds_writes_correct_registers() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // set_thresholds: 3 direct writes + 3 read-modify-writes
        expectations.push(I2cTransaction::write(address, vec![0x04, 0x05])); // X threshold = 5
        expectations.push(I2cTransaction::write(address, vec![0x05, 0xFB])); // Y threshold = -5 (0xFB)
        expectations.push(I2cTransaction::write(address, vec![0x06, 0x0A])); // Z threshold = 10
        // T_CONFIG read-modify-write: preserve T_CH_EN bit (0x01 from init)
        expectations.push(I2cTransaction::write_read(address, vec![0x07], vec![0x01]));
        expectations.push(I2cTransaction::write(address, vec![0x07, 0x07])); // temp=3 << 1 | T_CH_EN=1
        // DEVICE_CONFIG_2 read-modify-write: hysteresis=2 in bits [7:5]
        expectations.push(I2cTransaction::write_read(address, vec![0x01], vec![0x02]));
        expectations.push(I2cTransaction::write(address, vec![0x01, 0x42])); // 0x02 | (2 << 5) = 0x42
        // SENSOR_CONFIG_2 read-modify-write: direction=Below(1) in bit [5]
        expectations.push(I2cTransaction::write_read(address, vec![0x03], vec![0x03]));
        expectations.push(I2cTransaction::write(address, vec![0x03, 0x23])); // 0x03 | (1 << 5) = 0x23

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        use crate::config::{ThresholdConfig, ThresholdDirection};
        sensor
            .set_thresholds(&ThresholdConfig {
                x: 5,
                y: -5,
                z: 10,
                temperature: 3,
                hysteresis: 2,
                direction: ThresholdDirection::Below,
            })
            .unwrap();

        sensor.into_inner().done();
    }

    // -- Phase 2: Interrupts ------------------------------------------------

    #[test]
    fn set_interrupt_writes_correct_byte() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // INT_CONFIG_1 full write:
        // RSLT_INT=0 | THRESHOLD_INT=1 | INT_STATE=1 | INT_MODE=001 | MASK_INTB=0 | INT_PINSTATE=0
        // = 0b0110_0100 = 0x64 (matches SparkFun's setupWakeUpAndSleep hardcoded value)
        expectations.push(I2cTransaction::write(address, vec![0x08, 0x64]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        use crate::config::{IntPinBehavior, InterruptConfig, InterruptMode};
        sensor
            .set_interrupt(&InterruptConfig {
                mode: InterruptMode::IntPin,
                on_conversion_complete: false,
                on_threshold_crossing: true,
                pin_behavior: IntPinBehavior::Pulse10us,
                mask: false,
            })
            .unwrap();

        sensor.into_inner().done();
    }

    #[test]
    fn set_interrupt_all_modes() {
        use crate::config::{IntPinBehavior, InterruptConfig, InterruptMode};

        let modes = [
            (InterruptMode::None, 0b000),
            (InterruptMode::IntPin, 0b001),
            (InterruptMode::IntPinLatched, 0b010),
            (InterruptMode::Scl, 0b011),
            (InterruptMode::SclLatched, 0b100),
        ];

        for (mode, expected_bits) in modes {
            let address = 0x22u8;
            let mut expectations = init_expectations(address);

            let expected_byte = expected_bits << 2; // INT_MODE is bits [4:2]
            expectations.push(I2cTransaction::write(address, vec![0x08, expected_byte]));

            let i2c = I2cMock::new(&expectations);
            let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
            let config = ConfigBuilder::new().build().unwrap();
            let mut sensor = sensor.init(&config).unwrap();

            sensor
                .set_interrupt(&InterruptConfig {
                    mode,
                    on_conversion_complete: false,
                    on_threshold_crossing: false,
                    pin_behavior: IntPinBehavior::Latched,
                    mask: false,
                })
                .unwrap();

            sensor.into_inner().done();
        }
    }

    // -- Phase 3: Gain/Offset Calibration -----------------------------------

    #[test]
    fn set_calibration_writes_correct_registers() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // MAG_GAIN_CONFIG (0x09)
        expectations.push(I2cTransaction::write(address, vec![0x09, 0x80]));
        // MAG_OFFSET_CONFIG_1 (0x0A) — NOT 0x0B!
        expectations.push(I2cTransaction::write(address, vec![0x0A, 0xF0])); // -16 as u8
        // MAG_OFFSET_CONFIG_2 (0x0B) — correct register, avoids Arduino bug
        expectations.push(I2cTransaction::write(address, vec![0x0B, 0x10])); // +16
        // SENSOR_CONFIG_2 read-modify-write: gain_channel=Second(1) in bit [4]
        expectations.push(I2cTransaction::write_read(address, vec![0x03], vec![0x03]));
        expectations.push(I2cTransaction::write(address, vec![0x03, 0x13])); // 0x03 | (1 << 4) = 0x13

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        use crate::config::{CalibrationConfig, GainChannel};
        sensor
            .set_calibration(&CalibrationConfig {
                gain: 0x80,
                offset_1: -16,
                offset_2: 16,
                gain_channel: GainChannel::Second,
            })
            .unwrap();

        sensor.into_inner().done();
    }

    #[test]
    fn set_calibration_offset2_uses_register_0x0b_not_0x0a() {
        // Explicit regression test for the SparkFun Arduino copy-paste bug.
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        expectations.push(I2cTransaction::write(address, vec![0x09, 0x00])); // gain
        expectations.push(I2cTransaction::write(address, vec![0x0A, 0x00])); // offset_1 → 0x0A
        expectations.push(I2cTransaction::write(address, vec![0x0B, 0xFF])); // offset_2 → 0x0B (NOT 0x0A)
        expectations.push(I2cTransaction::write_read(address, vec![0x03], vec![0x03]));
        expectations.push(I2cTransaction::write(address, vec![0x03, 0x03])); // gain_channel=First, preserve bits

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        use crate::config::{CalibrationConfig, GainChannel};
        sensor
            .set_calibration(&CalibrationConfig {
                gain: 0,
                offset_1: 0,
                offset_2: -1, // 0xFF as u8
                gain_channel: GainChannel::First,
            })
            .unwrap();

        sensor.into_inner().done();
    }

    // -- I2C Address Change -------------------------------------------------

    #[test]
    fn change_address_writes_and_verifies() {
        let old_addr = 0x22u8;
        let new_addr = 0x30u8;
        let mut expectations = init_expectations(old_addr);

        // Write I2C_ADDRESS register (0x0C) at OLD address:
        // bits [7:1] = new_addr, bit [0] = 1 (update enable)
        let reg_val = (new_addr << 1) | 0x01;
        expectations.push(I2cTransaction::write(old_addr, vec![0x0C, reg_val]));

        // Verify at NEW address: read manufacturer ID
        expectations.push(I2cTransaction::write_read(new_addr, vec![0x0E], vec![0x49]));
        expectations.push(I2cTransaction::write_read(new_addr, vec![0x0F], vec![0x54]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        sensor.change_address(new_addr).unwrap();

        sensor.into_inner().done();
    }

    #[test]
    fn change_address_verification_failure_returns_error() {
        let old_addr = 0x22u8;
        let new_addr = 0x30u8;
        let mut expectations = init_expectations(old_addr);

        // Write succeeds at old address
        let reg_val = (new_addr << 1) | 0x01;
        expectations.push(I2cTransaction::write(old_addr, vec![0x0C, reg_val]));

        // Verify at new address: returns wrong manufacturer ID
        expectations.push(I2cTransaction::write_read(new_addr, vec![0x0E], vec![0x00]));
        expectations.push(I2cTransaction::write_read(new_addr, vec![0x0F], vec![0x00]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let err = sensor.change_address(new_addr).unwrap_err();
        assert_eq!(
            err,
            Error::AddressChangeFailed {
                new_address: new_addr
            }
        );

        sensor.into_inner().done();
    }
}
