//! Error types for the TMAG5273 driver.
//!
//! Three error types, each for a different phase:
//!
//! - [`ConfigError`] — returned by [`ConfigBuilder::build()`](crate::ConfigBuilder::build)
//!   when configuration validation fails. Not generic — no I2C involved.
//! - [`Error<E>`] — returned by all driver methods after initialization.
//!   Generic over the I2C bus error type `E`; supports the `?` operator
//!   via a blanket `From<E>` impl.
//! - [`InitError`] — returned by [`Tmag5273::init()`](crate::Tmag5273::init).
//!   Wraps an [`Error`] together with the I2C bus and delay backend so the
//!   caller can recover the peripherals on initialization failure.

use core::fmt;
use embedded_hal::i2c::I2c;

use crate::DeviceStatus;
use crate::types::{Micros, NoDelay};

// ---------------------------------------------------------------------------
// Error
// ---------------------------------------------------------------------------

/// Errors produced by the TMAG5273 driver.
///
/// Generic over `E`, the I2C bus error type.
///
/// # Examples
///
/// ```
/// use tmag5273::Error;
///
/// let err: Error<u8> = Error::I2c(42);
/// assert_eq!(err, Error::I2c(42));
/// ```
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// An I2C bus error occurred during communication with the sensor.
    #[error("I2C bus error: {0:?}")]
    I2c(E),

    /// The manufacturer ID read from the device does not match the expected
    /// value (`0x5449`).
    #[error("invalid manufacturer ID: 0x{0:04X} (expected 0x5449)")]
    InvalidManufacturerId(u16),

    /// The device version register does not match the expected value for the
    /// selected [`DeviceVariant`](crate::DeviceVariant).
    #[error("device version mismatch: expected 0x{expected:02X}, got 0x{got:02X}")]
    VersionMismatch {
        /// The version byte the driver expected.
        expected: u8,
        /// The version byte actually read from the device.
        got: u8,
    },

    /// CRC validation failed on an I2C read.
    #[error("CRC mismatch: device sent 0x{expected:02X}, computed 0x{computed:02X}")]
    CrcMismatch {
        /// The CRC byte received from the device.
        expected: u8,
        /// The CRC computed locally over the payload.
        computed: u8,
    },

    /// The conversion-complete flag was not set within the configured poll
    /// limit.
    #[error("conversion did not complete within the poll limit")]
    ConversionTimeout,

    /// Angle calculation requires at least two magnetic channels to be
    /// enabled.
    #[error("angle calculation requires at least two magnetic channels")]
    AngleRequiresTwoChannels,

    /// I2C address change could not be verified at the new address.
    ///
    /// The device may have already changed its address — the caller should
    /// try communicating at `new_address` first, then fall back to
    /// `old_address`, or power-cycle the device to restore the factory
    /// default.
    #[error("address change from 0x{old_address:02X} to 0x{new_address:02X} could not be verified")]
    AddressChangeFailed {
        /// The address the device was using before the change attempt.
        old_address: u8,
        /// The new address that was written to the device.
        new_address: u8,
    },

    /// A register returned a value that does not map to any known enum variant.
    #[error("register 0x{register:02X} returned invalid value 0x{value:02X}")]
    InvalidRegisterValue {
        /// The register address that was read.
        register: u8,
        /// The raw value that could not be decoded.
        value: u8,
    },

    /// High-level read methods require Standard I2C read mode.
    #[error("read methods require Standard I2C mode, current mode is 0x{mode:02X}")]
    NonStandardReadMode {
        /// The current I2C read mode that is incompatible.
        mode: u8,
    },

    /// `read_temperature()` was called but the temperature channel is disabled
    /// in the configuration.
    ///
    /// Enable the channel via
    /// [`ConfigBuilder::temperature_enabled(true)`](crate::ConfigBuilder) and
    /// re-initialize, or use [`Tmag5273::read_all`](crate::Tmag5273) which
    /// returns `None` for the temperature field when the channel is disabled.
    #[error("temperature channel is disabled; enable it in ConfigBuilder")]
    TemperatureDisabled,

    /// Enabling sensor CRC requires the `crc` Cargo feature.
    ///
    /// The driver was compiled without CRC support, so enabling the sensor's
    /// hardware CRC would cause it to append CRC bytes that the driver cannot
    /// validate, silently corrupting all subsequent reads.
    ///
    /// Either enable the `crc` feature in `Cargo.toml` and recompile, or only
    /// call [`set_crc_enabled(false)`](crate::Tmag5273::set_crc_enabled).
    #[error("enabling sensor CRC requires the `crc` Cargo feature")]
    CrcFeatureRequired,

    /// A sensor diagnostic failure was detected during a measurement read.
    ///
    /// The device's `CONV_STATUS.DIAG_FAIL` flag was set, indicating an
    /// internal fault (VCC undervoltage, oscillator error, OTP CRC error,
    /// or INT pin error). The [`DeviceStatus`](DeviceStatus) carries
    /// the specific fault flags.
    ///
    /// This error is only returned when
    /// [`diag_enabled(true)`](crate::ConfigBuilder::diag_enabled)
    /// is set in the configuration. Use
    /// [`ignore_diag_fail(true)`](crate::ConfigBuilder::ignore_diag_fail) to
    /// suppress this error in noisy environments.
    #[error("sensor diagnostic failure: {0:?}")]
    DiagnosticFailure(DeviceStatus),
}

// ---------------------------------------------------------------------------
// ConfigError
// ---------------------------------------------------------------------------

/// Errors that can occur during [`ConfigBuilder::build()`](crate::ConfigBuilder::build).
///
/// Unlike [`Error<E>`], this type is *not* generic — configuration validation
/// never touches the I2C bus, so there is no transport error to propagate.
///
/// # Examples
///
/// ```
/// use tmag5273::ConfigError;
///
/// let err = ConfigError::AngleRequiresTwoChannels;
/// assert_eq!(err, ConfigError::AngleRequiresTwoChannels);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    /// Angle calculation requires at least two magnetic channels to be
    /// enabled.
    #[error("angle calculation requires at least two magnetic channels")]
    AngleRequiresTwoChannels,

    /// Sleep duration is shorter than the conversion time for the configured
    /// averaging and channels.
    #[error("sleep duration ({sleep} µs) < conversion time ({conversion} µs)")]
    SleepShorterThanConversion {
        /// Sleep duration.
        sleep: Micros,
        /// Required conversion time.
        conversion: Micros,
    },
}

// ---------------------------------------------------------------------------
// From<E> — enables `?` on raw I2C calls
// ---------------------------------------------------------------------------

/// Blanket conversion from any I2C bus error into [`Error::I2c`].
///
/// This enables the `?` operator on raw `I2C::Error` values inside driver
/// methods. The trade-off is that Rust's coherence rules prevent adding
/// additional `impl<E> From<OtherError> for Error<E>` conversions in the
/// future — this is intentional, as all non-I2C error variants are
/// constructed explicitly by the driver.
impl<E> From<E> for Error<E> {
    #[inline]
    fn from(e: E) -> Self {
        Error::I2c(e)
    }
}

// ---------------------------------------------------------------------------
// InitError
// ---------------------------------------------------------------------------

/// Initialization failure that returns the I2C bus and delay backend to the caller.
///
/// When [`Tmag5273::init`](crate::Tmag5273) fails, the bus is not consumed.
/// `InitError` bundles the [`Error`] with the original `I2C` peripheral and
/// delay backend so the caller can retry or use them for another device.
#[non_exhaustive]
pub struct InitError<I2C: I2c, D = NoDelay> {
    /// The error that caused initialization to fail.
    pub error: Error<I2C::Error>,
    /// The I2C bus, returned so the caller can reuse it.
    pub i2c: I2C,
    /// The delay backend, returned so the caller can reuse it.
    pub delay: D,
}

// Manual Debug impl — `I2C` and `D` themselves may not implement `Debug`.
impl<I2C: I2c, D> fmt::Debug for InitError<I2C, D>
where
    I2C::Error: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("InitError")
            .field("error", &self.error)
            .field("i2c", &"<I2C bus>")
            .finish_non_exhaustive()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// A minimal I2C error type for testing.
    #[derive(Debug, Clone, Copy, PartialEq)]
    struct FakeI2cError(u8);

    #[test]
    fn from_i2c_error() {
        let raw = FakeI2cError(7);
        let err: Error<FakeI2cError> = Error::from(raw);
        assert_eq!(err, Error::I2c(FakeI2cError(7)));
    }

    #[test]
    fn from_i2c_error_via_into() {
        let raw = FakeI2cError(99);
        let err: Error<FakeI2cError> = raw.into();
        assert_eq!(err, Error::I2c(FakeI2cError(99)));
    }

    #[test]
    fn variant_invalid_manufacturer_id() {
        let err: Error<FakeI2cError> = Error::InvalidManufacturerId(0x0000);
        assert_eq!(err, Error::InvalidManufacturerId(0x0000));
    }

    #[test]
    fn variant_version_mismatch() {
        let err: Error<FakeI2cError> = Error::VersionMismatch {
            expected: 0x01,
            got: 0x02,
        };
        assert_eq!(
            err,
            Error::VersionMismatch {
                expected: 0x01,
                got: 0x02,
            }
        );
    }

    #[test]
    fn variant_crc_mismatch() {
        let err: Error<FakeI2cError> = Error::CrcMismatch {
            expected: 0xAB,
            computed: 0xCD,
        };
        assert_eq!(
            err,
            Error::CrcMismatch {
                expected: 0xAB,
                computed: 0xCD,
            }
        );
    }

    #[test]
    fn variant_conversion_timeout() {
        let err: Error<FakeI2cError> = Error::ConversionTimeout;
        assert_eq!(err, Error::ConversionTimeout);
    }

    #[test]
    fn variant_angle_requires_two_channels() {
        let err: Error<FakeI2cError> = Error::AngleRequiresTwoChannels;
        assert_eq!(err, Error::AngleRequiresTwoChannels);
    }

    #[test]
    fn variant_address_change_failed() {
        let err: Error<FakeI2cError> = Error::AddressChangeFailed {
            old_address: 0x22,
            new_address: 0x30,
        };
        assert_eq!(
            err,
            Error::AddressChangeFailed {
                old_address: 0x22,
                new_address: 0x30,
            }
        );
    }

    #[test]
    fn variant_invalid_register_value() {
        let err: Error<FakeI2cError> = Error::InvalidRegisterValue {
            register: 0x00,
            value: 0xFF,
        };
        assert_eq!(
            err,
            Error::InvalidRegisterValue {
                register: 0x00,
                value: 0xFF,
            }
        );
    }

    #[test]
    fn variant_non_standard_read_mode() {
        let err: Error<FakeI2cError> = Error::NonStandardReadMode { mode: 0x01 };
        assert_eq!(err, Error::NonStandardReadMode { mode: 0x01 });
    }

    #[test]
    fn variant_temperature_disabled() {
        let err: Error<FakeI2cError> = Error::TemperatureDisabled;
        assert_eq!(err, Error::TemperatureDisabled);
    }

    #[test]
    fn variant_crc_feature_required() {
        let err: Error<FakeI2cError> = Error::CrcFeatureRequired;
        assert_eq!(err, Error::CrcFeatureRequired);
    }

    #[test]
    fn error_is_copy() {
        let err: Error<FakeI2cError> = Error::ConversionTimeout;
        let copy = err;
        assert_eq!(err, copy);
    }

    #[test]
    fn error_clone() {
        let err: Error<FakeI2cError> = Error::I2c(FakeI2cError(1));
        #[expect(clippy::clone_on_copy, reason = "explicitly testing Clone impl")]
        let cloned = err.clone();
        assert_eq!(err, cloned);
    }

    // -- ConfigError tests --------------------------------------------------

    #[test]
    fn config_error_angle_requires_two_channels() {
        let err = ConfigError::AngleRequiresTwoChannels;
        assert_eq!(err, ConfigError::AngleRequiresTwoChannels);
    }

    #[test]
    fn config_error_sleep_shorter_than_conversion() {
        let err = ConfigError::SleepShorterThanConversion {
            sleep: Micros(1_000),
            conversion: Micros(3_225),
        };
        assert_eq!(
            err,
            ConfigError::SleepShorterThanConversion {
                sleep: Micros(1_000),
                conversion: Micros(3_225),
            }
        );
    }

    #[test]
    fn config_error_is_copy() {
        let err = ConfigError::AngleRequiresTwoChannels;
        let copy = err;
        assert_eq!(err, copy);
    }
}
