// Rust guideline compliant 2026-03-24

//! Error types for the TMAG5273 driver.
//!
//! [`Error`] is generic over the I2C bus error type `E`, allowing callers to
//! propagate transport errors through the `?` operator.  [`InitError`] wraps
//! an [`Error`] together with the I2C bus so callers can recover the bus on
//! initialization failure.

use core::fmt;
use embedded_hal::i2c::I2c;

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

    /// Wake-up-and-sleep operating mode requires a non-zero sleep duration.
    #[error("wake-up-and-sleep mode requires a sleep duration")]
    SleepDurationRequired,

    /// I2C address change could not be verified at the new address.
    ///
    /// The device may have already changed its address — the caller must
    /// decide whether to power-cycle or retry at the new address.
    #[error("address change to 0x{new_address:02X} could not be verified")]
    AddressChangeFailed {
        /// The new address that was written to the device.
        new_address: u8,
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

/// Initialization failure that returns the I2C bus to the caller.
///
/// When [`Tmag5273::init`](crate::Tmag5273) fails, the bus is not consumed.
/// `InitError` bundles the [`Error`] with the original `I2C` peripheral so
/// the caller can retry or use the bus for another device.
pub struct InitError<I2C: I2c> {
    /// The error that caused initialization to fail.
    pub error: Error<I2C::Error>,
    /// The I2C bus, returned so the caller can reuse it.
    pub i2c: I2C,
}

// Manual Debug impl — `I2C` itself may not implement `Debug`.
impl<I2C: I2c> fmt::Debug for InitError<I2C>
where
    I2C::Error: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("InitError")
            .field("error", &self.error)
            .field("i2c", &"<I2C bus>")
            .finish()
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
    fn variant_sleep_duration_required() {
        let err: Error<FakeI2cError> = Error::SleepDurationRequired;
        assert_eq!(err, Error::SleepDurationRequired);
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
}
