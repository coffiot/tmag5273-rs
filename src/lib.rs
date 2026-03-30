//! Platform-agnostic `no_std` driver for the TI TMAG5273 3-axis Hall-effect sensor.
//!
//! This crate provides a type-safe, zero-allocation driver for the TMAG5273 family
//! of linear 3D Hall-effect sensors over I2C, using [`embedded-hal`] 1.0 traits.
//!
//! # Supported Variants
//!
//! All eight TMAG5273 variants are supported (A1, A2, B1, B2, C1, C2, D1, D2),
//! differing in default I2C address and magnetic field sensitivity range.
//!
//! | Variant | Address | XY range (low / high) | Z range (low / high) |
//! |---------|---------|----------------------|---------------------|
//! | A1, B1, C1, D1 | 0x35, 0x22, 0x78, 0x44 | ±40 / ±80 mT | ±40 / ±80 mT |
//! | A2, B2, C2, D2 | 0x35, 0x22, 0x78, 0x44 | ±133 / ±266 mT | ±133 / ±266 mT |
//!
//! # Quick Start
//!
//! Three ways to construct the driver (simplest first):
//!
//! ```no_run
//! # use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction};
//! # let i2c = I2cMock::new(&[]);
//! use tmag5273::{Tmag5273, DeviceVariant, ConfigBuilder};
//!
//! // Option 1: Plug-and-play — scans all known addresses, auto-detects variant
//! // let sensor = Tmag5273::scan(i2c).expect("no TMAG5273 found");
//!
//! // Option 2: Known address, auto-detect variant
//! // let sensor = Tmag5273::detect(i2c, 0x22).expect("no sensor at 0x22");
//!
//! // Option 3: Fully explicit — no I2C activity until init()
//! let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
//!
//! // Configure and initialize
//! let config = ConfigBuilder::new().build().unwrap();
//! // let mut sensor = sensor.init(&config).unwrap();
//!
//! // Read measurements
//! // let mag = sensor.read_magnetic().unwrap();
//! // let temp = sensor.read_temperature().unwrap();
//! // let all = sensor.read_all().unwrap(); // coherent temp + mag in one burst
//! ```
//!
//! # Features
//!
//! - `crc` — Enables CRC-8 validation on I2C reads (disables burst reads)
//! - `defmt` — Enables `defmt::Format` derives on all public types
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/1.0

#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![cfg_attr(docsrs, feature(doc_auto_cfg))]

/// Gated `defmt` logging macros.
///
/// These wrap `defmt::trace!` / `debug!` / `warn!` / `error!` behind
/// `#[cfg(all(feature = "defmt", not(test)))]`. The `not(test)` gate is
/// needed because `defmt` requires a global logger (e.g., `defmt-rtt`)
/// at link time, which only exists on real hardware — not in host-side
/// `cargo test`.
macro_rules! defmt_trace {
    ($($arg:tt)*) => {
        #[cfg(all(feature = "defmt", not(test)))]
        defmt::trace!($($arg)*);
    };
}
macro_rules! defmt_debug {
    ($($arg:tt)*) => {
        #[cfg(all(feature = "defmt", not(test)))]
        defmt::debug!($($arg)*);
    };
}
macro_rules! defmt_warn {
    ($($arg:tt)*) => {
        #[cfg(all(feature = "defmt", not(test)))]
        defmt::warn!($($arg)*);
    };
}
macro_rules! defmt_error {
    ($($arg:tt)*) => {
        #[cfg(all(feature = "defmt", not(test)))]
        defmt::error!($($arg)*);
    };
}

/// Generates a `TryFrom<u8>` impl for a `#[repr(u8)]` enum.
///
/// Each variant is mapped from its discriminant value. Unknown values return
/// `Err(value)` and emit a `defmt::warn!` when the `defmt` feature is active.
macro_rules! impl_try_from_u8 {
    ($enum:ty { $($val:expr => $variant:ident),+ $(,)? }) => {
        impl TryFrom<u8> for $enum {
            type Error = u8;

            fn try_from(val: u8) -> Result<Self, u8> {
                match val {
                    $($val => Ok(Self::$variant),)+
                    other => {
                        defmt_warn!("invalid register value: 0x{:02X}", other);
                        Err(other)
                    }
                }
            }
        }
    };
}

mod config;
mod device;
mod error;
mod register;
mod types;

pub use config::{
    CalibrationConfig, Config, ConfigBuilder, InterruptConfig, ThresholdConfig, ThresholdHysteresis,
};
pub use device::{Configured, Tmag5273, Unconfigured};
pub use error::{ConfigError, Error, InitError};
pub use register::{X_THR_CONFIG, Y_THR_CONFIG, Z_THR_CONFIG};
pub use types::{
    AngleCalculation, AngleReading, Celsius, ConversionAverage, ConversionStatus, Degrees,
    DeviceStatus, DeviceVariant, Diagnostics, GainChannel, I2cReadMode, IntPinBehavior,
    InterruptMode, MagneticChannel, MagneticReading, Micros, MicrosRange, MilliTesla, NoDelay,
    OperatingMode, PowerMode, Range, SensorReading, SleepDuration, TempCoefficient,
    ThresholdCrossingCount, ThresholdDirection, TriggerMode, WAKE_DELAY,
};
