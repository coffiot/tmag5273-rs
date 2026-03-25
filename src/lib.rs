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
//! # Features
//!
//! - `crc` — Enables CRC-8 validation on I2C reads (disables burst reads)
//! - `defmt` — Enables `defmt::Format` derives on all public types
//!
//! # Quick Start
//!
//! ```no_run
//! # use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction};
//! # let i2c = I2cMock::new(&[]);
//! use tmag5273::{Tmag5273, DeviceVariant, ConfigBuilder};
//!
//! let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
//! let config = ConfigBuilder::new().build().unwrap();
//! // let mut sensor = sensor.init(&config).unwrap();
//! // let reading = sensor.read_magnetic().unwrap();
//! ```
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/1.0

// Rust guideline compliant 2026-03-24
#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![cfg_attr(docsrs, feature(doc_auto_cfg))]

mod config;
mod device;
mod error;
mod register;
mod types;

pub use config::{Config, ConfigBuilder};
pub use device::{Configured, Tmag5273, Unconfigured};
pub use error::{Error, InitError};
pub use types::{
    AngleReading, Celsius, ConversionStatus, Degrees, DeviceStatus, DeviceVariant, MagneticReading,
    Micros, MilliTesla, Range, SensorReading, magnitude_to_millitesla,
};

// Re-export config enums for consumer use
pub use config::{
    AngleCalculation, CalibrationConfig, ConversionAverage, GainChannel, I2cReadMode,
    IntPinBehavior, InterruptConfig, InterruptMode, MagneticChannel, OperatingMode, PowerMode,
    SleepDuration, TempCoefficient, ThresholdConfig, ThresholdDirection, TriggerMode,
};
