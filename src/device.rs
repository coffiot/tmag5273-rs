// Rust guideline compliant 2026-03-25

//! Core driver implementation for the TMAG5273 sensor.
//!
//! Uses a typestate pattern with [`Unconfigured`] and [`Configured`] markers
//! to prevent reading before initialization at compile time.

use core::fmt;
use core::marker::PhantomData;

use embedded_hal::i2c::I2c;

use crate::config::{
    AngleCalculation, Config, ConversionAverage, GainChannel, I2cReadMode, InterruptMode,
    MagneticChannel, OperatingMode, PowerMode, SleepDuration, TempCoefficient, ThresholdDirection,
    TriggerMode,
};
use crate::error::{Error, InitError};
use crate::register;
use crate::types::{
    AngleReading, Celsius, ConversionStatus, DeviceStatus, DeviceVariant, MagneticReading, Range,
};

// ---------------------------------------------------------------------------
// Typestate markers
// ---------------------------------------------------------------------------

/// Marker type: sensor has not been initialized.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Unconfigured;

/// Marker type: sensor has been initialized and is ready for measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
    angle_enabled: bool,
    max_polls: u16,
    /// Current I2C read mode — high-level reads guard against non-Standard
    /// mode to prevent silent data corruption.
    read_mode: I2cReadMode,
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

// Compile-time size assertion — should be ~15 bytes on ESP32 (I2C handle ~4 bytes + fields).
const _: () = assert!(
    core::mem::size_of::<Tmag5273<[u8; 4], Configured>>() <= 20,
    "Tmag5273 struct exceeds 20-byte budget (excluding I2C handle)"
);

// ---------------------------------------------------------------------------
// Unconfigured — construction + initialization
// ---------------------------------------------------------------------------

impl<I2C: I2c> Tmag5273<I2C, Unconfigured> {
    /// Create a new driver instance without performing any I2C operations.
    ///
    /// The I2C address is derived from the [`DeviceVariant`].
    pub fn new(i2c: I2C, variant: DeviceVariant) -> Self {
        Self::new_with_address(i2c, variant, variant.default_address())
    }

    /// Create a new driver instance with a custom I2C address.
    ///
    /// Use this when the sensor's address has been changed from the factory
    /// default.
    ///
    /// # Panics
    ///
    /// Panics in debug mode if `address > 0x7F` (I2C addresses are 7-bit).
    pub fn new_with_address(i2c: I2C, variant: DeviceVariant, address: u8) -> Self {
        debug_assert!(
            address <= 0x7F,
            "I2C address must be 7-bit (0x00-0x7F), got 0x{address:02X}"
        );
        Self {
            address,
            i2c,
            variant,
            xy_range: Range::Low,
            z_range: Range::Low,
            channels: MagneticChannel::Off,
            angle_enabled: false,
            max_polls: 10,
            read_mode: I2cReadMode::Standard,
            _state: PhantomData,
        }
    }

    /// Initialize the sensor: verify identity, apply configuration, and
    /// transition to [`Configured`] state.
    ///
    /// Wakes the device if it was left in sleep mode by previous firmware.
    /// After the wake-up write, two dummy `DEVICE_ID` reads provide ~100 us
    /// of I2C bus activity as a timing substitute for the 80 us wake-up
    /// delay specified in the TI datasheet (Linux kernel uses
    /// `usleep_range(80, 200)`).
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
            angle_enabled: !matches!(config.angle_calculation, AngleCalculation::None),
            max_polls: config.max_polls,
            read_mode: I2cReadMode::Standard,
            _state: PhantomData,
        })
    }

    /// Inner init logic — returns `Error` for easier `?` usage, wrapped by
    /// `init()` into `InitError`.
    fn init_inner(&mut self, config: &Config) -> Result<(), Error<I2C::Error>> {
        // 0a. Wake the device if it was left in SLEEP mode by previous firmware.
        // Any I2C activity triggers wake-up; an empty write is the cheapest.
        // Intentionally ignore errors — the device may NACK while waking.
        let _ = self.i2c.write(self.address, &[]);

        // The device needs ~80us to wake. Without a Delay trait, perform
        // dummy register reads as a timing substitute — each I2C read at
        // 400kHz takes ~50us minimum (start + addr + reg + restart + data + stop).
        // Two reads ≈ 100us, sufficient for the 80us wake-up requirement.
        let _ = self.read_register(register::DEVICE_ID);
        let _ = self.read_register(register::DEVICE_ID);

        // 0b. Disable interrupts before applying new configuration to prevent
        // spurious INT pin activity from stale settings.
        self.write_register(register::INT_CONFIG_1, 0x00)?;

        // 1. Read and verify manufacturer ID (registers 0x0E + 0x0F) = 0x5449.
        // Single 2-byte burst read starting at MANUFACTURER_ID_LSB — the two
        // ID registers are consecutive, so this halves the I2C transactions.
        let mut id_buf = [0u8; 2];
        self.i2c
            .write_read(self.address, &[register::MANUFACTURER_ID_LSB], &mut id_buf)?;
        let manufacturer_id = u16::from(id_buf[1]) << 8 | u16::from(id_buf[0]);

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

        // 3. Write configuration registers.
        // Registers 0x00-0x03 (DEVICE_CONFIG_1 through SENSOR_CONFIG_2) are
        // consecutive — write as a single 5-byte I2C transaction (address +
        // 4 data bytes) to halve the bus overhead.
        let writes = config.to_register_writes();
        self.i2c.write(
            self.address,
            &[
                register::DEVICE_CONFIG_1,
                writes[0].1,
                writes[1].1,
                writes[2].1,
                writes[3].1,
            ],
        )?;
        // T_CONFIG (0x07) is not contiguous with 0x03 — separate write.
        self.write_register(writes[4].0, writes[4].1)?;

        // 4. Read device status — hardware faults are not driver errors.
        // The caller can inspect status via `read_status()` after init.
        let _status_byte = self.read_register(register::DEVICE_STATUS)?;

        // 4b. Clear stale fault flags so subsequent read_status() returns fresh state.
        self.write_register(register::DEVICE_STATUS, 0x0F)?;

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
    /// **CRC limitation:** Even when the `crc` feature is enabled, this
    /// method does **not** validate CRC on the magnetic burst read. The
    /// TMAG5273 CRC scheme operates on 2-byte register-pair reads, not
    /// 6-byte bursts. CRC validation covers temperature and angle reads
    /// only (via [`read_register_pair`](Self::read_temperature)).
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn read_magnetic(&mut self) -> Result<MagneticReading, Error<I2C::Error>> {
        if self.read_mode != I2cReadMode::Standard {
            return Err(Error::NonStandardReadMode {
                mode: self.read_mode as u8,
            });
        }

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
        if self.read_mode != I2cReadMode::Standard {
            return Err(Error::NonStandardReadMode {
                mode: self.read_mode as u8,
            });
        }

        let buf = self.read_register_pair(register::T_MSB_RESULT)?;
        let raw = i16::from_be_bytes(buf);
        Ok(crate::types::raw_to_celsius(raw))
    }

    /// Read the CORDIC angle calculation result.
    ///
    /// Requires angle calculation to be enabled in the configuration
    /// (any [`AngleCalculation`] variant other than `None`).
    ///
    /// # Errors
    ///
    /// Returns [`Error::AngleRequiresTwoChannels`] if angle calculation was
    /// not enabled during initialization.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn read_angle(&mut self) -> Result<AngleReading, Error<I2C::Error>> {
        if self.read_mode != I2cReadMode::Standard {
            return Err(Error::NonStandardReadMode {
                mode: self.read_mode as u8,
            });
        }

        if !self.angle_enabled {
            return Err(Error::AngleRequiresTwoChannels);
        }

        // Without CRC, read all 3 registers (ANGLE_MSB, ANGLE_LSB,
        // MAGNITUDE) in a single burst — they are consecutive.
        #[cfg(not(feature = "crc"))]
        {
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

        // With CRC, use the existing 2-call approach — `read_register_pair`
        // validates the CRC over the angle register pair, and the magnitude
        // is a separate single-byte read.
        #[cfg(feature = "crc")]
        {
            let angle_buf = self.read_register_pair(register::ANGLE_RESULT_MSB)?;
            let raw_angle = u16::from_be_bytes(angle_buf);
            let magnitude = self.read_register(register::MAGNITUDE_RESULT)?;
            Ok(AngleReading {
                angle: crate::types::raw_to_degrees(raw_angle),
                magnitude,
            })
        }
    }

    /// Enable or disable CRC-8 on the sensor's I2C interface.
    ///
    /// This controls the sensor's hardware CRC generation (register
    /// `DEVICE_CONFIG_1` bit 7). It does NOT control driver-side CRC
    /// validation — that is controlled by the `crc` Cargo feature.
    ///
    /// # CRC Layer Interaction
    ///
    /// | `crc` feature | Sensor CRC | Behavior |
    /// |---|---|---|
    /// | OFF | OFF | Normal reads (default) |
    /// | OFF | ON | **Reads fail** — sensor sends extra CRC byte |
    /// | ON | OFF | **CRC mismatch** — driver expects CRC sensor omits |
    /// | ON | ON | Full CRC protection (correct) |
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn set_crc_enabled(&mut self, enabled: bool) -> Result<(), Error<I2C::Error>> {
        let current = self.read_register(register::DEVICE_CONFIG_1)?;
        let updated = register::insert(
            current,
            register::CRC_EN_MASK,
            register::CRC_EN_SHIFT,
            u8::from(enabled),
        );
        self.write_register(register::DEVICE_CONFIG_1, updated)?;
        Ok(())
    }

    /// Read the sensor's CRC mode from register `DEVICE_CONFIG_1` bit 7.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_crc_enabled(&mut self) -> Result<bool, Error<I2C::Error>> {
        let raw = self.read_register(register::DEVICE_CONFIG_1)?;
        Ok(register::extract(raw, register::CRC_EN_MASK, register::CRC_EN_SHIFT) != 0)
    }

    /// Set the I2C read mode on the sensor.
    ///
    /// Controls how the sensor returns data: standard 3-byte reads, 1-byte
    /// 16-bit, or 1-byte 8-bit. High-level read methods
    /// ([`read_magnetic`](Self::read_magnetic),
    /// [`read_temperature`](Self::read_temperature),
    /// [`read_angle`](Self::read_angle)) require
    /// [`Standard`](I2cReadMode::Standard) mode and will return
    /// [`Error::NonStandardReadMode`] if another mode is active.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn set_read_mode(&mut self, mode: I2cReadMode) -> Result<(), Error<I2C::Error>> {
        let current = self.read_register(register::DEVICE_CONFIG_1)?;
        let updated = register::insert(
            current,
            register::I2C_RD_MASK,
            register::I2C_RD_SHIFT,
            mode as u8,
        );
        self.write_register(register::DEVICE_CONFIG_1, updated)?;
        self.read_mode = mode;
        Ok(())
    }

    /// Read the sensor's current I2C read mode from `DEVICE_CONFIG_1`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_read_mode(&mut self) -> Result<I2cReadMode, Error<I2C::Error>> {
        let raw = self.read_register(register::DEVICE_CONFIG_1)?;
        let val = register::extract(raw, register::I2C_RD_MASK, register::I2C_RD_SHIFT);
        I2cReadMode::try_from(val).map_err(|_| Error::InvalidRegisterValue {
            register: register::DEVICE_CONFIG_1,
            value: val,
        })
    }

    /// Read the current state of the INT pin from the `DEVICE_STATUS` register.
    ///
    /// Returns `true` when the interrupt pin is asserted (active).
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn is_interrupt_active(&mut self) -> Result<bool, Error<I2C::Error>> {
        Ok(self.read_status()?.int_pin_active)
    }

    /// Switch the operating mode at runtime.
    ///
    /// When transitioning to an active mode the device may need ~80 us to
    /// wake from sleep (TI datasheet; Linux kernel uses
    /// `usleep_range(80, 200)`). Two dummy `DEVICE_ID` reads provide the
    /// required delay without a `Delay` trait.
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

        // If switching to an active mode, the device may need wake-up time
        // (80us per datasheet). Dummy reads provide the delay without
        // requiring a Delay trait.
        if !matches!(mode, OperatingMode::Sleep) {
            let _ = self.read_register(register::DEVICE_ID);
            let _ = self.read_register(register::DEVICE_ID);
        }

        Ok(())
    }

    /// Trigger a single conversion in standby or trigger mode.
    ///
    /// Uses the conversion start bit (0x80) OR-ed into a register read,
    /// which triggers a conversion and returns the register value in a
    /// single I2C transaction. This is more efficient than a separate
    /// read-modify-write cycle.
    ///
    /// After calling this, use [`wait_for_conversion`](Self::wait_for_conversion)
    /// to poll for completion.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn trigger_conversion(&mut self) -> Result<(), Error<I2C::Error>> {
        // Reading any register with bit 7 set triggers a conversion.
        // We read CONV_STATUS to also get the current status.
        let _ = self.read_register(register::CONV_STATUS | register::CONVERSION_START_BIT)?;
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

    /// Check sensor diagnostics by reading `CONV_STATUS` and, if needed,
    /// `DEVICE_STATUS`.
    ///
    /// Reads the conversion status register. If the `DIAG_FAIL` flag is set,
    /// reads `DEVICE_STATUS` for specific fault information. Returns the
    /// device status (all flags clear if no diagnostic failure).
    ///
    /// Inspired by the Zephyr RTOS driver's per-sample diagnostic check.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn check_diagnostics(&mut self) -> Result<DeviceStatus, Error<I2C::Error>> {
        let conv_raw = self.read_register(register::CONV_STATUS)?;
        let conv = ConversionStatus::from_reg(conv_raw);
        if conv.diag_fail {
            self.read_status()
        } else {
            Ok(DeviceStatus::default())
        }
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
        // Reject addresses outside the usable 7-bit I2C range.
        // 0x00-0x07 and 0x78-0x7F are reserved by the I2C specification.
        if !(0x08..=0x77).contains(&new_address) {
            return Err(Error::AddressChangeFailed { new_address });
        }

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

    // -----------------------------------------------------------------------
    // Configuration read-back getters
    // -----------------------------------------------------------------------

    /// Read the magnetic temperature compensation coefficient from register
    /// `DEVICE_CONFIG_1`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_temp_coefficient(&mut self) -> Result<TempCoefficient, Error<I2C::Error>> {
        let raw = self.read_register(register::DEVICE_CONFIG_1)?;
        let val = register::extract(raw, register::MAG_TEMPCO_MASK, register::MAG_TEMPCO_SHIFT);
        TempCoefficient::try_from(val).map_err(|_| Error::InvalidRegisterValue {
            register: register::DEVICE_CONFIG_1,
            value: val,
        })
    }

    /// Read the conversion averaging mode from register `DEVICE_CONFIG_1`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_conversion_average(&mut self) -> Result<ConversionAverage, Error<I2C::Error>> {
        let raw = self.read_register(register::DEVICE_CONFIG_1)?;
        let val = register::extract(raw, register::CONV_AVG_MASK, register::CONV_AVG_SHIFT);
        ConversionAverage::try_from(val).map_err(|_| Error::InvalidRegisterValue {
            register: register::DEVICE_CONFIG_1,
            value: val,
        })
    }

    /// Read the current operating mode from register `DEVICE_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_operating_mode(&mut self) -> Result<OperatingMode, Error<I2C::Error>> {
        let raw = self.read_register(register::DEVICE_CONFIG_2)?;
        let val = register::extract(
            raw,
            register::OPERATING_MODE_MASK,
            register::OPERATING_MODE_SHIFT,
        );
        OperatingMode::try_from(val).map_err(|_| Error::InvalidRegisterValue {
            register: register::DEVICE_CONFIG_2,
            value: val,
        })
    }

    /// Read the power mode from register `DEVICE_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_power_mode(&mut self) -> Result<PowerMode, Error<I2C::Error>> {
        let raw = self.read_register(register::DEVICE_CONFIG_2)?;
        let val = register::extract(raw, register::LP_LN_MASK, register::LP_LN_SHIFT);
        PowerMode::try_from(val).map_err(|_| Error::InvalidRegisterValue {
            register: register::DEVICE_CONFIG_2,
            value: val,
        })
    }

    /// Read the trigger mode from register `DEVICE_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_trigger_mode(&mut self) -> Result<TriggerMode, Error<I2C::Error>> {
        let raw = self.read_register(register::DEVICE_CONFIG_2)?;
        let val = register::extract(
            raw,
            register::TRIGGER_MODE_MASK,
            register::TRIGGER_MODE_SHIFT,
        );
        TriggerMode::try_from(val).map_err(|_| Error::InvalidRegisterValue {
            register: register::DEVICE_CONFIG_2,
            value: val,
        })
    }

    /// Read whether the I2C glitch filter is enabled from register
    /// `DEVICE_CONFIG_2`.
    ///
    /// The register bit is inverted: 0 = filter enabled, 1 = filter disabled.
    /// This getter returns `true` when the filter is enabled (register bit == 0).
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_glitch_filter(&mut self) -> Result<bool, Error<I2C::Error>> {
        let raw = self.read_register(register::DEVICE_CONFIG_2)?;
        Ok(register::extract(
            raw,
            register::I2C_GLITCH_FILTER_MASK,
            register::I2C_GLITCH_FILTER_SHIFT,
        ) == 0)
    }

    /// Read the threshold hysteresis value from register `DEVICE_CONFIG_2`.
    ///
    /// Returns the raw 3-bit hysteresis field (0--7).
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_hysteresis(&mut self) -> Result<u8, Error<I2C::Error>> {
        let raw = self.read_register(register::DEVICE_CONFIG_2)?;
        Ok(register::extract(
            raw,
            register::THR_HYST_MASK,
            register::THR_HYST_SHIFT,
        ))
    }

    /// Read the magnetic channel selection from register `SENSOR_CONFIG_1`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_magnetic_channels(&mut self) -> Result<MagneticChannel, Error<I2C::Error>> {
        let raw = self.read_register(register::SENSOR_CONFIG_1)?;
        let val = register::extract(raw, register::MAG_CH_EN_MASK, register::MAG_CH_EN_SHIFT);
        MagneticChannel::try_from(val).map_err(|_| Error::InvalidRegisterValue {
            register: register::SENSOR_CONFIG_1,
            value: val,
        })
    }

    /// Read the sleep duration from register `SENSOR_CONFIG_1`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_sleep_duration(&mut self) -> Result<SleepDuration, Error<I2C::Error>> {
        let raw = self.read_register(register::SENSOR_CONFIG_1)?;
        let val = register::extract(raw, register::SLEEPTIME_MASK, register::SLEEPTIME_SHIFT);
        SleepDuration::try_from(val).map_err(|_| Error::InvalidRegisterValue {
            register: register::SENSOR_CONFIG_1,
            value: val,
        })
    }

    /// Read the angle calculation mode from register `SENSOR_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_angle_calculation(&mut self) -> Result<AngleCalculation, Error<I2C::Error>> {
        let raw = self.read_register(register::SENSOR_CONFIG_2)?;
        let val = register::extract(raw, register::ANGLE_EN_MASK, register::ANGLE_EN_SHIFT);
        AngleCalculation::try_from(val).map_err(|_| Error::InvalidRegisterValue {
            register: register::SENSOR_CONFIG_2,
            value: val,
        })
    }

    /// Read the X/Y axis measurement range from register `SENSOR_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_xy_range(&mut self) -> Result<Range, Error<I2C::Error>> {
        let raw = self.read_register(register::SENSOR_CONFIG_2)?;
        let val = register::extract(raw, register::X_Y_RANGE_MASK, register::X_Y_RANGE_SHIFT);
        Range::try_from(val).map_err(|_| Error::InvalidRegisterValue {
            register: register::SENSOR_CONFIG_2,
            value: val,
        })
    }

    /// Read the Z axis measurement range from register `SENSOR_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_z_range(&mut self) -> Result<Range, Error<I2C::Error>> {
        let raw = self.read_register(register::SENSOR_CONFIG_2)?;
        let val = register::extract(raw, register::Z_RANGE_MASK, register::Z_RANGE_SHIFT);
        Range::try_from(val).map_err(|_| Error::InvalidRegisterValue {
            register: register::SENSOR_CONFIG_2,
            value: val,
        })
    }

    /// Read the threshold crossing direction from register `SENSOR_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_threshold_direction(&mut self) -> Result<ThresholdDirection, Error<I2C::Error>> {
        let raw = self.read_register(register::SENSOR_CONFIG_2)?;
        let val = register::extract(raw, register::MAG_THR_DIR_MASK, register::MAG_THR_DIR_SHIFT);
        ThresholdDirection::try_from(val).map_err(|_| Error::InvalidRegisterValue {
            register: register::SENSOR_CONFIG_2,
            value: val,
        })
    }

    /// Read the gain channel selection from register `SENSOR_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_gain_channel(&mut self) -> Result<GainChannel, Error<I2C::Error>> {
        let raw = self.read_register(register::SENSOR_CONFIG_2)?;
        let val = register::extract(raw, register::MAG_GAIN_CH_MASK, register::MAG_GAIN_CH_SHIFT);
        GainChannel::try_from(val).map_err(|_| Error::InvalidRegisterValue {
            register: register::SENSOR_CONFIG_2,
            value: val,
        })
    }

    /// Read whether the temperature channel is enabled from register
    /// `T_CONFIG`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_temperature_enabled(&mut self) -> Result<bool, Error<I2C::Error>> {
        let raw = self.read_register(register::T_CONFIG)?;
        Ok(register::extract(raw, register::T_CH_EN_MASK, register::T_CH_EN_SHIFT) != 0)
    }

    /// Read the temperature threshold value from register `T_CONFIG`.
    ///
    /// Returns the signed 7-bit threshold occupying the upper bits of
    /// `T_CONFIG`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_temperature_threshold(&mut self) -> Result<i8, Error<I2C::Error>> {
        let raw = self.read_register(register::T_CONFIG)?;
        let val = register::extract(
            raw,
            register::T_THR_CONFIG_MASK,
            register::T_THR_CONFIG_SHIFT,
        );
        Ok(val as i8)
    }

    /// Read the interrupt routing mode from register `INT_CONFIG_1`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_interrupt_mode(&mut self) -> Result<InterruptMode, Error<I2C::Error>> {
        let raw = self.read_register(register::INT_CONFIG_1)?;
        let val = register::extract(raw, register::INT_MODE_MASK, register::INT_MODE_SHIFT);
        InterruptMode::try_from(val).map_err(|_| Error::InvalidRegisterValue {
            register: register::INT_CONFIG_1,
            value: val,
        })
    }

    /// Read whether the interrupt-on-conversion-complete flag is set from
    /// register `INT_CONFIG_1`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_interrupt_on_conversion(&mut self) -> Result<bool, Error<I2C::Error>> {
        let raw = self.read_register(register::INT_CONFIG_1)?;
        Ok(register::extract(raw, register::RSLT_INT_MASK, register::RSLT_INT_SHIFT) != 0)
    }

    /// Read whether the interrupt-on-threshold-crossing flag is set from
    /// register `INT_CONFIG_1`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_interrupt_on_threshold(&mut self) -> Result<bool, Error<I2C::Error>> {
        let raw = self.read_register(register::INT_CONFIG_1)?;
        Ok(register::extract(
            raw,
            register::THRESHOLD_INT_MASK,
            register::THRESHOLD_INT_SHIFT,
        ) != 0)
    }

    /// Read the INT pin assertion behavior from register `INT_CONFIG_1`.
    ///
    /// Returns `true` for pulsed mode, `false` for latched mode.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_interrupt_pin_state(&mut self) -> Result<bool, Error<I2C::Error>> {
        let raw = self.read_register(register::INT_CONFIG_1)?;
        Ok(register::extract(raw, register::INT_STATE_MASK, register::INT_STATE_SHIFT) != 0)
    }

    /// Read whether the interrupt pin output is masked from register
    /// `INT_CONFIG_1`.
    ///
    /// Returns `true` when the INT output is disabled (masked).
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_interrupt_mask(&mut self) -> Result<bool, Error<I2C::Error>> {
        let raw = self.read_register(register::INT_CONFIG_1)?;
        Ok(register::extract(raw, register::MASK_INTB_MASK, register::MASK_INTB_SHIFT) != 0)
    }

    /// Read the X-axis threshold value from register `X_THR_CONFIG`.
    ///
    /// Returns the full register byte interpreted as a signed threshold.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_x_threshold(&mut self) -> Result<i8, Error<I2C::Error>> {
        let raw = self.read_register(register::X_THR_CONFIG)?;
        Ok(raw as i8)
    }

    /// Read the Y-axis threshold value from register `Y_THR_CONFIG`.
    ///
    /// Returns the full register byte interpreted as a signed threshold.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_y_threshold(&mut self) -> Result<i8, Error<I2C::Error>> {
        let raw = self.read_register(register::Y_THR_CONFIG)?;
        Ok(raw as i8)
    }

    /// Read the Z-axis threshold value from register `Z_THR_CONFIG`.
    ///
    /// Returns the full register byte interpreted as a signed threshold.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_z_threshold(&mut self) -> Result<i8, Error<I2C::Error>> {
        let raw = self.read_register(register::Z_THR_CONFIG)?;
        Ok(raw as i8)
    }

    /// Read the magnetic gain adjustment value from register
    /// `MAG_GAIN_CONFIG`.
    ///
    /// Returns the raw 8-bit gain value.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_magnetic_gain(&mut self) -> Result<u8, Error<I2C::Error>> {
        Ok(self.read_register(register::MAG_GAIN_CONFIG)?)
    }

    /// Read the first magnetic offset value from register
    /// `MAG_OFFSET_CONFIG_1`.
    ///
    /// Returns the full register byte interpreted as a signed offset.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_magnetic_offset_1(&mut self) -> Result<i8, Error<I2C::Error>> {
        let raw = self.read_register(register::MAG_OFFSET_CONFIG_1)?;
        Ok(raw as i8)
    }

    /// Read the second magnetic offset value from register
    /// `MAG_OFFSET_CONFIG_2`.
    ///
    /// Returns the full register byte interpreted as a signed offset.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_magnetic_offset_2(&mut self) -> Result<i8, Error<I2C::Error>> {
        let raw = self.read_register(register::MAG_OFFSET_CONFIG_2)?;
        Ok(raw as i8)
    }

    /// Read the conversion status register (`CONV_STATUS`, 0x18).
    ///
    /// Returns a parsed [`ConversionStatus`] containing the rolling set
    /// counter, power-on-reset flag, diagnostic status, and result-ready
    /// flag.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn read_conversion_status(&mut self) -> Result<ConversionStatus, Error<I2C::Error>> {
        let raw = self.read_register(register::CONV_STATUS)?;
        Ok(ConversionStatus::from_reg(raw))
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

    /// Read a register pair (MSB + LSB) and return as `i16` (big-endian).
    ///
    /// When the `crc` feature is enabled, reads 3 bytes (MSB + LSB + CRC)
    /// and validates the CRC-8 checksum. Without the feature, reads 2 bytes.
    #[cfg(not(feature = "crc"))]
    fn read_register_pair(&mut self, reg: u8) -> Result<[u8; 2], Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c.write_read(self.address, &[reg], &mut buf)?;
        Ok(buf)
    }

    /// Read a register pair with CRC-8 validation.
    ///
    /// Reads 3 bytes: MSB, LSB, CRC. Validates the CRC over `[address_W, reg, address_R, MSB, LSB]`
    /// per TI datasheet Section 7.5.3.2.
    #[cfg(feature = "crc")]
    fn read_register_pair(&mut self, reg: u8) -> Result<[u8; 2], Error<I2C::Error>> {
        let mut buf = [0u8; 3]; // MSB + LSB + CRC
        self.i2c.write_read(self.address, &[reg], &mut buf)?;

        // CRC is computed over the full I2C transaction bytes:
        // [device_addr << 1 | W, register_addr, device_addr << 1 | R, data_MSB, data_LSB]
        let crc_input = [
            self.address << 1,       // address + write bit
            reg,                     // register address
            (self.address << 1) | 1, // address + read bit
            buf[0],                  // data MSB
            buf[1],                  // data LSB
        ];
        let computed = crc8(&crc_input);
        let expected = buf[2];

        if computed != expected {
            return Err(Error::CrcMismatch { expected, computed });
        }

        Ok([buf[0], buf[1]])
    }
}

// ---------------------------------------------------------------------------
// CRC-8 (feature-gated)
// ---------------------------------------------------------------------------

/// Compute CRC-8 with polynomial 0x07 and initial value 0xFF.
///
/// This is the CRC algorithm specified in the TMAG5273 datasheet for I2C
/// data integrity validation. Bitwise implementation — no lookup table
/// (the TMAG5273 CRCs at most 5 bytes per transaction).
#[cfg(feature = "crc")]
fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0xFF;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x07
            } else {
                crc << 1
            };
        }
    }
    crc
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
        // DEVICE_CONFIG_1: CRC_EN is set when `crc` feature is active.
        #[cfg(feature = "crc")]
        let dc1: u8 = 0x80;
        #[cfg(not(feature = "crc"))]
        let dc1: u8 = 0x00;

        vec![
            // Wake-up: empty write (device may NACK — ignored by driver)
            I2cTransaction::write(address, vec![]),
            // Wake-up delay: 2 dummy DEVICE_ID reads (~100us at 400kHz)
            I2cTransaction::write_read(address, vec![0x0D], vec![0x01]),
            I2cTransaction::write_read(address, vec![0x0D], vec![0x01]),
            // Disable interrupts before config (INT_CONFIG_1 = 0x00)
            I2cTransaction::write(address, vec![0x08, 0x00]),
            // Burst read manufacturer ID (0x0E..=0x0F) → [LSB=0x49, MSB=0x54]
            I2cTransaction::write_read(address, vec![0x0E], vec![0x49, 0x54]),
            // Read device ID (0x0D) → version 1
            I2cTransaction::write_read(address, vec![0x0D], vec![0x01]),
            // Burst write config registers 0x00-0x03
            I2cTransaction::write(address, vec![0x00, dc1, 0x02, 0x70, 0x03]),
            // Separate write for T_CONFIG (0x07)
            I2cTransaction::write(address, vec![0x07, 0x01]),
            // Read device status
            I2cTransaction::write_read(address, vec![0x1C], vec![0x00]),
            // Clear stale fault flags (DEVICE_STATUS = 0x0F)
            I2cTransaction::write(address, vec![0x1C, 0x0F]),
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
            // Wake-up: empty write
            I2cTransaction::write(0x22, vec![]),
            // Wake-up delay: 2 dummy DEVICE_ID reads
            I2cTransaction::write_read(0x22, vec![0x0D], vec![0x01]),
            I2cTransaction::write_read(0x22, vec![0x0D], vec![0x01]),
            // Disable interrupts before config
            I2cTransaction::write(0x22, vec![0x08, 0x00]),
            // Burst read manufacturer ID — return wrong ID
            I2cTransaction::write_read(0x22, vec![0x0E], vec![0x00, 0x00]),
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
            // Wake-up: empty write
            I2cTransaction::write(0x22, vec![]),
            // Wake-up delay: 2 dummy DEVICE_ID reads
            I2cTransaction::write_read(0x22, vec![0x0D], vec![0x01]),
            I2cTransaction::write_read(0x22, vec![0x0D], vec![0x01]),
            // Disable interrupts before config
            I2cTransaction::write(0x22, vec![0x08, 0x00]),
            // Burst read manufacturer ID — correct
            I2cTransaction::write_read(0x22, vec![0x0E], vec![0x49, 0x54]),
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
        // Temperature read from register 0x10. raw = 17508 (0x4464) → 25.0°C.
        // With CRC feature: 3 bytes (data + CRC); without: 2 bytes.
        #[cfg(not(feature = "crc"))]
        expectations.push(I2cTransaction::write_read(
            0x22,
            vec![0x10],
            vec![0x44, 0x64],
        ));
        #[cfg(feature = "crc")]
        {
            // CRC input: [addr<<1|W=0x44, reg=0x10, addr<<1|R=0x45, MSB=0x44, LSB=0x64]
            let crc = super::crc8(&[0x44, 0x10, 0x45, 0x44, 0x64]);
            expectations.push(I2cTransaction::write_read(
                0x22,
                vec![0x10],
                vec![0x44, 0x64, crc],
            ));
        }

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
        let writes = config.to_register_writes();
        let mut expectations = vec![
            // Wake-up: empty write
            I2cTransaction::write(address, vec![]),
            // Wake-up delay: 2 dummy DEVICE_ID reads
            I2cTransaction::write_read(address, vec![0x0D], vec![0x01]),
            I2cTransaction::write_read(address, vec![0x0D], vec![0x01]),
            // Disable interrupts before config
            I2cTransaction::write(address, vec![0x08, 0x00]),
            // Burst manufacturer ID read
            I2cTransaction::write_read(address, vec![0x0E], vec![0x49, 0x54]),
            I2cTransaction::write_read(address, vec![0x0D], vec![0x01]),
            // Burst write registers 0x00-0x03
            I2cTransaction::write(
                address,
                vec![0x00, writes[0].1, writes[1].1, writes[2].1, writes[3].1],
            ),
            // Separate T_CONFIG write
            I2cTransaction::write(address, vec![writes[4].0, writes[4].1]),
            I2cTransaction::write_read(address, vec![0x1C], vec![0x00]),
            // Clear stale fault flags
            I2cTransaction::write(address, vec![0x1C, 0x0F]),
            I2cTransaction::write_read(address, vec![0x18], vec![0x00]),
        ];

        // trigger_conversion: read CONV_STATUS with bit 7 set (0x18 | 0x80 = 0x98)
        expectations.push(I2cTransaction::write_read(address, vec![0x98], vec![0x00]));

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

    // -- Runtime CRC toggle ---------------------------------------------------

    #[test]
    fn set_crc_enabled_writes_bit7() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // Read DEVICE_CONFIG_1, then write with CRC_EN=1 (bit 7 set).
        expectations.push(I2cTransaction::write_read(address, vec![0x00], vec![0x00]));
        expectations.push(I2cTransaction::write(address, vec![0x00, 0x80]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        sensor.set_crc_enabled(true).unwrap();

        sensor.into_inner().done();
    }

    #[test]
    fn set_crc_disabled_clears_bit7() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // Read DEVICE_CONFIG_1 with CRC_EN=1, then write with CRC_EN=0.
        expectations.push(I2cTransaction::write_read(address, vec![0x00], vec![0x80]));
        expectations.push(I2cTransaction::write(address, vec![0x00, 0x00]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        sensor.set_crc_enabled(false).unwrap();

        sensor.into_inner().done();
    }

    #[test]
    fn get_crc_enabled_reads_bit7() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // Return DEVICE_CONFIG_1 with CRC_EN=1.
        expectations.push(I2cTransaction::write_read(address, vec![0x00], vec![0x80]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        assert!(sensor.get_crc_enabled().unwrap());

        sensor.into_inner().done();
    }

    #[test]
    fn get_crc_disabled_reads_bit7_clear() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // Return DEVICE_CONFIG_1 with CRC_EN=0.
        expectations.push(I2cTransaction::write_read(address, vec![0x00], vec![0x00]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        assert!(!sensor.get_crc_enabled().unwrap());

        sensor.into_inner().done();
    }

    // -- I2C Read Mode -------------------------------------------------------

    #[test]
    fn set_read_mode_writes_correct_register_value() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // Read DEVICE_CONFIG_1, then write with I2C_RD=0x01 (OneByte16Bit).
        expectations.push(I2cTransaction::write_read(address, vec![0x00], vec![0x00]));
        expectations.push(I2cTransaction::write(address, vec![0x00, 0x01]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        sensor.set_read_mode(I2cReadMode::OneByte16Bit).unwrap();

        sensor.into_inner().done();
    }

    #[test]
    fn get_read_mode_reads_back_correct_enum() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // Return DEVICE_CONFIG_1 with I2C_RD=0x02 (OneByte8Bit).
        expectations.push(I2cTransaction::write_read(address, vec![0x00], vec![0x02]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        assert_eq!(sensor.get_read_mode().unwrap(), I2cReadMode::OneByte8Bit);

        sensor.into_inner().done();
    }

    #[test]
    fn read_magnetic_returns_error_when_non_standard_mode() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // set_read_mode to OneByte16Bit: read-modify-write on DEVICE_CONFIG_1
        expectations.push(I2cTransaction::write_read(address, vec![0x00], vec![0x00]));
        expectations.push(I2cTransaction::write(address, vec![0x00, 0x01]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        sensor.set_read_mode(I2cReadMode::OneByte16Bit).unwrap();
        let err = sensor.read_magnetic().unwrap_err();
        assert_eq!(err, Error::NonStandardReadMode { mode: 0x01 });

        sensor.into_inner().done();
    }

    #[test]
    fn read_temperature_returns_error_when_non_standard_mode() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // set_read_mode to OneByte8Bit
        expectations.push(I2cTransaction::write_read(address, vec![0x00], vec![0x00]));
        expectations.push(I2cTransaction::write(address, vec![0x00, 0x02]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        sensor.set_read_mode(I2cReadMode::OneByte8Bit).unwrap();
        let err = sensor.read_temperature().unwrap_err();
        assert_eq!(err, Error::NonStandardReadMode { mode: 0x02 });

        sensor.into_inner().done();
    }

    // -- INT Pin Readback ----------------------------------------------------

    #[test]
    fn is_interrupt_active_reads_intb_rb_bit() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // Return DEVICE_STATUS with INTB_RB=1 (bit 4 set) and no faults.
        expectations.push(I2cTransaction::write_read(address, vec![0x1C], vec![0x10]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        assert!(sensor.is_interrupt_active().unwrap());

        sensor.into_inner().done();
    }

    #[test]
    fn is_interrupt_not_active_when_bit_clear() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // Return DEVICE_STATUS with INTB_RB=0, no faults.
        expectations.push(I2cTransaction::write_read(address, vec![0x1C], vec![0x00]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        assert!(!sensor.is_interrupt_active().unwrap());

        sensor.into_inner().done();
    }

    // -- CRC-8 ---------------------------------------------------------------

    #[cfg(feature = "crc")]
    #[test]
    fn crc8_empty_is_init_value() {
        assert_eq!(super::crc8(&[]), 0xFF);
    }

    #[cfg(feature = "crc")]
    #[test]
    fn crc8_deterministic_and_order_dependent() {
        let a = super::crc8(&[0x01, 0x02]);
        let b = super::crc8(&[0x02, 0x01]);
        assert_ne!(a, b, "CRC must be order-dependent");

        // Same input produces same output.
        assert_eq!(super::crc8(&[0x01, 0x02]), a);
    }

    #[cfg(feature = "crc")]
    #[test]
    fn crc8_single_byte_values() {
        // Just verify these don't panic and produce non-trivial results.
        let c0 = super::crc8(&[0x00]);
        let c1 = super::crc8(&[0xFF]);
        assert_ne!(c0, c1);
        // 0x00 XOR 0xFF = 0xFF, shift through poly 8 times.
        // The exact value depends on the polynomial — just verify non-zero.
        assert_ne!(c0, 0);
    }

    // -- Phase 4: Configuration read-back getters ----------------------------

    #[test]
    fn get_operating_mode_reads_correct_value() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // DEVICE_CONFIG_2 with operating_mode = WakeUpAndSleep (0b11) in
        // bits [1:0], plus LP_LN=1 (bit 4) to verify field isolation.
        // Register value: 0b0001_0011 = 0x13
        expectations.push(I2cTransaction::write_read(address, vec![0x01], vec![0x13]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let mode = sensor.get_operating_mode().unwrap();
        assert_eq!(mode, OperatingMode::WakeUpAndSleep);

        sensor.into_inner().done();
    }

    #[test]
    fn get_magnetic_channels_reads_correct_value() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // SENSOR_CONFIG_1 with MAG_CH_EN = XYZ (0x7) in bits [7:4],
        // SLEEPTIME = Ms5 (0x1) in bits [3:0].
        // Register value: 0b0111_0001 = 0x71
        expectations.push(I2cTransaction::write_read(address, vec![0x02], vec![0x71]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let channels = sensor.get_magnetic_channels().unwrap();
        assert_eq!(channels, MagneticChannel::XYZ);

        sensor.into_inner().done();
    }

    #[test]
    fn read_conversion_status_parses_all_fields() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // CONV_STATUS with SET_COUNT=5 (bits [7:5] = 0b101), POR=1 (bit 4),
        // DIAG=1 (bit 1), RESULT=1 (bit 0).
        // 0b1011_0011 = 0xB3
        expectations.push(I2cTransaction::write_read(address, vec![0x18], vec![0xB3]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let status = sensor.read_conversion_status().unwrap();
        assert_eq!(status.set_count, 5);
        assert!(status.power_on_reset);
        assert!(status.diag_fail);
        assert!(status.result_ready);

        sensor.into_inner().done();
    }

    // -- check_diagnostics (Zephyr-inspired) --------------------------------

    #[test]
    fn check_diagnostics_no_fault() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // CONV_STATUS with DIAG_FAIL=0 (bit 1 clear)
        expectations.push(I2cTransaction::write_read(address, vec![0x18], vec![0x00]));
        // No DEVICE_STATUS read should follow.

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let status = sensor.check_diagnostics().unwrap();
        assert!(status.is_ok());
        assert_eq!(status, DeviceStatus::default());

        sensor.into_inner().done();
    }

    #[test]
    fn check_diagnostics_with_fault() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address);

        // CONV_STATUS with DIAG_FAIL=1 (bit 1 set) → 0x02
        expectations.push(I2cTransaction::write_read(address, vec![0x18], vec![0x02]));
        // Driver should then read DEVICE_STATUS (0x1C) to get fault details.
        // Return VCC undervoltage (bit 0) + oscillator error (bit 3) = 0x09.
        expectations.push(I2cTransaction::write_read(address, vec![0x1C], vec![0x09]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let status = sensor.check_diagnostics().unwrap();
        assert!(!status.is_ok());
        assert!(status.vcc_undervoltage);
        assert!(status.oscillator_error);
        assert!(!status.otp_crc_error);
        assert!(!status.int_error);

        sensor.into_inner().done();
    }
}
