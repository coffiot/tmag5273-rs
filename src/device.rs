//! Core driver implementation for the TMAG5273 sensor.
//!
//! Uses a typestate pattern with [`Unconfigured`] and [`Configured`] markers
//! to prevent reading before initialization at compile time.

use core::fmt;
use core::marker::PhantomData;

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::config::Config;
use crate::error::{Error, InitError};
use crate::register;
use crate::types::{
    AngleCalculation, AngleReading, Celsius, ConversionAverage, ConversionStatus, Degrees,
    DeviceStatus, DeviceVariant, Diagnostics, GainChannel, I2cReadMode, IntPinBehavior,
    InterruptMode, MagneticChannel, MagneticReading, Micros, MilliTesla, NoDelay, OperatingMode,
    PowerMode, Range, SensorReading, SleepDuration, TempCoefficient, ThresholdCrossingCount,
    ThresholdDirection, TriggerMode, WAKE_DELAY,
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
/// # Construction
///
/// Three ways to create the driver (simplest first):
///
/// 1. **[`scan()`](Self::scan)** — tries all known addresses, auto-detects variant.
///    Zero hardware knowledge required.
/// 2. **[`detect()`](Self::detect)** — auto-detects variant at a known address.
/// 3. **[`new()`](Self::new)** — explicit variant, no I2C activity until [`init()`](Self::init).
///
/// After construction, call [`init()`](Self::init) with a [`Config`] to
/// transition from [`Unconfigured`] to [`Configured`] and unlock measurement
/// methods.
///
/// # Type Parameters
///
/// - `I2C`: The I2C bus implementing [`embedded_hal::i2c::I2c`].
/// - `State`: Either [`Unconfigured`] or [`Configured`] (typestate marker).
/// - `D`: Delay backend implementing [`embedded_hal::delay::DelayNs`].
///   Defaults to [`NoDelay`] — a zero-sized no-op that compiles to nothing.
///   When a real delay is provided via [`new_with_delay`](Self::new_with_delay),
///   the driver inserts precise timing for wake-up and conversion waits.
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
pub struct Tmag5273<I2C, State = Unconfigured, D = NoDelay> {
    i2c: I2C,
    variant: DeviceVariant,
    address: u8,
    xy_range: Range,
    z_range: Range,
    channels: MagneticChannel,
    angle_enabled: bool,
    temperature_enabled: bool,
    conversion_average: ConversionAverage,
    /// Current I2C read mode — high-level reads guard against non-Standard
    /// mode to prevent silent data corruption.
    read_mode: I2cReadMode,
    /// Per-sample diagnostic checking policy.
    diagnostics: Diagnostics,
    delay: D,
    _state: PhantomData<State>,
}

impl<I2C, State, D> fmt::Debug for Tmag5273<I2C, State, D> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Tmag5273")
            .field("variant", &self.variant)
            .field("address", &self.address)
            .finish_non_exhaustive()
    }
}

// Compile-time size assertion — should be ~16 bytes on ESP32 (I2C handle ~4 bytes + fields).
// NoDelay is a ZST, so adding D = NoDelay has zero size impact.
const _: () = assert!(
    core::mem::size_of::<Tmag5273<[u8; 4], Configured, NoDelay>>() <= 24,
    "Tmag5273 struct exceeds 24-byte budget (excluding I2C handle)"
);

// ---------------------------------------------------------------------------
// Unconfigured — construction + initialization
// ---------------------------------------------------------------------------

impl<I2C: I2c> Tmag5273<I2C, Unconfigured> {
    /// Create a new driver instance without performing any I2C operations.
    ///
    /// The I2C address is derived from the [`DeviceVariant`].
    /// Uses [`NoDelay`] as the delay backend — wake-up and conversion timing
    /// rely on bounded I2C polling.
    pub fn new(i2c: I2C, variant: DeviceVariant) -> Self {
        Self::new_with_address(i2c, variant, variant.default_address())
    }

    /// Auto-detect the device variant by reading the `DEVICE_ID` register.
    ///
    /// Sends a wake-up write, reads the version field from the sensor at the
    /// given `address`, and resolves the [`DeviceVariant`] from the address
    /// and version. Returns the constructed driver in `Unconfigured` state.
    ///
    /// On failure, the I2C bus is returned alongside the error so the caller
    /// can retry at a different address or reuse the bus. This follows the
    /// same bus-recovery pattern as [`init()`](Self::init).
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] if the sensor does not respond.
    /// Returns [`Error::InvalidRegisterValue`] if the version byte is not
    /// 1 or 2, or the address does not match any known family (A–D).
    pub fn detect(mut i2c: I2C, address: u8) -> Result<Self, (Error<I2C::Error>, I2C)> {
        let address = address & 0x7F;

        // Wake the sensor (may NACK if already awake — that's fine).
        let _ = i2c.write(address, &[]);

        // Read DEVICE_ID register to get the version field.
        let mut buf = [0u8];
        if let Err(e) = i2c.write_read(address, &[register::DEVICE_ID], &mut buf) {
            return Err((Error::I2c(e), i2c));
        }
        let version = register::extract(buf[0], register::VERSION_MASK, register::VERSION_SHIFT);

        match DeviceVariant::from_address_version(address, version) {
            Some(variant) => Ok(Self::new_with_address(i2c, variant, address)),
            None => {
                defmt_error!(
                    "detect: unknown version {} at address 0x{:02X}",
                    version,
                    address,
                );
                Err((
                    Error::InvalidRegisterValue {
                        register: register::DEVICE_ID,
                        value: version,
                    },
                    i2c,
                ))
            }
        }
    }

    /// Scan all known TMAG5273 addresses and auto-detect the variant.
    ///
    /// Tries each address in [`DeviceVariant::KNOWN_ADDRESSES`] (0x22, 0x35,
    /// 0x44, 0x78). Returns the first sensor that responds with a valid
    /// `DEVICE_ID`. No knowledge of the hardware is required — true
    /// plug-and-play.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction};
    /// # let i2c = I2cMock::new(&[]);
    /// use tmag5273::{Tmag5273, ConfigBuilder};
    ///
    /// let sensor = Tmag5273::scan(i2c).expect("no TMAG5273 found on bus");
    /// let config = ConfigBuilder::new().build().unwrap();
    /// // let mut sensor = sensor.init(&config).unwrap();
    /// ```
    ///
    /// # Errors
    ///
    /// Returns the last [`Error`] and the I2C bus if no sensor responds at
    /// any known address.
    pub fn scan(i2c: I2C) -> Result<Self, (Error<I2C::Error>, I2C)> {
        let mut bus = i2c;
        let mut last_err = None;

        for &address in &DeviceVariant::KNOWN_ADDRESSES {
            defmt_debug!("scan: probing address 0x{:02X}", address);
            match Self::detect(bus, address) {
                Ok(sensor) => {
                    defmt_debug!("scan: found sensor at 0x{:02X}", address);
                    return Ok(sensor);
                }
                Err((err, recovered_bus)) => {
                    last_err = Some(err);
                    bus = recovered_bus;
                }
            }
        }

        // All addresses failed — return last error with the bus.
        // SAFETY: last_err is always Some because KNOWN_ADDRESSES is non-empty.
        Err((last_err.unwrap(), bus))
    }

    /// Create a new driver instance with a custom I2C address.
    ///
    /// Use this when the sensor's address has been changed from the factory
    /// default. Uses [`NoDelay`] as the delay backend.
    ///
    /// The address is masked to 7 bits (`address & 0x7F`).
    ///
    /// # Panics (debug only)
    ///
    /// Debug-asserts that the 7-bit address is not in the I2C reserved
    /// ranges (0x00–0x07 or 0x78–0x7F). This catches wiring mistakes
    /// during development without adding runtime cost in release builds.
    pub fn new_with_address(i2c: I2C, variant: DeviceVariant, address: u8) -> Self {
        let addr = address & 0x7F;
        debug_assert!(
            (0x08..=0x77).contains(&addr),
            "I2C address 0x{addr:02X} is in the reserved range (valid: 0x08–0x77)"
        );
        Self {
            address: addr,
            i2c,
            variant,
            xy_range: Range::Low,
            z_range: Range::Low,
            channels: MagneticChannel::Off,
            angle_enabled: false,
            temperature_enabled: false,
            conversion_average: ConversionAverage::X1,
            read_mode: I2cReadMode::Standard,
            diagnostics: Diagnostics::Off,
            delay: NoDelay,
            _state: PhantomData,
        }
    }
}

impl<I2C: I2c, D: DelayNs> Tmag5273<I2C, Unconfigured, D> {
    /// Create a driver instance with a delay backend for precise timing.
    ///
    /// When a real [`DelayNs`] is provided:
    /// - [`wait_for_conversion()`](Tmag5273::wait_for_conversion) sleeps the exact conversion
    ///   time before polling
    /// - [`set_mode()`](Tmag5273::set_mode) automatically inserts an 80 us wake-up delay
    ///   after non-sleep mode changes
    pub fn new_with_delay(i2c: I2C, variant: DeviceVariant, delay: D) -> Self {
        Self::new_with_delay_and_address(i2c, variant, variant.default_address(), delay)
    }

    /// Create with delay backend and custom I2C address.
    ///
    /// The address is masked to 7 bits (`address & 0x7F`).
    ///
    /// # Panics (debug only)
    ///
    /// Debug-asserts that the 7-bit address is not in the I2C reserved
    /// ranges (0x00–0x07 or 0x78–0x7F).
    pub fn new_with_delay_and_address(
        i2c: I2C,
        variant: DeviceVariant,
        address: u8,
        delay: D,
    ) -> Self {
        let addr = address & 0x7F;
        debug_assert!(
            (0x08..=0x77).contains(&addr),
            "I2C address 0x{addr:02X} is in the reserved range (valid: 0x08–0x77)"
        );
        Self {
            address: addr,
            i2c,
            variant,
            xy_range: Range::Low,
            z_range: Range::Low,
            channels: MagneticChannel::Off,
            angle_enabled: false,
            temperature_enabled: false,
            conversion_average: ConversionAverage::X1,
            read_mode: I2cReadMode::Standard,
            diagnostics: Diagnostics::Off,
            delay,
            _state: PhantomData,
        }
    }

    /// Initialize the sensor: verify identity, apply configuration, and
    /// transition to [`Configured`] state.
    ///
    /// Sends a wake-up write in case the device was left in sleep mode by
    /// previous firmware. When a real delay backend is provided, the driver
    /// inserts an 80 us wake-up delay after the empty write. With [`NoDelay`],
    /// the subsequent I2C transactions (interrupt disable, manufacturer ID
    /// read) provide implicit wake-up time (>100 us at 400 kHz).
    ///
    /// Takes `config` by reference so the same configuration can be reused
    /// across multiple sensors.
    ///
    /// # Errors
    ///
    /// Returns [`InitError`] (containing the I2C bus and delay backend) if
    /// initialization fails. This allows the caller to recover the bus and
    /// retry or use it for another device.
    pub fn init(
        mut self,
        config: &Config,
    ) -> Result<Tmag5273<I2C, Configured, D>, InitError<I2C, D>> {
        if let Err(error) = self.init_inner(config) {
            return Err(InitError {
                error,
                i2c: self.i2c,
                delay: self.delay,
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
            temperature_enabled: config.temperature_enabled,
            conversion_average: config.conversion_average,
            read_mode: I2cReadMode::Standard,
            diagnostics: config.diagnostics,
            delay: self.delay,
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

        // 0b. Wait for wake-up: 80 us per TI datasheet (Linux kernel uses
        // usleep_range(80, 200)). NoDelay: no-op — the I2C transactions that
        // follow provide implicit wake-up time (>100 us at 400 kHz).
        self.delay.delay_us(WAKE_DELAY.min.0);

        // 0c. Disable interrupts before applying new configuration to prevent
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
            defmt_error!(
                "invalid manufacturer ID: 0x{:04X} (expected 0x{:04X})",
                manufacturer_id,
                register::MANUFACTURER_ID_EXPECTED,
            );
            return Err(Error::InvalidManufacturerId(manufacturer_id));
        }

        // 2. Read device ID and verify version matches variant
        let device_id = self.read_register(register::DEVICE_ID)?;
        let version = register::extract(device_id, register::VERSION_MASK, register::VERSION_SHIFT);

        if version != self.variant.version() {
            defmt_error!(
                "version mismatch: expected 0x{:02X}, got 0x{:02X}",
                self.variant.version(),
                version,
            );
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

        defmt_debug!(
            "init OK: variant={:?}, addr=0x{:02X}",
            self.variant,
            self.address,
        );

        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Configured — measurement + control
// ---------------------------------------------------------------------------

impl<I2C: I2c, D: DelayNs> Tmag5273<I2C, Configured, D> {
    /// Returns the expected conversion time for the current configuration.
    ///
    /// Computed from the stored [`ConversionAverage`], active magnetic channels,
    /// and temperature measurement state. Use this to calculate duty cycles or
    /// to manually sleep before reading in standby mode.
    #[must_use]
    pub fn conversion_time(&self) -> Micros {
        self.conversion_average
            .conversion_time(self.channels, self.temperature_enabled)
    }

    /// Decode raw X/Y/Z register values into a [`MagneticReading`].
    ///
    /// Applies the correct range conversion per axis and suppresses disabled
    /// channels as `None`.
    #[inline]
    fn decode_magnetic(&self, raw_x: i16, raw_y: i16, raw_z: i16) -> MagneticReading {
        let xy_range = self.variant.range_mt_xy(self.xy_range);
        let z_range = self.variant.range_mt_z(self.z_range);
        MagneticReading {
            x: if self.channels.has_x() {
                Some(MilliTesla::from_raw(raw_x, xy_range))
            } else {
                None
            },
            y: if self.channels.has_y() {
                Some(MilliTesla::from_raw(raw_y, xy_range))
            } else {
                None
            },
            z: if self.channels.has_z() {
                Some(MilliTesla::from_raw(raw_z, z_range))
            } else {
                None
            },
        }
    }

    /// Guard that the sensor is in Standard I2C read mode.
    ///
    /// High-level read methods require Standard mode; other modes change the
    /// response format and would silently corrupt parsed data.
    #[inline]
    fn require_standard_mode(&self) -> Result<(), Error<I2C::Error>> {
        if self.read_mode != I2cReadMode::Standard {
            defmt_error!("non-standard read mode: 0x{:02X}", self.read_mode as u8);
            return Err(Error::NonStandardReadMode {
                mode: self.read_mode as u8,
            });
        }
        Ok(())
    }

    /// Check CONV_STATUS for diagnostic failure (from a raw register byte).
    ///
    /// Behaviour depends on [`Diagnostics`]:
    /// - `Off` → returns `Ok(())` immediately
    /// - `Halt` → returns `Err(Error::DiagnosticFailure)` on fault
    /// - `Warn` → emits `defmt::warn!` on fault, returns `Ok(())`
    /// - `Ignore` → returns `Ok(())`
    #[inline]
    fn check_diag_from_conv_status(&mut self, conv_raw: u8) -> Result<(), Error<I2C::Error>> {
        if matches!(self.diagnostics, Diagnostics::Off) {
            return Ok(());
        }
        let conv = ConversionStatus::from_register(conv_raw);
        if !conv.diag_fail {
            return Ok(());
        }
        match self.diagnostics {
            // Off is unreachable (early return above) but kept for exhaustiveness.
            Diagnostics::Off | Diagnostics::Ignore => Ok(()),
            Diagnostics::Halt => {
                let status =
                    DeviceStatus::from_register(self.read_register(register::DEVICE_STATUS)?);
                Err(Error::DiagnosticFailure(status))
            }
            Diagnostics::Warn => {
                // Best-effort: if the status read fails, swallow the error —
                // Warn must never promote a diagnostic fault to a measurement error.
                // Outer cfg gates both the I2C read and the log call — defmt
                // requires a global logger only available on real hardware.
                #[cfg(all(feature = "defmt", not(test)))]
                if let Ok(raw) = self.read_register(register::DEVICE_STATUS) {
                    let status = DeviceStatus::from_register(raw);
                    defmt_warn!("TMAG5273 diagnostic fault (ignored): {:?}", status);
                }
                Ok(())
            }
        }
    }

    /// Read CONV_STATUS and check for diagnostic failure.
    ///
    /// Used by `read_temperature` and `read_angle` where CONV_STATUS is
    /// not included in the burst read.
    #[inline]
    fn read_and_check_diag(&mut self) -> Result<(), Error<I2C::Error>> {
        // Fast path: skip the CONV_STATUS register read when diagnostics are off.
        if matches!(self.diagnostics, Diagnostics::Off) {
            return Ok(());
        }
        let conv_raw = self.read_register(register::CONV_STATUS)?;
        self.check_diag_from_conv_status(conv_raw)
    }

    /// Read a register field and decode it into a typed enum.
    fn read_field<T: TryFrom<u8, Error = u8>>(
        &mut self,
        reg: u8,
        mask: u8,
        shift: u8,
    ) -> Result<T, Error<I2C::Error>> {
        let raw = self.read_register(reg)?;
        let val = register::extract(raw, mask, shift);
        T::try_from(val).map_err(|_| Error::InvalidRegisterValue {
            register: reg,
            value: val,
        })
    }

    /// Read magnetic field for all enabled channels.
    ///
    /// Uses a burst read for data coherence and speed. Disabled channels
    /// return `None`.
    ///
    /// When the `crc` feature is enabled, uses two 4-byte block reads
    /// (from 0x10 and 0x14) with per-block CRC-8 validation. The
    /// temperature bytes in block 1 are discarded.
    ///
    /// # Errors
    ///
    /// Returns [`Error::CrcMismatch`] if CRC validation fails (`crc` feature).
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn read_magnetic(&mut self) -> Result<MagneticReading, Error<I2C::Error>> {
        self.require_standard_mode()?;

        // With CRC: two 4-byte block reads (0x10-0x13, 0x14-0x17) with CRC
        // validation per block. Reads from T_MSB (0x10) for 4-byte alignment;
        // discards the 2 temperature bytes from block 1.
        #[cfg(feature = "crc")]
        let (raw_x, raw_y, raw_z) = {
            let block1 = self.read_block_with_crc(register::T_MSB_RESULT)?;
            let block2 = self.read_block_with_crc(0x14)?;
            // block1: [T_MSB, T_LSB, X_MSB, X_LSB] — discard T
            // block2: [Y_MSB, Y_LSB, Z_MSB, Z_LSB]
            self.read_and_check_diag()?;
            (
                i16::from_be_bytes([block1[2], block1[3]]),
                i16::from_be_bytes([block2[0], block2[1]]),
                i16::from_be_bytes([block2[2], block2[3]]),
            )
        };

        // Without CRC: burst read with optional diagnostic extension.
        #[cfg(not(feature = "crc"))]
        let (raw_x, raw_y, raw_z) = if !matches!(self.diagnostics, Diagnostics::Off) {
            let mut buf = [0u8; 7];
            self.i2c
                .write_read(self.address, &[register::X_MSB_RESULT], &mut buf)?;
            self.check_diag_from_conv_status(buf[6])?;
            (
                i16::from_be_bytes([buf[0], buf[1]]),
                i16::from_be_bytes([buf[2], buf[3]]),
                i16::from_be_bytes([buf[4], buf[5]]),
            )
        } else {
            let mut buf = [0u8; 6];
            self.i2c
                .write_read(self.address, &[register::X_MSB_RESULT], &mut buf)?;
            (
                i16::from_be_bytes([buf[0], buf[1]]),
                i16::from_be_bytes([buf[2], buf[3]]),
                i16::from_be_bytes([buf[4], buf[5]]),
            )
        };

        Ok(self.decode_magnetic(raw_x, raw_y, raw_z))
    }

    /// Read temperature from the sensor.
    ///
    /// # Errors
    ///
    /// Returns [`Error::TemperatureDisabled`] if the temperature channel was
    /// disabled in the configuration passed to [`Tmag5273::init`]. Per the TI
    /// datasheet, the `T_MSB_RESULT` / `T_LSB_RESULT` registers contain
    /// *undefined data* when the channel is off — calling this method in that
    /// state would return garbage silently.
    ///
    /// Returns [`Error::NonStandardReadMode`] if the I2C read mode is not
    /// [`I2cReadMode::Standard`](crate::I2cReadMode).
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn read_temperature(&mut self) -> Result<Celsius, Error<I2C::Error>> {
        if !self.temperature_enabled {
            defmt_error!("read_temperature called but temperature channel is disabled");
            return Err(Error::TemperatureDisabled);
        }

        self.require_standard_mode()?;

        let buf = self.read_register_pair(register::T_MSB_RESULT)?;
        let raw = i16::from_be_bytes(buf);
        self.read_and_check_diag()?;
        Ok(Celsius::from_raw(raw))
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
        self.require_standard_mode()?;

        if !self.angle_enabled {
            return Err(Error::AngleRequiresTwoChannels);
        }

        // Without CRC, read all 3 registers (ANGLE_MSB, ANGLE_LSB,
        // MAGNITUDE) in a single burst — they are consecutive.
        #[cfg(not(feature = "crc"))]
        let reading = {
            let mut buf = [0u8; 3];
            self.i2c
                .write_read(self.address, &[register::ANGLE_RESULT_MSB], &mut buf)?;
            let raw_angle = u16::from_be_bytes([buf[0], buf[1]]);
            let magnitude = buf[2];
            AngleReading {
                angle: Degrees::from_raw(raw_angle),
                magnitude,
            }
        };

        // With CRC, use the existing 2-call approach — `read_register_pair`
        // validates the CRC over the angle register pair, and the magnitude
        // is a separate single-byte read.
        #[cfg(feature = "crc")]
        let reading = {
            let angle_buf = self.read_register_pair(register::ANGLE_RESULT_MSB)?;
            let raw_angle = u16::from_be_bytes(angle_buf);
            let magnitude = self.read_register(register::MAGNITUDE_RESULT)?;
            AngleReading {
                angle: Degrees::from_raw(raw_angle),
                magnitude,
            }
        };

        self.read_and_check_diag()?;
        Ok(reading)
    }

    /// Read temperature and magnetic field in a single coherent burst.
    ///
    /// Performs a single 8-byte I2C burst read from `T_MSB_RESULT` (0x10)
    /// through `Z_LSB_RESULT` (0x17), guaranteeing all values come from
    /// the same conversion cycle. This matches the Linux kernel driver's
    /// `tmag5273_get_measure()` pattern.
    ///
    /// For best results in standby mode, call
    /// [`trigger_conversion`](Self::trigger_conversion) and
    /// [`wait_for_conversion`](Self::wait_for_conversion) first.
    ///
    /// # CRC
    ///
    /// When the `crc` feature is enabled, uses two 4-byte block reads
    /// (0x10–0x13 and 0x14–0x17) with per-block CRC-8 validation instead
    /// of a single 8-byte burst.
    ///
    /// # Errors
    ///
    /// Returns [`Error::NonStandardReadMode`] if the sensor is not in
    /// Standard I2C read mode. Returns [`Error::I2c`] on bus failure.
    pub fn read_all(&mut self) -> Result<SensorReading, Error<I2C::Error>> {
        self.require_standard_mode()?;

        // With CRC: two perfectly-aligned 4-byte block reads with CRC validation.
        // Block 1: 0x10-0x13 (T_MSB, T_LSB, X_MSB, X_LSB)
        // Block 2: 0x14-0x17 (Y_MSB, Y_LSB, Z_MSB, Z_LSB)
        #[cfg(feature = "crc")]
        let (raw_t, raw_x, raw_y, raw_z) = {
            let block1 = self.read_block_with_crc(register::T_MSB_RESULT)?;
            let block2 = self.read_block_with_crc(0x14)?;
            self.read_and_check_diag()?;
            (
                i16::from_be_bytes([block1[0], block1[1]]),
                i16::from_be_bytes([block1[2], block1[3]]),
                i16::from_be_bytes([block2[0], block2[1]]),
                i16::from_be_bytes([block2[2], block2[3]]),
            )
        };

        // Without CRC: single burst with optional diagnostic extension.
        #[cfg(not(feature = "crc"))]
        let (raw_t, raw_x, raw_y, raw_z) = if !matches!(self.diagnostics, Diagnostics::Off) {
            let mut buf = [0u8; 9];
            self.i2c
                .write_read(self.address, &[register::T_MSB_RESULT], &mut buf)?;
            self.check_diag_from_conv_status(buf[8])?;
            (
                i16::from_be_bytes([buf[0], buf[1]]),
                i16::from_be_bytes([buf[2], buf[3]]),
                i16::from_be_bytes([buf[4], buf[5]]),
                i16::from_be_bytes([buf[6], buf[7]]),
            )
        } else {
            let mut buf = [0u8; 8];
            self.i2c
                .write_read(self.address, &[register::T_MSB_RESULT], &mut buf)?;
            (
                i16::from_be_bytes([buf[0], buf[1]]),
                i16::from_be_bytes([buf[2], buf[3]]),
                i16::from_be_bytes([buf[4], buf[5]]),
                i16::from_be_bytes([buf[6], buf[7]]),
            )
        };

        let temperature = if self.temperature_enabled {
            Some(Celsius::from_raw(raw_t))
        } else {
            None
        };

        Ok(SensorReading {
            temperature,
            magnetic: self.decode_magnetic(raw_x, raw_y, raw_z),
        })
    }

    /// Enable or disable CRC-8 on the sensor's I2C interface.
    ///
    /// This controls the sensor's hardware CRC generation (register
    /// `DEVICE_CONFIG_1` bit 7). It does NOT control driver-side CRC
    /// validation — that is controlled by the `crc` Cargo feature.
    ///
    /// # Feature Guard
    ///
    /// Enabling sensor CRC without the `crc` Cargo feature would cause
    /// the sensor to append CRC bytes that the driver does not expect,
    /// silently corrupting all subsequent reads. To prevent this, the
    /// method only accepts `enabled = true` when the `crc` feature is
    /// compiled in. Calling `set_crc_enabled(true)` without the feature
    /// is a compile error.
    ///
    /// Disabling sensor CRC (`set_crc_enabled(false)`) is always allowed.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    #[cfg(feature = "crc")]
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

    /// Disable CRC-8 on the sensor's I2C interface.
    ///
    /// When the `crc` Cargo feature is not enabled, only disabling sensor
    /// CRC is allowed. Attempting to enable CRC without the feature returns
    /// [`Error::CrcFeatureRequired`].
    ///
    /// See the `crc`-gated version for full enable/disable.
    ///
    /// # Errors
    ///
    /// Returns [`Error::CrcFeatureRequired`] if `enabled` is `true` without
    /// the `crc` Cargo feature.
    /// Returns [`Error::I2c`] on bus communication failure.
    #[cfg(not(feature = "crc"))]
    pub fn set_crc_enabled(&mut self, enabled: bool) -> Result<(), Error<I2C::Error>> {
        if enabled {
            defmt_error!("CRC enable requires the `crc` Cargo feature");
            return Err(Error::CrcFeatureRequired);
        }
        let current = self.read_register(register::DEVICE_CONFIG_1)?;
        let updated = register::insert(current, register::CRC_EN_MASK, register::CRC_EN_SHIFT, 0);
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
        self.read_field(
            register::DEVICE_CONFIG_1,
            register::I2C_RD_MASK,
            register::I2C_RD_SHIFT,
        )
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

    /// Maximum retry attempts for register access after wake-up from sleep.
    ///
    /// The TI datasheet specifies 50 µs typical wake time, but real hardware
    /// (ESP32-S3 + TMAG5273 B1) needs ~6 ms. Three attempts with 80 µs
    /// inter-retry delay covers this reliably. Neither the Linux kernel nor
    /// Zephyr RTOS drivers retry — they rely on implicit settling, which
    /// can fail on fast I2C controllers.
    const WAKE_RETRY_MAX: u8 = 3;
    // Compile-time guard: the retry loop assumes at least one iteration
    // so that `last_err` is always `Some` at the `unwrap()` call.
    #[allow(dead_code)]
    const WAKE_RETRY_ASSERT: () = assert!(Self::WAKE_RETRY_MAX > 0);

    /// Switch the operating mode at runtime.
    ///
    /// # Wake-up from sleep
    ///
    /// When the device may be in sleep mode, `set_mode()` automatically
    /// retries the register access up to 3 times to handle post-sleep
    /// I2C NACKs. An empty I2C write is sent first to trigger the
    /// device's wake-up circuit, followed by an 80 µs settling delay
    /// (per TI datasheet; Linux kernel uses `usleep_range(80, 200)`).
    ///
    /// With [`NoDelay`], the inter-retry delay is a no-op — the I2C bus
    /// transaction overhead (~100 µs at 400 kHz) provides implicit
    /// settling between attempts.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] if the register access fails after all
    /// retry attempts, indicating the device is unresponsive.
    pub fn set_mode(&mut self, mode: OperatingMode) -> Result<(), Error<I2C::Error>> {
        // Wake the sensor if it may be sleeping — any I2C activity wakes
        // the device (TI datasheet §7.5.1). Errors are expected and
        // tolerated (Zephyr: "operation is known to fail").
        let _ = self.i2c.write(self.address, &[]);
        self.delay.delay_us(WAKE_DELAY.min.0);

        // Bounded retry on the register read — the device may NACK the
        // first access after deep sleep even with the wake delay above.
        // Only NACK errors are retried (expected during wake). Bus faults,
        // arbitration loss, and other hard errors propagate immediately.
        // Non-sleep transitions succeed on the first attempt (no overhead).
        let mut last_err = None;
        for _ in 0..Self::WAKE_RETRY_MAX {
            match self.read_register(register::DEVICE_CONFIG_2) {
                Ok(current) => {
                    let updated = register::insert(
                        current,
                        register::OPERATING_MODE_MASK,
                        register::OPERATING_MODE_SHIFT,
                        mode as u8,
                    );
                    self.write_register(register::DEVICE_CONFIG_2, updated)?;

                    if !matches!(mode, OperatingMode::Sleep) {
                        self.delay.delay_us(WAKE_DELAY.min.0);
                    }
                    defmt_debug!("set_mode: {:?}", mode);
                    return Ok(());
                }
                Err(e) => {
                    use embedded_hal::i2c::{Error as I2cError, ErrorKind};
                    if matches!(e.kind(), ErrorKind::NoAcknowledge(_)) {
                        defmt_debug!("set_mode: NACK, retrying wake");
                        last_err = Some(e);
                        self.delay.delay_us(WAKE_DELAY.min.0);
                    } else {
                        // Hard bus fault — propagate immediately, no retry.
                        return Err(Error::I2c(e));
                    }
                }
            }
        }

        defmt_error!(
            "set_mode: wake failed after {} retries",
            Self::WAKE_RETRY_MAX
        );
        Err(Error::I2c(last_err.unwrap()))
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
    /// When a real delay backend is provided, sleeps the exact conversion time
    /// before polling — the poll loop then acts as a safety net (typically
    /// exits on the first read). With [`NoDelay`], the delay is a no-op and
    /// the bounded poll loop is the primary completion mechanism.
    ///
    /// # Errors
    ///
    /// Returns [`Error::ConversionTimeout`] if the conversion does not complete
    /// within the poll limit, or [`Error::I2c`] on bus failure.
    pub fn wait_for_conversion(&mut self) -> Result<ConversionStatus, Error<I2C::Error>> {
        // Sleep the exact conversion time.
        // NoDelay: no-op (compiles to nothing). Real delay: precise sleep.
        let ct = self
            .conversion_average
            .conversion_time(self.channels, self.temperature_enabled);
        self.delay.delay_us(ct.0);

        // Bounded poll — primary loop for NoDelay, safety net for real delays.
        for _ in 0..self.conversion_average.max_polls() {
            let raw = self.read_register(register::CONV_STATUS)?;
            let status = ConversionStatus::from_register(raw);
            if status.result_ready {
                return Ok(status);
            }
        }
        defmt_error!(
            "conversion timeout after {} polls",
            self.conversion_average.max_polls()
        );
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
        Ok(DeviceStatus::from_register(raw))
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
    pub fn check_diag(&mut self) -> Result<DeviceStatus, Error<I2C::Error>> {
        let conv_raw = self.read_register(register::CONV_STATUS)?;
        let conv = ConversionStatus::from_register(conv_raw);
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

        // Threshold hysteresis: DEVICE_CONFIG_2[7:5] — read-modify-write.
        // THR_HYST is a mode selector (0=Signed, 1=SymmetricBand), NOT a
        // hysteresis magnitude.  Values 2–7 are reserved per SBASAI4 Table 8-4.
        let dc2 = self.read_register(register::DEVICE_CONFIG_2)?;
        let dc2 = register::insert(
            dc2,
            register::THR_HYST_MASK,
            register::THR_HYST_SHIFT,
            config.hysteresis as u8,
        );
        self.write_register(register::DEVICE_CONFIG_2, dc2)?;

        // Direction + crossing count: SENSOR_CONFIG_2[6:5] — read-modify-write.
        let sc2 = self.read_register(register::SENSOR_CONFIG_2)?;
        let sc2 = register::insert(
            sc2,
            register::THRX_COUNT_MASK,
            register::THRX_COUNT_SHIFT,
            config.crossing_count as u8,
        );
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
        let register = register::insert(
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
        self.write_register(register::INT_CONFIG_1, register)?;
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
        let old_address = self.address;

        // Reject addresses outside the usable 7-bit I2C range.
        // 0x00-0x07 and 0x78-0x7F are reserved by the I2C specification.
        if !(0x08..=0x77).contains(&new_address) {
            return Err(Error::AddressChangeFailed {
                old_address,
                new_address,
            });
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

        // Switch to new address and verify with a single 2-byte burst read
        // of the manufacturer ID registers (LSB=0x0E, MSB=0x0F), matching the
        // pattern used in `init_inner()`.
        self.address = new_address;

        let mut id_buf = [0u8; 2];
        match self
            .i2c
            .write_read(self.address, &[register::MANUFACTURER_ID_LSB], &mut id_buf)
        {
            Ok(()) => {
                let id = u16::from(id_buf[1]) << 8 | u16::from(id_buf[0]);
                if id == register::MANUFACTURER_ID_EXPECTED {
                    defmt_debug!(
                        "address changed: 0x{:02X} -> 0x{:02X}",
                        old_address,
                        new_address,
                    );
                    Ok(())
                } else {
                    defmt_error!(
                        "address change failed: 0x{:02X} -> 0x{:02X} (verification)",
                        old_address,
                        new_address,
                    );
                    self.address = old_address;
                    Err(Error::AddressChangeFailed {
                        old_address,
                        new_address,
                    })
                }
            }
            Err(_) => {
                defmt_error!(
                    "address change failed: 0x{:02X} -> 0x{:02X} (no response)",
                    old_address,
                    new_address,
                );
                self.address = old_address;
                Err(Error::AddressChangeFailed {
                    old_address,
                    new_address,
                })
            }
        }
    }

    // -----------------------------------------------------------------------
    // Advanced: Wake-Up-and-Sleep Lifecycle
    // -----------------------------------------------------------------------

    /// Configure the sensor for wake-up-and-sleep mode with recommended
    /// TI datasheet settings (section 8.2.1.2).
    ///
    /// This writes:
    /// - `INT_CONFIG_1`: threshold interrupt enabled, INT pin mode, 10 us pulse
    /// - `DEVICE_CONFIG_2`: wake-up-and-sleep operating mode, hysteresis = 1
    ///   (preserves glitch filter and trigger mode via read-modify-write)
    ///
    /// **Warning:** This overwrites the interrupt configuration and operating
    /// mode. Call this AFTER [`init`](crate::Tmag5273::init) and configure
    /// thresholds with [`set_thresholds`](Self::set_thresholds) before calling
    /// this method.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn configure_wake_and_sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        // INT_CONFIG_1: threshold crossing interrupt on INT pin, 10 us pulse.
        // Decomposed from TI datasheet value 0x64 = 0b0110_0100:
        //   RSLT_INT=0, THRESHOLD_INT=1, INT_STATE=1 (Pulse10us),
        //   INT_MODE=001 (ThroughInt), MASK_INTB=0
        let int_config = register::insert(
            0,
            register::THRESHOLD_INT_MASK,
            register::THRESHOLD_INT_SHIFT,
            1,
        ) | register::insert(
            0,
            register::INT_STATE_MASK,
            register::INT_STATE_SHIFT,
            IntPinBehavior::Pulse10us as u8,
        ) | register::insert(
            0,
            register::INT_MODE_MASK,
            register::INT_MODE_SHIFT,
            InterruptMode::ThroughInt as u8,
        );
        self.write_register(register::INT_CONFIG_1, int_config)?;

        // DEVICE_CONFIG_2: wake-up-and-sleep mode + hysteresis = 1.
        // Read-modify-write to preserve glitch filter and trigger mode settings.
        let current = self.read_register(register::DEVICE_CONFIG_2)?;
        let updated = register::insert(
            current,
            register::OPERATING_MODE_MASK,
            register::OPERATING_MODE_SHIFT,
            OperatingMode::WakeUpAndSleep as u8,
        );
        let updated = register::insert(
            updated,
            register::THR_HYST_MASK,
            register::THR_HYST_SHIFT,
            1, // ThresholdMode::SymmetricBand — TI datasheet recommended for wake-up-and-sleep
        );
        self.write_register(register::DEVICE_CONFIG_2, updated)?;

        // Wake delay for mode transition (bypasses set_mode, so must delay explicitly).
        // NoDelay: no-op. Real delay: automatic wake-up timing.
        self.delay.delay_us(WAKE_DELAY.min.0);

        Ok(())
    }

    /// Put the sensor into sleep mode for power savings.
    ///
    /// After calling this, the next [`set_mode()`](Self::set_mode) call will
    /// automatically retry the register access to handle the post-sleep
    /// wake-up delay — no caller-side retry is needed.
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
        self.read_field(
            register::DEVICE_CONFIG_1,
            register::MAG_TEMPCO_MASK,
            register::MAG_TEMPCO_SHIFT,
        )
    }

    /// Read the conversion averaging mode from register `DEVICE_CONFIG_1`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_conversion_average(&mut self) -> Result<ConversionAverage, Error<I2C::Error>> {
        self.read_field(
            register::DEVICE_CONFIG_1,
            register::CONV_AVG_MASK,
            register::CONV_AVG_SHIFT,
        )
    }

    /// Read the current operating mode from register `DEVICE_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_operating_mode(&mut self) -> Result<OperatingMode, Error<I2C::Error>> {
        self.read_field(
            register::DEVICE_CONFIG_2,
            register::OPERATING_MODE_MASK,
            register::OPERATING_MODE_SHIFT,
        )
    }

    /// Read the power mode from register `DEVICE_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_power_mode(&mut self) -> Result<PowerMode, Error<I2C::Error>> {
        self.read_field(
            register::DEVICE_CONFIG_2,
            register::LP_LN_MASK,
            register::LP_LN_SHIFT,
        )
    }

    /// Read the trigger mode from register `DEVICE_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_trigger_mode(&mut self) -> Result<TriggerMode, Error<I2C::Error>> {
        self.read_field(
            register::DEVICE_CONFIG_2,
            register::TRIGGER_MODE_MASK,
            register::TRIGGER_MODE_SHIFT,
        )
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

    /// Read the Threshold hysteresis from register `DEVICE_CONFIG_2`.
    ///
    /// Returns the raw `THR_HYST` field value.  Only 0 and 1 are valid
    /// per the TI datasheet; values 2–7 are reserved.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_threshold_hysteresis_raw(&mut self) -> Result<u8, Error<I2C::Error>> {
        let raw = self.read_register(register::DEVICE_CONFIG_2)?;
        Ok(register::extract(
            raw,
            register::THR_HYST_MASK,
            register::THR_HYST_SHIFT,
        ))
    }

    /// Read the threshold crossing count from register `SENSOR_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value, or [`Error::I2c`] on bus failure.
    pub fn get_threshold_crossing_count(
        &mut self,
    ) -> Result<ThresholdCrossingCount, Error<I2C::Error>> {
        self.read_field(
            register::SENSOR_CONFIG_2,
            register::THRX_COUNT_MASK,
            register::THRX_COUNT_SHIFT,
        )
    }

    /// Read the magnetic channel selection from register `SENSOR_CONFIG_1`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_magnetic_channels(&mut self) -> Result<MagneticChannel, Error<I2C::Error>> {
        self.read_field(
            register::SENSOR_CONFIG_1,
            register::MAG_CH_EN_MASK,
            register::MAG_CH_EN_SHIFT,
        )
    }

    /// Read the sleep duration from register `SENSOR_CONFIG_1`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_sleep_duration(&mut self) -> Result<SleepDuration, Error<I2C::Error>> {
        self.read_field(
            register::SENSOR_CONFIG_1,
            register::SLEEPTIME_MASK,
            register::SLEEPTIME_SHIFT,
        )
    }

    /// Read the angle calculation mode from register `SENSOR_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_angle_calculation(&mut self) -> Result<AngleCalculation, Error<I2C::Error>> {
        self.read_field(
            register::SENSOR_CONFIG_2,
            register::ANGLE_EN_MASK,
            register::ANGLE_EN_SHIFT,
        )
    }

    /// Read the X/Y axis measurement range from register `SENSOR_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_xy_range(&mut self) -> Result<Range, Error<I2C::Error>> {
        self.read_field(
            register::SENSOR_CONFIG_2,
            register::X_Y_RANGE_MASK,
            register::X_Y_RANGE_SHIFT,
        )
    }

    /// Read the Z axis measurement range from register `SENSOR_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_z_range(&mut self) -> Result<Range, Error<I2C::Error>> {
        self.read_field(
            register::SENSOR_CONFIG_2,
            register::Z_RANGE_MASK,
            register::Z_RANGE_SHIFT,
        )
    }

    /// Read the threshold crossing direction from register `SENSOR_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_threshold_direction(&mut self) -> Result<ThresholdDirection, Error<I2C::Error>> {
        self.read_field(
            register::SENSOR_CONFIG_2,
            register::MAG_THR_DIR_MASK,
            register::MAG_THR_DIR_SHIFT,
        )
    }

    /// Read the gain channel selection from register `SENSOR_CONFIG_2`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_gain_channel(&mut self) -> Result<GainChannel, Error<I2C::Error>> {
        self.read_field(
            register::SENSOR_CONFIG_2,
            register::MAG_GAIN_CH_MASK,
            register::MAG_GAIN_CH_SHIFT,
        )
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
    /// Returns the signed 7-bit threshold occupying bits \[7:1\] of
    /// `T_CONFIG`. The 7-bit value is sign-extended to `i8` so negative
    /// thresholds round-trip correctly through
    /// [`set_thresholds`](Self::set_thresholds).
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
        // Sign-extend from 7 bits: if bit 6 is set, the value is negative
        // in two's complement. Set bit 7 to propagate the sign into `i8`.
        let signed = if val & 0x40 != 0 {
            (val | 0x80) as i8
        } else {
            val as i8
        };
        Ok(signed)
    }

    /// Read the interrupt routing mode from register `INT_CONFIG_1`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidRegisterValue`] if the register contains an
    /// unrecognized value.
    /// Returns [`Error::I2c`] on bus communication failure.
    pub fn get_interrupt_mode(&mut self) -> Result<InterruptMode, Error<I2C::Error>> {
        self.read_field(
            register::INT_CONFIG_1,
            register::INT_MODE_MASK,
            register::INT_MODE_SHIFT,
        )
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
        Ok(ConversionStatus::from_register(raw))
    }
}

// ---------------------------------------------------------------------------
// Private I2C helpers (shared across both states)
// ---------------------------------------------------------------------------

impl<I2C: I2c, State, D> Tmag5273<I2C, State, D> {
    /// Release the I2C bus, consuming the driver.
    ///
    /// The sensor is left in whatever state it was in.
    ///
    /// See also [`into_parts()`](Self::into_parts) to recover both the I2C
    /// bus and the delay backend.
    pub fn into_inner(self) -> I2C {
        self.i2c
    }

    /// Consume the driver, returning the I2C bus and delay backend.
    ///
    /// Use this when you need both the bus and delay back — for example,
    /// to re-initialize with a different configuration or to reuse the
    /// peripherals with another device.
    pub fn into_parts(self) -> (I2C, D) {
        (self.i2c, self.delay)
    }

    /// Read a single register byte.
    fn read_register(&mut self, register: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0u8];
        self.i2c.write_read(self.address, &[register], &mut buf)?;
        defmt_trace!("read  reg 0x{:02X} = 0x{:02X}", register, buf[0]);
        Ok(buf[0])
    }

    /// Write a single register byte.
    fn write_register(&mut self, register: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[register, value])?;
        defmt_trace!("write reg 0x{:02X} = 0x{:02X}", register, value);
        Ok(())
    }

    /// Read a register pair (MSB + LSB) and return as `i16` (big-endian).
    ///
    /// When the `crc` feature is enabled, reads 3 bytes (MSB + LSB + CRC)
    /// and validates the CRC-8 checksum. Without the feature, reads 2 bytes.
    #[cfg(not(feature = "crc"))]
    fn read_register_pair(&mut self, register: u8) -> Result<[u8; 2], Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c.write_read(self.address, &[register], &mut buf)?;
        Ok(buf)
    }

    /// Read a 4-byte block with CRC-8 validation.
    ///
    /// Reads 5 bytes (4 data + 1 CRC) from the given register address.
    /// CRC covers `[addr_W, register, addr_R, data0, data1, data2, data3]`.
    /// Per TI datasheet SLYS045B §6.5.1.3.3: "the secondary sends the fifth
    /// CRC byte based off the CRC calculation of immediate past four register
    /// bytes."
    #[cfg(feature = "crc")]
    fn read_block_with_crc(&mut self, register: u8) -> Result<[u8; 4], Error<I2C::Error>> {
        let mut buf = [0u8; 5]; // 4 data + 1 CRC
        self.i2c.write_read(self.address, &[register], &mut buf)?;

        let crc_input = [
            self.address << 1,       // address + write bit
            register,                // register address
            (self.address << 1) | 1, // address + read bit
            buf[0],
            buf[1],
            buf[2],
            buf[3],
        ];
        let computed = crc8(&crc_input);
        let expected = buf[4];

        if computed != expected {
            defmt_warn!(
                "CRC mismatch at reg 0x{:02X}: expected 0x{:02X}, computed 0x{:02X}",
                register,
                expected,
                computed,
            );
            return Err(Error::CrcMismatch { expected, computed });
        }

        Ok([buf[0], buf[1], buf[2], buf[3]])
    }

    /// Read a register pair with CRC-8 validation.
    ///
    /// Reads 3 bytes: MSB, LSB, CRC. Validates the CRC over `[address_W, register, address_R, MSB, LSB]`
    /// per TI datasheet Section 7.5.3.2.
    #[cfg(feature = "crc")]
    fn read_register_pair(&mut self, register: u8) -> Result<[u8; 2], Error<I2C::Error>> {
        let mut buf = [0u8; 3]; // MSB + LSB + CRC
        self.i2c.write_read(self.address, &[register], &mut buf)?;

        // CRC is computed over the full I2C transaction bytes:
        // [device_addr << 1 | W, register_addr, device_addr << 1 | R, data_MSB, data_LSB]
        let crc_input = [
            self.address << 1,       // address + write bit
            register,                // register address
            (self.address << 1) | 1, // address + read bit
            buf[0],                  // data MSB
            buf[1],                  // data LSB
        ];
        let computed = crc8(&crc_input);
        let expected = buf[2];

        if computed != expected {
            defmt_warn!(
                "CRC mismatch at reg 0x{:02X}: expected 0x{:02X}, computed 0x{:02X}",
                register,
                expected,
                computed,
            );
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
    use crate::config::{CalibrationConfig, ConfigBuilder, InterruptConfig, ThresholdConfig};
    use crate::types::{
        AngleCalculation, ConversionAverage, MagneticChannel, OperatingMode, PowerMode, Range,
        SleepDuration, TempCoefficient, TriggerMode,
    };
    use crate::types::{
        Degrees, GainChannel, IntPinBehavior, InterruptMode, ThresholdCrossingCount,
        ThresholdDirection,
    };
    use embedded_hal_mock::eh1::delay::{CheckedDelay, Transaction as DelayTransaction};
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    // Compile-time assertion: Config must be Copy because to_register_writes()
    // consumes self, and we pass &Config to init_expectations.
    const _: () = {
        fn _assert_copy<T: Copy>() {}
        fn _check() {
            _assert_copy::<Config>();
        }
    };

    // -- detect() -----------------------------------------------------------

    #[test]
    fn detect_b1_succeeds() {
        let address = 0x22u8;
        let expectations = [
            I2cTransaction::write(address, vec![]),
            I2cTransaction::write_read(address, vec![0x0D], vec![0x01]),
        ];
        let i2c = I2cMock::new(&expectations);

        let sensor = Tmag5273::detect(i2c, address).unwrap();
        sensor.into_inner().done();
    }

    #[test]
    fn detect_b2_succeeds() {
        let address = 0x22u8;
        let expectations = [
            I2cTransaction::write(address, vec![]),
            I2cTransaction::write_read(address, vec![0x0D], vec![0x02]),
        ];
        let i2c = I2cMock::new(&expectations);

        let sensor = Tmag5273::detect(i2c, address).unwrap();
        sensor.into_inner().done();
    }

    #[test]
    fn detect_a1_succeeds() {
        let expectations = [
            I2cTransaction::write(0x35, vec![]),
            I2cTransaction::write_read(0x35, vec![0x0D], vec![0x01]),
        ];
        let i2c = I2cMock::new(&expectations);

        let sensor = Tmag5273::detect(i2c, 0x35).unwrap();
        sensor.into_inner().done();
    }

    #[test]
    fn detect_nack_returns_bus() {
        let address = 0x22u8;
        let expectations = [
            // Wake-up write
            I2cTransaction::write(address, vec![]),
            // DEVICE_ID read NACKs
            I2cTransaction::write_read(address, vec![0x0D], vec![0x00]).with_error(
                embedded_hal::i2c::ErrorKind::NoAcknowledge(
                    embedded_hal::i2c::NoAcknowledgeSource::Address,
                ),
            ),
        ];
        let i2c = I2cMock::new(&expectations);

        let (err, mut bus) = Tmag5273::detect(i2c, address).unwrap_err();
        assert!(matches!(err, Error::I2c(_)));
        bus.done(); // bus recovered, mock expectations met
    }

    #[test]
    fn detect_unknown_version_returns_bus() {
        let address = 0x22u8;
        let expectations = [
            I2cTransaction::write(address, vec![]),
            // Returns version 3 — not a valid TMAG5273 version
            I2cTransaction::write_read(address, vec![0x0D], vec![0x03]),
        ];
        let i2c = I2cMock::new(&expectations);

        let (err, mut bus) = Tmag5273::detect(i2c, address).unwrap_err();
        assert!(matches!(err, Error::InvalidRegisterValue { .. }));
        bus.done();
    }

    #[test]
    fn detect_unknown_address_returns_bus() {
        let address = 0x30u8;
        let expectations = [
            I2cTransaction::write(address, vec![]),
            I2cTransaction::write_read(address, vec![0x0D], vec![0x01]),
        ];
        let i2c = I2cMock::new(&expectations);

        let (err, mut bus) = Tmag5273::detect(i2c, address).unwrap_err();
        assert!(matches!(err, Error::InvalidRegisterValue { .. }));
        bus.done();
    }

    // -- scan() --------------------------------------------------------------

    /// Helper: mock expectations for a NACK at a given address (wake + failed read).
    fn nack_at(address: u8) -> Vec<I2cTransaction> {
        vec![
            I2cTransaction::write(address, vec![]),
            I2cTransaction::write_read(address, vec![0x0D], vec![0x00]).with_error(
                embedded_hal::i2c::ErrorKind::NoAcknowledge(
                    embedded_hal::i2c::NoAcknowledgeSource::Address,
                ),
            ),
        ]
    }

    /// Helper: mock expectations for a successful detect at a given address.
    fn respond_at(address: u8, version: u8) -> Vec<I2cTransaction> {
        vec![
            I2cTransaction::write(address, vec![]),
            I2cTransaction::write_read(address, vec![0x0D], vec![version]),
        ]
    }

    #[test]
    fn scan_finds_first_address() {
        // B-family (0x22) responds first — scan should stop immediately
        let expectations = respond_at(0x22, 0x01);
        let i2c = I2cMock::new(&expectations);

        let sensor = Tmag5273::scan(i2c).unwrap();
        sensor.into_inner().done();
    }

    #[test]
    fn scan_finds_third_address() {
        // B(0x22) NACKs, A(0x35) NACKs, D(0x44) responds
        let mut expectations = nack_at(0x22);
        expectations.extend(nack_at(0x35));
        expectations.extend(respond_at(0x44, 0x02)); // D2
        let i2c = I2cMock::new(&expectations);

        let sensor = Tmag5273::scan(i2c).unwrap();
        sensor.into_inner().done();
    }

    #[test]
    fn scan_all_nack_returns_error() {
        let mut expectations = nack_at(0x22);
        expectations.extend(nack_at(0x35));
        expectations.extend(nack_at(0x44));
        expectations.extend(nack_at(0x78));
        let i2c = I2cMock::new(&expectations);

        let (err, mut bus) = Tmag5273::scan(i2c).unwrap_err();
        assert!(matches!(err, Error::I2c(_)));
        bus.done();
    }

    /// Default config used by most tests: ContinuousMeasure, XYZ, temp on,
    /// High range, no angle, X1 averaging — matches `ConfigBuilder::new()`.
    fn default_config() -> Config {
        ConfigBuilder::new().build().unwrap()
    }

    /// Helper: create mock expectations for a successful init sequence.
    ///
    /// Uses `config.to_register_writes()` to compute register bytes, so the
    /// helper works for any valid `Config` — not just the default.
    fn init_expectations(address: u8, config: &Config) -> Vec<I2cTransaction> {
        let writes = config.to_register_writes();

        vec![
            // Wake-up: empty write (device may NACK — ignored by driver)
            I2cTransaction::write(address, vec![]),
            // Disable interrupts before config (INT_CONFIG_1 = 0x00)
            I2cTransaction::write(address, vec![0x08, 0x00]),
            // Burst read manufacturer ID (0x0E..=0x0F) → [LSB=0x49, MSB=0x54]
            I2cTransaction::write_read(address, vec![0x0E], vec![0x49, 0x54]),
            // Read device ID (0x0D) → version 1 (B1 variant)
            I2cTransaction::write_read(address, vec![0x0D], vec![0x01]),
            // Burst write config registers 0x00-0x03
            // dc1 already includes CRC_EN bit via to_register_writes()
            I2cTransaction::write(
                address,
                vec![0x00, writes[0].1, writes[1].1, writes[2].1, writes[3].1],
            ),
            // Separate write for T_CONFIG (0x07)
            I2cTransaction::write(address, vec![writes[4].0, writes[4].1]),
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
        let expectations = init_expectations(0x22, &default_config());
        // Regression guard: init uses exactly 9 I2C transactions (see DESIGN.md).
        // wake-up(1) + disable INT(1) + mfr ID(1) + version(1) + burst config(1)
        // + T_CONFIG(1) + device status(1) + clear faults(1) + conv status(1) = 9
        assert_eq!(
            expectations.len(),
            9,
            "init requires exactly 9 I2C transactions"
        );
        let i2c = I2cMock::new(&expectations);
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
    #[cfg(not(feature = "crc"))]
    fn read_magnetic_burst() {
        let mut expectations = init_expectations(0x22, &default_config());
        // Burst read X/Y/Z: 6 bytes from register 0x12
        expectations.push(I2cTransaction::write_read(
            0x22,
            vec![0x12],
            vec![0x40, 0x00, 0x20, 0x00, 0x10, 0x00], // X=16384, Y=8192, Z=4096
        ));

        // Regression guard: all 3 axes in a single burst read (see DESIGN.md).
        // init(9) + 1 burst read = 10 total — NOT 3 separate register reads.
        assert_eq!(
            expectations.len(),
            10,
            "read_magnetic: init(9) + 1 burst read for all 3 axes"
        );
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let reading = sensor.read_magnetic().unwrap();
        // Exact: power-of-two raw × power-of-two reciprocal → exact f32
        assert_eq!(reading.x.unwrap(), MilliTesla(-40.0)); // 16384 × 80/32768
        assert_eq!(reading.y.unwrap(), MilliTesla(-20.0)); // 8192 × 80/32768
        assert_eq!(reading.z.unwrap(), MilliTesla(-10.0)); // 4096 × 80/32768

        sensor.into_inner().done();
    }

    #[test]
    fn read_temperature() {
        let mut expectations = init_expectations(0x22, &default_config());
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
            // CRC input: [addr<<1|W=0x44, register=0x10, addr<<1|R=0x45, MSB=0x44, LSB=0x64]
            let crc = super::crc8(&[0x44, 0x10, 0x45, 0x44, 0x64]);
            expectations.push(I2cTransaction::write_read(
                0x22,
                vec![0x10],
                vec![0x44, 0x64, crc],
            ));
        }

        // Regression guard: temperature is a single register-pair read (see DESIGN.md).
        // init(9) + 1 read (2 bytes without CRC, 3 bytes with CRC) = 10 total.
        assert_eq!(
            expectations.len(),
            10,
            "read_temperature: init(9) + 1 register pair read"
        );
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let temp = sensor.read_temperature().unwrap();
        // Exact: raw=17508 is the TADC_T0 reference point → (17508-17508)*k = 0
        assert_eq!(temp, Celsius(25.0));

        sensor.into_inner().done();
    }

    #[test]
    fn wait_for_conversion_timeout() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .operating_mode(OperatingMode::Standby)
            .build()
            .unwrap();

        let mut expectations = init_expectations(address, &config);

        // trigger_conversion: read CONV_STATUS with bit 7 set (0x18 | 0x80 = 0x98)
        expectations.push(I2cTransaction::write_read(address, vec![0x98], vec![0x00]));

        // Poll max_polls times (10 for X1), all return not-ready
        for _ in 0..config.conversion_average.max_polls() {
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
        let mut expectations = init_expectations(address, &default_config());

        // set_thresholds: 3 direct writes + 3 read-modify-writes
        expectations.push(I2cTransaction::write(address, vec![0x04, 0x05])); // X threshold = 5
        expectations.push(I2cTransaction::write(address, vec![0x05, 0xFB])); // Y threshold = -5 (0xFB)
        expectations.push(I2cTransaction::write(address, vec![0x06, 0x0A])); // Z threshold = 10
        // T_CONFIG read-modify-write: preserve T_CH_EN bit (0x01 from init)
        expectations.push(I2cTransaction::write_read(address, vec![0x07], vec![0x01]));
        expectations.push(I2cTransaction::write(address, vec![0x07, 0x07])); // temp=3 << 1 | T_CH_EN=1
        // DEVICE_CONFIG_2 read-modify-write: threshold_hysteresis=SymmetricBand(1) in bits [7:5]
        expectations.push(I2cTransaction::write_read(address, vec![0x01], vec![0x02]));
        expectations.push(I2cTransaction::write(address, vec![0x01, 0x22])); // 0x02 | (1 << 5) = 0x22
        // SENSOR_CONFIG_2 read-modify-write: crossing_count=Four(1) in bit [6], direction=Below(1) in bit [5]
        expectations.push(I2cTransaction::write_read(address, vec![0x03], vec![0x03]));
        expectations.push(I2cTransaction::write(address, vec![0x03, 0x63])); // 0x03 | (1 << 6) | (1 << 5) = 0x63

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        sensor
            .set_thresholds(&ThresholdConfig {
                x: 5,
                y: -5,
                z: 10,
                temperature: 3,
                hysteresis: crate::config::ThresholdHysteresis::SymmetricBand,
                crossing_count: ThresholdCrossingCount::Four,
                direction: ThresholdDirection::Below,
            })
            .unwrap();

        sensor.into_inner().done();
    }

    // -- Phase 2: Interrupts ------------------------------------------------

    #[test]
    fn set_interrupt_writes_correct_byte() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

        // INT_CONFIG_1 full write:
        // RSLT_INT=0 | THRESHOLD_INT=1 | INT_STATE=1 | INT_MODE=001 | MASK_INTB=0 | INT_PINSTATE=0
        // = 0b0110_0100 = 0x64 (matches SparkFun's setupWakeUpAndSleep hardcoded value)
        expectations.push(I2cTransaction::write(address, vec![0x08, 0x64]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        sensor
            .set_interrupt(&InterruptConfig {
                mode: InterruptMode::ThroughInt,
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
        let modes = [
            (InterruptMode::None, 0b000),
            (InterruptMode::ThroughInt, 0b001),
            (InterruptMode::ThroughIntExceptI2cBusy, 0b010),
            (InterruptMode::ThroughScl, 0b011),
            (InterruptMode::ThroughSclExceptI2cBusy, 0b100),
        ];

        for (mode, expected_bits) in modes {
            let address = 0x22u8;
            let mut expectations = init_expectations(address, &default_config());

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

    #[test]
    fn set_interrupt_mask_true_writes_bit_0() {
        // Verifies MASK_INTB lands at bit 0 per datasheet (SLYS045B Table 8-11).
        // With mask=true: 0x64 | 0x01 = 0x65 (bit 0 set).
        // The old bug had MASK_INTB at bit 1, which would produce 0x66.
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

        // INT_CONFIG_1: THRESHOLD_INT=1 | INT_STATE=1 | INT_MODE=001 | MASK_INTB=1
        // = 0b0110_0101 = 0x65
        expectations.push(I2cTransaction::write(address, vec![0x08, 0x65]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        sensor
            .set_interrupt(&InterruptConfig {
                mode: InterruptMode::ThroughInt,
                on_conversion_complete: false,
                on_threshold_crossing: true,
                pin_behavior: IntPinBehavior::Pulse10us,
                mask: true,
            })
            .unwrap();

        sensor.into_inner().done();
    }

    #[test]
    fn get_interrupt_mask_reads_bit_0() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

        // Read INT_CONFIG_1 returning 0x01 (only MASK_INTB set at bit 0)
        expectations.push(I2cTransaction::write_read(address, vec![0x08], vec![0x01]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        assert!(sensor.get_interrupt_mask().unwrap());
        sensor.into_inner().done();
    }

    // -- Phase 3: Gain/Offset Calibration -----------------------------------

    #[test]
    fn set_calibration_writes_correct_registers() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

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
        let mut expectations = init_expectations(address, &default_config());

        expectations.push(I2cTransaction::write(address, vec![0x09, 0x00])); // gain
        expectations.push(I2cTransaction::write(address, vec![0x0A, 0x00])); // offset_1 → 0x0A
        expectations.push(I2cTransaction::write(address, vec![0x0B, 0xFF])); // offset_2 → 0x0B (NOT 0x0A)
        expectations.push(I2cTransaction::write_read(address, vec![0x03], vec![0x03]));
        expectations.push(I2cTransaction::write(address, vec![0x03, 0x03])); // gain_channel=First, preserve bits

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

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
        let mut expectations = init_expectations(old_addr, &default_config());

        // Write I2C_ADDRESS register (0x0C) at OLD address:
        // bits [7:1] = new_addr, bit [0] = 1 (update enable)
        let reg_val = (new_addr << 1) | 0x01;
        expectations.push(I2cTransaction::write(old_addr, vec![0x0C, reg_val]));

        // Verify at NEW address: burst read manufacturer ID [LSB=0x49, MSB=0x54]
        expectations.push(I2cTransaction::write_read(
            new_addr,
            vec![0x0E],
            vec![0x49, 0x54],
        ));

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
        let mut expectations = init_expectations(old_addr, &default_config());

        // Write succeeds at old address
        let reg_val = (new_addr << 1) | 0x01;
        expectations.push(I2cTransaction::write(old_addr, vec![0x0C, reg_val]));

        // Verify at new address: burst read returns wrong manufacturer ID
        expectations.push(I2cTransaction::write_read(
            new_addr,
            vec![0x0E],
            vec![0x00, 0x00],
        ));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let err = sensor.change_address(new_addr).unwrap_err();
        assert_eq!(
            err,
            Error::AddressChangeFailed {
                old_address: old_addr,
                new_address: new_addr,
            }
        );

        sensor.into_inner().done();
    }

    #[test]
    fn change_address_reserved_range_rejected() {
        let address = 0x22u8;
        let i2c = I2cMock::new(&init_expectations(address, &default_config()));
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        // I2C reserved: 0x00-0x07
        let err = sensor.change_address(0x03).unwrap_err();
        assert_eq!(
            err,
            Error::AddressChangeFailed {
                old_address: 0x22,
                new_address: 0x03,
            }
        );

        // I2C reserved: 0x78-0x7F
        let err = sensor.change_address(0x7A).unwrap_err();
        assert_eq!(
            err,
            Error::AddressChangeFailed {
                old_address: 0x22,
                new_address: 0x7A,
            }
        );

        sensor.into_inner().done();
    }

    // -- Runtime CRC toggle ---------------------------------------------------

    #[test]
    #[cfg(feature = "crc")]
    fn set_crc_enabled_writes_bit7() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

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
        let mut expectations = init_expectations(address, &default_config());

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
    #[cfg(not(feature = "crc"))]
    fn set_crc_enabled_true_without_feature_returns_error() {
        let address = 0x22u8;
        // No I2C operations expected — error returned before bus access
        let i2c = I2cMock::new(&init_expectations(address, &default_config()));
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let err = sensor.set_crc_enabled(true).unwrap_err();
        assert_eq!(err, Error::CrcFeatureRequired);

        sensor.into_inner().done();
    }

    #[test]
    fn get_crc_enabled_reads_bit7() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

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
        let mut expectations = init_expectations(address, &default_config());

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
        let mut expectations = init_expectations(address, &default_config());

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
        let mut expectations = init_expectations(address, &default_config());

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
        let mut expectations = init_expectations(address, &default_config());

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
        let mut expectations = init_expectations(address, &default_config());

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
        let mut expectations = init_expectations(address, &default_config());

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
        let mut expectations = init_expectations(address, &default_config());

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
        let mut expectations = init_expectations(address, &default_config());

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
        let mut expectations = init_expectations(address, &default_config());

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
        let mut expectations = init_expectations(address, &default_config());

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

    // -- check_diag (Zephyr-inspired) --------------------------------

    #[test]
    fn check_diag_no_fault() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

        // CONV_STATUS with DIAG_FAIL=0 (bit 1 clear)
        expectations.push(I2cTransaction::write_read(address, vec![0x18], vec![0x00]));
        // No DEVICE_STATUS read should follow.

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let status = sensor.check_diag().unwrap();
        assert!(status.is_ok());
        assert_eq!(status, DeviceStatus::default());

        sensor.into_inner().done();
    }

    #[test]
    fn check_diag_with_fault() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

        // CONV_STATUS with DIAG_FAIL=1 (bit 1 set) → 0x02
        expectations.push(I2cTransaction::write_read(address, vec![0x18], vec![0x02]));
        // Driver should then read DEVICE_STATUS (0x1C) to get fault details.
        // Return VCC undervoltage (bit 0) + oscillator error (bit 3) = 0x09.
        expectations.push(I2cTransaction::write_read(address, vec![0x1C], vec![0x09]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let status = sensor.check_diag().unwrap();
        assert!(!status.is_ok());
        assert!(status.vcc_undervoltage);
        assert!(status.oscillator_error);
        assert!(!status.otp_crc_error);
        assert!(!status.int_error);

        sensor.into_inner().done();
    }

    // -- Diagnostics per-sample tests ----------------------------------------

    /// Helper: build expectations for read_magnetic with diag (7-byte burst: 6 mag + 1 CONV_STATUS).
    #[cfg(not(feature = "crc"))]
    fn diag_read_magnetic_expectations(address: u8, conv_status: u8) -> Vec<I2cTransaction> {
        // 7-byte burst: X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB, CONV_STATUS
        vec![I2cTransaction::write_read(
            address,
            vec![0x12],
            vec![0x40, 0x00, 0x20, 0x00, 0x10, 0x00, conv_status],
        )]
    }

    #[test]
    #[cfg(not(feature = "crc"))]
    fn read_magnetic_diagnostics_halt_returns_error_on_fault() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .diagnostics(Diagnostics::Halt)
            .build()
            .unwrap();
        let mut expectations = init_expectations(address, &config);
        // 7-byte burst with DIAG_FAIL=1 (bit 1 set → 0x02)
        expectations.extend(diag_read_magnetic_expectations(address, 0x02));
        // Halt reads DEVICE_STATUS (0x1C) on fault → VCC undervoltage (0x01)
        expectations.push(I2cTransaction::write_read(address, vec![0x1C], vec![0x01]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        let err = sensor.read_magnetic().unwrap_err();
        match err {
            Error::DiagnosticFailure(status) => {
                assert!(status.vcc_undervoltage);
            }
            other => panic!("expected DiagnosticFailure, got {other:?}"),
        }
        sensor.into_inner().done();
    }

    #[test]
    #[cfg(not(feature = "crc"))]
    fn read_magnetic_diagnostics_halt_succeeds_without_fault() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .diagnostics(Diagnostics::Halt)
            .build()
            .unwrap();
        let mut expectations = init_expectations(address, &config);
        // 7-byte burst with DIAG_FAIL=0
        expectations.extend(diag_read_magnetic_expectations(address, 0x00));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        let mag = sensor.read_magnetic().unwrap();
        assert!(mag.x.is_some());
        sensor.into_inner().done();
    }

    #[test]
    #[cfg(not(feature = "crc"))]
    fn read_magnetic_diagnostics_warn_succeeds_on_fault() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .diagnostics(Diagnostics::Warn)
            .build()
            .unwrap();
        let mut expectations = init_expectations(address, &config);
        // 7-byte burst with DIAG_FAIL=1 (bit 1 → 0x02)
        expectations.extend(diag_read_magnetic_expectations(address, 0x02));
        // Warn+defmt logging is gated with not(test) so no DEVICE_STATUS read in tests.

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        // Warn: read succeeds even with fault
        let mag = sensor.read_magnetic().unwrap();
        assert!(mag.x.is_some());
        sensor.into_inner().done();
    }

    #[test]
    #[cfg(not(feature = "crc"))]
    fn read_magnetic_diagnostics_ignore_succeeds_on_fault() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .diagnostics(Diagnostics::Ignore)
            .build()
            .unwrap();
        let mut expectations = init_expectations(address, &config);
        // 7-byte burst with DIAG_FAIL=1 — Ignore discards, no DEVICE_STATUS read
        expectations.extend(diag_read_magnetic_expectations(address, 0x02));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        // Ignore: read succeeds silently
        let mag = sensor.read_magnetic().unwrap();
        assert!(mag.x.is_some());
        sensor.into_inner().done();
    }

    #[test]
    #[cfg(not(feature = "crc"))]
    fn read_magnetic_diagnostics_off_skips_conv_status_byte() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .diagnostics(Diagnostics::Off)
            .build()
            .unwrap();
        let mut expectations = init_expectations(address, &config);
        // Off: standard 6-byte burst, no CONV_STATUS byte
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x12],
            vec![0x40, 0x00, 0x20, 0x00, 0x10, 0x00],
        ));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        let mag = sensor.read_magnetic().unwrap();
        assert!(mag.x.is_some());
        sensor.into_inner().done();
    }

    // -- Diagnostics: read_all fault path (P2 #4) ---------------------------

    #[test]
    #[cfg(not(feature = "crc"))]
    fn diagnostics_halt_read_all_fault_returns_error() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .diagnostics(Diagnostics::Halt)
            .build()
            .unwrap();
        let mut expectations = init_expectations(address, &config);
        // 9-byte burst from T_MSB_RESULT (0x10): T + X + Y + Z + CONV_STATUS
        // CONV_STATUS with DIAG_FAIL=1 (bit 1 → 0x02) at buf[8]
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x10],
            vec![0x44, 0x64, 0x40, 0x00, 0x20, 0x00, 0x10, 0x00, 0x02],
        ));
        // Halt reads DEVICE_STATUS → VCC undervoltage (0x01)
        expectations.push(I2cTransaction::write_read(address, vec![0x1C], vec![0x01]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        let err = sensor.read_all().unwrap_err();
        match err {
            Error::DiagnosticFailure(status) => assert!(status.vcc_undervoltage),
            other => panic!("expected DiagnosticFailure, got {other:?}"),
        }
        sensor.into_inner().done();
    }

    // -- Diagnostics: read_temperature fault path via read_and_check_diag (P2 #3) --

    #[test]
    #[cfg(not(feature = "crc"))]
    fn diagnostics_halt_read_temperature_fault() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .diagnostics(Diagnostics::Halt)
            .build()
            .unwrap();
        let mut expectations = init_expectations(address, &config);
        // read_register_pair(T_MSB_RESULT) → 2 bytes
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x10],
            vec![0x44, 0x64],
        ));
        // read_and_check_diag → read CONV_STATUS (0x18) with DIAG_FAIL=1
        expectations.push(I2cTransaction::write_read(address, vec![0x18], vec![0x02]));
        // Halt reads DEVICE_STATUS → oscillator error (0x08)
        expectations.push(I2cTransaction::write_read(address, vec![0x1C], vec![0x08]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        let err = sensor.read_temperature().unwrap_err();
        match err {
            Error::DiagnosticFailure(status) => assert!(status.oscillator_error),
            other => panic!("expected DiagnosticFailure, got {other:?}"),
        }
        sensor.into_inner().done();
    }

    #[test]
    #[cfg(not(feature = "crc"))]
    fn diagnostics_warn_read_temperature_fault_succeeds() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .diagnostics(Diagnostics::Warn)
            .build()
            .unwrap();
        let mut expectations = init_expectations(address, &config);
        // read_register_pair(T_MSB_RESULT)
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x10],
            vec![0x44, 0x64],
        ));
        // read_and_check_diag → read CONV_STATUS with DIAG_FAIL=1
        expectations.push(I2cTransaction::write_read(address, vec![0x18], vec![0x02]));
        // Warn+defmt logging is gated with not(test) so no DEVICE_STATUS read in tests.

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        // Warn: temperature read succeeds despite fault
        let temp = sensor.read_temperature().unwrap();
        assert!(temp.0 > 20.0 && temp.0 < 30.0);
        sensor.into_inner().done();
    }

    #[test]
    #[cfg(not(feature = "crc"))]
    fn diagnostics_ignore_read_temperature_fault_succeeds() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .diagnostics(Diagnostics::Ignore)
            .build()
            .unwrap();
        let mut expectations = init_expectations(address, &config);
        // read_register_pair(T_MSB_RESULT)
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x10],
            vec![0x44, 0x64],
        ));
        // read_and_check_diag → read CONV_STATUS with DIAG_FAIL=1
        expectations.push(I2cTransaction::write_read(address, vec![0x18], vec![0x02]));
        // Ignore: no DEVICE_STATUS read

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        // Ignore: read succeeds silently
        let temp = sensor.read_temperature().unwrap();
        assert!(temp.0 > 20.0 && temp.0 < 30.0);
        sensor.into_inner().done();
    }

    // -- Coherent read_all() -------------------------------------------------

    #[test]
    #[cfg(not(feature = "crc"))]
    fn read_all_returns_coherent_data() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

        // 8-byte burst read from T_MSB_RESULT (0x10):
        // T = 17508 (0x4464) → 25.0°C
        // X = 0x4000 (16384), Y = 0x2000 (8192), Z = 0x1000 (4096)
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x10],
            vec![0x44, 0x64, 0x40, 0x00, 0x20, 0x00, 0x10, 0x00],
        ));

        // Regression guard: temp + 3 axes in a single 8-byte burst (see DESIGN.md).
        // init(9) + 1 burst read = 10 total — NOT 4 separate register reads.
        assert_eq!(
            expectations.len(),
            10,
            "read_all: init(9) + 1 burst read for temp + 3 axes"
        );
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let reading = sensor.read_all().unwrap();

        // Exact: raw=17508 is the TADC_T0 reference point
        assert_eq!(reading.temperature.unwrap(), Celsius(25.0));
        // Exact: power-of-two raw × power-of-two reciprocal → exact f32
        assert_eq!(reading.magnetic.x.unwrap(), MilliTesla(-40.0));
        assert_eq!(reading.magnetic.y.unwrap(), MilliTesla(-20.0));
        assert_eq!(reading.magnetic.z.unwrap(), MilliTesla(-10.0));

        sensor.into_inner().done();
    }

    #[test]
    fn read_all_rejects_non_standard_mode() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

        // set_read_mode to OneByte16Bit: read-modify-write on DEVICE_CONFIG_1
        expectations.push(I2cTransaction::write_read(address, vec![0x00], vec![0x00]));
        expectations.push(I2cTransaction::write(address, vec![0x00, 0x01]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        sensor.set_read_mode(I2cReadMode::OneByte16Bit).unwrap();
        let err = sensor.read_all().unwrap_err();
        assert_eq!(err, Error::NonStandardReadMode { mode: 0x01 });

        sensor.into_inner().done();
    }

    // -- read_all() temperature gating --------------------------------------

    #[test]
    #[cfg(not(feature = "crc"))]
    fn read_all_returns_none_temperature_when_disabled() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .temperature_enabled(false)
            .build()
            .unwrap();

        let mut expectations = init_expectations(address, &config);

        // 8-byte burst read — temperature registers contain undefined data
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x10],
            vec![0xFF, 0xFF, 0x40, 0x00, 0x20, 0x00, 0x10, 0x00],
        ));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        let reading = sensor.read_all().unwrap();
        assert!(
            reading.temperature.is_none(),
            "temperature should be None when channel is disabled"
        );
        // Magnetic channels should still be present
        assert!(reading.magnetic.x.is_some());

        sensor.into_inner().done();
    }

    // -- configure_wake_and_sleep() structured writes -----------------------

    #[test]
    fn configure_wake_and_sleep_uses_structured_registers() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

        // INT_CONFIG_1 full write: threshold INT + Pulse10us + ThroughInt mode
        // = 0b0110_0100 = 0x64 (same result as original magic value)
        expectations.push(I2cTransaction::write(address, vec![0x08, 0x64]));

        // DEVICE_CONFIG_2 read-modify-write:
        // Read current value (e.g. 0x02 = ContinuousMeasure mode from init)
        expectations.push(I2cTransaction::write_read(address, vec![0x01], vec![0x02]));
        // Write back with OperatingMode=WakeUpAndSleep (0b11) + THR_HYST=1 (0b001 << 5)
        // 0x02 & ~0b1110_0011 = 0x00, then | (1 << 5) | 0x03 = 0x23
        expectations.push(I2cTransaction::write(address, vec![0x01, 0x23]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        sensor.configure_wake_and_sleep().unwrap();

        sensor.into_inner().done();
    }

    // -- Temperature threshold sign extension --------------------------------

    #[test]
    fn temperature_threshold_negative_round_trip() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

        // Write threshold -5 via set_thresholds, then read it back.
        // -5i8 as u8 = 0xFB, insert into T_CONFIG bits [7:1]:
        //   insert(current, 0xFE, 1, 0xFB) = (0xFB << 1) & 0xFE = 0xF6
        // With T_CH_EN=1 preserved: 0xF6 | 0x01 = 0xF7.
        // Read back: extract(0xF7, 0xFE, 1) = 0x7B = 123.
        // Sign extend 7-bit: bit 6 set (0x40), so 123 | 0x80 = 0xFB as i8 = -5.

        // Return T_CONFIG with threshold=-5 encoded: 0xF7
        expectations.push(I2cTransaction::write_read(address, vec![0x07], vec![0xF7]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let threshold = sensor.get_temperature_threshold().unwrap();
        assert_eq!(threshold, -5, "expected -5, got {threshold}");

        sensor.into_inner().done();
    }

    #[test]
    fn temperature_threshold_positive_value() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

        // Positive threshold = 10. Insert: (10 << 1) & 0xFE = 0x14.
        // With T_CH_EN=1: 0x14 | 0x01 = 0x15.
        // Extract: (0x15 & 0xFE) >> 1 = 0x0A = 10.
        // Bit 6 clear, so no sign extension: result = 10.
        expectations.push(I2cTransaction::write_read(address, vec![0x07], vec![0x15]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = ConfigBuilder::new().build().unwrap();
        let mut sensor = sensor.init(&config).unwrap();

        let threshold = sensor.get_temperature_threshold().unwrap();
        assert_eq!(threshold, 10, "expected 10, got {threshold}");

        sensor.into_inner().done();
    }

    // -- read_temperature() error path --------------------------------------

    #[test]
    fn read_temperature_returns_error_when_disabled() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .temperature_enabled(false)
            .build()
            .unwrap();

        // No I2C read expected — error returned before bus access
        let i2c = I2cMock::new(&init_expectations(address, &config));
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        let err = sensor.read_temperature().unwrap_err();
        assert_eq!(err, Error::TemperatureDisabled);

        sensor.into_inner().done();
    }

    // -- read_magnetic() partial channels -----------------------------------

    #[test]
    #[cfg(not(feature = "crc"))]
    fn read_magnetic_partial_channel_xy() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .magnetic_channels(MagneticChannel::XY)
            .build()
            .unwrap();

        let mut expectations = init_expectations(address, &config);
        // Burst read X/Y/Z: 6 bytes from register 0x12 (same raw bytes)
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x12],
            vec![0x40, 0x00, 0x20, 0x00, 0x10, 0x00],
        ));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        let reading = sensor.read_magnetic().unwrap();
        // Exact: power-of-two raw × power-of-two reciprocal → exact f32
        assert_eq!(reading.x.unwrap(), MilliTesla(-40.0));
        assert_eq!(reading.y.unwrap(), MilliTesla(-20.0));
        assert!(reading.z.is_none(), "Z should be None for XY-only config");

        sensor.into_inner().done();
    }

    #[test]
    #[cfg(not(feature = "crc"))]
    fn read_magnetic_off_returns_all_none() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .magnetic_channels(MagneticChannel::Off)
            .build()
            .unwrap();

        let mut expectations = init_expectations(address, &config);
        // Burst read still happens (6 bytes), raw values don't matter
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x12],
            vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        ));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        let reading = sensor.read_magnetic().unwrap();
        assert!(reading.x.is_none(), "X should be None when channels off");
        assert!(reading.y.is_none(), "Y should be None when channels off");
        assert!(reading.z.is_none(), "Z should be None when channels off");

        sensor.into_inner().done();
    }

    // -- read_angle() -------------------------------------------------------

    #[test]
    fn read_angle_xy_happy_path() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .angle_calculation(AngleCalculation::XY)
            .magnetic_channels(MagneticChannel::XY)
            .build()
            .unwrap();

        let mut expectations = init_expectations(address, &config);
        // Without CRC: 3-byte burst [MSB, LSB, MAGNITUDE] from 0x19
        #[cfg(not(feature = "crc"))]
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x19],
            vec![0x0B, 0x40, 0x2A], // raw_angle=0x0B40 → 180.0°, magnitude=42
        ));
        // With CRC: read_register_pair(0x19) + read_register(0x1B)
        #[cfg(feature = "crc")]
        {
            expectations.push(I2cTransaction::write_read(
                address,
                vec![0x19],
                vec![0x0B, 0x40, 0xA9], // CRC = 0xA9
            ));
            expectations.push(I2cTransaction::write_read(
                address,
                vec![0x1B],
                vec![0x2A], // magnitude = 42
            ));
        }

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        let reading = sensor.read_angle().unwrap();
        // Exact: Degrees::from_raw(0x0B40) = 180.0
        assert_eq!(reading.angle, Degrees(180.0));
        assert_eq!(reading.magnitude, 42);

        sensor.into_inner().done();
    }

    #[test]
    fn read_angle_disabled_returns_error() {
        // Default config has angle=None
        let i2c = I2cMock::new(&init_expectations(0x22, &default_config()));
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = default_config();
        let mut sensor = sensor.init(&config).unwrap();

        // No I2C read expected — error returned before bus access
        let err = sensor.read_angle().unwrap_err();
        assert_eq!(err, Error::AngleRequiresTwoChannels);

        sensor.into_inner().done();
    }

    #[test]
    fn read_angle_non_standard_mode_returns_error() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

        // set_read_mode to OneByte16Bit: read-modify-write on DEVICE_CONFIG_1 (0x00)
        expectations.push(I2cTransaction::write_read(address, vec![0x00], vec![0x00]));
        expectations.push(I2cTransaction::write(address, vec![0x00, 0x01]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = default_config();
        let mut sensor = sensor.init(&config).unwrap();

        sensor.set_read_mode(I2cReadMode::OneByte16Bit).unwrap();
        let err = sensor.read_angle().unwrap_err();
        assert_eq!(err, Error::NonStandardReadMode { mode: 0x01 });

        sensor.into_inner().done();
    }

    // -- set_mode() and sleep() ---------------------------------------------

    #[test]
    fn set_mode_standby() {
        let address = 0x22u8;
        let config = default_config();
        // Default DC2 has ContinuousMeasure (0x02) in bits [1:0]
        let dc2 = config.to_register_writes()[1].1;

        let mut expectations = init_expectations(address, &config);
        // Wake-up write before set_mode
        expectations.push(I2cTransaction::write(address, vec![]));
        // Read DEVICE_CONFIG_2 → current value
        expectations.push(I2cTransaction::write_read(address, vec![0x01], vec![dc2]));
        // Write back with mode bits [1:0] changed to Standby (0x00)
        expectations.push(I2cTransaction::write(address, vec![0x01, dc2 & !0x03]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        sensor.set_mode(OperatingMode::Standby).unwrap();

        sensor.into_inner().done();
    }

    #[test]
    fn sleep_writes_sleep_mode_bits() {
        let address = 0x22u8;
        let config = default_config();
        let dc2 = config.to_register_writes()[1].1;

        let mut expectations = init_expectations(address, &config);
        // Wake-up write before set_mode (via sleep())
        expectations.push(I2cTransaction::write(address, vec![]));
        // Read DEVICE_CONFIG_2 → current value
        expectations.push(I2cTransaction::write_read(address, vec![0x01], vec![dc2]));
        // Write back with mode bits [1:0] = 0x01 (Sleep)
        expectations.push(I2cTransaction::write(
            address,
            vec![0x01, (dc2 & !0x03) | 0x01],
        ));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        sensor.sleep().unwrap();

        sensor.into_inner().done();
    }

    #[test]
    fn set_mode_retries_on_nack_then_succeeds() {
        // Simulates wake-from-sleep: first read_register NACKs, second succeeds.
        let address = 0x22u8;
        let config = default_config();
        let dc2 = config.to_register_writes()[1].1;

        let mut expectations = init_expectations(address, &config);
        // Wake write
        expectations.push(I2cTransaction::write(address, vec![]));
        // First read_register NACKs
        expectations.push(
            I2cTransaction::write_read(address, vec![0x01], vec![dc2]).with_error(
                embedded_hal::i2c::ErrorKind::NoAcknowledge(
                    embedded_hal::i2c::NoAcknowledgeSource::Address,
                ),
            ),
        );
        // Retry: second read_register succeeds
        expectations.push(I2cTransaction::write_read(address, vec![0x01], vec![dc2]));
        // Write-back with Standby mode
        expectations.push(I2cTransaction::write(address, vec![0x01, dc2 & !0x03]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        sensor.set_mode(OperatingMode::Standby).unwrap();
        sensor.into_inner().done();
    }

    #[test]
    fn set_mode_exhausts_retries_returns_error() {
        // All 3 read_register attempts NACK — device unresponsive.
        let address = 0x22u8;
        let config = default_config();
        let dc2 = config.to_register_writes()[1].1;

        let mut expectations = init_expectations(address, &config);
        // Wake write
        expectations.push(I2cTransaction::write(address, vec![]));
        // All 3 attempts NACK
        for _ in 0..3 {
            expectations.push(
                I2cTransaction::write_read(address, vec![0x01], vec![dc2]).with_error(
                    embedded_hal::i2c::ErrorKind::NoAcknowledge(
                        embedded_hal::i2c::NoAcknowledgeSource::Address,
                    ),
                ),
            );
        }

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        let result = sensor.set_mode(OperatingMode::Standby);
        assert!(result.is_err(), "should fail after 3 NACKs");
        sensor.into_inner().done();
    }

    #[test]
    fn set_mode_bus_error_propagates_immediately() {
        // A bus fault (not NACK) should fail on the first attempt — no retry.
        let address = 0x22u8;
        let config = default_config();
        let dc2 = config.to_register_writes()[1].1;

        let mut expectations = init_expectations(address, &config);
        // Wake write
        expectations.push(I2cTransaction::write(address, vec![]));
        // First read_register returns Bus error — should NOT retry
        expectations.push(
            I2cTransaction::write_read(address, vec![0x01], vec![dc2])
                .with_error(embedded_hal::i2c::ErrorKind::Bus),
        );
        // No further I2C transactions expected — retry is skipped

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        let result = sensor.set_mode(OperatingMode::Standby);
        assert!(result.is_err(), "bus error should propagate immediately");
        sensor.into_inner().done();
    }

    // -- Conversion lifecycle -----------------------------------------------

    #[test]
    fn wait_for_conversion_success_first_poll() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .operating_mode(OperatingMode::Standby)
            .build()
            .unwrap();

        let mut expectations = init_expectations(address, &config);
        // trigger_conversion: read CONV_STATUS with start bit (0x18 | 0x80 = 0x98)
        expectations.push(I2cTransaction::write_read(address, vec![0x98], vec![0x00]));
        // First poll of CONV_STATUS returns result_ready (bit 0 set)
        expectations.push(I2cTransaction::write_read(address, vec![0x18], vec![0x01]));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        sensor.trigger_conversion().unwrap();
        let status = sensor.wait_for_conversion().unwrap();
        assert!(status.result_ready);

        sensor.into_inner().done();
    }

    #[test]
    #[cfg(not(feature = "crc"))]
    fn full_lifecycle_trigger_wait_read_all() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .operating_mode(OperatingMode::Standby)
            .magnetic_channels(MagneticChannel::XYZ)
            .temperature_enabled(true)
            .build()
            .unwrap();

        let mut expectations = init_expectations(address, &config);
        // trigger_conversion: read 0x98 → 0x00
        expectations.push(I2cTransaction::write_read(address, vec![0x98], vec![0x00]));
        // wait_for_conversion: poll 0x18 → ready on first poll
        expectations.push(I2cTransaction::write_read(address, vec![0x18], vec![0x01]));
        // read_all: 8-byte burst from T_MSB_RESULT (0x10)
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x10],
            vec![0x44, 0x64, 0x40, 0x00, 0x20, 0x00, 0x10, 0x00],
        ));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        sensor.trigger_conversion().unwrap();
        let status = sensor.wait_for_conversion().unwrap();
        assert!(status.result_ready);

        let reading = sensor.read_all().unwrap();
        // Exact: raw=17508 is the TADC_T0 reference point
        assert_eq!(reading.temperature.unwrap(), Celsius(25.0));
        // Exact: power-of-two raw × power-of-two reciprocal → exact f32
        assert_eq!(reading.magnetic.x.unwrap(), MilliTesla(-40.0));
        assert_eq!(reading.magnetic.y.unwrap(), MilliTesla(-20.0));
        assert_eq!(reading.magnetic.z.unwrap(), MilliTesla(-10.0));

        sensor.into_inner().done();
    }

    // -- CRC feature-gated tests --------------------------------------------

    #[cfg(feature = "crc")]
    #[test]
    fn crc_read_angle_validates_checksum() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .angle_calculation(AngleCalculation::XY)
            .magnetic_channels(MagneticChannel::XY)
            .build()
            .unwrap();

        let mut expectations = init_expectations(address, &config);
        // CRC path: read_register_pair(0x19) → 3 bytes [MSB, LSB, CRC]
        // CRC input: [addr<<1|W=0x44, register=0x19, addr<<1|R=0x45, MSB=0x0B, LSB=0x40]
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x19],
            vec![0x0B, 0x40, 0xA9], // CRC = 0xA9
        ));
        // read_register(MAGNITUDE_RESULT=0x1B) → 1 byte
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x1B],
            vec![0x2A], // magnitude = 42
        ));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&config).unwrap();

        let reading = sensor.read_angle().unwrap();
        // Exact: 0x0B40 → Degrees(180.0)
        assert_eq!(reading.angle, Degrees(180.0));
        assert_eq!(reading.magnitude, 42);

        sensor.into_inner().done();
    }

    #[cfg(feature = "crc")]
    #[test]
    fn crc_mismatch_returns_error() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

        // read_temperature → read_register_pair(0x10) with wrong CRC
        // Correct CRC for [0x44, 0x10, 0x45, 0x44, 0x64] is 0x6B
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x10],
            vec![0x44, 0x64, 0xFF], // 0xFF is wrong CRC
        ));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let config = default_config();
        let mut sensor = sensor.init(&config).unwrap();

        let err = sensor.read_temperature().unwrap_err();
        assert_eq!(
            err,
            Error::CrcMismatch {
                expected: 0xFF,
                computed: 0x6B,
            }
        );

        sensor.into_inner().done();
    }

    #[cfg(feature = "crc")]
    #[test]
    fn crc_read_all_validates_both_blocks() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

        // Block 1: T_MSB(0x10) → [T_MSB=0x44, T_LSB=0x64, X_MSB=0x10, X_LSB=0x00, CRC]
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x10],
            vec![0x44, 0x64, 0x10, 0x00, 0x35],
        ));
        // Block 2: 0x14 → [Y_MSB=0x20, Y_LSB=0x00, Z_MSB=0x30, Z_LSB=0x00, CRC]
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x14],
            vec![0x20, 0x00, 0x30, 0x00, 0x5C],
        ));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&default_config()).unwrap();

        let reading = sensor.read_all().unwrap();
        assert!(reading.temperature.is_some());
        assert!(reading.magnetic.x.is_some());
        assert!(reading.magnetic.y.is_some());
        assert!(reading.magnetic.z.is_some());
        sensor.into_inner().done();
    }

    #[cfg(feature = "crc")]
    #[test]
    fn crc_read_magnetic_validates_both_blocks() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

        // Block 1: T_MSB(0x10) → [T_MSB, T_LSB, X_MSB, X_LSB, CRC]
        // Temperature bytes are discarded by read_magnetic
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x10],
            vec![0x44, 0x64, 0x10, 0x00, 0x35],
        ));
        // Block 2: 0x14 → [Y_MSB, Y_LSB, Z_MSB, Z_LSB, CRC]
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x14],
            vec![0x20, 0x00, 0x30, 0x00, 0x5C],
        ));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&default_config()).unwrap();

        let reading = sensor.read_magnetic().unwrap();
        assert!(reading.x.is_some());
        assert!(reading.y.is_some());
        assert!(reading.z.is_some());
        sensor.into_inner().done();
    }

    #[cfg(feature = "crc")]
    #[test]
    fn crc_read_all_block2_mismatch_returns_error() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());

        // Block 1: valid CRC
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x10],
            vec![0x44, 0x64, 0x10, 0x00, 0x35],
        ));
        // Block 2: wrong CRC (0xFF instead of 0x5C)
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x14],
            vec![0x20, 0x00, 0x30, 0x00, 0xFF],
        ));

        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&default_config()).unwrap();

        let err = sensor.read_all().unwrap_err();
        assert!(matches!(err, Error::CrcMismatch { .. }));
        sensor.into_inner().done();
    }

    // -- Non-default init: X32 averaging timeout ----------------------------

    #[test]
    fn init_x32_averaging_timeout() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .conversion_average(ConversionAverage::X32)
            .operating_mode(OperatingMode::Standby)
            .build()
            .unwrap();

        let mut expectations = init_expectations(address, &config);
        // trigger_conversion: read 0x98
        expectations.push(I2cTransaction::write_read(address, vec![0x98], vec![0x00]));
        // 100 polls (max_polls for X32), all returning not-ready (0x00)
        for _ in 0..config.conversion_average.max_polls() {
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

    // -- CheckedDelay timing verification -----------------------------------

    #[test]
    fn init_with_checked_delay_calls_wake_delay() {
        let i2c = I2cMock::new(&init_expectations(0x22, &default_config()));
        let delay = CheckedDelay::new(&[DelayTransaction::delay_us(80)]);

        let sensor = Tmag5273::new_with_delay(i2c, DeviceVariant::B1, delay);
        let config = ConfigBuilder::new().build().unwrap();
        let sensor = sensor.init(&config).unwrap();

        let (mut i2c, mut delay) = sensor.into_parts();
        i2c.done();
        delay.done();
    }

    #[test]
    fn set_mode_with_checked_delay_wakes_on_active() {
        let address = 0x22u8;
        let config = default_config();
        let dc2 = config.to_register_writes()[1].1;

        let mut i2c_expectations = init_expectations(address, &config);
        // Wake-up write before set_mode
        i2c_expectations.push(I2cTransaction::write(address, vec![]));
        // set_mode(Standby): read DEVICE_CONFIG_2 → current value
        i2c_expectations.push(I2cTransaction::write_read(address, vec![0x01], vec![dc2]));
        // Write back with mode bits [1:0] = Standby (0x00)
        i2c_expectations.push(I2cTransaction::write(address, vec![0x01, dc2 & !0x03]));

        let i2c = I2cMock::new(&i2c_expectations);
        // Three wake delays: init(80), set_mode pre-read(80), set_mode post-write(80)
        let delay = CheckedDelay::new(&[
            DelayTransaction::delay_us(80),
            DelayTransaction::delay_us(80),
            DelayTransaction::delay_us(80),
        ]);

        let sensor = Tmag5273::new_with_delay(i2c, DeviceVariant::B1, delay);
        let mut sensor = sensor.init(&config).unwrap();

        sensor.set_mode(OperatingMode::Standby).unwrap();

        let (mut i2c, mut delay) = sensor.into_parts();
        i2c.done();
        delay.done();
    }

    #[test]
    fn sleep_with_checked_delay_skips_post_wake() {
        let address = 0x22u8;
        let config = default_config();
        let dc2 = config.to_register_writes()[1].1;

        let mut i2c_expectations = init_expectations(address, &config);
        // Wake-up write before set_mode (via sleep())
        i2c_expectations.push(I2cTransaction::write(address, vec![]));
        // sleep() → set_mode(Sleep): read DEVICE_CONFIG_2
        i2c_expectations.push(I2cTransaction::write_read(address, vec![0x01], vec![dc2]));
        // Write back with mode bits [1:0] = Sleep (0x01)
        i2c_expectations.push(I2cTransaction::write(
            address,
            vec![0x01, (dc2 & !0x03) | 0x01],
        ));

        let i2c = I2cMock::new(&i2c_expectations);
        // Two delays: init(80) + set_mode pre-read(80). No post-write delay for sleep.
        let delay = CheckedDelay::new(&[
            DelayTransaction::delay_us(80),
            DelayTransaction::delay_us(80),
        ]);

        let sensor = Tmag5273::new_with_delay(i2c, DeviceVariant::B1, delay);
        let mut sensor = sensor.init(&config).unwrap();

        sensor.sleep().unwrap();

        let (mut i2c, mut delay) = sensor.into_parts();
        i2c.done();
        delay.done();
    }

    #[test]
    fn wait_for_conversion_with_checked_delay() {
        let address = 0x22u8;
        let config = ConfigBuilder::new()
            .operating_mode(OperatingMode::Standby)
            .build()
            .unwrap();

        let mut i2c_expectations = init_expectations(address, &config);
        // trigger_conversion: read CONV_STATUS with start bit (0x98)
        i2c_expectations.push(I2cTransaction::write_read(address, vec![0x98], vec![0x00]));
        // wait_for_conversion: first poll returns ready (bit 0 set)
        i2c_expectations.push(I2cTransaction::write_read(address, vec![0x18], vec![0x01]));

        let i2c = I2cMock::new(&i2c_expectations);
        // Two delays: 80 µs wake from init, 125 µs conversion time
        // (X1 averaging, XYZ channels, temp enabled = (1×25)×4 + 25 = 125 µs)
        let delay = CheckedDelay::new(&[
            DelayTransaction::delay_us(80),
            DelayTransaction::delay_us(125),
        ]);

        let sensor = Tmag5273::new_with_delay(i2c, DeviceVariant::B1, delay);
        let mut sensor = sensor.init(&config).unwrap();

        sensor.trigger_conversion().unwrap();
        let status = sensor.wait_for_conversion().unwrap();
        assert!(status.result_ready);

        let (mut i2c, mut delay) = sensor.into_parts();
        i2c.done();
        delay.done();
    }

    // -- coverage: getter round-trips ----------------------------------------
    // Each test inits a sensor with default config, mocks one register read,
    // and verifies the getter returns the expected decoded value.

    macro_rules! getter_test {
        ($name:ident, $getter:ident, $reg:expr, $raw:expr, $expected:expr) => {
            #[test]
            fn $name() {
                let address = 0x22u8;
                let mut expectations = init_expectations(address, &default_config());
                expectations.push(I2cTransaction::write_read(address, vec![$reg], vec![$raw]));
                let i2c = I2cMock::new(&expectations);
                let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
                let mut sensor = sensor.init(&default_config()).unwrap();
                assert_eq!(sensor.$getter().unwrap(), $expected);
                sensor.into_inner().done();
            }
        };
    }

    // DEVICE_CONFIG_1 (0x00) getters
    getter_test!(
        get_temp_coefficient_reads_ndbfe,
        get_temp_coefficient,
        0x00,
        0b0010_0000, // MAG_TEMPCO = 1 (NdBFe)
        TempCoefficient::NdBFe
    );

    getter_test!(
        get_conversion_average_reads_x16,
        get_conversion_average,
        0x00,
        0b0001_0000, // CONV_AVG = 0b100 (X16)
        ConversionAverage::X16
    );

    // DEVICE_CONFIG_2 (0x01) getters
    getter_test!(
        get_power_mode_reads_low_noise,
        get_power_mode,
        0x01,
        0b0001_0000, // LP_LN = 1
        PowerMode::LowNoise
    );

    getter_test!(
        get_trigger_mode_reads_int_pin,
        get_trigger_mode,
        0x01,
        0b0000_0100, // TRIGGER_MODE = 1
        TriggerMode::IntPin
    );

    #[test]
    fn get_glitch_filter_enabled_when_bit_clear() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());
        // I2C_GLITCH_FILTER bit 3 = 0 means filter ENABLED
        expectations.push(I2cTransaction::write_read(address, vec![0x01], vec![0x00]));
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&default_config()).unwrap();
        assert!(sensor.get_glitch_filter().unwrap());
        sensor.into_inner().done();
    }

    #[test]
    fn get_glitch_filter_disabled_when_bit_set() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());
        // I2C_GLITCH_FILTER bit 3 = 1 means filter DISABLED
        expectations.push(I2cTransaction::write_read(address, vec![0x01], vec![0x08]));
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&default_config()).unwrap();
        assert!(!sensor.get_glitch_filter().unwrap());
        sensor.into_inner().done();
    }

    #[test]
    fn get_threshold_hysteresis_raw_reads_field() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());
        // THR_HYST = 0b001 (1 = SymmetricBand) in bits [7:5]
        expectations.push(I2cTransaction::write_read(
            address,
            vec![0x01],
            vec![0b0010_0000],
        ));
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&default_config()).unwrap();
        assert_eq!(sensor.get_threshold_hysteresis_raw().unwrap(), 1);
        sensor.into_inner().done();
    }

    // SENSOR_CONFIG_1 (0x02) getters
    getter_test!(
        get_sleep_duration_reads_ms10,
        get_sleep_duration,
        0x02,
        0b0000_0010, // SLEEPTIME = 0x2 (Ms10)
        SleepDuration::Ms10
    );

    // SENSOR_CONFIG_2 (0x03) getters
    getter_test!(
        get_angle_calculation_reads_xy,
        get_angle_calculation,
        0x03,
        0b0000_0100, // ANGLE_EN = 1 (XY)
        AngleCalculation::XY
    );

    getter_test!(
        get_xy_range_reads_low,
        get_xy_range,
        0x03,
        0b0000_0000, // X_Y_RANGE = 0 (Low)
        Range::Low
    );

    getter_test!(
        get_z_range_reads_high,
        get_z_range,
        0x03,
        0b0000_0001, // Z_RANGE = 1 (High)
        Range::High
    );

    getter_test!(
        get_threshold_direction_reads_below,
        get_threshold_direction,
        0x03,
        0b0010_0000, // MAG_THR_DIR = 1 (Below)
        ThresholdDirection::Below
    );

    getter_test!(
        get_gain_channel_reads_second,
        get_gain_channel,
        0x03,
        0b0001_0000, // MAG_GAIN_CH = 1 (Second)
        GainChannel::Second
    );

    // T_CONFIG (0x07) getters
    #[test]
    fn get_temperature_enabled_true() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());
        expectations.push(I2cTransaction::write_read(address, vec![0x07], vec![0x01]));
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&default_config()).unwrap();
        assert!(sensor.get_temperature_enabled().unwrap());
        sensor.into_inner().done();
    }

    #[test]
    fn get_temperature_enabled_false() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());
        expectations.push(I2cTransaction::write_read(address, vec![0x07], vec![0x00]));
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&default_config()).unwrap();
        assert!(!sensor.get_temperature_enabled().unwrap());
        sensor.into_inner().done();
    }

    // INT_CONFIG_1 (0x08) getters
    getter_test!(
        get_interrupt_mode_reads_int_only,
        get_interrupt_mode,
        0x08,
        0b0000_0100, // INT_MODE = 1 (ThroughInt)
        InterruptMode::ThroughInt
    );

    #[test]
    fn get_interrupt_on_conversion_true() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());
        // RSLT_INT bit 7 = 1
        expectations.push(I2cTransaction::write_read(address, vec![0x08], vec![0x80]));
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&default_config()).unwrap();
        assert!(sensor.get_interrupt_on_conversion().unwrap());
        sensor.into_inner().done();
    }

    #[test]
    fn get_interrupt_on_threshold_true() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());
        // THRESHOLD_INT bit 6 = 1
        expectations.push(I2cTransaction::write_read(address, vec![0x08], vec![0x40]));
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&default_config()).unwrap();
        assert!(sensor.get_interrupt_on_threshold().unwrap());
        sensor.into_inner().done();
    }

    #[test]
    fn get_interrupt_pin_state_pulsed() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());
        // INT_STATE bit 5 = 1 (pulsed)
        expectations.push(I2cTransaction::write_read(address, vec![0x08], vec![0x20]));
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&default_config()).unwrap();
        assert!(sensor.get_interrupt_pin_state().unwrap());
        sensor.into_inner().done();
    }

    // Threshold getters (0x04, 0x05, 0x06)
    #[test]
    fn get_xyz_thresholds_signed() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());
        expectations.push(I2cTransaction::write_read(address, vec![0x04], vec![0x7F]));
        expectations.push(I2cTransaction::write_read(address, vec![0x05], vec![0x80])); // -128
        expectations.push(I2cTransaction::write_read(address, vec![0x06], vec![0x00]));
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&default_config()).unwrap();
        assert_eq!(sensor.get_x_threshold().unwrap(), 127);
        assert_eq!(sensor.get_y_threshold().unwrap(), -128);
        assert_eq!(sensor.get_z_threshold().unwrap(), 0);
        sensor.into_inner().done();
    }

    // Gain/offset getters (0x09, 0x0A, 0x0B)
    #[test]
    fn get_magnetic_gain_and_offsets() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());
        expectations.push(I2cTransaction::write_read(address, vec![0x09], vec![0xAB]));
        expectations.push(I2cTransaction::write_read(address, vec![0x0A], vec![0xFF])); // -1
        expectations.push(I2cTransaction::write_read(address, vec![0x0B], vec![0x55]));
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&default_config()).unwrap();
        assert_eq!(sensor.get_magnetic_gain().unwrap(), 0xAB);
        assert_eq!(sensor.get_magnetic_offset_1().unwrap(), -1);
        assert_eq!(sensor.get_magnetic_offset_2().unwrap(), 0x55);
        sensor.into_inner().done();
    }

    // is_conversion_complete
    #[test]
    fn is_conversion_complete_returns_true_when_set() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());
        // CONV_STATUS with RESULT_STATUS bit set
        expectations.push(I2cTransaction::write_read(address, vec![0x18], vec![0x01]));
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&default_config()).unwrap();
        assert!(sensor.is_conversion_complete().unwrap());
        sensor.into_inner().done();
    }

    #[test]
    fn is_conversion_complete_returns_false_when_clear() {
        let address = 0x22u8;
        let mut expectations = init_expectations(address, &default_config());
        expectations.push(I2cTransaction::write_read(address, vec![0x18], vec![0x00]));
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let mut sensor = sensor.init(&default_config()).unwrap();
        assert!(!sensor.is_conversion_complete().unwrap());
        sensor.into_inner().done();
    }

    // conversion_time
    #[test]
    fn conversion_time_returns_expected_micros() {
        let address = 0x22u8;
        let expectations = init_expectations(address, &default_config());
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let sensor = sensor.init(&default_config()).unwrap();
        let ct = sensor.conversion_time();
        // Default: X1 averaging, XYZ+temp → should be > 0
        assert!(u32::from(ct) > 0);
        sensor.into_inner().done();
    }

    // Debug impl
    #[test]
    fn debug_impl_contains_variant_and_address() {
        let address = 0x22u8;
        let expectations = init_expectations(address, &default_config());
        let i2c = I2cMock::new(&expectations);
        let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
        let sensor = sensor.init(&default_config()).unwrap();
        let debug = alloc::format!("{:?}", sensor);
        assert!(debug.contains("Tmag5273"));
        assert!(debug.contains("B1"));
        sensor.into_inner().done();
    }

    // ConfigBuilder::default()
    #[test]
    fn config_builder_default_equals_new() {
        let from_default = ConfigBuilder::default().build().unwrap();
        let from_new = ConfigBuilder::new().build().unwrap();
        assert_eq!(
            from_default.to_register_writes(),
            from_new.to_register_writes()
        );
    }

    // ConfigBuilder setter coverage — exercise setters that aren't used elsewhere
    #[test]
    fn config_builder_all_setters() {
        let config = ConfigBuilder::new()
            .operating_mode(OperatingMode::Standby)
            .xy_range(Range::Low)
            .z_range(Range::Low)
            .power_mode(PowerMode::LowNoise)
            .trigger_mode(TriggerMode::IntPin)
            .glitch_filter(false)
            .temp_coefficient(TempCoefficient::Ceramic)
            .diagnostics(crate::Diagnostics::Warn)
            .build()
            .unwrap();
        // Just verify it builds without error — the register values are
        // tested by the getter round-trip tests above.
        let _ = config.to_register_writes();
    }
}
