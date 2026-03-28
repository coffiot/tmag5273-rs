//! Configuration builder and enums for the TMAG5273 sensor.
//!
//! Use [`ConfigBuilder`] to construct a validated [`Config`], then pass it to
//! [`Tmag5273::init`](crate::Tmag5273).

use crate::error::ConfigError;
use crate::register;
use crate::types::{
    AngleCalculation, ConversionAverage, Diagnostics, GainChannel, I2cReadMode, IntPinBehavior,
    InterruptMode, MagneticChannel, Micros, OperatingMode, PowerMode, Range, SleepDuration,
    TempCoefficient, ThresholdCrossingCount, ThresholdDirection, TriggerMode,
};

/// Validated device configuration for the TMAG5273.
///
/// Construct via [`ConfigBuilder`]. The `#[non_exhaustive]` attribute allows
/// pub field reads but prevents construction outside this crate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[must_use]
#[non_exhaustive]
pub struct Config {
    /// Operating mode.
    pub operating_mode: OperatingMode,
    /// Magnetic channel selection.
    pub magnetic_channels: MagneticChannel,
    /// Whether the temperature channel is enabled.
    pub temperature_enabled: bool,
    /// Angle calculation axis pair.
    pub angle_calculation: AngleCalculation,
    /// Conversion averaging.
    pub conversion_average: ConversionAverage,
    /// X/Y axis measurement range.
    pub xy_range: Range,
    /// Z axis measurement range.
    pub z_range: Range,
    /// Power mode.
    pub power_mode: PowerMode,
    /// Sleep duration (used in wake-up-and-sleep mode).
    pub sleep_duration: SleepDuration,
    /// Trigger mode.
    pub trigger_mode: TriggerMode,
    /// Whether the I2C glitch filter is enabled.
    pub glitch_filter: bool,
    /// Magnetic temperature compensation coefficient.
    pub temp_coefficient: TempCoefficient,
    /// Per-sample diagnostic checking policy.
    ///
    /// Controls whether each `read_magnetic`, `read_temperature`, `read_all`,
    /// and `read_angle` call checks `CONV_STATUS.DIAG_FAIL`, and what
    /// happens when a fault is detected.
    ///
    /// Default: [`Diagnostics::Off`] (no overhead — existing behavior unchanged).
    pub diagnostics: Diagnostics,
}

// ---------------------------------------------------------------------------
// ConfigBuilder
// ---------------------------------------------------------------------------

/// Builder for validated [`Config`].
///
/// Defaults match the SparkFun Arduino `begin()` production configuration:
/// continuous mode, XYZ channels, temperature enabled, high range (80 mT),
/// no angle calculation.
///
/// # Examples
///
/// ```
/// use tmag5273::ConfigBuilder;
///
/// let config = ConfigBuilder::new().build().unwrap();
/// assert_eq!(config.magnetic_channels, tmag5273::MagneticChannel::XYZ);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct ConfigBuilder {
    operating_mode: OperatingMode,
    magnetic_channels: MagneticChannel,
    temperature_enabled: bool,
    angle_calculation: AngleCalculation,
    conversion_average: ConversionAverage,
    xy_range: Range,
    z_range: Range,
    power_mode: PowerMode,
    sleep_duration: SleepDuration,
    trigger_mode: TriggerMode,
    glitch_filter: bool,
    temp_coefficient: TempCoefficient,
    diagnostics: Diagnostics,
}

impl Default for ConfigBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl ConfigBuilder {
    /// Create a new builder with defaults matching the SparkFun `begin()`
    /// production configuration.
    pub const fn new() -> Self {
        Self {
            operating_mode: OperatingMode::ContinuousMeasure,
            magnetic_channels: MagneticChannel::XYZ,
            temperature_enabled: true,
            angle_calculation: AngleCalculation::None,
            conversion_average: ConversionAverage::X1,
            xy_range: Range::High,
            z_range: Range::High,
            power_mode: PowerMode::LowActiveCurrent,
            sleep_duration: SleepDuration::Ms1,
            trigger_mode: TriggerMode::I2cCommand,
            glitch_filter: true,
            temp_coefficient: TempCoefficient::None,
            diagnostics: Diagnostics::Off,
        }
    }

    /// Set the operating mode.
    pub const fn operating_mode(mut self, mode: OperatingMode) -> Self {
        self.operating_mode = mode;
        self
    }

    /// Set the magnetic channel selection.
    pub const fn magnetic_channels(mut self, channels: MagneticChannel) -> Self {
        self.magnetic_channels = channels;
        self
    }

    /// Enable or disable the temperature channel.
    pub const fn temperature_enabled(mut self, enabled: bool) -> Self {
        self.temperature_enabled = enabled;
        self
    }

    /// Set the angle calculation axis pair.
    pub const fn angle_calculation(mut self, angle: AngleCalculation) -> Self {
        self.angle_calculation = angle;
        self
    }

    /// Set the conversion averaging.
    pub const fn conversion_average(mut self, avg: ConversionAverage) -> Self {
        self.conversion_average = avg;
        self
    }

    /// Set the X/Y axis measurement range.
    pub const fn xy_range(mut self, range: Range) -> Self {
        self.xy_range = range;
        self
    }

    /// Set the Z axis measurement range.
    pub const fn z_range(mut self, range: Range) -> Self {
        self.z_range = range;
        self
    }

    /// Set the power mode.
    pub const fn power_mode(mut self, mode: PowerMode) -> Self {
        self.power_mode = mode;
        self
    }

    /// Set the sleep duration (for wake-up-and-sleep mode).
    pub const fn sleep_duration(mut self, dur: SleepDuration) -> Self {
        self.sleep_duration = dur;
        self
    }

    /// Set the trigger mode.
    pub const fn trigger_mode(mut self, mode: TriggerMode) -> Self {
        self.trigger_mode = mode;
        self
    }

    /// Enable or disable the I2C glitch filter.
    pub const fn glitch_filter(mut self, enabled: bool) -> Self {
        self.glitch_filter = enabled;
        self
    }

    /// Set the temperature compensation coefficient.
    pub const fn temp_coefficient(mut self, coeff: TempCoefficient) -> Self {
        self.temp_coefficient = coeff;
        self
    }

    /// Set the per-sample diagnostic checking policy.
    ///
    /// Controls whether each `read_*` method checks `CONV_STATUS.DIAG_FAIL`
    /// and what happens when a fault is detected. See [`Diagnostics`] for
    /// details on each variant.
    ///
    /// Default: [`Diagnostics::Off`].
    pub const fn diagnostics(mut self, policy: Diagnostics) -> Self {
        self.diagnostics = policy;
        self
    }

    /// Validate and build the [`Config`].
    ///
    /// # Errors
    ///
    /// Returns [`ConfigError::AngleRequiresTwoChannels`]
    /// if angle calculation is enabled with fewer than two magnetic channels.
    ///
    /// Returns [`ConfigError::IntPinTriggerRequiresStandby`]
    /// if `TriggerMode::IntPin` is used with any mode other than
    /// `OperatingMode::Standby`.
    ///
    /// Returns [`ConfigError::SleepShorterThanConversion`]
    /// if the sleep duration is shorter than the conversion time for the
    /// configured averaging and channels.
    pub fn build(self) -> Result<Config, ConfigError> {
        // Angle calculation requires at least two axes.
        if !matches!(self.angle_calculation, AngleCalculation::None)
            && self.magnetic_channels.axis_count() < 2
        {
            return Err(ConfigError::AngleRequiresTwoChannels);
        }

        // INT-pin trigger is only meaningful in standby mode.
        if matches!(self.trigger_mode, TriggerMode::IntPin)
            && !matches!(self.operating_mode, OperatingMode::Standby)
        {
            return Err(ConfigError::IntPinTriggerRequiresStandby);
        }

        // Sleep must be long enough for at least one full conversion.
        if matches!(self.operating_mode, OperatingMode::WakeUpAndSleep) {
            let sleep = self.sleep_duration.as_micros();
            let conversion = self
                .conversion_average
                .conversion_time(self.magnetic_channels, self.temperature_enabled);
            if sleep.0 < conversion.0 {
                return Err(ConfigError::SleepShorterThanConversion { sleep, conversion });
            }
        }

        Ok(Config {
            operating_mode: self.operating_mode,
            magnetic_channels: self.magnetic_channels,
            temperature_enabled: self.temperature_enabled,
            angle_calculation: self.angle_calculation,
            conversion_average: self.conversion_average,
            xy_range: self.xy_range,
            z_range: self.z_range,
            power_mode: self.power_mode,
            sleep_duration: self.sleep_duration,
            trigger_mode: self.trigger_mode,
            glitch_filter: self.glitch_filter,
            temp_coefficient: self.temp_coefficient,
            diagnostics: self.diagnostics,
        })
    }
}

// ---------------------------------------------------------------------------
// Config → register serialization
// ---------------------------------------------------------------------------

impl Config {
    /// Returns the expected conversion time in microseconds for this
    /// configuration.
    #[must_use]
    pub const fn conversion_time(&self) -> Micros {
        self.conversion_average
            .conversion_time(self.magnetic_channels, self.temperature_enabled)
    }

    /// Serialize this configuration into register address/value pairs for
    /// writing to the device.
    pub(crate) fn to_register_writes(self) -> [(u8, u8); 5] {
        // DEVICE_CONFIG_1 (0x00)
        //
        // CRC_EN (bit 7): set when the `crc` Cargo feature is active so the
        // sensor sends a CRC byte on every 2-byte read. Cleared otherwise to
        // avoid the extra byte that the non-CRC read path does not expect.
        //
        // I2C_RD (bits 1:0): always Standard (0x00) — the driver's high-level
        // read methods rely on standard 3-byte I2C responses.
        #[cfg(feature = "crc")]
        let crc_en: u8 = 1;
        #[cfg(not(feature = "crc"))]
        let crc_en: u8 = 0;

        let dc1 = register::insert(0, register::CRC_EN_MASK, register::CRC_EN_SHIFT, crc_en)
            | register::insert(
                0,
                register::MAG_TEMPCO_MASK,
                register::MAG_TEMPCO_SHIFT,
                self.temp_coefficient as u8,
            )
            | register::insert(
                0,
                register::CONV_AVG_MASK,
                register::CONV_AVG_SHIFT,
                self.conversion_average as u8,
            )
            | register::insert(
                0,
                register::I2C_RD_MASK,
                register::I2C_RD_SHIFT,
                I2cReadMode::Standard as u8,
            );

        // DEVICE_CONFIG_2 (0x01)
        let dc2 = register::insert(
            0,
            register::LP_LN_MASK,
            register::LP_LN_SHIFT,
            self.power_mode as u8,
        ) | register::insert(
            0,
            register::I2C_GLITCH_FILTER_MASK,
            register::I2C_GLITCH_FILTER_SHIFT,
            u8::from(!self.glitch_filter), // 0 = enabled, 1 = disabled (inverted)
        ) | register::insert(
            0,
            register::TRIGGER_MODE_MASK,
            register::TRIGGER_MODE_SHIFT,
            self.trigger_mode as u8,
        ) | register::insert(
            0,
            register::OPERATING_MODE_MASK,
            register::OPERATING_MODE_SHIFT,
            self.operating_mode as u8,
        );

        // SENSOR_CONFIG_1 (0x02)
        let sc1 = register::insert(
            0,
            register::MAG_CH_EN_MASK,
            register::MAG_CH_EN_SHIFT,
            self.magnetic_channels as u8,
        ) | register::insert(
            0,
            register::SLEEPTIME_MASK,
            register::SLEEPTIME_SHIFT,
            self.sleep_duration as u8,
        );

        // SENSOR_CONFIG_2 (0x03)
        let sc2 = register::insert(
            0,
            register::ANGLE_EN_MASK,
            register::ANGLE_EN_SHIFT,
            self.angle_calculation as u8,
        ) | register::insert(
            0,
            register::X_Y_RANGE_MASK,
            register::X_Y_RANGE_SHIFT,
            self.xy_range as u8,
        ) | register::insert(
            0,
            register::Z_RANGE_MASK,
            register::Z_RANGE_SHIFT,
            self.z_range as u8,
        );

        // T_CONFIG (0x07)
        let tc = register::insert(
            0,
            register::T_CH_EN_MASK,
            register::T_CH_EN_SHIFT,
            u8::from(self.temperature_enabled),
        );

        [
            (register::DEVICE_CONFIG_1, dc1),
            (register::DEVICE_CONFIG_2, dc2),
            (register::SENSOR_CONFIG_1, sc1),
            (register::SENSOR_CONFIG_2, sc2),
            (register::T_CONFIG, tc),
        ]
    }
}

// ---------------------------------------------------------------------------
// Threshold configuration
// ---------------------------------------------------------------------------

/// Threshold hysteresis mode — maps to the `THR_HYST` field in
/// `DEVICE_CONFIG_2[7:5]`.
///
/// **WARNING:** Despite the TI register name suggesting a magnitude,
/// `THR_HYST` is a **mode selector** with only two valid values.
/// Values 2–7 are **reserved** in the TMAG5273 datasheet (SBASAI4
/// Rev B, Table 8-4) and produce undefined hardware behavior
/// (observed: continuous INT firing regardless of threshold values).
///
/// The `MAG_THR_DIR` bit (threshold direction) is **ignored** when
/// `THR_HYST > 1` according to the datasheet.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ThresholdHysteresis {
    /// `THR_HYST = 0`: Uses the full 8-bit 2's complement value of each
    /// `x_THR_CONFIG` register as a signed threshold.  Combined with
    /// `ThresholdDirection`, creates a one-sided limit check.
    ///
    /// Example: X_THR = +10, direction = Above → INT fires when X > +10 LSB.
    Signed = 0,

    /// `THR_HYST = 1`: Uses the 7 LSB bits of each `x_THR_CONFIG` register
    /// to create two symmetric thresholds of equal magnitude (one positive,
    /// one negative).  Creates a ±band around zero.
    ///
    /// Example: X_THR = 10 → band is [-10, +10] LSB.  With direction = Above,
    /// INT fires when |X| > 10 LSB (field goes outside the band).
    SymmetricBand = 1,
}

/// Per-axis threshold configuration for magnetic field and temperature.
///
/// Threshold values are signed 8-bit integers in sensor-native units.
/// Each LSB corresponds to `full_scale_range / 128` mT (e.g., for ±80 mT
/// range, 1 LSB ≈ 0.625 mT). Use with
/// [`set_thresholds`](crate::Tmag5273::set_thresholds).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ThresholdConfig {
    /// X-axis threshold in sensor-native units (each LSB ≈ range / 128 mT).
    pub x: i8,
    /// Y-axis threshold in sensor-native units (each LSB ≈ range / 128 mT).
    pub y: i8,
    /// Z-axis threshold in sensor-native units (each LSB ≈ range / 128 mT).
    pub z: i8,
    /// Temperature threshold in sensor-native units (each LSB ≈ 1 °C).
    pub temperature: i8,
    /// Threshold hysteresis mode (maps to `THR_HYST` register field).
    ///
    /// Selects between signed one-sided thresholds
    /// ([`ThresholdHysteresis::Signed`]) and symmetric ±band thresholds
    /// ([`ThresholdHysteresis::SymmetricBand`]).
    ///
    /// **Only values 0 and 1 are valid.**  The enum enforces this at the
    /// type level — values 2–7 are reserved and cause undefined hardware
    /// behavior (continuous INT firing).
    pub hysteresis: ThresholdHysteresis,
    /// Number of threshold crossings before the interrupt fires.
    ///
    /// Use [`ThresholdCrossingCount::Four`] in noisy environments to
    /// debounce the threshold comparison.
    ///
    /// Default: [`ThresholdCrossingCount::One`] (interrupt on first crossing).
    pub crossing_count: ThresholdCrossingCount,
    /// Whether to trigger on field above or below the threshold.
    ///
    /// **Note:** This field is ignored by hardware when
    /// [`hysteresis`](ThresholdConfig::hysteresis) is
    /// [`SymmetricBand`](ThresholdHysteresis::SymmetricBand).
    pub direction: ThresholdDirection,
}

// ---------------------------------------------------------------------------
// Interrupt configuration
// ---------------------------------------------------------------------------

/// Interrupt configuration for the TMAG5273.
///
/// Use with [`set_interrupt`](crate::Tmag5273::set_interrupt).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptConfig {
    /// Interrupt routing mode.
    pub mode: InterruptMode,
    /// Assert interrupt on conversion complete.
    pub on_conversion_complete: bool,
    /// Assert interrupt on threshold crossing.
    pub on_threshold_crossing: bool,
    /// INT pin behavior (latched vs pulsed).
    pub pin_behavior: IntPinBehavior,
    /// Mask the interrupt pin (true = INT output disabled).
    pub mask: bool,
}

// ---------------------------------------------------------------------------
// Gain/offset calibration
// ---------------------------------------------------------------------------

/// Magnetic gain and offset calibration configuration.
///
/// Gain and offset corrections are applied by the sensor hardware to
/// improve measurement accuracy. Use with
/// [`set_calibration`](crate::Tmag5273::set_calibration).
///
/// Reference: TI TMAG5273 datasheet SBASAI4, Tables 8-12 through 8-14.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CalibrationConfig {
    /// Magnetic gain adjustment (raw 8-bit value for `GAIN_VALUE` register).
    ///
    /// The hardware interprets this as a fractional multiplier: `gain / 256`.
    /// **A value of 0 is treated by the device as 1.0** (no adjustment).
    ///
    /// | `gain` | Effective multiplier |
    /// |--------|---------------------|
    /// | `0`    | 1.0 (no adjustment) |
    /// | `128`  | 0.5                 |
    /// | `192`  | 0.75                |
    /// | `255`  | ≈ 0.996             |
    ///
    /// The target axis is selected by [`gain_channel`](Self::gain_channel)
    /// and [`AngleCalculation`](crate::AngleCalculation).
    pub gain: u8,
    /// Magnetic offset for axis 1 (signed 8-bit, register 0x0A).
    pub offset_1: i8,
    /// Magnetic offset for axis 2 (signed 8-bit, register 0x0B).
    pub offset_2: i8,
    /// Which axis pair receives the gain adjustment.
    pub gain_channel: GainChannel,
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builder_defaults_match_sparkfun_begin() {
        let config = ConfigBuilder::new().build().unwrap();
        assert_eq!(config.operating_mode, OperatingMode::ContinuousMeasure);
        assert_eq!(config.magnetic_channels, MagneticChannel::XYZ);
        assert!(config.temperature_enabled);
        assert_eq!(config.angle_calculation, AngleCalculation::None);
        assert_eq!(config.xy_range, Range::High);
        assert_eq!(config.z_range, Range::High);
        assert_eq!(config.power_mode, PowerMode::LowActiveCurrent);
    }

    #[test]
    fn angle_requires_two_channels_x_only() {
        let result = ConfigBuilder::new()
            .magnetic_channels(MagneticChannel::X)
            .angle_calculation(AngleCalculation::XY)
            .build();
        assert_eq!(result.unwrap_err(), ConfigError::AngleRequiresTwoChannels);
    }

    #[test]
    fn angle_with_two_channels_ok() {
        let result = ConfigBuilder::new()
            .magnetic_channels(MagneticChannel::XY)
            .angle_calculation(AngleCalculation::XY)
            .build();
        assert!(result.is_ok());
    }

    #[test]
    fn build_wakeup_sleep_accepts_ms1_with_default_x1_averaging() {
        // Ms1 (1000 µs) is a valid sleep with the default X1 averaging.
        // X1 + XYZ + temp = 125 µs conversion, well under 1000 µs.
        let result = ConfigBuilder::new()
            .operating_mode(OperatingMode::WakeUpAndSleep)
            .sleep_duration(SleepDuration::Ms1)
            .build();
        assert!(result.is_ok());
    }

    #[test]
    fn register_writes_default_config() {
        let config = ConfigBuilder::new().build().unwrap();
        let writes = config.to_register_writes();

        // 5 registers written
        assert_eq!(writes.len(), 5);

        // Verify SENSOR_CONFIG_1: channels = XYZ (0x7 << 4 = 0x70), sleep = 0
        let sc1 = writes.iter().find(|(addr, _)| *addr == 0x02).unwrap().1;
        assert_eq!(sc1 & 0xF0, 0x70); // MAG_CH_EN = 0x7

        // Verify SENSOR_CONFIG_2: angle=none, xy_range=high, z_range=high
        let sc2 = writes.iter().find(|(addr, _)| *addr == 0x03).unwrap().1;
        assert_eq!(sc2 & 0x03, 0x03); // both ranges = High (1)
        assert_eq!(sc2 & 0x0C, 0x00); // angle = None

        // Verify T_CONFIG: temperature enabled
        let tc = writes.iter().find(|(addr, _)| *addr == 0x07).unwrap().1;
        assert_eq!(tc & 0x01, 0x01);
    }

    #[test]
    fn conversion_time_default_config_returns_125us() {
        let config = ConfigBuilder::new().build().unwrap();
        // Default: X1, XYZ, temp enabled → 125 us
        assert_eq!(config.conversion_time().0, 125);
    }

    #[test]
    fn build_wakeup_sleep_rejects_ms1_when_x32_xyz_temp_exceeds_1000us() {
        // Ms1 (1000 µs) < X32 + XYZ + temp conversion time (3225 µs).
        let result = ConfigBuilder::new()
            .operating_mode(OperatingMode::WakeUpAndSleep)
            .sleep_duration(SleepDuration::Ms1)
            .conversion_average(ConversionAverage::X32)
            .magnetic_channels(MagneticChannel::XYZ)
            .temperature_enabled(true)
            .build();
        assert_eq!(
            result.unwrap_err(),
            ConfigError::SleepShorterThanConversion {
                sleep: Micros(1_000),
                conversion: Micros(3_225),
            },
        );
    }

    #[test]
    fn build_wakeup_sleep_accepts_ms5_with_x32_xyz_temp() {
        // Ms5 (5000 µs) > X32 + XYZ + temp (3225 µs) — always valid.
        let result = ConfigBuilder::new()
            .operating_mode(OperatingMode::WakeUpAndSleep)
            .sleep_duration(SleepDuration::Ms5)
            .conversion_average(ConversionAverage::X32)
            .magnetic_channels(MagneticChannel::XYZ)
            .temperature_enabled(true)
            .build();
        assert!(result.is_ok());
    }

    #[test]
    fn int_pin_trigger_with_standby_ok() {
        let result = ConfigBuilder::new()
            .operating_mode(OperatingMode::Standby)
            .trigger_mode(TriggerMode::IntPin)
            .build();
        assert!(result.is_ok());
    }

    #[test]
    fn int_pin_trigger_with_continuous_rejected() {
        let result = ConfigBuilder::new()
            .operating_mode(OperatingMode::ContinuousMeasure)
            .trigger_mode(TriggerMode::IntPin)
            .build();
        assert_eq!(
            result.unwrap_err(),
            ConfigError::IntPinTriggerRequiresStandby,
        );
    }

    #[test]
    fn int_pin_trigger_with_wakeup_sleep_rejected() {
        let result = ConfigBuilder::new()
            .operating_mode(OperatingMode::WakeUpAndSleep)
            .trigger_mode(TriggerMode::IntPin)
            .sleep_duration(SleepDuration::Ms5)
            .build();
        assert_eq!(
            result.unwrap_err(),
            ConfigError::IntPinTriggerRequiresStandby,
        );
    }

    #[test]
    fn i2c_command_trigger_with_continuous_ok() {
        // I2cCommand trigger is the default and works in all modes.
        let result = ConfigBuilder::new()
            .operating_mode(OperatingMode::ContinuousMeasure)
            .trigger_mode(TriggerMode::I2cCommand)
            .build();
        assert!(result.is_ok());
    }
}
