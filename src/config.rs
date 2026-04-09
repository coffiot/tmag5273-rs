//! Configuration builder and enums for the TMAG5273 sensor.
//!
//! Use [`ConfigBuilder`] to construct a validated [`Config`], then pass it to
//! [`Tmag5273::init`](crate::Tmag5273).

use crate::error::ConfigError;
use crate::register;
use crate::types::{
    AngleEnable, ConversionAverage, I2cReadMode, InterruptMode, InterruptState, Lsb,
    MagneticChannel, MagneticGainChannel, MagneticTempCoefficient, MagneticThresholdDirection,
    MicrosIsr, MilliTesla, OperatingMode, PowerNoiseMode, Range, SleepTime, TempThresholdConfig,
    ThresholdCrossingCount, TriggerMode,
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
    pub magnetic_channels_enabled: MagneticChannel,
    /// Whether the temperature channel is enabled.
    pub temp_channel_enabled: bool,
    /// Angle calculation axis pair.
    pub angle_enabled: AngleEnable,
    /// Conversion averaging.
    pub conversion_average: ConversionAverage,
    /// X/Y axis measurement range.
    pub xy_range: Range,
    /// Z axis measurement range.
    pub z_range: Range,
    /// Power mode.
    pub power_noise_mode: PowerNoiseMode,
    /// Sleep duration (used in wake-up-and-sleep mode).
    pub sleep_time: SleepTime,
    /// Trigger mode.
    pub trigger_mode: TriggerMode,
    /// Whether the I2C glitch filter is enabled.
    pub i2c_glitch_filter: bool,
    /// Magnetic temperature compensation coefficient.
    pub magnetic_temp_coefficient: MagneticTempCoefficient,
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
/// assert_eq!(config.magnetic_channels_enabled, tmag5273::MagneticChannel::XYZ);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct ConfigBuilder {
    operating_mode: OperatingMode,
    magnetic_channels_enabled: MagneticChannel,
    temp_channel_enabled: bool,
    angle_enabled: AngleEnable,
    conversion_average: ConversionAverage,
    xy_range: Range,
    z_range: Range,
    power_noise_mode: PowerNoiseMode,
    sleep_time: SleepTime,
    trigger_mode: TriggerMode,
    i2c_glitch_filter: bool,
    magnetic_temp_coefficient: MagneticTempCoefficient,
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
            magnetic_channels_enabled: MagneticChannel::XYZ,
            temp_channel_enabled: true,
            angle_enabled: AngleEnable::None,
            conversion_average: ConversionAverage::X1,
            xy_range: Range::High,
            z_range: Range::High,
            power_noise_mode: PowerNoiseMode::LowActiveCurrent,
            sleep_time: SleepTime::Ms1,
            trigger_mode: TriggerMode::I2cCommand,
            i2c_glitch_filter: true,
            magnetic_temp_coefficient: MagneticTempCoefficient::None,
        }
    }

    /// Set the operating mode.
    pub const fn operating_mode(mut self, mode: OperatingMode) -> Self {
        self.operating_mode = mode;
        self
    }

    /// Set the magnetic channel selection.
    pub const fn magnetic_channels_enabled(mut self, magnetic_channels: MagneticChannel) -> Self {
        self.magnetic_channels_enabled = magnetic_channels;
        self
    }

    /// Enable or disable the temperature channel.
    pub const fn temp_channel_enabled(mut self, enabled: bool) -> Self {
        self.temp_channel_enabled = enabled;
        self
    }

    /// Set the angle calculation axis pair.
    pub const fn angle_enabled(mut self, enabled: AngleEnable) -> Self {
        self.angle_enabled = enabled;
        self
    }

    /// Set the conversion averaging.
    pub const fn conversion_average(mut self, conversion_average: ConversionAverage) -> Self {
        self.conversion_average = conversion_average;
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
    pub const fn power_noise_mode(mut self, mode: PowerNoiseMode) -> Self {
        self.power_noise_mode = mode;
        self
    }

    /// Set the sleep duration (for wake-up-and-sleep mode).
    pub const fn sleep_time(mut self, sleep_time: SleepTime) -> Self {
        self.sleep_time = sleep_time;
        self
    }

    /// Set the trigger mode.
    pub const fn trigger_mode(mut self, mode: TriggerMode) -> Self {
        self.trigger_mode = mode;
        self
    }

    /// Enable or disable the I2C glitch filter.
    pub const fn i2c_glitch_filter(mut self, enabled: bool) -> Self {
        self.i2c_glitch_filter = enabled;
        self
    }

    /// Set the temperature compensation coefficient.
    pub const fn magnetic_temp_coefficient(mut self, coefficient: MagneticTempCoefficient) -> Self {
        self.magnetic_temp_coefficient = coefficient;
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
    ///
    /// Returns [`ConfigError::AngleMixedRanges`]
    /// if `AngleEnable::YZ` or `AngleEnable::XZ` is used with different
    /// XY and Z range settings.
    pub fn build(self) -> Result<Config, ConfigError> {
        // Angle calculation requires at least two axes.
        if !matches!(self.angle_enabled, AngleEnable::None)
            && self.magnetic_channels_enabled.axis_count() < 2
        {
            return Err(ConfigError::AngleRequiresTwoChannels);
        }

        // INT-pin trigger is only meaningful in standby mode.
        if matches!(self.trigger_mode, TriggerMode::IntPin)
            && !matches!(self.operating_mode, OperatingMode::Standby)
        {
            return Err(ConfigError::IntPinTriggerRequiresStandby);
        }

        // YZ/XZ angle calculation mixes axes with independent range
        // registers (X_Y_RANGE vs Z_RANGE). The CORDIC magnitude output
        // cannot be scaled to mT when the two ranges differ — §7.5.1.3.
        if matches!(self.angle_enabled, AngleEnable::YZ | AngleEnable::XZ)
            && self.xy_range != self.z_range
        {
            return Err(ConfigError::AngleMixedRanges);
        }

        // Sleep must be long enough for at least one full conversion.
        if matches!(self.operating_mode, OperatingMode::WakeUpAndSleep) {
            let sleep = self.sleep_time.as_micros_isr();
            let conversion = self
                .conversion_average
                .conversion_time(self.magnetic_channels_enabled, self.temp_channel_enabled);
            if sleep.0 < conversion.0 {
                return Err(ConfigError::SleepShorterThanConversion { sleep, conversion });
            }
        }

        Ok(Config {
            operating_mode: self.operating_mode,
            magnetic_channels_enabled: self.magnetic_channels_enabled,
            temp_channel_enabled: self.temp_channel_enabled,
            angle_enabled: self.angle_enabled,
            conversion_average: self.conversion_average,
            xy_range: self.xy_range,
            z_range: self.z_range,
            power_noise_mode: self.power_noise_mode,
            sleep_time: self.sleep_time,
            trigger_mode: self.trigger_mode,
            i2c_glitch_filter: self.i2c_glitch_filter,
            magnetic_temp_coefficient: self.magnetic_temp_coefficient,
        })
    }
}

// ---------------------------------------------------------------------------
// Config → register serialization
// ---------------------------------------------------------------------------

impl Config {
    /// Returns the expected conversion time in micros_isreconds for this
    /// configuration.
    pub const fn conversion_time(&self) -> MicrosIsr {
        self.conversion_average
            .conversion_time(self.magnetic_channels_enabled, self.temp_channel_enabled)
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
                self.magnetic_temp_coefficient as u8,
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
            self.power_noise_mode as u8,
        ) | register::insert(
            0,
            register::I2C_GLITCH_FILTER_MASK,
            register::I2C_GLITCH_FILTER_SHIFT,
            u8::from(!self.i2c_glitch_filter), // 0 = enabled, 1 = disabled (inverted)
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
            self.magnetic_channels_enabled as u8,
        ) | register::insert(
            0,
            register::SLEEPTIME_MASK,
            register::SLEEPTIME_SHIFT,
            self.sleep_time as u8,
        );

        // SENSOR_CONFIG_2 (0x03)
        let sc2 = register::insert(
            0,
            register::ANGLE_EN_MASK,
            register::ANGLE_EN_SHIFT,
            self.angle_enabled as u8,
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
            u8::from(self.temp_channel_enabled),
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
    /// `MagneticThresholdDirection`, creates a one-sided limit cross check
    /// (datasheet §7.2.1.1, Figures 7-4 and 7-5).
    ///
    /// Example: X_THR = +10, direction = Above → INT fires when X > +10 LSB.
    LimitCross = 0,

    /// `THR_HYST = 1`: Uses the 7 LSB bits of each `x_THR_CONFIG` register
    /// to create two symmetric thresholds of equal magnitude (one positive,
    /// one negative).  Creates a ±band around zero.
    ///
    /// Example: X_THR = 10 → band is [-10, +10] LSB.  With direction = Above,
    /// INT fires when |X| > 10 LSB (field goes outside the band).
    SymmetricBand = 1,
}

impl_try_from_u8!(ThresholdHysteresis {
    0 => LimitCross,
    1 => SymmetricBand,
});

impl ThresholdHysteresis {
    /// Decode a threshold register LSB back to milliTesla.
    ///
    /// The threshold registers share the same 8-bit scale as the CORDIC
    /// magnitude register: 1 LSB = `range / 128` mT (datasheet §8.1.5–§8.1.7).
    ///
    /// - [`LimitCross`](Self::LimitCross): full signed 8-bit → mT (preserves sign)
    /// - [`SymmetricBand`](Self::SymmetricBand): masks to 7 bits → mT (always ≥ 0,
    ///   represents the ± band width around zero)
    ///
    /// Caller must pass the correct per-axis range: `range_xy` for X/Y axes,
    /// `range_z` for Z axis (these may differ by variant — see
    /// [`DeviceVariant::range_xy`](crate::types::DeviceVariant::range_xy) and
    /// [`DeviceVariant::range_z`](crate::types::DeviceVariant::range_z)).
    #[inline]
    pub fn decode_threshold(&self, lsb: Lsb, range: MilliTesla) -> MilliTesla {
        let raw = match self {
            Self::LimitCross => lsb.0 as f32,
            Self::SymmetricBand => (lsb.0 as u8 & register::LSB_7BIT_MASK) as f32,
        };
        MilliTesla(raw * range.mt_per_lsb())
    }
}

#[cfg(feature = "libm")]
impl ThresholdHysteresis {
    /// Encode a milliTesla value to a threshold register LSB for this mode.
    ///
    /// Operation order: `round(scaled_mT) + margin`, then clamp to mode range.
    ///
    /// - [`LimitCross`](Self::LimitCross): `clamp(round(mT × 128 / range) + margin, −128, 127)`.
    ///   Negative margin is valid (shifts threshold toward zero).
    /// - [`SymmetricBand`](Self::SymmetricBand): `clamp(round(|mT| × 128 / range) + margin, 0, 127)`.
    ///   Returns `None` if `margin < 0` (band width cannot be negative).
    ///
    /// Returns `None` for NaN, infinity, or zero-range inputs.
    ///
    /// Clamping to register bounds is intentional domain policy — the caller
    /// requests the closest representable threshold. The hardware register
    /// would saturate identically. See datasheet §8.1.5–§8.1.7.
    #[inline]
    pub fn encode_threshold(&self, mt: MilliTesla, range: MilliTesla, margin: i8) -> Option<Lsb> {
        if !mt.0.is_finite() || !range.0.is_finite() || range.0 == 0.0 {
            return None;
        }
        match self {
            Self::LimitCross => {
                let scaled = libm::roundf(mt.0 * range.lsb_per_mt()) as i32;
                let with_margin = scaled + i32::from(margin);
                Some(Lsb(with_margin.clamp(-128, 127) as i8))
            }
            Self::SymmetricBand => {
                if margin < 0 {
                    return None;
                }
                let scaled = libm::roundf(mt.abs().0 * range.lsb_per_mt()) as i32;
                let with_margin = scaled + i32::from(margin);
                Some(Lsb(with_margin.clamp(0, 127) as i8))
            }
        }
    }
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
    pub x: Lsb,
    /// Y-axis threshold in sensor-native units (each LSB ≈ range / 128 mT).
    pub y: Lsb,
    /// Z-axis threshold in sensor-native units (each LSB ≈ range / 128 mT).
    pub z: Lsb,
    /// Temperature threshold code. Valid codes are enforced by the
    /// [`TempThresholdConfig`](crate::TempThresholdConfig) type:
    /// `0x00` (disabled) or `0x1A`–`0x34` (≈ −41 °C to ≈ 170 °C, ~8 °C/LSB).
    pub temperature: TempThresholdConfig,
    /// Threshold hysteresis mode (maps to `THR_HYST` register field).
    ///
    /// Selects between signed one-sided thresholds
    /// ([`ThresholdHysteresis::LimitCross`]) and symmetric ±band thresholds
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
    pub direction: MagneticThresholdDirection,
}

impl Default for ThresholdConfig {
    fn default() -> Self {
        Self {
            x: Lsb(0),
            y: Lsb(0),
            z: Lsb(0),
            temperature: TempThresholdConfig::DISABLED,
            hysteresis: ThresholdHysteresis::LimitCross,
            crossing_count: ThresholdCrossingCount::One,
            direction: MagneticThresholdDirection::Above,
        }
    }
}

#[cfg(feature = "libm")]
impl ThresholdConfig {
    /// Build threshold configuration from milliTesla baselines.
    ///
    /// Encodes each axis using [`ThresholdHysteresis::encode_threshold`]:
    /// X and Y with `range_xy`, Z with `range_z`.
    ///
    /// `temperature` is a pre-encoded [`TempThresholdConfig`] (pass-through,
    /// not converted from °C).
    ///
    /// Returns `None` if any axis encoding fails (e.g., negative `margin`
    /// with [`ThresholdHysteresis::SymmetricBand`]).
    ///
    /// `range_xy` and `range_z` should come from
    /// [`DeviceVariant::range_xy`](crate::types::DeviceVariant::range_xy) and
    /// [`DeviceVariant::range_z`](crate::types::DeviceVariant::range_z).
    #[expect(
        clippy::too_many_arguments,
        reason = "mirrors ThresholdConfig fields 1:1 — a builder would add complexity without value"
    )]
    pub fn from_mt(
        baselines: [MilliTesla; 3],
        range_xy: MilliTesla,
        range_z: MilliTesla,
        margin: i8,
        hysteresis: ThresholdHysteresis,
        direction: MagneticThresholdDirection,
        crossing_count: ThresholdCrossingCount,
        temperature: TempThresholdConfig,
    ) -> Option<Self> {
        let x = hysteresis.encode_threshold(baselines[0], range_xy, margin)?;
        let y = hysteresis.encode_threshold(baselines[1], range_xy, margin)?;
        let z = hysteresis.encode_threshold(baselines[2], range_z, margin)?;
        Some(Self {
            x,
            y,
            z,
            temperature,
            hysteresis,
            crossing_count,
            direction,
        })
    }
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
    pub pin_behavior: InterruptState,
    /// Mask the interrupt pin (true = INT output disabled).
    pub mask: bool,
}

impl Default for InterruptConfig {
    fn default() -> Self {
        Self {
            mode: InterruptMode::None,
            on_conversion_complete: false,
            on_threshold_crossing: false,
            pin_behavior: InterruptState::Latched,
            mask: false,
        }
    }
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
    /// and [`AngleEnable`](crate::AngleEnable).
    pub gain: u8,
    /// Magnetic offset for axis 1 (signed 8-bit, register 0x0A).
    pub offset_1: i8,
    /// Magnetic offset for axis 2 (signed 8-bit, register 0x0B).
    pub offset_2: i8,
    /// Which axis pair receives the gain adjustment.
    pub gain_channel: MagneticGainChannel,
}

impl Default for CalibrationConfig {
    fn default() -> Self {
        Self {
            gain: 0,
            offset_1: 0,
            offset_2: 0,
            gain_channel: MagneticGainChannel::First,
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::register;

    #[test]
    fn builder_defaults_match_sparkfun_begin() {
        let config = ConfigBuilder::new().build().unwrap();
        assert_eq!(config.operating_mode, OperatingMode::ContinuousMeasure);
        assert_eq!(config.magnetic_channels_enabled, MagneticChannel::XYZ);
        assert!(config.temp_channel_enabled);
        assert_eq!(config.angle_enabled, AngleEnable::None);
        assert_eq!(config.xy_range, Range::High);
        assert_eq!(config.z_range, Range::High);
        assert_eq!(config.power_noise_mode, PowerNoiseMode::LowActiveCurrent);
    }

    #[test]
    fn angle_requires_two_channels_x_only() {
        let result = ConfigBuilder::new()
            .magnetic_channels_enabled(MagneticChannel::X)
            .angle_enabled(AngleEnable::XY)
            .build();
        assert_eq!(result.unwrap_err(), ConfigError::AngleRequiresTwoChannels);
    }

    #[test]
    fn angle_with_two_channels_ok() {
        let result = ConfigBuilder::new()
            .magnetic_channels_enabled(MagneticChannel::XY)
            .angle_enabled(AngleEnable::XY)
            .build();
        assert!(result.is_ok());
    }

    #[test]
    fn build_wakeup_sleep_accepts_ms1_with_default_x1_averaging() {
        // Ms1 (1000 µs) is a valid sleep with the default X1 averaging.
        // X1 + XYZ + temp = 100 µs conversion, well under 1000 µs.
        let result = ConfigBuilder::new()
            .operating_mode(OperatingMode::WakeUpAndSleep)
            .sleep_time(SleepTime::Ms1)
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
        let tc = writes
            .iter()
            .find(|(addr, _)| *addr == register::T_CONFIG)
            .unwrap()
            .1;
        assert_eq!(tc & 0x01, 0x01);
    }

    #[test]
    fn conversion_time_default_config_returns_100us() {
        let config = ConfigBuilder::new().build().unwrap();
        // Default: X1, XYZ, temp enabled → 100 us
        // (temp doesn't add time at CONV_AVG=000b per datasheet §5.11 note 2)
        assert_eq!(config.conversion_time().0, 100);
    }

    #[test]
    fn build_wakeup_sleep_rejects_ms1_when_x32_xyz_temp_exceeds_1000us() {
        // Ms1 (1000 µs) < X32 + XYZ + temp conversion time (3225 µs).
        let result = ConfigBuilder::new()
            .operating_mode(OperatingMode::WakeUpAndSleep)
            .sleep_time(SleepTime::Ms1)
            .conversion_average(ConversionAverage::X32)
            .magnetic_channels_enabled(MagneticChannel::XYZ)
            .temp_channel_enabled(true)
            .build();
        assert_eq!(
            result.unwrap_err(),
            ConfigError::SleepShorterThanConversion {
                sleep: MicrosIsr(1_000),
                conversion: MicrosIsr(3_225),
            },
        );
    }

    #[test]
    fn build_wakeup_sleep_accepts_ms5_with_x32_xyz_temp() {
        // Ms5 (5000 µs) > X32 + XYZ + temp (3225 µs) — always valid.
        let result = ConfigBuilder::new()
            .operating_mode(OperatingMode::WakeUpAndSleep)
            .sleep_time(SleepTime::Ms5)
            .conversion_average(ConversionAverage::X32)
            .magnetic_channels_enabled(MagneticChannel::XYZ)
            .temp_channel_enabled(true)
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
            .sleep_time(SleepTime::Ms5)
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

    // -- AngleMixedRanges validation -----------------------------------------

    #[test]
    fn angle_yz_rejects_mixed_ranges() {
        let result = ConfigBuilder::new()
            .magnetic_channels_enabled(MagneticChannel::YZ)
            .angle_enabled(AngleEnable::YZ)
            .xy_range(Range::Low)
            .z_range(Range::High)
            .build();
        assert_eq!(result.unwrap_err(), ConfigError::AngleMixedRanges);
    }

    #[test]
    fn angle_xz_rejects_mixed_ranges() {
        let result = ConfigBuilder::new()
            .magnetic_channels_enabled(MagneticChannel::ZX)
            .angle_enabled(AngleEnable::XZ)
            .xy_range(Range::High)
            .z_range(Range::Low)
            .build();
        assert_eq!(result.unwrap_err(), ConfigError::AngleMixedRanges);
    }

    #[test]
    fn angle_yz_accepts_matching_ranges() {
        let result = ConfigBuilder::new()
            .magnetic_channels_enabled(MagneticChannel::YZ)
            .angle_enabled(AngleEnable::YZ)
            .xy_range(Range::High)
            .z_range(Range::High)
            .build();
        assert!(result.is_ok());
    }

    #[test]
    fn angle_xy_ignores_z_range_mismatch() {
        // XY angle only uses xy_range — z_range doesn't matter.
        let result = ConfigBuilder::new()
            .magnetic_channels_enabled(MagneticChannel::XY)
            .angle_enabled(AngleEnable::XY)
            .xy_range(Range::High)
            .z_range(Range::Low)
            .build();
        assert!(result.is_ok());
    }

    // -- ThresholdHysteresis::decode_threshold ---------------------------------

    #[test]
    fn decode_threshold_limit_cross_zero_lsb() {
        let mt = ThresholdHysteresis::LimitCross.decode_threshold(Lsb(0), MilliTesla(80.0));
        assert_eq!(mt, MilliTesla(0.0));
    }

    #[test]
    fn decode_threshold_limit_cross_positive_max() {
        // 127 × 80 / 128 = 79.375
        let mt = ThresholdHysteresis::LimitCross.decode_threshold(Lsb(127), MilliTesla(80.0));
        assert_eq!(mt, MilliTesla(79.375));
    }

    #[test]
    fn decode_threshold_limit_cross_negative_max() {
        // -128 × 80 / 128 = -80.0
        let mt = ThresholdHysteresis::LimitCross.decode_threshold(Lsb(-128), MilliTesla(80.0));
        assert_eq!(mt, MilliTesla(-80.0));
    }

    #[test]
    fn decode_threshold_symmetric_band_positive_max() {
        // 127 & 0x7F = 127; 127 × 80 / 128 = 79.375
        let mt = ThresholdHysteresis::SymmetricBand.decode_threshold(Lsb(127), MilliTesla(80.0));
        assert_eq!(mt, MilliTesla(79.375));
    }

    #[test]
    fn decode_threshold_symmetric_band_masks_sign_bit() {
        // -128_i8 = 0x80; 0x80 & 0x7F = 0 → result is 0.0 mT
        let mt = ThresholdHysteresis::SymmetricBand.decode_threshold(Lsb(-128), MilliTesla(80.0));
        assert_eq!(mt, MilliTesla(0.0));
    }

    #[test]
    fn decode_threshold_zero_range() {
        // 1 × 0.0 / 128 = 0.0
        let mt = ThresholdHysteresis::LimitCross.decode_threshold(Lsb(1), MilliTesla(0.0));
        assert_eq!(mt, MilliTesla(0.0));
    }

    // -- ThresholdHysteresis::encode_threshold --------------------------------

    #[cfg(feature = "libm")]
    mod encode_threshold_tests {
        use super::*;

        #[test]
        fn limit_cross_zero_mt() {
            let r = ThresholdHysteresis::LimitCross.encode_threshold(
                MilliTesla(0.0),
                MilliTesla(80.0),
                0,
            );
            assert_eq!(r, Some(Lsb(0)));
        }

        #[test]
        fn limit_cross_half_range() {
            // 40 / 80 × 128 = 64
            let r = ThresholdHysteresis::LimitCross.encode_threshold(
                MilliTesla(40.0),
                MilliTesla(80.0),
                0,
            );
            assert_eq!(r, Some(Lsb(64)));
        }

        #[test]
        fn symmetric_band_abs_with_margin() {
            // |−5| / 80 × 128 = 8, +2 = 10
            let r = ThresholdHysteresis::SymmetricBand.encode_threshold(
                MilliTesla(-5.0),
                MilliTesla(80.0),
                2,
            );
            assert_eq!(r, Some(Lsb(10)));
        }

        #[test]
        fn limit_cross_clamps_above_127() {
            // 80/80 × 128 = 128, +5 = 133 → clamped to 127
            let r = ThresholdHysteresis::LimitCross.encode_threshold(
                MilliTesla(80.0),
                MilliTesla(80.0),
                5,
            );
            assert_eq!(r, Some(Lsb(127)));
        }

        #[test]
        fn symmetric_band_negative_margin_returns_none() {
            let r = ThresholdHysteresis::SymmetricBand.encode_threshold(
                MilliTesla(1.0),
                MilliTesla(80.0),
                -1,
            );
            assert_eq!(r, None);
        }

        #[test]
        fn zero_range_returns_none() {
            let r = ThresholdHysteresis::LimitCross.encode_threshold(
                MilliTesla(1.0),
                MilliTesla(0.0),
                0,
            );
            assert_eq!(r, None);
        }

        #[test]
        fn nan_returns_none() {
            let r = ThresholdHysteresis::LimitCross.encode_threshold(
                MilliTesla(f32::NAN),
                MilliTesla(80.0),
                0,
            );
            assert_eq!(r, None);
        }

        #[test]
        fn round_trip_limit_cross() {
            let mt = MilliTesla(10.0);
            let range = MilliTesla(80.0);
            let lsb = ThresholdHysteresis::LimitCross
                .encode_threshold(mt, range, 0)
                .unwrap();
            let decoded = ThresholdHysteresis::LimitCross.decode_threshold(lsb, range);
            // Tolerance: ±range/256 = ±0.3125
            assert!((decoded.0 - mt.0).abs() <= 0.3125);
        }
    }

    // -- ThresholdConfig::from_mt ---------------------------------------------

    #[cfg(feature = "libm")]
    mod from_mt_tests {
        use super::*;

        #[test]
        fn all_zeros() {
            let cfg = ThresholdConfig::from_mt(
                [MilliTesla(0.0), MilliTesla(0.0), MilliTesla(0.0)],
                MilliTesla(80.0),
                MilliTesla(80.0),
                0,
                ThresholdHysteresis::LimitCross,
                MagneticThresholdDirection::Above,
                ThresholdCrossingCount::One,
                TempThresholdConfig::DISABLED,
            )
            .unwrap();
            assert_eq!(cfg.x, Lsb(0));
            assert_eq!(cfg.y, Lsb(0));
            assert_eq!(cfg.z, Lsb(0));
        }

        #[test]
        fn mixed_axes_with_margin() {
            // x: 40/80 × 128 = 64, +5 = 69
            // y: 20/80 × 128 = 32, +5 = 37
            // z: 10/80 × 128 = 16, +5 = 21
            let cfg = ThresholdConfig::from_mt(
                [MilliTesla(40.0), MilliTesla(20.0), MilliTesla(10.0)],
                MilliTesla(80.0),
                MilliTesla(80.0),
                5,
                ThresholdHysteresis::LimitCross,
                MagneticThresholdDirection::Above,
                ThresholdCrossingCount::One,
                TempThresholdConfig::DISABLED,
            )
            .unwrap();
            assert_eq!(cfg.x, Lsb(69));
            assert_eq!(cfg.y, Lsb(37));
            assert_eq!(cfg.z, Lsb(21));
        }

        #[test]
        fn symmetric_band_negative_margin_returns_none() {
            let r = ThresholdConfig::from_mt(
                [MilliTesla(1.0), MilliTesla(1.0), MilliTesla(1.0)],
                MilliTesla(80.0),
                MilliTesla(80.0),
                -1,
                ThresholdHysteresis::SymmetricBand,
                MagneticThresholdDirection::Above,
                ThresholdCrossingCount::One,
                TempThresholdConfig::DISABLED,
            );
            assert_eq!(r, None);
        }

        #[test]
        fn different_xy_z_ranges() {
            // XY: 10/80 × 128 = 16
            // Z:  10/40 × 128 = 32
            let cfg = ThresholdConfig::from_mt(
                [MilliTesla(10.0), MilliTesla(10.0), MilliTesla(10.0)],
                MilliTesla(80.0),
                MilliTesla(40.0),
                0,
                ThresholdHysteresis::LimitCross,
                MagneticThresholdDirection::Above,
                ThresholdCrossingCount::One,
                TempThresholdConfig::DISABLED,
            )
            .unwrap();
            assert_eq!(cfg.x, Lsb(16));
            assert_eq!(cfg.y, Lsb(16));
            assert_eq!(cfg.z, Lsb(32));
        }
    }
}
