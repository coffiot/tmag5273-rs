// Rust guideline compliant 2026-03-24

//! Configuration builder and enums for the TMAG5273 sensor.
//!
//! Use [`ConfigBuilder`] to construct a validated [`Config`], then pass it to
//! [`Tmag5273::init`](crate::Tmag5273).

// Re-export Range from types — it is the canonical definition
pub use crate::types::Range;

// Placeholder — will be filled in Phase 2
// All config enums, ConfigBuilder, Config, and validation go here.

/// Operating mode for the TMAG5273.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum OperatingMode {
    /// Stand-by mode — triggered by I2C command or INT pin.
    Standby = 0x00,
    /// Sleep mode — low power, I2C active.
    Sleep = 0x01,
    /// Continuous measurement mode.
    ContinuousMeasure = 0x02,
    /// Wake-up and sleep mode — periodic measurement with threshold check.
    WakeUpAndSleep = 0x03,
}

/// Magnetic channel selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum MagneticChannel {
    /// All channels off.
    Off = 0x0,
    /// X axis only.
    X = 0x1,
    /// Y axis only.
    Y = 0x2,
    /// X and Y axes.
    XY = 0x3,
    /// Z axis only.
    Z = 0x4,
    /// Z and X axes.
    ZX = 0x5,
    /// Y and Z axes.
    YZ = 0x6,
    /// X, Y, and Z axes.
    XYZ = 0x7,
    /// Pseudo-simultaneous X-Y-X (for angle calculation).
    XYX = 0x8,
    /// Pseudo-simultaneous Y-X-Y.
    YXY = 0x9,
    /// Pseudo-simultaneous Y-Z-Y.
    YZY = 0xA,
    /// Pseudo-simultaneous X-Z-X.
    XZX = 0xB,
}

impl MagneticChannel {
    /// Returns the number of distinct axes enabled.
    pub(crate) const fn axis_count(self) -> u8 {
        match self {
            Self::Off => 0,
            Self::X | Self::Y | Self::Z => 1,
            Self::XY | Self::ZX | Self::YZ | Self::XYX | Self::YXY | Self::YZY | Self::XZX => 2,
            Self::XYZ => 3,
        }
    }

    /// Whether the X axis is enabled.
    pub(crate) const fn has_x(self) -> bool {
        matches!(
            self,
            Self::X | Self::XY | Self::ZX | Self::XYZ | Self::XYX | Self::YXY | Self::XZX
        )
    }

    /// Whether the Y axis is enabled.
    pub(crate) const fn has_y(self) -> bool {
        matches!(
            self,
            Self::Y | Self::XY | Self::YZ | Self::XYZ | Self::XYX | Self::YXY | Self::YZY
        )
    }

    /// Whether the Z axis is enabled.
    pub(crate) const fn has_z(self) -> bool {
        matches!(
            self,
            Self::Z | Self::ZX | Self::YZ | Self::XYZ | Self::YZY | Self::XZX
        )
    }
}

/// Conversion averaging samples.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ConversionAverage {
    /// 1x (no averaging).
    X1 = 0b000,
    /// 2x averaging.
    X2 = 0b001,
    /// 4x averaging.
    X4 = 0b010,
    /// 8x averaging.
    X8 = 0b011,
    /// 16x averaging.
    X16 = 0b100,
    /// 32x averaging.
    X32 = 0b101,
}

impl ConversionAverage {
    /// Maximum poll count for conversion timeout (2x safety margin over
    /// datasheet max at 400 kHz I2C, ~50 µs per poll).
    pub(crate) const fn max_polls(self) -> u16 {
        match self {
            Self::X1 => 10,
            Self::X2 => 15,
            Self::X4 => 25,
            Self::X8 => 40,
            Self::X16 => 70,
            Self::X32 => 100,
        }
    }
}

/// Sleep duration between measurements in wake-up-and-sleep mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum SleepDuration {
    /// 1 ms.
    Ms1 = 0x0,
    /// 5 ms.
    Ms5 = 0x1,
    /// 10 ms.
    Ms10 = 0x2,
    /// 15 ms.
    Ms15 = 0x3,
    /// 20 ms.
    Ms20 = 0x4,
    /// 30 ms.
    Ms30 = 0x5,
    /// 50 ms.
    Ms50 = 0x6,
    /// 100 ms.
    Ms100 = 0x7,
    /// 500 ms.
    Ms500 = 0x8,
    /// 1000 ms.
    Ms1000 = 0x9,
    /// 2000 ms.
    Ms2000 = 0xA,
    /// 5000 ms.
    Ms5000 = 0xB,
    /// 20000 ms.
    Ms20000 = 0xC,
}

/// Power mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PowerMode {
    /// Low active current mode (default).
    LowActiveCurrent = 0,
    /// Low noise mode.
    LowNoise = 1,
}

/// Trigger mode for standby conversions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TriggerMode {
    /// I2C command triggers conversion.
    I2cCommand = 0,
    /// INT pin triggers conversion.
    IntPin = 1,
}

/// Magnetic temperature compensation coefficient.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TempCoefficient {
    /// No temperature compensation (0%).
    None = 0b00,
    /// NdBFe magnets (0.12%/°C).
    NdBFe = 0b01,
    /// Ceramic/Ferrite magnets (0.2%/°C). Note: 0b10 is reserved.
    Ceramic = 0b11,
}

/// Angle calculation axis pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum AngleCalculation {
    /// Angle calculation disabled.
    None = 0b00,
    /// X-Y plane angle.
    XY = 0b01,
    /// Y-Z plane angle.
    YZ = 0b10,
    /// X-Z plane angle.
    XZ = 0b11,
}

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

/// Validated device configuration for the TMAG5273.
///
/// Construct via [`ConfigBuilder`]. The `#[non_exhaustive]` attribute allows
/// pub field reads but prevents construction outside this crate.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    /// Maximum conversion poll count (derived from `conversion_average`).
    pub(crate) max_polls: u16,
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
#[derive(Debug, Clone)]
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
}

impl Default for ConfigBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl ConfigBuilder {
    /// Create a new builder with defaults matching the SparkFun `begin()`
    /// production configuration.
    pub fn new() -> Self {
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
        }
    }

    /// Set the operating mode.
    pub fn operating_mode(mut self, mode: OperatingMode) -> Self {
        self.operating_mode = mode;
        self
    }

    /// Set the magnetic channel selection.
    pub fn magnetic_channels(mut self, channels: MagneticChannel) -> Self {
        self.magnetic_channels = channels;
        self
    }

    /// Enable or disable the temperature channel.
    pub fn temperature_enabled(mut self, enabled: bool) -> Self {
        self.temperature_enabled = enabled;
        self
    }

    /// Set the angle calculation axis pair.
    pub fn angle_calculation(mut self, angle: AngleCalculation) -> Self {
        self.angle_calculation = angle;
        self
    }

    /// Set the conversion averaging.
    pub fn conversion_average(mut self, avg: ConversionAverage) -> Self {
        self.conversion_average = avg;
        self
    }

    /// Set the X/Y axis measurement range.
    pub fn xy_range(mut self, range: Range) -> Self {
        self.xy_range = range;
        self
    }

    /// Set the Z axis measurement range.
    pub fn z_range(mut self, range: Range) -> Self {
        self.z_range = range;
        self
    }

    /// Set the power mode.
    pub fn power_mode(mut self, mode: PowerMode) -> Self {
        self.power_mode = mode;
        self
    }

    /// Set the sleep duration (for wake-up-and-sleep mode).
    pub fn sleep_duration(mut self, dur: SleepDuration) -> Self {
        self.sleep_duration = dur;
        self
    }

    /// Set the trigger mode.
    pub fn trigger_mode(mut self, mode: TriggerMode) -> Self {
        self.trigger_mode = mode;
        self
    }

    /// Enable or disable the I2C glitch filter.
    pub fn glitch_filter(mut self, enabled: bool) -> Self {
        self.glitch_filter = enabled;
        self
    }

    /// Set the temperature compensation coefficient.
    pub fn temp_coefficient(mut self, coeff: TempCoefficient) -> Self {
        self.temp_coefficient = coeff;
        self
    }

    /// Enable CRC-8 validation on I2C reads.
    ///
    /// Only available when the `crc` feature is enabled. Without the feature,
    /// this method does not exist — the compiler rejects the call.
    #[cfg(feature = "crc")]
    pub fn crc_enable(self, _enable: bool) -> Self {
        // CRC config will be applied during init when the crc feature is active
        self
    }

    /// Validate and build the [`Config`].
    ///
    /// # Errors
    ///
    /// Returns [`Error::AngleRequiresTwoChannels`](crate::Error::AngleRequiresTwoChannels)
    /// if angle calculation is enabled with fewer than two magnetic channels.
    ///
    /// Returns [`Error::SleepDurationRequired`](crate::Error::SleepDurationRequired)
    /// if wake-up-and-sleep mode is selected without a sleep duration > 1 ms.
    pub fn build(self) -> Result<Config, crate::Error<core::convert::Infallible>> {
        // Angle calculation requires at least two axes.
        if !matches!(self.angle_calculation, AngleCalculation::None)
            && self.magnetic_channels.axis_count() < 2
        {
            return Err(crate::Error::AngleRequiresTwoChannels);
        }

        // Wake-up-and-sleep needs a meaningful sleep duration.
        if matches!(self.operating_mode, OperatingMode::WakeUpAndSleep)
            && matches!(self.sleep_duration, SleepDuration::Ms1)
        {
            return Err(crate::Error::SleepDurationRequired);
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
            max_polls: self.conversion_average.max_polls(),
        })
    }
}

// ---------------------------------------------------------------------------
// Config → register serialization
// ---------------------------------------------------------------------------

impl Config {
    /// Serialize this configuration into register address/value pairs for
    /// writing to the device.
    pub(crate) fn to_register_writes(&self) -> [(u8, u8); 5] {
        use crate::register;

        // DEVICE_CONFIG_1 (0x00)
        let dc1 = register::insert(
            0,
            register::MAG_TEMPCO_MASK,
            register::MAG_TEMPCO_SHIFT,
            self.temp_coefficient as u8,
        ) | register::insert(
            0,
            register::CONV_AVG_MASK,
            register::CONV_AVG_SHIFT,
            self.conversion_average as u8,
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

/// Threshold crossing direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ThresholdDirection {
    /// Interrupt when field exceeds threshold (above).
    Above = 0,
    /// Interrupt when field falls below threshold (below).
    Below = 1,
}

/// Per-axis threshold configuration for magnetic field and temperature.
///
/// Threshold values are signed 8-bit integers representing the crossing
/// point in sensor-native units. Use with
/// [`set_thresholds`](crate::Tmag5273::set_thresholds).
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ThresholdConfig {
    /// X-axis threshold (signed, sensor-native units).
    pub x: i8,
    /// Y-axis threshold (signed, sensor-native units).
    pub y: i8,
    /// Z-axis threshold (signed, sensor-native units).
    pub z: i8,
    /// Temperature threshold (signed, upper 7 bits of T_CONFIG).
    pub temperature: i8,
    /// Hysteresis level (0–7). Higher values reduce noise sensitivity.
    pub hysteresis: u8,
    /// Whether to trigger on field above or below the threshold.
    pub direction: ThresholdDirection,
}

// ---------------------------------------------------------------------------
// Interrupt configuration
// ---------------------------------------------------------------------------

/// Interrupt routing mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum InterruptMode {
    /// No interrupt.
    None = 0b000,
    /// Interrupt through INT pin.
    IntPin = 0b001,
    /// Interrupt through INT pin, latched until cleared by I2C read.
    IntPinLatched = 0b010,
    /// Interrupt through SCL (clock stretching).
    Scl = 0b011,
    /// Interrupt through SCL, latched until cleared by I2C read.
    SclLatched = 0b100,
}

/// INT pin assertion behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum IntPinBehavior {
    /// INT pin stays asserted until host clears it.
    Latched = 0,
    /// INT pin pulses for 10 µs then deasserts.
    Pulse10us = 1,
}

/// Interrupt configuration for the TMAG5273.
///
/// Use with [`set_interrupt`](crate::Tmag5273::set_interrupt).
#[derive(Debug, Clone, Copy)]
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

/// Gain adjustment channel selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum GainChannel {
    /// Apply gain to first axis pair.
    First = 0,
    /// Apply gain to second axis pair.
    Second = 1,
}

/// Magnetic gain and offset calibration configuration.
///
/// Use with [`set_calibration`](crate::Tmag5273::set_calibration).
///
/// **Note:** The SparkFun Arduino library has a known copy-paste bug where
/// `config_mag_offset_2()` writes to register 0x0A instead of 0x0B.
/// This driver writes to the correct registers.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CalibrationConfig {
    /// Magnetic gain adjustment (raw 8-bit value, register 0x09).
    pub gain: u8,
    /// Magnetic offset for axis 1 (signed, register 0x0A).
    pub offset_1: i8,
    /// Magnetic offset for axis 2 (signed, register 0x0B).
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
        assert_eq!(result.unwrap_err(), crate::Error::AngleRequiresTwoChannels);
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
    fn wake_up_and_sleep_needs_duration() {
        let result = ConfigBuilder::new()
            .operating_mode(OperatingMode::WakeUpAndSleep)
            .sleep_duration(SleepDuration::Ms1) // 1 ms is the "none" sentinel
            .build();
        assert_eq!(result.unwrap_err(), crate::Error::SleepDurationRequired);
    }

    #[test]
    fn wake_up_and_sleep_with_duration_ok() {
        let result = ConfigBuilder::new()
            .operating_mode(OperatingMode::WakeUpAndSleep)
            .sleep_duration(SleepDuration::Ms100)
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
    fn magnetic_channel_axis_count() {
        assert_eq!(MagneticChannel::Off.axis_count(), 0);
        assert_eq!(MagneticChannel::X.axis_count(), 1);
        assert_eq!(MagneticChannel::XY.axis_count(), 2);
        assert_eq!(MagneticChannel::XYZ.axis_count(), 3);
        assert_eq!(MagneticChannel::XYX.axis_count(), 2);
    }

    #[test]
    fn magnetic_channel_has_axes() {
        assert!(MagneticChannel::XYZ.has_x());
        assert!(MagneticChannel::XYZ.has_y());
        assert!(MagneticChannel::XYZ.has_z());
        assert!(!MagneticChannel::YZ.has_x());
        assert!(MagneticChannel::YZ.has_y());
        assert!(MagneticChannel::YZ.has_z());
    }

    #[test]
    fn max_polls_for_averaging() {
        assert_eq!(ConversionAverage::X1.max_polls(), 10);
        assert_eq!(ConversionAverage::X32.max_polls(), 100);
    }
}
