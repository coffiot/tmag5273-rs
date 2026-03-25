// Rust guideline compliant 2026-03-24

//! Configuration builder and enums for the TMAG5273 sensor.
//!
//! Use [`ConfigBuilder`] to construct a validated [`Config`], then pass it to
//! [`Tmag5273::init`](crate::Tmag5273).

use crate::types::Range;

/// I2C read mode for the TMAG5273.
///
/// Controls how the sensor formats I2C read responses. All high-level
/// read methods ([`read_magnetic`](crate::Tmag5273::read_magnetic),
/// [`read_temperature`](crate::Tmag5273::read_temperature),
/// [`read_angle`](crate::Tmag5273::read_angle)) require
/// [`Standard`](Self::Standard) mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum I2cReadMode {
    /// Standard 3-byte read (register address + 2 data bytes). Default.
    Standard = 0x00,
    /// 1-byte read returning 16-bit sensor data + conversion status.
    OneByte16Bit = 0x01,
    /// 1-byte read returning 8-bit MSB data + conversion status.
    OneByte8Bit = 0x02,
}

impl TryFrom<u8> for I2cReadMode {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, u8> {
        match val {
            0x00 => Ok(Self::Standard),
            0x01 => Ok(Self::OneByte16Bit),
            0x02 => Ok(Self::OneByte8Bit),
            other => Err(other),
        }
    }
}

/// Operating mode for the TMAG5273.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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

impl TryFrom<u8> for OperatingMode {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, u8> {
        match val {
            0x00 => Ok(Self::Standby),
            0x01 => Ok(Self::Sleep),
            0x02 => Ok(Self::ContinuousMeasure),
            0x03 => Ok(Self::WakeUpAndSleep),
            other => Err(other),
        }
    }
}

/// Magnetic channel selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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

impl TryFrom<u8> for MagneticChannel {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, u8> {
        match val {
            0x0 => Ok(Self::Off),
            0x1 => Ok(Self::X),
            0x2 => Ok(Self::Y),
            0x3 => Ok(Self::XY),
            0x4 => Ok(Self::Z),
            0x5 => Ok(Self::ZX),
            0x6 => Ok(Self::YZ),
            0x7 => Ok(Self::XYZ),
            0x8 => Ok(Self::XYX),
            0x9 => Ok(Self::YXY),
            0xA => Ok(Self::YZY),
            0xB => Ok(Self::XZX),
            other => Err(other),
        }
    }
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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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

impl TryFrom<u8> for ConversionAverage {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, u8> {
        match val {
            0b000 => Ok(Self::X1),
            0b001 => Ok(Self::X2),
            0b010 => Ok(Self::X4),
            0b011 => Ok(Self::X8),
            0b100 => Ok(Self::X16),
            0b101 => Ok(Self::X32),
            other => Err(other),
        }
    }
}

impl ConversionAverage {
    /// Returns the expected conversion time in microseconds for the given
    /// channel configuration.
    ///
    /// Formula from TI datasheet (SBASAI4 Rev C, Table 7-5):
    /// `((1 << avg) * 25) * num_channels + 25`
    /// where `avg` is the averaging exponent (0 for X1, 5 for X32) and
    /// `num_channels` includes magnetic axes plus temperature (if enabled).
    ///
    /// This follows the Zephyr RTOS driver's conversion time calculation.
    pub const fn conversion_time_us(self, channels: MagneticChannel, temperature_enabled: bool) -> u32 {
        let nb = channels.axis_count() as u32 + temperature_enabled as u32;
        ((1u32 << self as u32) * 25) * nb + 25
    }

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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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

impl TryFrom<u8> for SleepDuration {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, u8> {
        match val {
            0x0 => Ok(Self::Ms1),
            0x1 => Ok(Self::Ms5),
            0x2 => Ok(Self::Ms10),
            0x3 => Ok(Self::Ms15),
            0x4 => Ok(Self::Ms20),
            0x5 => Ok(Self::Ms30),
            0x6 => Ok(Self::Ms50),
            0x7 => Ok(Self::Ms100),
            0x8 => Ok(Self::Ms500),
            0x9 => Ok(Self::Ms1000),
            0xA => Ok(Self::Ms2000),
            0xB => Ok(Self::Ms5000),
            0xC => Ok(Self::Ms20000),
            other => Err(other),
        }
    }
}

/// Power mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PowerMode {
    /// Low active current mode (default).
    LowActiveCurrent = 0,
    /// Low noise mode.
    LowNoise = 1,
}

impl TryFrom<u8> for PowerMode {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, u8> {
        match val {
            0 => Ok(Self::LowActiveCurrent),
            1 => Ok(Self::LowNoise),
            other => Err(other),
        }
    }
}

/// Trigger mode for standby conversions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TriggerMode {
    /// I2C command triggers conversion.
    I2cCommand = 0,
    /// INT pin triggers conversion.
    IntPin = 1,
}

impl TryFrom<u8> for TriggerMode {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, u8> {
        match val {
            0 => Ok(Self::I2cCommand),
            1 => Ok(Self::IntPin),
            other => Err(other),
        }
    }
}

/// Magnetic temperature compensation coefficient.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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

impl TryFrom<u8> for TempCoefficient {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, u8> {
        match val {
            0b00 => Ok(Self::None),
            0b01 => Ok(Self::NdBFe),
            0b11 => Ok(Self::Ceramic),
            other => Err(other),
        }
    }
}

/// Angle calculation axis pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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

impl TryFrom<u8> for AngleCalculation {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, u8> {
        match val {
            0b00 => Ok(Self::None),
            0b01 => Ok(Self::XY),
            0b10 => Ok(Self::YZ),
            0b11 => Ok(Self::XZ),
            other => Err(other),
        }
    }
}

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

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
    /// Returns the expected conversion time in microseconds for this
    /// configuration.
    ///
    /// Delegates to [`ConversionAverage::conversion_time_us`] using the
    /// magnetic channel and temperature settings from this config.
    pub const fn conversion_time_us(&self) -> u32 {
        self.conversion_average
            .conversion_time_us(self.magnetic_channels, self.temperature_enabled)
    }

    /// Serialize this configuration into register address/value pairs for
    /// writing to the device.
    pub(crate) fn to_register_writes(self) -> [(u8, u8); 5] {
        use crate::register;

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

/// Threshold crossing direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ThresholdDirection {
    /// Interrupt when field exceeds threshold (above).
    Above = 0,
    /// Interrupt when field falls below threshold (below).
    Below = 1,
}

impl TryFrom<u8> for ThresholdDirection {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, u8> {
        match val {
            0 => Ok(Self::Above),
            1 => Ok(Self::Below),
            other => Err(other),
        }
    }
}

/// Per-axis threshold configuration for magnetic field and temperature.
///
/// Threshold values are signed 8-bit integers representing the crossing
/// point in sensor-native units. Use with
/// [`set_thresholds`](crate::Tmag5273::set_thresholds).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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
    /// Hysteresis level (3-bit field). Higher values reduce noise sensitivity.
    ///
    /// Valid range: 0--7. Values above 7 are silently clamped to 3 bits
    /// (`& 0x07`) when written to the device register.
    pub hysteresis: u8,
    /// Whether to trigger on field above or below the threshold.
    pub direction: ThresholdDirection,
}

// ---------------------------------------------------------------------------
// Interrupt configuration
// ---------------------------------------------------------------------------

/// Interrupt routing mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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

impl TryFrom<u8> for InterruptMode {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, u8> {
        match val {
            0b000 => Ok(Self::None),
            0b001 => Ok(Self::IntPin),
            0b010 => Ok(Self::IntPinLatched),
            0b011 => Ok(Self::Scl),
            0b100 => Ok(Self::SclLatched),
            other => Err(other),
        }
    }
}

/// INT pin assertion behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum IntPinBehavior {
    /// INT pin stays asserted until host clears it.
    Latched = 0,
    /// INT pin pulses for 10 µs then deasserts.
    Pulse10us = 1,
}

impl TryFrom<u8> for IntPinBehavior {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, u8> {
        match val {
            0 => Ok(Self::Latched),
            1 => Ok(Self::Pulse10us),
            other => Err(other),
        }
    }
}

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

/// Gain adjustment channel selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum GainChannel {
    /// Apply gain to first axis pair.
    First = 0,
    /// Apply gain to second axis pair.
    Second = 1,
}

impl TryFrom<u8> for GainChannel {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, u8> {
        match val {
            0 => Ok(Self::First),
            1 => Ok(Self::Second),
            other => Err(other),
        }
    }
}

/// Magnetic gain and offset calibration configuration.
///
/// Use with [`set_calibration`](crate::Tmag5273::set_calibration).
///
/// **Note:** The SparkFun Arduino library has a known copy-paste bug where
/// `config_mag_offset_2()` writes to register 0x0A instead of 0x0B.
/// This driver writes to the correct registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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

    // -- TryFrom<u8> round-trip tests ----------------------------------------

    #[test]
    fn operating_mode_try_from_round_trip() {
        for variant in [
            OperatingMode::Standby,
            OperatingMode::Sleep,
            OperatingMode::ContinuousMeasure,
            OperatingMode::WakeUpAndSleep,
        ] {
            assert_eq!(OperatingMode::try_from(variant as u8), Ok(variant));
        }
    }

    #[test]
    fn operating_mode_try_from_invalid() {
        assert_eq!(OperatingMode::try_from(0x04), Err(0x04));
        assert_eq!(OperatingMode::try_from(0xFF), Err(0xFF));
    }

    #[test]
    fn magnetic_channel_try_from_round_trip() {
        for variant in [
            MagneticChannel::Off,
            MagneticChannel::X,
            MagneticChannel::Y,
            MagneticChannel::XY,
            MagneticChannel::Z,
            MagneticChannel::ZX,
            MagneticChannel::YZ,
            MagneticChannel::XYZ,
            MagneticChannel::XYX,
            MagneticChannel::YXY,
            MagneticChannel::YZY,
            MagneticChannel::XZX,
        ] {
            assert_eq!(MagneticChannel::try_from(variant as u8), Ok(variant));
        }
    }

    #[test]
    fn magnetic_channel_try_from_invalid() {
        assert_eq!(MagneticChannel::try_from(0x0C), Err(0x0C));
        assert_eq!(MagneticChannel::try_from(0xFF), Err(0xFF));
    }

    #[test]
    fn conversion_average_try_from_round_trip() {
        for variant in [
            ConversionAverage::X1,
            ConversionAverage::X2,
            ConversionAverage::X4,
            ConversionAverage::X8,
            ConversionAverage::X16,
            ConversionAverage::X32,
        ] {
            assert_eq!(ConversionAverage::try_from(variant as u8), Ok(variant));
        }
    }

    #[test]
    fn conversion_average_try_from_invalid() {
        assert_eq!(ConversionAverage::try_from(0b110), Err(0b110));
        assert_eq!(ConversionAverage::try_from(0xFF), Err(0xFF));
    }

    #[test]
    fn sleep_duration_try_from_round_trip() {
        for variant in [
            SleepDuration::Ms1,
            SleepDuration::Ms5,
            SleepDuration::Ms10,
            SleepDuration::Ms15,
            SleepDuration::Ms20,
            SleepDuration::Ms30,
            SleepDuration::Ms50,
            SleepDuration::Ms100,
            SleepDuration::Ms500,
            SleepDuration::Ms1000,
            SleepDuration::Ms2000,
            SleepDuration::Ms5000,
            SleepDuration::Ms20000,
        ] {
            assert_eq!(SleepDuration::try_from(variant as u8), Ok(variant));
        }
    }

    #[test]
    fn sleep_duration_try_from_invalid() {
        assert_eq!(SleepDuration::try_from(0x0D), Err(0x0D));
        assert_eq!(SleepDuration::try_from(0xFF), Err(0xFF));
    }

    #[test]
    fn power_mode_try_from_round_trip() {
        for variant in [PowerMode::LowActiveCurrent, PowerMode::LowNoise] {
            assert_eq!(PowerMode::try_from(variant as u8), Ok(variant));
        }
    }

    #[test]
    fn power_mode_try_from_invalid() {
        assert_eq!(PowerMode::try_from(2), Err(2));
        assert_eq!(PowerMode::try_from(0xFF), Err(0xFF));
    }

    #[test]
    fn trigger_mode_try_from_round_trip() {
        for variant in [TriggerMode::I2cCommand, TriggerMode::IntPin] {
            assert_eq!(TriggerMode::try_from(variant as u8), Ok(variant));
        }
    }

    #[test]
    fn trigger_mode_try_from_invalid() {
        assert_eq!(TriggerMode::try_from(2), Err(2));
        assert_eq!(TriggerMode::try_from(0xFF), Err(0xFF));
    }

    #[test]
    fn temp_coefficient_try_from_round_trip() {
        for variant in [
            TempCoefficient::None,
            TempCoefficient::NdBFe,
            TempCoefficient::Ceramic,
        ] {
            assert_eq!(TempCoefficient::try_from(variant as u8), Ok(variant));
        }
    }

    #[test]
    fn temp_coefficient_try_from_invalid() {
        // 0b10 is reserved
        assert_eq!(TempCoefficient::try_from(0b10), Err(0b10));
        assert_eq!(TempCoefficient::try_from(0xFF), Err(0xFF));
    }

    #[test]
    fn angle_calculation_try_from_round_trip() {
        for variant in [
            AngleCalculation::None,
            AngleCalculation::XY,
            AngleCalculation::YZ,
            AngleCalculation::XZ,
        ] {
            assert_eq!(AngleCalculation::try_from(variant as u8), Ok(variant));
        }
    }

    #[test]
    fn angle_calculation_try_from_invalid() {
        assert_eq!(AngleCalculation::try_from(0x04), Err(0x04));
        assert_eq!(AngleCalculation::try_from(0xFF), Err(0xFF));
    }

    #[test]
    fn threshold_direction_try_from_round_trip() {
        for variant in [ThresholdDirection::Above, ThresholdDirection::Below] {
            assert_eq!(ThresholdDirection::try_from(variant as u8), Ok(variant));
        }
    }

    #[test]
    fn threshold_direction_try_from_invalid() {
        assert_eq!(ThresholdDirection::try_from(2), Err(2));
        assert_eq!(ThresholdDirection::try_from(0xFF), Err(0xFF));
    }

    #[test]
    fn interrupt_mode_try_from_round_trip() {
        for variant in [
            InterruptMode::None,
            InterruptMode::IntPin,
            InterruptMode::IntPinLatched,
            InterruptMode::Scl,
            InterruptMode::SclLatched,
        ] {
            assert_eq!(InterruptMode::try_from(variant as u8), Ok(variant));
        }
    }

    #[test]
    fn interrupt_mode_try_from_invalid() {
        assert_eq!(InterruptMode::try_from(0b101), Err(0b101));
        assert_eq!(InterruptMode::try_from(0xFF), Err(0xFF));
    }

    #[test]
    fn int_pin_behavior_try_from_round_trip() {
        for variant in [IntPinBehavior::Latched, IntPinBehavior::Pulse10us] {
            assert_eq!(IntPinBehavior::try_from(variant as u8), Ok(variant));
        }
    }

    #[test]
    fn int_pin_behavior_try_from_invalid() {
        assert_eq!(IntPinBehavior::try_from(2), Err(2));
        assert_eq!(IntPinBehavior::try_from(0xFF), Err(0xFF));
    }

    #[test]
    fn gain_channel_try_from_round_trip() {
        for variant in [GainChannel::First, GainChannel::Second] {
            assert_eq!(GainChannel::try_from(variant as u8), Ok(variant));
        }
    }

    #[test]
    fn gain_channel_try_from_invalid() {
        assert_eq!(GainChannel::try_from(2), Err(2));
        assert_eq!(GainChannel::try_from(0xFF), Err(0xFF));
    }

    #[test]
    fn i2c_read_mode_try_from_round_trip() {
        for variant in [
            I2cReadMode::Standard,
            I2cReadMode::OneByte16Bit,
            I2cReadMode::OneByte8Bit,
        ] {
            assert_eq!(I2cReadMode::try_from(variant as u8), Ok(variant));
        }
    }

    #[test]
    fn i2c_read_mode_try_from_invalid() {
        assert_eq!(I2cReadMode::try_from(0x03), Err(0x03));
        assert_eq!(I2cReadMode::try_from(0xFF), Err(0xFF));
    }

    #[test]
    fn range_try_from_round_trip() {
        for variant in [Range::Low, Range::High] {
            assert_eq!(Range::try_from(variant as u8), Ok(variant));
        }
    }

    #[test]
    fn range_try_from_invalid() {
        assert_eq!(Range::try_from(2), Err(2));
        assert_eq!(Range::try_from(0xFF), Err(0xFF));
    }

    // -- Conversion time formula (Zephyr-inspired) ----------------------------

    #[test]
    fn conversion_time_x1_xyz_temp() {
        // X1 averaging, XYZ + temp = 4 channels: (1*25)*4 + 25 = 125 us
        let t = ConversionAverage::X1.conversion_time_us(MagneticChannel::XYZ, true);
        assert_eq!(t, 125);
    }

    #[test]
    fn conversion_time_x32_xyz_temp() {
        // X32, 4 channels: (32*25)*4 + 25 = 3225 us
        let t = ConversionAverage::X32.conversion_time_us(MagneticChannel::XYZ, true);
        assert_eq!(t, 3225);
    }

    #[test]
    fn conversion_time_x1_single_axis() {
        // X1, X only, no temp: (1*25)*1 + 25 = 50 us
        let t = ConversionAverage::X1.conversion_time_us(MagneticChannel::X, false);
        assert_eq!(t, 50);
    }

    #[test]
    fn config_conversion_time_delegates() {
        let config = ConfigBuilder::new().build().unwrap();
        // Default: X1, XYZ, temp enabled → 125 us
        assert_eq!(config.conversion_time_us(), 125);
    }
}
