//! Public types, newtypes, and raw-to-physical conversion helpers for the
//! TMAG5273 Hall-effect sensor driver.

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

impl_try_from_u8!(I2cReadMode {
    0x00 => Standard,
    0x01 => OneByte16Bit,
    0x02 => OneByte8Bit,
});

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

impl_try_from_u8!(OperatingMode {
    0x00 => Standby,
    0x01 => Sleep,
    0x02 => ContinuousMeasure,
    0x03 => WakeUpAndSleep,
});

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
    /// Pseudo-simultaneous X-Y-X — use with [`AngleCalculation::XY`].
    XYX = 0x8,
    /// Pseudo-simultaneous Y-X-Y — use with [`AngleCalculation::XY`].
    YXY = 0x9,
    /// Pseudo-simultaneous Y-Z-Y — use with [`AngleCalculation::YZ`].
    YZY = 0xA,
    /// Pseudo-simultaneous X-Z-X — use with [`AngleCalculation::XZ`].
    XZX = 0xB,
}

impl_try_from_u8!(MagneticChannel {
    0x0 => Off,
    0x1 => X,
    0x2 => Y,
    0x3 => XY,
    0x4 => Z,
    0x5 => ZX,
    0x6 => YZ,
    0x7 => XYZ,
    0x8 => XYX,
    0x9 => YXY,
    0xA => YZY,
    0xB => XZX,
});

impl MagneticChannel {
    /// Returns the number of distinct physical axes enabled.
    ///
    /// Pseudo-simultaneous modes (XYX, YXY, YZY, XZX) measure 2 distinct
    /// axes even though they perform 3 measurement slots. Use
    /// [`measurement_slots`](Self::measurement_slots) for conversion time
    /// calculations.
    pub(crate) const fn axis_count(self) -> u8 {
        match self {
            Self::Off => 0,
            Self::X | Self::Y | Self::Z => 1,
            Self::XY | Self::ZX | Self::YZ | Self::XYX | Self::YXY | Self::YZY | Self::XZX => 2,
            Self::XYZ => 3,
        }
    }

    /// Returns the number of measurement slots per conversion cycle.
    ///
    /// Pseudo-simultaneous modes perform 3 measurements (e.g., XYX = X, Y, X).
    /// This is the value used by [`ConversionAverage::conversion_time`] since
    /// the sensor's conversion duration depends on measurement slots, not
    /// distinct axes.
    pub(crate) const fn measurement_slots(self) -> u8 {
        match self {
            Self::Off => 0,
            Self::X | Self::Y | Self::Z => 1,
            Self::XY | Self::ZX | Self::YZ => 2,
            Self::XYZ | Self::XYX | Self::YXY | Self::YZY | Self::XZX => 3,
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

impl_try_from_u8!(ConversionAverage {
    0b000 => X1,
    0b001 => X2,
    0b010 => X4,
    0b011 => X8,
    0b100 => X16,
    0b101 => X32,
});

impl ConversionAverage {
    /// Returns the expected conversion time in microseconds for the given
    /// channel configuration.
    ///
    /// Formula from TI datasheet (SBASAI4 Rev C, Table 7-5):
    /// `((1 << avg) * 25) * num_channels + 25` microseconds,
    /// where `avg` is the averaging exponent (0 for X1, 5 for X32) and
    /// `num_channels` includes magnetic axes plus temperature (if enabled).
    ///
    /// This follows the Zephyr RTOS driver's conversion time calculation.
    pub const fn conversion_time(
        self,
        channels: MagneticChannel,
        temperature_enabled: bool,
    ) -> Micros {
        let temp_channel: u32 = if temperature_enabled { 1 } else { 0 };
        let num_channels = channels.measurement_slots() as u32 + temp_channel;
        let samples_per_channel = 1u32 << (self as u32);
        let us = samples_per_channel * 25 * num_channels + 25;
        Micros(us)
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

impl_try_from_u8!(SleepDuration {
    0x0 => Ms1,
    0x1 => Ms5,
    0x2 => Ms10,
    0x3 => Ms15,
    0x4 => Ms20,
    0x5 => Ms30,
    0x6 => Ms50,
    0x7 => Ms100,
    0x8 => Ms500,
    0x9 => Ms1000,
    0xA => Ms2000,
    0xB => Ms5000,
    0xC => Ms20000,
});

impl SleepDuration {
    /// Returns the sleep duration in microseconds.
    pub const fn as_micros(self) -> Micros {
        Micros(match self {
            Self::Ms1 => 1_000,
            Self::Ms5 => 5_000,
            Self::Ms10 => 10_000,
            Self::Ms15 => 15_000,
            Self::Ms20 => 20_000,
            Self::Ms30 => 30_000,
            Self::Ms50 => 50_000,
            Self::Ms100 => 100_000,
            Self::Ms500 => 500_000,
            Self::Ms1000 => 1_000_000,
            Self::Ms2000 => 2_000_000,
            Self::Ms5000 => 5_000_000,
            Self::Ms20000 => 20_000_000,
        })
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

impl_try_from_u8!(PowerMode {
    0 => LowActiveCurrent,
    1 => LowNoise,
});

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

impl_try_from_u8!(TriggerMode {
    0 => I2cCommand,
    1 => IntPin,
});

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

impl_try_from_u8!(TempCoefficient {
    0b00 => None,
    0b01 => NdBFe,
    0b11 => Ceramic,
});

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

impl_try_from_u8!(AngleCalculation {
    0b00 => None,
    0b01 => XY,
    0b10 => YZ,
    0b11 => XZ,
});

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

impl_try_from_u8!(ThresholdDirection {
    0 => Above,
    1 => Below,
});

/// Number of threshold crossings before the interrupt is asserted
/// (maps to `THRX_COUNT` field in `SENSOR_CONFIG_2[6]`).
///
/// Use [`Four`](Self::Four) in noisy environments to debounce
/// the threshold comparison — the interrupt only fires after 4
/// consecutive threshold crossings.
///
/// Reference: TI TMAG5273 datasheet SBASAI4, Table 8-6.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ThresholdCrossingCount {
    /// Interrupt after 1 threshold crossing (default).
    #[default]
    One = 0,
    /// Interrupt after 4 threshold crossings (debounce).
    Four = 1,
}

impl_try_from_u8!(ThresholdCrossingCount {
    0 => One,
    1 => Four,
});

/// Interrupt routing mode (maps to `INT_MODE` field in `INT_CONFIG_1[4:2]`).
///
/// Controls which pin carries the interrupt signal and whether the interrupt
/// is suppressed when the I2C bus is busy with another device.
///
/// **Note:** Latched vs pulsed interrupt behavior is controlled separately
/// by [`IntPinBehavior`], not by this enum.
///
/// Reference: TI TMAG5273 datasheet SBASAI4, Table 8-11.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum InterruptMode {
    /// No interrupt (`INT_MODE = 000b`).
    None = 0b000,
    /// Interrupt through INT pin (`INT_MODE = 001b`).
    ThroughInt = 0b001,
    /// Interrupt through INT pin, except when I2C bus is busy
    /// (`INT_MODE = 010b`).
    ///
    /// The interrupt is suppressed while the I2C bus is in use by another
    /// device (not this TMAG5273). Use this mode on shared I2C buses to
    /// prevent interrupt assertions from corrupting ongoing transactions
    /// with other devices.
    ThroughIntExceptI2cBusy = 0b010,
    /// Interrupt through SCL pin (`INT_MODE = 011b`).
    ///
    /// TI does not recommend sharing the I2C bus with other devices when
    /// using SCL for interrupts — the SCL interrupt may corrupt transactions
    /// with other secondary devices.
    ThroughScl = 0b011,
    /// Interrupt through SCL pin, except when I2C bus is busy
    /// (`INT_MODE = 100b`).
    ///
    /// The interrupt and result register updates are suppressed unless the
    /// TMAG5273 is being actively addressed by the primary.
    ThroughSclExceptI2cBusy = 0b100,
}

impl_try_from_u8!(InterruptMode {
    0b000 => None,
    0b001 => ThroughInt,
    0b010 => ThroughIntExceptI2cBusy,
    0b011 => ThroughScl,
    0b100 => ThroughSclExceptI2cBusy,
});

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

impl_try_from_u8!(IntPinBehavior {
    0 => Latched,
    1 => Pulse10us,
});

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

impl_try_from_u8!(GainChannel {
    0 => First,
    1 => Second,
});

/// Per-sample diagnostic checking policy.
///
/// Controls whether the driver checks `CONV_STATUS.DIAG_FAIL` after each
/// measurement read, and what happens when a fault is detected.
///
/// | Variant  | Checks diag? | On fault                                  |
/// |----------|-------------|-------------------------------------------|
/// | `Off`    | No          | —                                         |
/// | `Halt`   | Yes         | Returns `Error::DiagnosticFailure`        |
/// | `Warn`   | Yes         | Emits `defmt::warn!` (no error returned)  |
/// | `Ignore` | Yes         | Silently discards the fault               |
///
/// `Warn` without the `defmt` feature behaves identically to `Ignore`.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Diagnostics {
    /// Diagnostics disabled — no overhead. (default)
    #[default]
    Off = 0,
    /// Check diagnostics; return [`Error::DiagnosticFailure`](crate::Error::DiagnosticFailure) on fault.
    Halt = 1,
    /// Check diagnostics; emit `defmt::warn!` on fault but do not return an error.
    ///
    /// Without the `defmt` Cargo feature this behaves like [`Ignore`](Self::Ignore).
    Warn = 2,
    /// Check diagnostics; silently discard faults.
    Ignore = 3,
}

/// Magnetic field measurement range.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Range {
    /// Low range (±40 mT for version 1, ±133 mT for version 2).
    Low = 0,
    /// High range (±80 mT for version 1, ±266 mT for version 2).
    High = 1,
}

impl_try_from_u8!(Range {
    0 => Low,
    1 => High,
});

// ---------------------------------------------------------------------------
// Device variant
// ---------------------------------------------------------------------------

/// Default I2C address for device family letter A.
const I2C_ADDR_A: u8 = 0x35;
/// Default I2C address for device family letter B.
const I2C_ADDR_B: u8 = 0x22;
/// Default I2C address for device family letter C.
const I2C_ADDR_C: u8 = 0x78;
/// Default I2C address for device family letter D.
const I2C_ADDR_D: u8 = 0x44;

/// Full-scale XY range in mT — version 1, low range.
const RANGE_V1_LOW_XY: f32 = 40.0;
/// Full-scale XY range in mT — version 1, high range.
const RANGE_V1_HIGH_XY: f32 = 80.0;
/// Full-scale XY range in mT — version 2, low range.
const RANGE_V2_LOW_XY: f32 = 133.0;
/// Full-scale XY range in mT — version 2, high range.
const RANGE_V2_HIGH_XY: f32 = 266.0;

/// Full-scale Z range in mT — version 1, low range.
const RANGE_V1_LOW_Z: f32 = 40.0;
/// Full-scale Z range in mT — version 1, high range.
const RANGE_V1_HIGH_Z: f32 = 80.0;
/// Full-scale Z range in mT — version 2, low range.
const RANGE_V2_LOW_Z: f32 = 133.0;
/// Full-scale Z range in mT — version 2, high range.
const RANGE_V2_HIGH_Z: f32 = 266.0;

/// TMAG5273 device variant.
///
/// The letter (A–D) determines the default I2C address and the number (1/2)
/// determines the magnetic sensitivity range (version).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceVariant {
    /// TMAG5273A1 — address 0x35, version 1.
    A1,
    /// TMAG5273A2 — address 0x35, version 2.
    A2,
    /// TMAG5273B1 — address 0x22, version 1.
    B1,
    /// TMAG5273B2 — address 0x22, version 2.
    B2,
    /// TMAG5273C1 — address 0x78, version 1.
    C1,
    /// TMAG5273C2 — address 0x78, version 2.
    C2,
    /// TMAG5273D1 — address 0x44, version 1.
    D1,
    /// TMAG5273D2 — address 0x44, version 2.
    D2,
}

impl DeviceVariant {
    /// Factory-default I2C addresses for all TMAG5273 families.
    ///
    /// Ordered B, A, D, C — B-family (0x22, SparkFun default) first since
    /// it's the most common breakout board.
    pub const KNOWN_ADDRESSES: [u8; 4] = [I2C_ADDR_B, I2C_ADDR_A, I2C_ADDR_D, I2C_ADDR_C];

    /// Resolve the device variant from a known I2C address and version byte.
    ///
    /// Returns `None` if the address does not match any known family (A–D)
    /// or the version is not 1 or 2.
    #[inline]
    pub const fn from_address_version(address: u8, version: u8) -> Option<Self> {
        match (address, version) {
            (I2C_ADDR_A, 1) => Some(Self::A1),
            (I2C_ADDR_A, 2) => Some(Self::A2),
            (I2C_ADDR_B, 1) => Some(Self::B1),
            (I2C_ADDR_B, 2) => Some(Self::B2),
            (I2C_ADDR_C, 1) => Some(Self::C1),
            (I2C_ADDR_C, 2) => Some(Self::C2),
            (I2C_ADDR_D, 1) => Some(Self::D1),
            (I2C_ADDR_D, 2) => Some(Self::D2),
            _ => None,
        }
    }

    /// Returns the factory-default I2C address for this variant.
    #[inline]
    pub const fn default_address(&self) -> u8 {
        match self {
            Self::A1 | Self::A2 => I2C_ADDR_A,
            Self::B1 | Self::B2 => I2C_ADDR_B,
            Self::C1 | Self::C2 => I2C_ADDR_C,
            Self::D1 | Self::D2 => I2C_ADDR_D,
        }
    }

    /// Returns the sensitivity version number (1 or 2).
    #[inline]
    pub const fn version(&self) -> u8 {
        match self {
            Self::A1 | Self::B1 | Self::C1 | Self::D1 => 1,
            Self::A2 | Self::B2 | Self::C2 | Self::D2 => 2,
        }
    }

    /// Returns the full-scale magnetic range in mT for the X and Y axes.
    #[inline]
    pub const fn range_mt_xy(&self, range: Range) -> f32 {
        match (self, range) {
            (Self::A1 | Self::B1 | Self::C1 | Self::D1, Range::Low) => RANGE_V1_LOW_XY,
            (Self::A1 | Self::B1 | Self::C1 | Self::D1, Range::High) => RANGE_V1_HIGH_XY,
            (Self::A2 | Self::B2 | Self::C2 | Self::D2, Range::Low) => RANGE_V2_LOW_XY,
            (Self::A2 | Self::B2 | Self::C2 | Self::D2, Range::High) => RANGE_V2_HIGH_XY,
        }
    }

    /// Returns the full-scale magnetic range in mT for the Z axis.
    #[inline]
    pub const fn range_mt_z(&self, range: Range) -> f32 {
        match (self, range) {
            (Self::A1 | Self::B1 | Self::C1 | Self::D1, Range::Low) => RANGE_V1_LOW_Z,
            (Self::A1 | Self::B1 | Self::C1 | Self::D1, Range::High) => RANGE_V1_HIGH_Z,
            (Self::A2 | Self::B2 | Self::C2 | Self::D2, Range::Low) => RANGE_V2_LOW_Z,
            (Self::A2 | Self::B2 | Self::C2 | Self::D2, Range::High) => RANGE_V2_HIGH_Z,
        }
    }
}

// ---------------------------------------------------------------------------
// Measurement newtypes
// ---------------------------------------------------------------------------

/// Magnetic flux density in millitesla.
#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MilliTesla(pub f32);

impl From<MilliTesla> for f32 {
    #[inline]
    fn from(v: MilliTesla) -> f32 {
        v.0
    }
}

impl MilliTesla {
    /// Construct from a raw 16-bit 2's complement ADC code (datasheet eq. 10).
    #[inline]
    pub(crate) fn from_raw(raw: i16, range_mt: f32) -> Self {
        Self(-(raw as f32) * range_mt * INV_32768)
    }

    /// Construct from the 8-bit magnitude register value (0x1B).
    ///
    /// Treats the byte as the upper 8 bits of a 16-bit signed value and
    /// applies the standard magnetic conversion. Follows the Zephyr RTOS
    /// driver's `tmag5273_magnitude_convert()` pattern.
    #[inline]
    pub(crate) fn from_magnitude_raw(raw: u8, range_mt: f32) -> Self {
        Self::from_raw(((raw as u16) << 8) as i16, range_mt)
    }
}

/// Software `sqrt` for `no_std` — fast inverse square root (Quake-style)
/// refined with three Newton–Raphson iterations for maximum `f32` precision
/// (~1 ULP error, matching hardware `fsqrt` accuracy).
#[inline]
pub(crate) fn sqrt(x: f32) -> f32 {
    if x <= 0.0 {
        return 0.0;
    }
    let half = 0.5 * x;
    let bits = x.to_bits();
    let guess = f32::from_bits(0x5f37_5a86 - (bits >> 1)); // initial estimate
    let guess = guess * (1.5 - half * guess * guess); // 1st Newton iteration
    let guess = guess * (1.5 - half * guess * guess); // 2nd Newton iteration
    let guess = guess * (1.5 - half * guess * guess); // 3rd Newton iteration
    x * guess // convert 1/sqrt(x) → sqrt(x)
}

/// Temperature in degrees Celsius.
#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Celsius(pub f32);

impl From<Celsius> for f32 {
    #[inline]
    fn from(v: Celsius) -> f32 {
        v.0
    }
}

impl Celsius {
    /// Construct from a raw 16-bit 2's complement ADC code (datasheet eq. 12).
    #[inline]
    pub(crate) fn from_raw(raw: i16) -> Self {
        Self(TSENSE_T0 + ((raw as f32) - TADC_T0) * TADC_RES_INV)
    }
}

/// Angle in degrees (0–360).
#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Degrees(pub f32);

impl From<Degrees> for f32 {
    #[inline]
    fn from(v: Degrees) -> f32 {
        v.0
    }
}

impl Degrees {
    /// Construct from a raw 16-bit angle register value (9.4 fixed-point).
    ///
    /// Bits \[12:4\]: 9-bit integer part (0–360).
    /// Bits \[3:0\]: 4-bit fractional part (resolution 1/16 degree).
    #[inline]
    pub(crate) fn from_raw(raw: u16) -> Self {
        let integer = ((raw >> ANGLE_INT_SHIFT) & ANGLE_INT_MASK) as f32;
        let fraction = (raw & ANGLE_FRAC_MASK) as f32 * ANGLE_FRAC_SCALE;
        Self(integer + fraction)
    }
}

/// Duration in microseconds.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Micros(pub u32);

impl From<Micros> for u32 {
    #[inline]
    fn from(v: Micros) -> u32 {
        v.0
    }
}

impl core::fmt::Display for Micros {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// A range of microsecond durations, matching Linux `usleep_range(min, max)` semantics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MicrosRange {
    /// Minimum delay (used by the driver for actual `delay_us` calls).
    pub min: Micros,
    /// Maximum acceptable delay (documentary — for RTOS scheduler batching).
    pub max: Micros,
}

/// Wake-up delay range after transitioning from Sleep to an active mode.
///
/// From TI TMAG5273 datasheet: device needs >=80 µs to wake. The Linux kernel
/// driver uses `usleep_range(80, 200)` for this boundary.
pub const WAKE_DELAY: MicrosRange = MicrosRange {
    min: Micros(80),
    max: Micros(200),
};

/// Zero-sized no-op delay backend (default).
///
/// All `delay_*()` calls compile to nothing. When used as the delay backend
/// for [`Tmag5273`](crate::Tmag5273), the driver falls back to bounded I2C
/// polling for conversion completion and does not insert wake-up delays.
///
/// This is the default — existing callers need no changes.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NoDelay;

impl embedded_hal::delay::DelayNs for NoDelay {
    #[inline]
    fn delay_ns(&mut self, _ns: u32) {}
}

// ---------------------------------------------------------------------------
// Composite reading structs
// ---------------------------------------------------------------------------

/// Magnetic field reading from all enabled axes.
///
/// Disabled channels are represented as `None`.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[must_use]
pub struct MagneticReading {
    /// X-axis magnetic flux density, or `None` if the channel is disabled.
    pub x: Option<MilliTesla>,
    /// Y-axis magnetic flux density, or `None` if the channel is disabled.
    pub y: Option<MilliTesla>,
    /// Z-axis magnetic flux density, or `None` if the channel is disabled.
    pub z: Option<MilliTesla>,
}

impl MagneticReading {
    /// Compute the total magnetic field magnitude: `sqrt(x² + y² + z²)`.
    ///
    /// Disabled channels (represented as `None`) are treated as zero.
    /// Uses a software `sqrt` implementation — no `libm` or `std` required.
    ///
    /// # Examples
    ///
    /// ```
    /// use tmag5273::{MagneticReading, MilliTesla};
    ///
    /// let reading = MagneticReading {
    ///     x: Some(MilliTesla(3.0)),
    ///     y: Some(MilliTesla(4.0)),
    ///     z: None,
    /// };
    /// let mag = reading.magnitude();
    /// assert!((mag.0 - 5.0).abs() < 0.01); // 3-4-5 triangle
    /// ```
    #[inline]
    pub fn magnitude(&self) -> MilliTesla {
        let x = self.x.map_or(0.0, |v| v.0);
        let y = self.y.map_or(0.0, |v| v.0);
        let z = self.z.map_or(0.0, |v| v.0);
        MilliTesla(sqrt(x * x + y * y + z * z))
    }
}

/// Angle measurement result from the TMAG5273 angle engine.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[must_use]
pub struct AngleReading {
    /// Computed angle in degrees (0–360).
    pub angle: Degrees,
    /// Magnitude of the magnetic field vector (unitless, 0–255).
    pub magnitude: u8,
}

impl AngleReading {
    /// Convert the raw magnitude to physical units (millitesla).
    ///
    /// `range_mt` is the full-scale magnetic range for the axis pair used
    /// in the angle calculation. Obtain it from
    /// [`DeviceVariant::range_mt_xy`] or [`DeviceVariant::range_mt_z`].
    #[inline]
    pub fn magnitude_millitesla(&self, range_mt: f32) -> MilliTesla {
        MilliTesla::from_magnitude_raw(self.magnitude, range_mt)
    }
}

/// Complete sensor reading from a single conversion cycle.
///
/// All values are guaranteed to come from the same conversion — no risk
/// of data from different cycles mixing. Produced by
/// [`Tmag5273::read_all`](crate::Tmag5273::read_all).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[must_use]
pub struct SensorReading {
    /// Temperature in degrees Celsius, or `None` if the channel is disabled.
    pub temperature: Option<Celsius>,
    /// Magnetic field reading from all enabled axes.
    pub magnetic: MagneticReading,
}

// ---------------------------------------------------------------------------
// Status structs
// ---------------------------------------------------------------------------

/// Parsed contents of the `DEVICE_STATUS` register (0x1C).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[must_use]
pub struct DeviceStatus {
    /// VCC supply is below the undervoltage threshold.
    pub vcc_undervoltage: bool,
    /// OTP memory CRC check failed.
    pub otp_crc_error: bool,
    /// Internal error detected.
    pub int_error: bool,
    /// Oscillator error detected.
    pub oscillator_error: bool,
    /// Current state of the INT pin (true = asserted/active).
    pub int_pin_active: bool,
}

/// Bit position of VCC undervoltage flag in `DEVICE_STATUS`.
const DEVICE_STATUS_VCC_UV_BIT: u8 = 0;
/// Bit position of OTP CRC error flag in `DEVICE_STATUS`.
const DEVICE_STATUS_OTP_CRC_BIT: u8 = 1;
/// Bit position of internal error flag in `DEVICE_STATUS`.
const DEVICE_STATUS_INT_ERR_BIT: u8 = 2;
/// Bit position of oscillator error flag in `DEVICE_STATUS`.
const DEVICE_STATUS_OSC_ERR_BIT: u8 = 3;
/// Bit position of INT pin readback flag in `DEVICE_STATUS`.
const DEVICE_STATUS_INTB_RB_BIT: u8 = 4;

impl DeviceStatus {
    /// Returns `true` if no fault flags are set.
    #[inline]
    pub const fn is_ok(&self) -> bool {
        !self.vcc_undervoltage && !self.otp_crc_error && !self.int_error && !self.oscillator_error
    }

    /// Parses a raw `DEVICE_STATUS` register byte into structured flags.
    #[inline]
    pub const fn from_register(register: u8) -> Self {
        Self {
            vcc_undervoltage: (register >> DEVICE_STATUS_VCC_UV_BIT) & 1 != 0,
            otp_crc_error: (register >> DEVICE_STATUS_OTP_CRC_BIT) & 1 != 0,
            int_error: (register >> DEVICE_STATUS_INT_ERR_BIT) & 1 != 0,
            oscillator_error: (register >> DEVICE_STATUS_OSC_ERR_BIT) & 1 != 0,
            int_pin_active: (register >> DEVICE_STATUS_INTB_RB_BIT) & 1 != 0,
        }
    }
}

/// Parsed contents of the `CONV_STATUS` register (0x18).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[must_use]
pub struct ConversionStatus {
    /// Rolling conversion set counter (0–7).
    pub set_count: u8,
    /// A power-on-reset event was detected since the last read.
    pub power_on_reset: bool,
    /// Diagnostic self-test failed.
    pub diag_fail: bool,
    /// A new conversion result is available.
    pub result_ready: bool,
}

// CONV_STATUS register bit layout (from TI datasheet SBASAI4 Rev C):
//   [7:5] SET_COUNT   — rolling conversion counter (3 bits)
//   [4]   POR         — power-on-reset flag
//   [3:2] reserved
//   [1]   DIAG_STATUS — diagnostic self-test fail
//   [0]   RESULT_STATUS — conversion result ready

/// Bit shift for the 3-bit set counter (bits [7:5]).
const CONV_STATUS_SET_COUNT_SHIFT: u8 = 5;
/// Mask for the set counter AFTER shifting.
const CONV_STATUS_SET_COUNT_MASK: u8 = 0x07;
/// Bit position of the power-on-reset flag (bit 4).
const CONV_STATUS_POR_BIT: u8 = 4;
/// Bit position of the diagnostic fail flag (bit 1).
const CONV_STATUS_DIAG_BIT: u8 = 1;
/// Bit position of the result-ready flag (bit 0).
const CONV_STATUS_READY_BIT: u8 = 0;

impl ConversionStatus {
    /// Returns `true` if a new conversion result is available.
    #[inline]
    pub const fn is_ready(&self) -> bool {
        self.result_ready
    }

    /// Parses a raw `CONV_STATUS` register byte into structured fields.
    #[inline]
    pub const fn from_register(register: u8) -> Self {
        Self {
            set_count: (register >> CONV_STATUS_SET_COUNT_SHIFT) & CONV_STATUS_SET_COUNT_MASK,
            power_on_reset: (register >> CONV_STATUS_POR_BIT) & 1 != 0,
            diag_fail: (register >> CONV_STATUS_DIAG_BIT) & 1 != 0,
            result_ready: (register >> CONV_STATUS_READY_BIT) & 1 != 0,
        }
    }
}

// ---------------------------------------------------------------------------
// Conversion constants (precomputed reciprocals for FPU-less targets)
// ---------------------------------------------------------------------------

/// Reciprocal of 32768 — avoids runtime division in magnetic conversion.
const INV_32768: f32 = 1.0 / 32768.0;

/// Temperature sensor reference temperature (degrees Celsius).
const TSENSE_T0: f32 = 25.0;

/// Temperature ADC code at the reference temperature.
const TADC_T0: f32 = 17508.0;

/// Reciprocal of the temperature ADC resolution (60.1 codes per degree C).
const TADC_RES_INV: f32 = 1.0 / 60.1;

/// Bit mask for the 9-bit integer part of the angle result.
const ANGLE_INT_MASK: u16 = 0x1FF;

/// Bit shift to extract the integer part of the angle result.
const ANGLE_INT_SHIFT: u16 = 4;

/// Bit mask for the 4-bit fractional part of the angle result.
const ANGLE_FRAC_MASK: u16 = 0x0F;

/// Reciprocal of 16 — converts the 4-bit fractional angle field to degrees.
const ANGLE_FRAC_SCALE: f32 = 1.0 / 16.0;

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Floating-point comparison tolerance.
    const EPSILON: f32 = 0.15;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPSILON
    }

    // -- DeviceVariant: default addresses ------------------------------------

    #[test]
    fn variant_a1_address() {
        assert_eq!(DeviceVariant::A1.default_address(), 0x35);
    }

    #[test]
    fn variant_a2_address() {
        assert_eq!(DeviceVariant::A2.default_address(), 0x35);
    }

    #[test]
    fn variant_b1_address() {
        assert_eq!(DeviceVariant::B1.default_address(), 0x22);
    }

    #[test]
    fn variant_b2_address() {
        assert_eq!(DeviceVariant::B2.default_address(), 0x22);
    }

    #[test]
    fn variant_c1_address() {
        assert_eq!(DeviceVariant::C1.default_address(), 0x78);
    }

    #[test]
    fn variant_c2_address() {
        assert_eq!(DeviceVariant::C2.default_address(), 0x78);
    }

    #[test]
    fn variant_d1_address() {
        assert_eq!(DeviceVariant::D1.default_address(), 0x44);
    }

    #[test]
    fn variant_d2_address() {
        assert_eq!(DeviceVariant::D2.default_address(), 0x44);
    }

    // -- DeviceVariant: version ----------------------------------------------

    #[test]
    fn version_1_variants() {
        assert_eq!(DeviceVariant::A1.version(), 1);
        assert_eq!(DeviceVariant::B1.version(), 1);
        assert_eq!(DeviceVariant::C1.version(), 1);
        assert_eq!(DeviceVariant::D1.version(), 1);
    }

    #[test]
    fn version_2_variants() {
        assert_eq!(DeviceVariant::A2.version(), 2);
        assert_eq!(DeviceVariant::B2.version(), 2);
        assert_eq!(DeviceVariant::C2.version(), 2);
        assert_eq!(DeviceVariant::D2.version(), 2);
    }

    // -- DeviceVariant: from_address_version ----------------------------------

    #[test]
    fn from_address_version_all_variants() {
        assert_eq!(
            DeviceVariant::from_address_version(0x35, 1),
            Some(DeviceVariant::A1)
        );
        assert_eq!(
            DeviceVariant::from_address_version(0x35, 2),
            Some(DeviceVariant::A2)
        );
        assert_eq!(
            DeviceVariant::from_address_version(0x22, 1),
            Some(DeviceVariant::B1)
        );
        assert_eq!(
            DeviceVariant::from_address_version(0x22, 2),
            Some(DeviceVariant::B2)
        );
        assert_eq!(
            DeviceVariant::from_address_version(0x78, 1),
            Some(DeviceVariant::C1)
        );
        assert_eq!(
            DeviceVariant::from_address_version(0x78, 2),
            Some(DeviceVariant::C2)
        );
        assert_eq!(
            DeviceVariant::from_address_version(0x44, 1),
            Some(DeviceVariant::D1)
        );
        assert_eq!(
            DeviceVariant::from_address_version(0x44, 2),
            Some(DeviceVariant::D2)
        );
    }

    #[test]
    fn from_address_version_unknown_address() {
        assert_eq!(DeviceVariant::from_address_version(0x30, 1), None);
    }

    #[test]
    fn from_address_version_unknown_version() {
        assert_eq!(DeviceVariant::from_address_version(0x22, 3), None);
        assert_eq!(DeviceVariant::from_address_version(0x22, 0), None);
    }

    // -- DeviceVariant: range tables -----------------------------------------

    #[test]
    fn range_v1_low_xy() {
        assert_eq!(DeviceVariant::A1.range_mt_xy(Range::Low), 40.0);
    }

    #[test]
    fn range_v1_high_xy() {
        assert_eq!(DeviceVariant::B1.range_mt_xy(Range::High), 80.0);
    }

    #[test]
    fn range_v2_low_xy() {
        assert_eq!(DeviceVariant::C2.range_mt_xy(Range::Low), 133.0);
    }

    #[test]
    fn range_v2_high_xy() {
        assert_eq!(DeviceVariant::D2.range_mt_xy(Range::High), 266.0);
    }

    #[test]
    fn range_v1_low_z() {
        assert_eq!(DeviceVariant::A1.range_mt_z(Range::Low), 40.0);
    }

    #[test]
    fn range_v1_high_z() {
        assert_eq!(DeviceVariant::B1.range_mt_z(Range::High), 80.0);
    }

    #[test]
    fn range_v2_low_z() {
        assert_eq!(DeviceVariant::C2.range_mt_z(Range::Low), 133.0);
    }

    #[test]
    fn range_v2_high_z() {
        assert_eq!(DeviceVariant::D2.range_mt_z(Range::High), 266.0);
    }

    // -- Temperature conversion ----------------------------------------------

    #[test]
    fn temp_reference_point() {
        let c = Celsius::from_raw(17508);
        assert!(approx_eq(c.0, 25.0), "expected ~25.0, got {}", c.0);
    }

    #[test]
    fn temp_raw_zero() {
        // T = 25.0 + (0 - 17508) / 60.1 ≈ -266.36
        let c = Celsius::from_raw(0);
        assert!(approx_eq(c.0, -266.36), "expected ~-266.36, got {}", c.0);
    }

    #[test]
    fn temp_raw_max() {
        // T = 25.0 + (32767 - 17508) / 60.1 ≈ 279.0
        let c = Celsius::from_raw(i16::MAX);
        assert!(approx_eq(c.0, 279.0), "expected ~279.0, got {}", c.0);
    }

    // -- Magnetic conversion -------------------------------------------------

    #[test]
    fn from_raw_zero_returns_zero_mt() {
        let m = MilliTesla::from_raw(0, 40.0);
        assert_eq!(m.0, 0.0);
    }

    #[test]
    fn from_raw_i16_min_returns_positive_range() {
        // raw = i16::MIN = -32768 → B = -(-32768) * range / 32768 = +range
        let m = MilliTesla::from_raw(i16::MIN, 40.0);
        assert!(approx_eq(m.0, 40.0), "expected ~40.0, got {}", m.0);
    }

    #[test]
    fn from_raw_i16_max_returns_negative_range() {
        // raw = 32767 → B = -(32767) * 40 / 32768 ≈ -39.9988
        let m = MilliTesla::from_raw(i16::MAX, 40.0);
        assert!(approx_eq(m.0, -40.0), "expected ~-40.0, got {}", m.0);
    }

    #[test]
    fn from_raw_i16_min_v2_high_returns_266mt() {
        let m = MilliTesla::from_raw(i16::MIN, 266.0);
        assert!(approx_eq(m.0, 266.0), "expected ~266.0, got {}", m.0);
    }

    // -- Angle conversion ----------------------------------------------------

    #[test]
    fn angle_zero() {
        let d = Degrees::from_raw(0x0000);
        assert_eq!(d.0, 0.0);
    }

    #[test]
    fn angle_180() {
        // 180 << 4 = 0x0B40
        let d = Degrees::from_raw(0x0B40);
        assert!(approx_eq(d.0, 180.0), "expected 180.0, got {}", d.0);
    }

    #[test]
    fn angle_360() {
        // 360 << 4 = 0x1680
        let d = Degrees::from_raw(0x1680);
        assert!(approx_eq(d.0, 360.0), "expected 360.0, got {}", d.0);
    }

    #[test]
    fn angle_fractional() {
        // 90 degrees + 8/16 = 90.5 degrees → (90 << 4) | 8 = 0x05A8
        let d = Degrees::from_raw(0x05A8);
        assert!(approx_eq(d.0, 90.5), "expected 90.5, got {}", d.0);
    }

    // -- DeviceStatus parsing ------------------------------------------------

    #[test]
    fn device_status_all_clear() {
        let s = DeviceStatus::from_register(0x00);
        assert!(!s.vcc_undervoltage);
        assert!(!s.otp_crc_error);
        assert!(!s.int_error);
        assert!(!s.oscillator_error);
        assert!(!s.int_pin_active);
    }

    #[test]
    fn device_status_all_set() {
        let s = DeviceStatus::from_register(0x1F);
        assert!(s.vcc_undervoltage);
        assert!(s.otp_crc_error);
        assert!(s.int_error);
        assert!(s.oscillator_error);
        assert!(s.int_pin_active);
    }

    #[test]
    fn device_status_individual_bits() {
        let s = DeviceStatus::from_register(0b0000_0001);
        assert!(s.vcc_undervoltage);
        assert!(!s.otp_crc_error);

        let s = DeviceStatus::from_register(0b0000_0010);
        assert!(!s.vcc_undervoltage);
        assert!(s.otp_crc_error);

        let s = DeviceStatus::from_register(0b0000_0100);
        assert!(s.int_error);
        assert!(!s.oscillator_error);

        let s = DeviceStatus::from_register(0b0000_1000);
        assert!(!s.int_error);
        assert!(s.oscillator_error);
    }

    #[test]
    fn device_status_int_pin_active() {
        let s = DeviceStatus::from_register(0b0001_0000);
        assert!(s.int_pin_active);
        assert!(s.is_ok()); // INT is not a fault
    }

    // -- ConversionStatus parsing --------------------------------------------

    #[test]
    fn conv_status_all_clear() {
        let s = ConversionStatus::from_register(0x00);
        assert_eq!(s.set_count, 0);
        assert!(!s.power_on_reset);
        assert!(!s.diag_fail);
        assert!(!s.result_ready);
    }

    #[test]
    fn conv_status_set_count() {
        // set_count occupies bits [7:5] — value 5 = 0b101 << 5 = 0xA0
        let s = ConversionStatus::from_register(0b1010_0000);
        assert_eq!(s.set_count, 5);
        assert!(!s.power_on_reset);
    }

    #[test]
    fn conv_status_set_count_max() {
        // 7 = 0b111 << 5 = 0xE0
        let s = ConversionStatus::from_register(0b1110_0000);
        assert_eq!(s.set_count, 7);
    }

    #[test]
    fn conv_status_power_on_reset() {
        // POR = bit 4 = 0x10
        let s = ConversionStatus::from_register(0b0001_0000);
        assert_eq!(s.set_count, 0);
        assert!(s.power_on_reset);
        assert!(!s.diag_fail);
        assert!(!s.result_ready);
    }

    #[test]
    fn conv_status_diag_fail() {
        // DIAG_STATUS = bit 1 = 0x02
        let s = ConversionStatus::from_register(0b0000_0010);
        assert!(s.diag_fail);
        assert!(!s.result_ready);
    }

    #[test]
    fn conv_status_result_ready() {
        // RESULT_STATUS = bit 0 = 0x01
        let s = ConversionStatus::from_register(0b0000_0001);
        assert!(s.result_ready);
        assert!(!s.diag_fail);
    }

    #[test]
    fn conv_status_all_flags() {
        // SET_COUNT=7 (bits [7:5]) | POR (bit 4) | DIAG (bit 1) | RESULT (bit 0)
        // = 0b1110_0000 | 0b0001_0000 | 0b0000_0010 | 0b0000_0001 = 0xF3
        let s = ConversionStatus::from_register(0b1111_0011);
        assert_eq!(s.set_count, 7);
        assert!(s.power_on_reset);
        assert!(s.diag_fail);
        assert!(s.result_ready);
    }

    // -- Magnitude conversion (Zephyr-inspired) ------------------------------

    #[test]
    fn from_magnitude_raw_zero_returns_zero() {
        let m = MilliTesla::from_magnitude_raw(0, 80.0);
        assert_eq!(m.0, 0.0);
    }

    #[test]
    fn from_magnitude_raw_127_returns_negative() {
        // raw=127, range=80.0 → (127 << 8) = 32512 as i16
        // B = -(32512) * 80 / 32768 ≈ -79.375 mT
        let m = MilliTesla::from_magnitude_raw(127, 80.0);
        assert!(approx_eq(m.0, -79.375), "expected ~-79.375, got {}", m.0);
    }

    // -- Wake delay constants ------------------------------------------------

    #[test]
    fn wake_delay_constants() {
        assert_eq!(WAKE_DELAY.min.0, 80);
        assert_eq!(WAKE_DELAY.max.0, 200);
    }

    // -- sqrt edge cases -------------------------------------------------

    // Note: sqrt is a crate-private function, imported via `use super::*`

    #[test]
    fn sqrt_zero() {
        assert_eq!(sqrt(0.0), 0.0);
    }

    #[test]
    fn sqrt_negative_returns_zero() {
        assert_eq!(sqrt(-1.0), 0.0);
        assert_eq!(sqrt(-100.0), 0.0);
        assert_eq!(sqrt(f32::NEG_INFINITY), 0.0);
    }

    #[test]
    fn sqrt_perfect_squares() {
        assert!((sqrt(1.0) - 1.0).abs() < 0.001);
        assert!((sqrt(4.0) - 2.0).abs() < 0.001);
        assert!((sqrt(9.0) - 3.0).abs() < 0.001);
        assert!((sqrt(25.0) - 5.0).abs() < 0.001);
        assert!((sqrt(100.0) - 10.0).abs() < 0.001);
    }

    #[test]
    fn sqrt_non_perfect() {
        assert!((sqrt(2.0) - 1.41421).abs() < 0.001);
        assert!((sqrt(3.0) - 1.73205).abs() < 0.001);
        assert!((sqrt(0.5) - 0.70711).abs() < 0.001);
    }

    #[test]
    fn sqrt_very_small() {
        assert!((sqrt(1e-6) - 1e-3).abs() < 1e-5);
    }

    #[test]
    fn sqrt_large_values() {
        assert!((sqrt(10000.0) - 100.0).abs() < 0.01);
        assert!((sqrt(1e10) - 1e5).abs() < 1.0);
    }

    #[test]
    fn sqrt_sensor_typical_values() {
        // Typical magnitude_squared values from TMAG5273 readings
        assert!((sqrt(0.1) - 0.31623).abs() < 0.001); // ~0.3 mT ambient
        assert!((sqrt(6400.0) - 80.0).abs() < 0.01); // full-scale v1
        assert!((sqrt(70756.0) - 266.0).abs() < 0.1); // full-scale v2
    }

    // -- MagneticReading::magnitude edge cases --------------------------------

    #[test]
    fn magnitude_all_none() {
        let r = MagneticReading {
            x: None,
            y: None,
            z: None,
        };
        assert_eq!(r.magnitude().0, 0.0);
    }

    #[test]
    fn magnitude_single_axis() {
        let r = MagneticReading {
            x: Some(MilliTesla(5.0)),
            y: None,
            z: None,
        };
        assert!((r.magnitude().0 - 5.0).abs() < 0.01);
    }

    #[test]
    fn magnitude_negative_fields() {
        let r = MagneticReading {
            x: Some(MilliTesla(-3.0)),
            y: Some(MilliTesla(-4.0)),
            z: None,
        };
        // sqrt(9 + 16) = 5, sign doesn't matter
        assert!((r.magnitude().0 - 5.0).abs() < 0.01);
    }

    #[test]
    fn magnitude_all_axes() {
        let r = MagneticReading {
            x: Some(MilliTesla(1.0)),
            y: Some(MilliTesla(2.0)),
            z: Some(MilliTesla(2.0)),
        };
        // sqrt(1 + 4 + 4) = 3
        assert!((r.magnitude().0 - 3.0).abs() < 0.01);
    }

    #[test]
    fn magnitude_zero_fields() {
        let r = MagneticReading {
            x: Some(MilliTesla(0.0)),
            y: Some(MilliTesla(0.0)),
            z: Some(MilliTesla(0.0)),
        };
        assert_eq!(r.magnitude().0, 0.0);
    }

    // -- MilliTesla::from_magnitude_raw overflow edge cases -------------------

    #[test]
    fn magnitude_raw_128_no_overflow() {
        // This was the overflow bug: (128_i16) << 8 = 32768 > i16::MAX
        // Fixed by casting through u16: ((128_u16) << 8) as i16 = -32768
        let m = MilliTesla::from_magnitude_raw(128, 80.0);
        assert!(approx_eq(m.0, 80.0), "expected ~80.0, got {}", m.0);
    }

    #[test]
    fn magnitude_raw_255_no_overflow() {
        // Maximum raw value: ((255_u16) << 8) as i16 = -256
        let m = MilliTesla::from_magnitude_raw(255, 80.0);
        assert!(m.0.is_finite(), "expected finite, got {}", m.0);
    }

    // -- MilliTesla::from_raw boundary values ---------------------------------

    #[test]
    fn millitesla_max_positive_raw() {
        // i16::MAX = 32767 → -(32767) * 80 / 32768 ≈ -79.9976 mT
        let m = MilliTesla::from_raw(i16::MAX, 80.0);
        assert!(m.0 < 0.0, "max positive raw should give negative mT");
        assert!(approx_eq(m.0, -80.0), "expected ~-80.0, got {}", m.0);
    }

    #[test]
    fn millitesla_min_negative_raw() {
        // i16::MIN = -32768 → -(-32768) * 80 / 32768 = 80.0 mT
        let m = MilliTesla::from_raw(i16::MIN, 80.0);
        assert!(m.0 > 0.0, "min negative raw should give positive mT");
        assert!(approx_eq(m.0, 80.0), "expected ~80.0, got {}", m.0);
    }

    #[test]
    fn millitesla_zero_raw() {
        let m = MilliTesla::from_raw(0, 80.0);
        assert_eq!(m.0, 0.0);
    }

    #[test]
    fn millitesla_zero_range() {
        let m = MilliTesla::from_raw(16384, 0.0);
        assert_eq!(m.0, 0.0);
    }

    // -- Celsius::from_raw boundary values ------------------------------------

    // -- From impls -----------------------------------------------------------

    #[test]
    fn millitesla_into_f32() {
        assert_eq!(f32::from(MilliTesla(3.14)), 3.14);
    }

    #[test]
    fn celsius_into_f32() {
        assert_eq!(f32::from(Celsius(25.0)), 25.0);
    }

    #[test]
    fn degrees_into_f32() {
        assert_eq!(f32::from(Degrees(180.0)), 180.0);
    }

    #[test]
    fn micros_into_u32() {
        assert_eq!(u32::from(Micros(1000)), 1000);
    }

    // -- AngleReading::magnitude_millitesla -----------------------------------

    #[test]
    fn angle_reading_magnitude_millitesla() {
        let reading = AngleReading {
            angle: Degrees(90.0),
            magnitude: 128,
        };
        // from_magnitude_raw(128, 80.0) = 80.0 mT (tested above)
        let m = reading.magnitude_millitesla(80.0);
        assert!(approx_eq(m.0, 80.0), "expected ~80.0, got {}", m.0);
    }

    #[test]
    fn angle_reading_magnitude_millitesla_zero() {
        let reading = AngleReading {
            angle: Degrees(0.0),
            magnitude: 0,
        };
        assert_eq!(reading.magnitude_millitesla(80.0).0, 0.0);
    }

    // -- MagneticChannel helpers ----------------------------------------------

    #[test]
    fn magnetic_channel_axis_count() {
        assert_eq!(MagneticChannel::Off.axis_count(), 0);
        assert_eq!(MagneticChannel::X.axis_count(), 1);
        assert_eq!(MagneticChannel::XY.axis_count(), 2);
        assert_eq!(MagneticChannel::XYZ.axis_count(), 3);
        // Pseudo-sim: 2 distinct axes even though 3 measurement slots
        assert_eq!(MagneticChannel::XYX.axis_count(), 2);
        assert_eq!(MagneticChannel::YXY.axis_count(), 2);
        assert_eq!(MagneticChannel::YZY.axis_count(), 2);
        assert_eq!(MagneticChannel::XZX.axis_count(), 2);
    }

    #[test]
    fn magnetic_channel_measurement_slots() {
        assert_eq!(MagneticChannel::Off.measurement_slots(), 0);
        assert_eq!(MagneticChannel::X.measurement_slots(), 1);
        assert_eq!(MagneticChannel::XY.measurement_slots(), 2);
        assert_eq!(MagneticChannel::XYZ.measurement_slots(), 3);
        // Pseudo-sim: 3 measurement slots (e.g. X, Y, X)
        assert_eq!(MagneticChannel::XYX.measurement_slots(), 3);
        assert_eq!(MagneticChannel::YXY.measurement_slots(), 3);
        assert_eq!(MagneticChannel::YZY.measurement_slots(), 3);
        assert_eq!(MagneticChannel::XZX.measurement_slots(), 3);
    }

    #[test]
    fn magnetic_channel_has_axes_all_variants() {
        //                          has_x  has_y  has_z
        let cases: &[(MagneticChannel, bool, bool, bool)] = &[
            (MagneticChannel::Off, false, false, false),
            (MagneticChannel::X, true, false, false),
            (MagneticChannel::Y, false, true, false),
            (MagneticChannel::XY, true, true, false),
            (MagneticChannel::Z, false, false, true),
            (MagneticChannel::ZX, true, false, true),
            (MagneticChannel::YZ, false, true, true),
            (MagneticChannel::XYZ, true, true, true),
            // Pseudo-simultaneous: physical axes measured, not register layout
            (MagneticChannel::XYX, true, true, false), // measures X, Y
            (MagneticChannel::YXY, true, true, false), // measures Y, X
            (MagneticChannel::YZY, false, true, true), // measures Y, Z
            (MagneticChannel::XZX, true, false, true), // measures X, Z
        ];
        for &(ch, x, y, z) in cases {
            assert_eq!(ch.has_x(), x, "{ch:?}.has_x()");
            assert_eq!(ch.has_y(), y, "{ch:?}.has_y()");
            assert_eq!(ch.has_z(), z, "{ch:?}.has_z()");
        }
    }

    #[test]
    fn i2c_read_mode_try_from() {
        for v in [
            I2cReadMode::Standard,
            I2cReadMode::OneByte16Bit,
            I2cReadMode::OneByte8Bit,
        ] {
            assert_eq!(I2cReadMode::try_from(v as u8), Ok(v));
        }
        assert_eq!(I2cReadMode::try_from(0x03), Err(0x03));
    }

    // -- Exhaustive from_register fuzz ----------------------------------------

    #[test]
    fn fuzz_device_status_from_register() {
        for byte in 0u8..=255 {
            let s = DeviceStatus::from_register(byte);

            // All fields must be valid bools (no panic) — the mere act of
            // reading them is the check. Additionally verify is_ok() doesn't
            // panic.
            let _ = s.vcc_undervoltage;
            let _ = s.otp_crc_error;
            let _ = s.int_error;
            let _ = s.oscillator_error;
            let _ = s.int_pin_active;
            let _ = s.is_ok();
        }

        // Spot-check: byte 0x00 — all flags clear.
        let s = DeviceStatus::from_register(0x00);
        assert!(!s.vcc_undervoltage);
        assert!(!s.otp_crc_error);
        assert!(!s.int_error);
        assert!(!s.oscillator_error);
        assert!(!s.int_pin_active);
        assert!(s.is_ok());

        // Spot-check: byte 0x1F — all five defined bits set.
        let s = DeviceStatus::from_register(0x1F);
        assert!(s.vcc_undervoltage);
        assert!(s.otp_crc_error);
        assert!(s.int_error);
        assert!(s.oscillator_error);
        assert!(s.int_pin_active);
        assert!(!s.is_ok()); // faults present
    }

    #[test]
    fn fuzz_conversion_status_from_register() {
        for byte in 0u8..=255 {
            let s = ConversionStatus::from_register(byte);

            // set_count must always be in 0..=7 (3-bit field).
            assert!(
                s.set_count <= 7,
                "set_count out of range for byte 0x{byte:02X}: {}",
                s.set_count
            );

            // Bool fields readable without panic.
            let _ = s.power_on_reset;
            let _ = s.diag_fail;
            let _ = s.result_ready;
            let _ = s.is_ready();
        }

        // Spot-check: byte 0x00 — all defaults.
        let s = ConversionStatus::from_register(0x00);
        assert_eq!(s.set_count, 0);
        assert!(!s.power_on_reset);
        assert!(!s.diag_fail);
        assert!(!s.result_ready);

        // Spot-check: byte 0xFF — all bits set.
        // SET_COUNT=7 (bits [7:5]), POR (bit 4), reserved [3:2], DIAG (bit 1), RESULT (bit 0)
        let s = ConversionStatus::from_register(0xFF);
        assert_eq!(s.set_count, 7);
        assert!(s.power_on_reset);
        assert!(s.diag_fail);
        assert!(s.result_ready);
    }

    // -- Exhaustive TryFrom fuzz ----------------------------------------------

    /// Test helper: for every byte 0..=255, calls `T::try_from(byte)` and
    /// asserts:
    ///   - `Ok(v)` ⇒ `v as u8 == byte` (round-trip) — via the caller-supplied
    ///     `to_u8` closure (needed because `#[repr(u8)]` does not provide
    ///     `Into<u8>`).
    ///   - `Err(e)` ⇒ `e == byte`.
    ///   - The total valid count matches `expected_valid`.
    fn assert_exhaustive_try_from<T>(name: &str, expected_valid: usize, to_u8: impl Fn(T) -> u8)
    where
        T: TryFrom<u8, Error = u8> + Copy + core::fmt::Debug + PartialEq,
    {
        let mut valid = 0usize;
        for byte in 0..=255u8 {
            match T::try_from(byte) {
                Ok(v) => {
                    let back = to_u8(v);
                    assert_eq!(
                        back, byte,
                        "{name}: round-trip failed for Ok({v:?}): {back} != {byte}"
                    );
                    valid += 1;
                }
                Err(e) => {
                    assert_eq!(e, byte, "{name}: Err payload mismatch: {e} != {byte}");
                }
            }
        }
        assert_eq!(
            valid, expected_valid,
            "{name}: expected {expected_valid} valid variants, got {valid}"
        );
    }

    #[test]
    fn exhaustive_i2c_read_mode() {
        assert_exhaustive_try_from::<I2cReadMode>("I2cReadMode", 3, |v| v as u8);
    }

    #[test]
    fn exhaustive_operating_mode() {
        assert_exhaustive_try_from::<OperatingMode>("OperatingMode", 4, |v| v as u8);
    }

    #[test]
    fn exhaustive_magnetic_channel() {
        assert_exhaustive_try_from::<MagneticChannel>("MagneticChannel", 12, |v| v as u8);
    }

    #[test]
    fn exhaustive_conversion_average() {
        assert_exhaustive_try_from::<ConversionAverage>("ConversionAverage", 6, |v| v as u8);
    }

    #[test]
    fn exhaustive_sleep_duration() {
        assert_exhaustive_try_from::<SleepDuration>("SleepDuration", 13, |v| v as u8);
    }

    #[test]
    fn exhaustive_power_mode() {
        assert_exhaustive_try_from::<PowerMode>("PowerMode", 2, |v| v as u8);
    }

    #[test]
    fn exhaustive_trigger_mode() {
        assert_exhaustive_try_from::<TriggerMode>("TriggerMode", 2, |v| v as u8);
    }

    #[test]
    fn exhaustive_temp_coefficient() {
        assert_exhaustive_try_from::<TempCoefficient>("TempCoefficient", 3, |v| v as u8);
    }

    #[test]
    fn exhaustive_angle_calculation() {
        assert_exhaustive_try_from::<AngleCalculation>("AngleCalculation", 4, |v| v as u8);
    }

    #[test]
    fn exhaustive_threshold_direction() {
        assert_exhaustive_try_from::<ThresholdDirection>("ThresholdDirection", 2, |v| v as u8);
    }

    #[test]
    fn exhaustive_threshold_crossing_count() {
        assert_exhaustive_try_from::<ThresholdCrossingCount>("ThresholdCrossingCount", 2, |v| {
            v as u8
        });
    }

    #[test]
    fn exhaustive_interrupt_mode() {
        assert_exhaustive_try_from::<InterruptMode>("InterruptMode", 5, |v| v as u8);
    }

    #[test]
    fn exhaustive_int_pin_behavior() {
        assert_exhaustive_try_from::<IntPinBehavior>("IntPinBehavior", 2, |v| v as u8);
    }

    #[test]
    fn exhaustive_gain_channel() {
        assert_exhaustive_try_from::<GainChannel>("GainChannel", 2, |v| v as u8);
    }

    #[test]
    fn exhaustive_range() {
        assert_exhaustive_try_from::<Range>("Range", 2, |v| v as u8);
    }

    // -- Linux kernel driver cross-reference (v6.14) --------------------------
    // Cross-reference: Linux kernel drivers/iio/magnetometer/tmag5273.c (v6.14)

    #[test]
    fn linux_xref_averaging_modes() {
        // Linux stores pre-shifted values via FIELD_PREP into CONV_AVG_MASK.
        // Our enum stores unshifted raw values (0-5).
        // Comparison: (variant as u8) << CONV_AVG_SHIFT == linux_value
        use crate::register::CONV_AVG_SHIFT;

        let table: &[(ConversionAverage, u8, &str)] = &[
            (ConversionAverage::X1, 0x00, "TMAG5273_AVG_1"),
            (ConversionAverage::X2, 0x04, "TMAG5273_AVG_2"),
            (ConversionAverage::X4, 0x08, "TMAG5273_AVG_4"),
            (ConversionAverage::X8, 0x0C, "TMAG5273_AVG_8"),
            (ConversionAverage::X16, 0x10, "TMAG5273_AVG_16"),
            (ConversionAverage::X32, 0x14, "TMAG5273_AVG_32"),
        ];
        for &(variant, linux_shifted, name) in table {
            let shifted = (variant as u8) << CONV_AVG_SHIFT;
            assert_eq!(
                shifted, linux_shifted,
                "averaging mode mismatch for {name}: (rust {variant:?} as u8) << {CONV_AVG_SHIFT} = 0x{shifted:02X}, linux=0x{linux_shifted:02X}"
            );
        }
    }

    #[test]
    fn linux_xref_operating_modes() {
        let table: &[(OperatingMode, u8, &str)] = &[
            (OperatingMode::Standby, 0, "TMAG5273_OP_MODE_STANDBY"),
            (OperatingMode::Sleep, 1, "TMAG5273_OP_MODE_SLEEP"),
            (
                OperatingMode::ContinuousMeasure,
                2,
                "TMAG5273_OP_MODE_CONTINUOUS",
            ),
            (OperatingMode::WakeUpAndSleep, 3, "TMAG5273_OP_MODE_WAKEUP"),
        ];
        for &(variant, linux_value, name) in table {
            assert_eq!(
                variant as u8, linux_value,
                "operating mode mismatch for {name}: rust={}, linux={linux_value}",
                variant as u8
            );
        }
    }

    #[test]
    fn linux_xref_angle_enable() {
        let table: &[(AngleCalculation, u8, &str)] = &[
            (AngleCalculation::None, 0, "TMAG5273_ANGLE_EN_OFF"),
            (AngleCalculation::XY, 1, "TMAG5273_ANGLE_EN_X_Y"),
            (AngleCalculation::YZ, 2, "TMAG5273_ANGLE_EN_Y_Z"),
            (AngleCalculation::XZ, 3, "TMAG5273_ANGLE_EN_X_Z"),
        ];
        for &(variant, linux_value, name) in table {
            assert_eq!(
                variant as u8, linux_value,
                "angle enable mismatch for {name}: rust={}, linux={linux_value}",
                variant as u8
            );
        }
    }

    #[test]
    fn linux_xref_mag_channel_xyz() {
        // Linux: TMAG5273_MAG_CH_EN_X_Y_Z = 7
        assert_eq!(
            MagneticChannel::XYZ as u8,
            7,
            "MagneticChannel::XYZ should be 7 (TMAG5273_MAG_CH_EN_X_Y_Z)"
        );
    }

    #[test]
    fn linux_xref_magnetic_scale() {
        // Linux tmag5273_scale table: microgauss per LSB for each range.
        // Our range constants are in mT; convert:
        //   1 mT = 10 Gauss = 10_000_000 microgauss
        //   microgauss_per_lsb = range_mt * 10_000_000 / 32768
        // Linux rounds these to integers (e.g., 12200 vs exact 12207.03).
        let table: &[(f64, f64, &str)] = &[
            (40.0, 12200.0, "v1 low (40 mT)"),
            (80.0, 24400.0, "v1 high (80 mT)"),
            (133.0, 40600.0, "v2 low (133 mT)"),
            (266.0, 81200.0, "v2 high (266 mT)"),
        ];
        for &(range_mt, linux_ugauss_per_lsb, name) in table {
            let our_ugauss_per_lsb = range_mt * 10_000_000.0 / 32768.0;
            let rel_error =
                ((our_ugauss_per_lsb - linux_ugauss_per_lsb) / linux_ugauss_per_lsb).abs();
            assert!(
                rel_error < 0.005,
                "magnetic scale mismatch for {name}: ours={our_ugauss_per_lsb:.2}, linux={linux_ugauss_per_lsb}, error={:.3}%",
                rel_error * 100.0
            );
        }
    }

    #[test]
    fn linux_xref_temperature_formula() {
        // Linux integer formula: millidegrees = (raw - 16005) * 10000 / 601
        // Our float formula: celsius = 25.0 + (raw - 17508.0) / 60.1
        // Both implement the same datasheet curve but diverge by ~8 millidegrees
        // due to integer vs float rounding. Assert within 0.5°C at 3 reference points.
        let table: &[(i16, f64, &str)] = &[
            (0, -266.305, "raw=0"),
            (17508, 25.008, "raw=17508 (reference)"),
            (32767, 278.902, "raw=i16::MAX"),
        ];
        for &(raw, linux_millideg_approx, name) in table {
            // Linux: (raw - 16005) * 10000 / 601 millidegrees
            let linux_celsius = ((raw as i64 - 16005) * 10000) as f64 / 601.0 / 1000.0;
            // Verify our Linux reference values are correct
            assert!(
                (linux_celsius - linux_millideg_approx).abs() < 0.01,
                "Linux formula sanity check failed for {name}: computed={linux_celsius}, expected={linux_millideg_approx}"
            );

            // Our formula
            let our_celsius = 25.0_f64 + (raw as f64 - 17508.0) / 60.1;

            let diff = (our_celsius - linux_celsius).abs();
            assert!(
                diff < 0.5,
                "temperature formula mismatch for {name}: ours={our_celsius:.3}°C, linux={linux_celsius:.3}°C, diff={diff:.3}°C"
            );
        }
    }

    // -- coverage: max_polls for all ConversionAverage variants ---------------

    #[test]
    fn max_polls_all_variants() {
        assert_eq!(ConversionAverage::X1.max_polls(), 10);
        assert_eq!(ConversionAverage::X2.max_polls(), 15);
        assert_eq!(ConversionAverage::X4.max_polls(), 25);
        assert_eq!(ConversionAverage::X8.max_polls(), 40);
        assert_eq!(ConversionAverage::X16.max_polls(), 70);
        assert_eq!(ConversionAverage::X32.max_polls(), 100);
    }

    // -- coverage: SleepDuration::as_micros for all variants ------------------

    #[test]
    fn sleep_duration_as_micros_all_variants() {
        assert_eq!(SleepDuration::Ms1.as_micros(), Micros(1_000));
        assert_eq!(SleepDuration::Ms5.as_micros(), Micros(5_000));
        assert_eq!(SleepDuration::Ms10.as_micros(), Micros(10_000));
        assert_eq!(SleepDuration::Ms15.as_micros(), Micros(15_000));
        assert_eq!(SleepDuration::Ms20.as_micros(), Micros(20_000));
        assert_eq!(SleepDuration::Ms30.as_micros(), Micros(30_000));
        assert_eq!(SleepDuration::Ms50.as_micros(), Micros(50_000));
        assert_eq!(SleepDuration::Ms100.as_micros(), Micros(100_000));
        assert_eq!(SleepDuration::Ms500.as_micros(), Micros(500_000));
        assert_eq!(SleepDuration::Ms1000.as_micros(), Micros(1_000_000));
        assert_eq!(SleepDuration::Ms2000.as_micros(), Micros(2_000_000));
        assert_eq!(SleepDuration::Ms5000.as_micros(), Micros(5_000_000));
        assert_eq!(SleepDuration::Ms20000.as_micros(), Micros(20_000_000));
    }

    // -- coverage: Micros Display impl ----------------------------------------

    #[test]
    fn micros_display_impl() {
        extern crate alloc;
        let m = Micros(12345);
        let s = alloc::format!("{}", m);
        assert_eq!(s, "12345");
    }
}
