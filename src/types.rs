//! Public types, newtypes, and raw-to-physical conversion helpers for the
//! TMAG5273 Hall-effect sensor driver.

// ---------------------------------------------------------------------------
// Magnetic field measurement range
// ---------------------------------------------------------------------------

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

/// Angle in degrees (0–360).
#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Degrees(pub f32);

/// Duration in microseconds.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Micros(pub u32);

impl core::fmt::Display for Micros {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// A range of microsecond durations, matching Linux `usleep_range(min, max)` semantics.
///
/// - `.0` — minimum delay (used by the driver for actual `delay_us` calls).
/// - `.1` — maximum acceptable delay (documentary — for RTOS scheduler batching).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MicrosRange(pub Micros, pub Micros);

/// Wake-up delay range after transitioning from Sleep to an active mode.
///
/// From TI TMAG5273 datasheet: device needs >=80 µs to wake. The Linux kernel
/// driver uses `usleep_range(80, 200)` for this boundary.
pub const WAKE_DELAY: MicrosRange = MicrosRange(Micros(80), Micros(200));

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
    pub const fn from_reg(register: u8) -> Self {
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
    pub const fn from_reg(register: u8) -> Self {
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
// Conversion functions
// ---------------------------------------------------------------------------

/// Converts a raw magnetic field ADC code to millitesla.
///
/// Implements datasheet equation 10:
/// `B_mT = (-raw * range_mt) / 32768.0`
#[inline]
pub(crate) fn raw_to_millitesla(raw: i16, range_mt: f32) -> MilliTesla {
    MilliTesla(-(raw as f32) * range_mt * INV_32768)
}

/// Converts a raw temperature ADC code to degrees Celsius.
///
/// Implements datasheet equation 12:
/// `T = 25.0 + (raw - 17508) / 60.1`
#[inline]
pub(crate) fn raw_to_celsius(raw: i16) -> Celsius {
    Celsius(TSENSE_T0 + ((raw as f32) - TADC_T0) * TADC_RES_INV)
}

/// Converts a raw angle register value to degrees.
///
/// The register uses a 9.4 fixed-point format:
/// - Bits \[12:4\]: 9-bit integer part (0–360).
/// - Bits \[3:0\]: 4-bit fractional part (resolution 1/16 degree).
#[inline]
pub(crate) fn raw_to_degrees(raw: u16) -> Degrees {
    let integer = ((raw >> ANGLE_INT_SHIFT) & ANGLE_INT_MASK) as f32;
    let fraction = (raw & ANGLE_FRAC_MASK) as f32 * ANGLE_FRAC_SCALE;
    Degrees(integer + fraction)
}

// ---------------------------------------------------------------------------
// Public conversion helpers
// ---------------------------------------------------------------------------

/// Converts a raw magnitude register value to millitesla.
///
/// The magnitude register (0x1B) contains the MSB of the vector magnitude.
/// To get physical units, treat it as the upper byte of a 16-bit value
/// and apply the same conversion as magnetic channels.
///
/// This follows the Zephyr RTOS driver's `tmag5273_magnitude_convert()` pattern.
#[inline]
pub fn magnitude_to_millitesla(raw_magnitude: u8, range_mt: f32) -> MilliTesla {
    raw_to_millitesla(((raw_magnitude as u16) << 8) as i16, range_mt)
}

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
        let c = raw_to_celsius(17508);
        assert!(approx_eq(c.0, 25.0), "expected ~25.0, got {}", c.0);
    }

    #[test]
    fn temp_raw_zero() {
        // T = 25.0 + (0 - 17508) / 60.1 ≈ -266.36
        let c = raw_to_celsius(0);
        assert!(approx_eq(c.0, -266.36), "expected ~-266.36, got {}", c.0);
    }

    #[test]
    fn temp_raw_max() {
        // T = 25.0 + (32767 - 17508) / 60.1 ≈ 279.0
        let c = raw_to_celsius(i16::MAX);
        assert!(approx_eq(c.0, 279.0), "expected ~279.0, got {}", c.0);
    }

    // -- Magnetic conversion -------------------------------------------------

    #[test]
    fn mag_raw_zero() {
        let m = raw_to_millitesla(0, 40.0);
        assert_eq!(m.0, 0.0);
    }

    #[test]
    fn mag_raw_min_yields_positive_range() {
        // raw = i16::MIN = -32768 → B = -(-32768) * range / 32768 = +range
        let m = raw_to_millitesla(i16::MIN, 40.0);
        assert!(approx_eq(m.0, 40.0), "expected ~40.0, got {}", m.0);
    }

    #[test]
    fn mag_raw_max_yields_negative_range() {
        // raw = 32767 → B = -(32767) * 40 / 32768 ≈ -39.9988
        let m = raw_to_millitesla(i16::MAX, 40.0);
        assert!(approx_eq(m.0, -40.0), "expected ~-40.0, got {}", m.0);
    }

    #[test]
    fn mag_high_range_v2() {
        let m = raw_to_millitesla(i16::MIN, 266.0);
        assert!(approx_eq(m.0, 266.0), "expected ~266.0, got {}", m.0);
    }

    // -- Angle conversion ----------------------------------------------------

    #[test]
    fn angle_zero() {
        let d = raw_to_degrees(0x0000);
        assert_eq!(d.0, 0.0);
    }

    #[test]
    fn angle_180() {
        // 180 << 4 = 0x0B40
        let d = raw_to_degrees(0x0B40);
        assert!(approx_eq(d.0, 180.0), "expected 180.0, got {}", d.0);
    }

    #[test]
    fn angle_360() {
        // 360 << 4 = 0x1680
        let d = raw_to_degrees(0x1680);
        assert!(approx_eq(d.0, 360.0), "expected 360.0, got {}", d.0);
    }

    #[test]
    fn angle_fractional() {
        // 90 degrees + 8/16 = 90.5 degrees → (90 << 4) | 8 = 0x05A8
        let d = raw_to_degrees(0x05A8);
        assert!(approx_eq(d.0, 90.5), "expected 90.5, got {}", d.0);
    }

    // -- DeviceStatus parsing ------------------------------------------------

    #[test]
    fn device_status_all_clear() {
        let s = DeviceStatus::from_reg(0x00);
        assert!(!s.vcc_undervoltage);
        assert!(!s.otp_crc_error);
        assert!(!s.int_error);
        assert!(!s.oscillator_error);
        assert!(!s.int_pin_active);
    }

    #[test]
    fn device_status_all_set() {
        let s = DeviceStatus::from_reg(0x1F);
        assert!(s.vcc_undervoltage);
        assert!(s.otp_crc_error);
        assert!(s.int_error);
        assert!(s.oscillator_error);
        assert!(s.int_pin_active);
    }

    #[test]
    fn device_status_individual_bits() {
        let s = DeviceStatus::from_reg(0b0000_0001);
        assert!(s.vcc_undervoltage);
        assert!(!s.otp_crc_error);

        let s = DeviceStatus::from_reg(0b0000_0010);
        assert!(!s.vcc_undervoltage);
        assert!(s.otp_crc_error);

        let s = DeviceStatus::from_reg(0b0000_0100);
        assert!(s.int_error);
        assert!(!s.oscillator_error);

        let s = DeviceStatus::from_reg(0b0000_1000);
        assert!(!s.int_error);
        assert!(s.oscillator_error);
    }

    #[test]
    fn device_status_int_pin_active() {
        let s = DeviceStatus::from_reg(0b0001_0000);
        assert!(s.int_pin_active);
        assert!(s.is_ok()); // INT is not a fault
    }

    // -- ConversionStatus parsing --------------------------------------------

    #[test]
    fn conv_status_all_clear() {
        let s = ConversionStatus::from_reg(0x00);
        assert_eq!(s.set_count, 0);
        assert!(!s.power_on_reset);
        assert!(!s.diag_fail);
        assert!(!s.result_ready);
    }

    #[test]
    fn conv_status_set_count() {
        // set_count occupies bits [7:5] — value 5 = 0b101 << 5 = 0xA0
        let s = ConversionStatus::from_reg(0b1010_0000);
        assert_eq!(s.set_count, 5);
        assert!(!s.power_on_reset);
    }

    #[test]
    fn conv_status_set_count_max() {
        // 7 = 0b111 << 5 = 0xE0
        let s = ConversionStatus::from_reg(0b1110_0000);
        assert_eq!(s.set_count, 7);
    }

    #[test]
    fn conv_status_power_on_reset() {
        // POR = bit 4 = 0x10
        let s = ConversionStatus::from_reg(0b0001_0000);
        assert_eq!(s.set_count, 0);
        assert!(s.power_on_reset);
        assert!(!s.diag_fail);
        assert!(!s.result_ready);
    }

    #[test]
    fn conv_status_diag_fail() {
        // DIAG_STATUS = bit 1 = 0x02
        let s = ConversionStatus::from_reg(0b0000_0010);
        assert!(s.diag_fail);
        assert!(!s.result_ready);
    }

    #[test]
    fn conv_status_result_ready() {
        // RESULT_STATUS = bit 0 = 0x01
        let s = ConversionStatus::from_reg(0b0000_0001);
        assert!(s.result_ready);
        assert!(!s.diag_fail);
    }

    #[test]
    fn conv_status_all_flags() {
        // SET_COUNT=7 (bits [7:5]) | POR (bit 4) | DIAG (bit 1) | RESULT (bit 0)
        // = 0b1110_0000 | 0b0001_0000 | 0b0000_0010 | 0b0000_0001 = 0xF3
        let s = ConversionStatus::from_reg(0b1111_0011);
        assert_eq!(s.set_count, 7);
        assert!(s.power_on_reset);
        assert!(s.diag_fail);
        assert!(s.result_ready);
    }

    // -- Magnitude conversion (Zephyr-inspired) ------------------------------

    #[test]
    fn magnitude_zero() {
        let m = magnitude_to_millitesla(0, 80.0);
        assert_eq!(m.0, 0.0);
    }

    #[test]
    fn magnitude_max() {
        // raw=127, range=80.0 → (127 << 8) = 32512 as i16
        // B = -(32512) * 80 / 32768 ≈ -79.375 mT (magnitude is always positive in
        // practice, but the formula applies the standard signed conversion).
        let m = magnitude_to_millitesla(127, 80.0);
        assert!(approx_eq(m.0, -79.375), "expected ~-79.375, got {}", m.0);
    }

    // -- Wake delay constants ------------------------------------------------

    #[test]
    fn wake_delay_constants() {
        assert_eq!(WAKE_DELAY.0.0, 80);
        assert_eq!(WAKE_DELAY.1.0, 200);
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

    // -- magnitude_to_millitesla overflow edge cases --------------------------

    #[test]
    fn magnitude_raw_128_no_overflow() {
        // This was the overflow bug: (128_i16) << 8 = 32768 > i16::MAX
        // Fixed by casting through u16: ((128_u16) << 8) as i16 = -32768
        let m = magnitude_to_millitesla(128, 80.0);
        // (128 << 8) as i16 = -32768, then raw_to_millitesla(-(-32768) * 80 / 32768) = 80.0
        assert!(approx_eq(m.0, 80.0), "expected ~80.0, got {}", m.0);
    }

    #[test]
    fn magnitude_raw_255_no_overflow() {
        // Maximum raw value: ((255_u16) << 8) as i16 = -256
        let m = magnitude_to_millitesla(255, 80.0);
        assert!(m.0.is_finite(), "expected finite, got {}", m.0);
    }

    #[test]
    fn magnitude_raw_0() {
        let m = magnitude_to_millitesla(0, 80.0);
        assert_eq!(m.0, 0.0);
    }

    // -- raw_to_millitesla boundary values ------------------------------------

    #[test]
    fn millitesla_max_positive_raw() {
        // i16::MAX = 32767 → -(32767) * 80 / 32768 ≈ -79.9976 mT
        let m = raw_to_millitesla(i16::MAX, 80.0);
        assert!(m.0 < 0.0, "max positive raw should give negative mT");
        assert!(approx_eq(m.0, -80.0), "expected ~-80.0, got {}", m.0);
    }

    #[test]
    fn millitesla_min_negative_raw() {
        // i16::MIN = -32768 → -(-32768) * 80 / 32768 = 80.0 mT
        let m = raw_to_millitesla(i16::MIN, 80.0);
        assert!(m.0 > 0.0, "min negative raw should give positive mT");
        assert!(approx_eq(m.0, 80.0), "expected ~80.0, got {}", m.0);
    }

    #[test]
    fn millitesla_zero_raw() {
        let m = raw_to_millitesla(0, 80.0);
        assert_eq!(m.0, 0.0);
    }

    #[test]
    fn millitesla_zero_range() {
        let m = raw_to_millitesla(16384, 0.0);
        assert_eq!(m.0, 0.0);
    }

    // -- raw_to_celsius boundary values ---------------------------------------

    #[test]
    fn celsius_reference_point() {
        // At raw = TADC_T25 (17508), temperature should be 25.0°C
        let t = raw_to_celsius(17508);
        assert!(approx_eq(t.0, 25.0), "expected 25.0, got {}", t.0);
    }

    #[test]
    fn celsius_zero_raw() {
        // raw = 0 → 25.0 + (0 - 17508) / 60.1 ≈ -266.4°C
        let t = raw_to_celsius(0);
        assert!(t.0 < -200.0, "expected very negative, got {}", t.0);
    }

    // -- raw_to_degrees boundary values ---------------------------------------

    #[test]
    fn degrees_zero() {
        let d = raw_to_degrees(0);
        assert_eq!(d.0, 0.0);
    }

    #[test]
    fn degrees_full_scale() {
        // 0x0B40 → 180.0° per datasheet
        let d = raw_to_degrees(0x0B40);
        assert!(approx_eq(d.0, 180.0), "expected ~180.0, got {}", d.0);
    }
}
