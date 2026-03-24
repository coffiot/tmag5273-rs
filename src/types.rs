// Rust guideline compliant 2026-03-24

//! Public types, newtypes, and raw-to-physical conversion helpers for the
//! TMAG5273 Hall-effect sensor driver.

// ---------------------------------------------------------------------------
// Magnetic field measurement range
// ---------------------------------------------------------------------------

/// Magnetic field measurement range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Range {
    /// Low range (±40 mT for version 1, ±133 mT for version 2).
    Low = 0,
    /// High range (±80 mT for version 1, ±266 mT for version 2).
    High = 1,
}

impl TryFrom<u8> for Range {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, u8> {
        match val {
            0 => Ok(Self::Low),
            1 => Ok(Self::High),
            other => Err(other),
        }
    }
}

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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

/// Temperature in degrees Celsius.
#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Celsius(pub f32);

/// Angle in degrees (0–360).
#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Degrees(pub f32);

// ---------------------------------------------------------------------------
// Composite reading structs
// ---------------------------------------------------------------------------

/// Magnetic field reading from all enabled axes.
///
/// Disabled channels are represented as `None`.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MagneticReading {
    /// X-axis magnetic flux density, or `None` if the channel is disabled.
    pub x: Option<MilliTesla>,
    /// Y-axis magnetic flux density, or `None` if the channel is disabled.
    pub y: Option<MilliTesla>,
    /// Z-axis magnetic flux density, or `None` if the channel is disabled.
    pub z: Option<MilliTesla>,
}

/// Angle measurement result from the TMAG5273 angle engine.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AngleReading {
    /// Computed angle in degrees (0–360).
    pub angle: Degrees,
    /// Magnitude of the magnetic field vector (unitless, 0–255).
    pub magnitude: u8,
}

// ---------------------------------------------------------------------------
// Status structs
// ---------------------------------------------------------------------------

/// Parsed contents of the `DEVICE_STATUS` register (0x1C).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceStatus {
    /// VCC supply is below the undervoltage threshold.
    pub vcc_undervoltage: bool,
    /// OTP memory CRC check failed.
    pub otp_crc_error: bool,
    /// Internal error detected.
    pub int_error: bool,
    /// Oscillator error detected.
    pub oscillator_error: bool,
}

/// Bit position of VCC undervoltage flag in `DEVICE_STATUS`.
const DEVICE_STATUS_VCC_UV_BIT: u8 = 0;
/// Bit position of OTP CRC error flag in `DEVICE_STATUS`.
const DEVICE_STATUS_OTP_CRC_BIT: u8 = 1;
/// Bit position of internal error flag in `DEVICE_STATUS`.
const DEVICE_STATUS_INT_ERR_BIT: u8 = 2;
/// Bit position of oscillator error flag in `DEVICE_STATUS`.
const DEVICE_STATUS_OSC_ERR_BIT: u8 = 3;

impl DeviceStatus {
    /// Parses a raw `DEVICE_STATUS` register byte into structured flags.
    #[inline]
    pub const fn from_reg(reg: u8) -> Self {
        Self {
            vcc_undervoltage: (reg >> DEVICE_STATUS_VCC_UV_BIT) & 1 != 0,
            otp_crc_error: (reg >> DEVICE_STATUS_OTP_CRC_BIT) & 1 != 0,
            int_error: (reg >> DEVICE_STATUS_INT_ERR_BIT) & 1 != 0,
            oscillator_error: (reg >> DEVICE_STATUS_OSC_ERR_BIT) & 1 != 0,
        }
    }
}

/// Parsed contents of the `CONV_STATUS` register (0x18).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    /// Parses a raw `CONV_STATUS` register byte into structured fields.
    #[inline]
    pub const fn from_reg(reg: u8) -> Self {
        Self {
            set_count: (reg >> CONV_STATUS_SET_COUNT_SHIFT) & CONV_STATUS_SET_COUNT_MASK,
            power_on_reset: (reg >> CONV_STATUS_POR_BIT) & 1 != 0,
            diag_fail: (reg >> CONV_STATUS_DIAG_BIT) & 1 != 0,
            result_ready: (reg >> CONV_STATUS_READY_BIT) & 1 != 0,
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
    }

    #[test]
    fn device_status_all_set() {
        let s = DeviceStatus::from_reg(0x0F);
        assert!(s.vcc_undervoltage);
        assert!(s.otp_crc_error);
        assert!(s.int_error);
        assert!(s.oscillator_error);
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
}
