// TMAG5273 register addresses, bit-field masks/shifts, and helpers.
// Reference: TI TMAG5273 datasheet SBASAI4 Rev C.
//
// Register addresses and bitfield masks for all 29 registers (0x00–0x1C).
// Status register field parsing (CONV_STATUS, DEVICE_STATUS) lives in types.rs.
// Result register LSB addresses are implicit (MSB + 1 via burst reads).

// ---------------------------------------------------------------------------
// Bit-field helpers
// ---------------------------------------------------------------------------

/// Extract a field from a register value.
#[inline]
pub(crate) const fn extract(register: u8, mask: u8, shift: u8) -> u8 {
    (register & mask) >> shift
}

/// Insert a field value into a register value, preserving other bits.
#[inline]
pub(crate) const fn insert(register: u8, mask: u8, shift: u8, value: u8) -> u8 {
    (register & !mask) | ((value << shift) & mask)
}

/// OR this bit into the register address of a read to simultaneously
/// trigger a new conversion (standby/trigger mode only).
///
/// From TI datasheet SBASAI4 Rev C, Section 7.5.1.1: setting bit 7 of
/// the register address during an I2C read initiates a conversion and
/// returns the register value in a single transaction.
pub(crate) const CONVERSION_START_BIT: u8 = 0x80;

// ---------------------------------------------------------------------------
// 0x00 — DEVICE_CONFIG_1
// ---------------------------------------------------------------------------
pub(crate) const DEVICE_CONFIG_1: u8 = 0x00;

pub(crate) const CRC_EN_MASK: u8 = 0b1000_0000;
pub(crate) const CRC_EN_SHIFT: u8 = 7;

pub(crate) const MAG_TEMPCO_MASK: u8 = 0b0110_0000;
pub(crate) const MAG_TEMPCO_SHIFT: u8 = 5;

pub(crate) const CONV_AVG_MASK: u8 = 0b0001_1100;
pub(crate) const CONV_AVG_SHIFT: u8 = 2;

pub(crate) const I2C_RD_MASK: u8 = 0b0000_0011;
pub(crate) const I2C_RD_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x01 — DEVICE_CONFIG_2
// ---------------------------------------------------------------------------
pub(crate) const DEVICE_CONFIG_2: u8 = 0x01;

pub(crate) const THR_HYST_MASK: u8 = 0b1110_0000;
pub(crate) const THR_HYST_SHIFT: u8 = 5;

pub(crate) const LP_LN_MASK: u8 = 0b0001_0000;
pub(crate) const LP_LN_SHIFT: u8 = 4;

pub(crate) const I2C_GLITCH_FILTER_MASK: u8 = 0b0000_1000;
pub(crate) const I2C_GLITCH_FILTER_SHIFT: u8 = 3;

pub(crate) const TRIGGER_MODE_MASK: u8 = 0b0000_0100;
pub(crate) const TRIGGER_MODE_SHIFT: u8 = 2;

pub(crate) const OPERATING_MODE_MASK: u8 = 0b0000_0011;
pub(crate) const OPERATING_MODE_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x02 — SENSOR_CONFIG_1
// ---------------------------------------------------------------------------
pub(crate) const SENSOR_CONFIG_1: u8 = 0x02;

pub(crate) const MAG_CH_EN_MASK: u8 = 0b1111_0000;
pub(crate) const MAG_CH_EN_SHIFT: u8 = 4;

pub(crate) const SLEEPTIME_MASK: u8 = 0b0000_1111;
pub(crate) const SLEEPTIME_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x03 — SENSOR_CONFIG_2
// ---------------------------------------------------------------------------
pub(crate) const SENSOR_CONFIG_2: u8 = 0x03;

pub(crate) const MAG_THR_DIR_MASK: u8 = 0b0010_0000;
pub(crate) const MAG_THR_DIR_SHIFT: u8 = 5;

pub(crate) const MAG_GAIN_CH_MASK: u8 = 0b0001_0000;
pub(crate) const MAG_GAIN_CH_SHIFT: u8 = 4;

pub(crate) const ANGLE_EN_MASK: u8 = 0b0000_1100;
pub(crate) const ANGLE_EN_SHIFT: u8 = 2;

pub(crate) const X_Y_RANGE_MASK: u8 = 0b0000_0010;
pub(crate) const X_Y_RANGE_SHIFT: u8 = 1;

pub(crate) const Z_RANGE_MASK: u8 = 0b0000_0001;
pub(crate) const Z_RANGE_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x04 — X_THR_CONFIG
// ---------------------------------------------------------------------------
pub(crate) const X_THR_CONFIG: u8 = 0x04;

// ---------------------------------------------------------------------------
// 0x05 — Y_THR_CONFIG
// ---------------------------------------------------------------------------
pub(crate) const Y_THR_CONFIG: u8 = 0x05;

// ---------------------------------------------------------------------------
// 0x06 — Z_THR_CONFIG
// ---------------------------------------------------------------------------
pub(crate) const Z_THR_CONFIG: u8 = 0x06;

// ---------------------------------------------------------------------------
// 0x07 — T_CONFIG
// ---------------------------------------------------------------------------
pub(crate) const T_CONFIG: u8 = 0x07;

pub(crate) const T_THR_CONFIG_MASK: u8 = 0b1111_1110;
pub(crate) const T_THR_CONFIG_SHIFT: u8 = 1;

pub(crate) const T_CH_EN_MASK: u8 = 0b0000_0001;
pub(crate) const T_CH_EN_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x08 — INT_CONFIG_1
// ---------------------------------------------------------------------------
pub(crate) const INT_CONFIG_1: u8 = 0x08;

pub(crate) const RSLT_INT_MASK: u8 = 0b1000_0000;
pub(crate) const RSLT_INT_SHIFT: u8 = 7;

pub(crate) const THRESHOLD_INT_MASK: u8 = 0b0100_0000;
pub(crate) const THRESHOLD_INT_SHIFT: u8 = 6;

pub(crate) const INT_STATE_MASK: u8 = 0b0010_0000;
pub(crate) const INT_STATE_SHIFT: u8 = 5;

pub(crate) const INT_MODE_MASK: u8 = 0b0001_1100;
pub(crate) const INT_MODE_SHIFT: u8 = 2;

pub(crate) const MASK_INTB_MASK: u8 = 0b0000_0001;
pub(crate) const MASK_INTB_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x09 — MAG_GAIN_CONFIG
// ---------------------------------------------------------------------------
pub(crate) const MAG_GAIN_CONFIG: u8 = 0x09;

// ---------------------------------------------------------------------------
// 0x0A — MAG_OFFSET_CONFIG_1
// ---------------------------------------------------------------------------
pub(crate) const MAG_OFFSET_CONFIG_1: u8 = 0x0A;

// ---------------------------------------------------------------------------
// 0x0B — MAG_OFFSET_CONFIG_2
// ---------------------------------------------------------------------------
pub(crate) const MAG_OFFSET_CONFIG_2: u8 = 0x0B;

// ---------------------------------------------------------------------------
// 0x0C — I2C_ADDRESS (read-only)
// ---------------------------------------------------------------------------
pub(crate) const I2C_ADDRESS: u8 = 0x0C;

pub(crate) const I2C_ADDRESS_BITS_MASK: u8 = 0b1111_1110;
pub(crate) const I2C_ADDRESS_BITS_SHIFT: u8 = 1;

pub(crate) const I2C_ADDRESS_UPDATE_EN_MASK: u8 = 0b0000_0001;
pub(crate) const I2C_ADDRESS_UPDATE_EN_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x0D — DEVICE_ID (read-only)
// ---------------------------------------------------------------------------
pub(crate) const DEVICE_ID: u8 = 0x0D;

pub(crate) const VERSION_MASK: u8 = 0b0000_0011;
pub(crate) const VERSION_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x0E — MANUFACTURER_ID_LSB (read-only)
// ---------------------------------------------------------------------------
pub(crate) const MANUFACTURER_ID_LSB: u8 = 0x0E;

/// Expected 16-bit manufacturer ID: ASCII "TI" = 0x5449.
pub(crate) const MANUFACTURER_ID_EXPECTED: u16 = 0x5449;

// ---------------------------------------------------------------------------
// 0x10 — T_MSB_RESULT (read-only)
// ---------------------------------------------------------------------------
pub(crate) const T_MSB_RESULT: u8 = 0x10;
// 0x11 T_LSB_RESULT — implicit via burst read (MSB + 1)

// ---------------------------------------------------------------------------
// 0x12 — X_MSB_RESULT (read-only, burst reads X/Y/Z from here)
// ---------------------------------------------------------------------------
#[cfg_attr(feature = "crc", allow(dead_code))]
pub(crate) const X_MSB_RESULT: u8 = 0x12;
// 0x13–0x17 X_LSB, Y_MSB/LSB, Z_MSB/LSB — implicit via 6-byte burst from X_MSB

// ---------------------------------------------------------------------------
// 0x18 — CONV_STATUS (read-only)
// ---------------------------------------------------------------------------
pub(crate) const CONV_STATUS: u8 = 0x18;

// CONV_STATUS field masks live in types.rs (ConversionStatus::from_register)
pub(crate) const RESULT_STATUS_MASK: u8 = 0b0000_0001;
pub(crate) const RESULT_STATUS_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x19 — ANGLE_RESULT_MSB (read-only)
// ---------------------------------------------------------------------------
pub(crate) const ANGLE_RESULT_MSB: u8 = 0x19;
// 0x1A ANGLE_RESULT_LSB — implicit via burst read (MSB + 1)

// ---------------------------------------------------------------------------
// 0x1B — MAGNITUDE_RESULT (read-only, used in CRC path of read_angle)
// ---------------------------------------------------------------------------
#[cfg_attr(not(feature = "crc"), allow(dead_code))]
pub(crate) const MAGNITUDE_RESULT: u8 = 0x1B;

// ---------------------------------------------------------------------------
// 0x1C — DEVICE_STATUS (read-only)
// ---------------------------------------------------------------------------
pub(crate) const DEVICE_STATUS: u8 = 0x1C;

// DEVICE_STATUS field masks live in types.rs (DeviceStatus::from_register)

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -- Full-range extract/insert round-trip ----------------------------------

    /// For every mask/shift pair and every possible input byte (0..=255),
    /// extracting the field and re-inserting it into zero must reproduce
    /// exactly the masked bits of the original byte.
    #[test]
    fn extract_insert_full_range_all_fields() {
        // (mask, shift, name) for every field defined in this module.
        let fields: &[(u8, u8, &str)] = &[
            // DEVICE_CONFIG_1 (0x00)
            (CRC_EN_MASK, CRC_EN_SHIFT, "CRC_EN"),
            (MAG_TEMPCO_MASK, MAG_TEMPCO_SHIFT, "MAG_TEMPCO"),
            (CONV_AVG_MASK, CONV_AVG_SHIFT, "CONV_AVG"),
            (I2C_RD_MASK, I2C_RD_SHIFT, "I2C_RD"),
            // DEVICE_CONFIG_2 (0x01)
            (THR_HYST_MASK, THR_HYST_SHIFT, "THR_HYST"),
            (LP_LN_MASK, LP_LN_SHIFT, "LP_LN"),
            (
                I2C_GLITCH_FILTER_MASK,
                I2C_GLITCH_FILTER_SHIFT,
                "I2C_GLITCH_FILTER",
            ),
            (TRIGGER_MODE_MASK, TRIGGER_MODE_SHIFT, "TRIGGER_MODE"),
            (OPERATING_MODE_MASK, OPERATING_MODE_SHIFT, "OPERATING_MODE"),
            // SENSOR_CONFIG_1 (0x02)
            (MAG_CH_EN_MASK, MAG_CH_EN_SHIFT, "MAG_CH_EN"),
            (SLEEPTIME_MASK, SLEEPTIME_SHIFT, "SLEEPTIME"),
            // SENSOR_CONFIG_2 (0x03)
            (MAG_THR_DIR_MASK, MAG_THR_DIR_SHIFT, "MAG_THR_DIR"),
            (MAG_GAIN_CH_MASK, MAG_GAIN_CH_SHIFT, "MAG_GAIN_CH"),
            (ANGLE_EN_MASK, ANGLE_EN_SHIFT, "ANGLE_EN"),
            (X_Y_RANGE_MASK, X_Y_RANGE_SHIFT, "X_Y_RANGE"),
            (Z_RANGE_MASK, Z_RANGE_SHIFT, "Z_RANGE"),
            // T_CONFIG (0x07)
            (T_THR_CONFIG_MASK, T_THR_CONFIG_SHIFT, "T_THR_CONFIG"),
            (T_CH_EN_MASK, T_CH_EN_SHIFT, "T_CH_EN"),
            // INT_CONFIG_1 (0x08)
            (RSLT_INT_MASK, RSLT_INT_SHIFT, "RSLT_INT"),
            (THRESHOLD_INT_MASK, THRESHOLD_INT_SHIFT, "THRESHOLD_INT"),
            (INT_STATE_MASK, INT_STATE_SHIFT, "INT_STATE"),
            (INT_MODE_MASK, INT_MODE_SHIFT, "INT_MODE"),
            (MASK_INTB_MASK, MASK_INTB_SHIFT, "MASK_INTB"),
            // I2C_ADDRESS (0x0C)
            (
                I2C_ADDRESS_BITS_MASK,
                I2C_ADDRESS_BITS_SHIFT,
                "I2C_ADDRESS_BITS",
            ),
            (
                I2C_ADDRESS_UPDATE_EN_MASK,
                I2C_ADDRESS_UPDATE_EN_SHIFT,
                "I2C_ADDRESS_UPDATE_EN",
            ),
            // DEVICE_ID (0x0D)
            (VERSION_MASK, VERSION_SHIFT, "VERSION"),
            // CONV_STATUS (0x18)
            (RESULT_STATUS_MASK, RESULT_STATUS_SHIFT, "RESULT_STATUS"),
        ];

        for &(mask, shift, name) in fields {
            for byte in 0u8..=255 {
                let extracted = extract(byte, mask, shift);
                let reinserted = insert(0, mask, shift, extracted);
                assert_eq!(
                    reinserted,
                    byte & mask,
                    "extract-then-insert mismatch for field {name} \
                     (mask=0x{mask:02X}, shift={shift}) at byte 0x{byte:02X}: \
                     extracted={extracted}, reinserted=0x{reinserted:02X}, \
                     expected=0x{:02X}",
                    byte & mask,
                );
            }
        }
    }

    // -- Linux kernel driver cross-reference (v6.14) --------------------------
    // Cross-reference: Linux kernel drivers/iio/magnetometer/tmag5273.c (v6.14)

    #[test]
    fn linux_xref_register_addresses() {
        let table: &[(u8, u8, &str)] = &[
            (DEVICE_CONFIG_1, 0x00, "DEVICE_CONFIG_1"),
            (DEVICE_CONFIG_2, 0x01, "DEVICE_CONFIG_2"),
            (SENSOR_CONFIG_1, 0x02, "SENSOR_CONFIG_1"),
            (SENSOR_CONFIG_2, 0x03, "SENSOR_CONFIG_2"),
            (X_THR_CONFIG, 0x04, "X_THR_CONFIG"),
            (Y_THR_CONFIG, 0x05, "Y_THR_CONFIG"),
            (Z_THR_CONFIG, 0x06, "Z_THR_CONFIG"),
            (T_CONFIG, 0x07, "T_CONFIG"),
            (INT_CONFIG_1, 0x08, "INT_CONFIG_1"),
            (MAG_GAIN_CONFIG, 0x09, "MAG_GAIN_CONFIG"),
            (MAG_OFFSET_CONFIG_1, 0x0A, "MAG_OFFSET_CONFIG_1"),
            (MAG_OFFSET_CONFIG_2, 0x0B, "MAG_OFFSET_CONFIG_2"),
            (I2C_ADDRESS, 0x0C, "I2C_ADDRESS"),
            (DEVICE_ID, 0x0D, "DEVICE_ID"),
            (MANUFACTURER_ID_LSB, 0x0E, "MANUFACTURER_ID_LSB"),
            (T_MSB_RESULT, 0x10, "T_MSB_RESULT"),
            (X_MSB_RESULT, 0x12, "X_MSB_RESULT"),
            (CONV_STATUS, 0x18, "CONV_STATUS"),
            (ANGLE_RESULT_MSB, 0x19, "ANGLE_RESULT_MSB"),
            (MAGNITUDE_RESULT, 0x1B, "MAGNITUDE_RESULT"),
            (DEVICE_STATUS, 0x1C, "DEVICE_STATUS"),
        ];
        for &(rust_const, linux_value, name) in table {
            assert_eq!(
                rust_const, linux_value,
                "register address mismatch for {name}: rust=0x{rust_const:02X}, linux=0x{linux_value:02X}"
            );
        }
    }

    #[test]
    fn linux_xref_bit_masks() {
        // Linux GENMASK / BIT equivalents from tmag5273.c (v6.14)
        let table: &[(u8, u8, &str)] = &[
            (CONV_AVG_MASK, 0x1C, "CONV_AVG_MASK = GENMASK(4,2)"),
            (
                OPERATING_MODE_MASK,
                0x03,
                "OPERATING_MODE_MASK = GENMASK(1,0)",
            ),
            (MAG_CH_EN_MASK, 0xF0, "MAG_CH_EN_MASK = GENMASK(7,4)"),
            (Z_RANGE_MASK, 0x01, "Z_RANGE_MASK = BIT(0)"),
            (X_Y_RANGE_MASK, 0x02, "X_Y_RANGE_MASK = BIT(1)"),
            (ANGLE_EN_MASK, 0x0C, "ANGLE_EN_MASK = GENMASK(3,2)"),
            (T_CH_EN_MASK, 0x01, "T_CH_EN_MASK = BIT(0)"),
            (VERSION_MASK, 0x03, "VERSION_MASK = GENMASK(1,0)"),
            (RESULT_STATUS_MASK, 0x01, "RESULT_STATUS_MASK = BIT(0)"),
        ];
        for &(rust_const, linux_value, name) in table {
            assert_eq!(
                rust_const, linux_value,
                "bit mask mismatch for {name}: rust=0x{rust_const:02X}, linux=0x{linux_value:02X}"
            );
        }
    }

    #[test]
    fn linux_xref_manufacturer_id() {
        assert_eq!(
            MANUFACTURER_ID_EXPECTED, 0x5449,
            "manufacturer ID mismatch: rust=0x{:04X}, linux=0x5449",
            MANUFACTURER_ID_EXPECTED
        );
    }

    // -- TI datasheet SBASAI4 Rev C cross-reference ---------------------------

    // TI datasheet SBASAI4 Rev C, Table 7-5
    #[test]
    fn datasheet_xref_register_addresses() {
        assert_eq!(X_THR_CONFIG, 0x04);
        assert_eq!(Y_THR_CONFIG, 0x05);
        assert_eq!(Z_THR_CONFIG, 0x06);
        assert_eq!(T_CONFIG, 0x07);
        assert_eq!(INT_CONFIG_1, 0x08);
        assert_eq!(MAG_GAIN_CONFIG, 0x09);
        assert_eq!(MAG_OFFSET_CONFIG_1, 0x0A);
        assert_eq!(MAG_OFFSET_CONFIG_2, 0x0B);
        assert_eq!(I2C_ADDRESS, 0x0C);
    }

    // TI datasheet SBASAI4 Rev C, Table 7-6
    #[test]
    fn datasheet_xref_device_config_1_fields() {
        assert_eq!(DEVICE_CONFIG_1, 0x00);
        assert_eq!(CRC_EN_MASK, 0x80);
        assert_eq!(CRC_EN_SHIFT, 7);
        assert_eq!(MAG_TEMPCO_MASK, 0x60);
        assert_eq!(MAG_TEMPCO_SHIFT, 5);
        assert_eq!(CONV_AVG_MASK, 0x1C);
        assert_eq!(CONV_AVG_SHIFT, 2);
        assert_eq!(I2C_RD_MASK, 0x03);
        assert_eq!(I2C_RD_SHIFT, 0);
        assert_eq!(
            CRC_EN_MASK | MAG_TEMPCO_MASK | CONV_AVG_MASK | I2C_RD_MASK,
            0xFF,
            "DEVICE_CONFIG_1 masks must cover all 8 bits"
        );
    }

    // TI datasheet SBASAI4 Rev C, Table 7-7
    #[test]
    fn datasheet_xref_device_config_2_fields() {
        assert_eq!(DEVICE_CONFIG_2, 0x01);
        assert_eq!(THR_HYST_MASK, 0xE0);
        assert_eq!(THR_HYST_SHIFT, 5);
        assert_eq!(LP_LN_MASK, 0x10);
        assert_eq!(LP_LN_SHIFT, 4);
        assert_eq!(I2C_GLITCH_FILTER_MASK, 0x08);
        assert_eq!(I2C_GLITCH_FILTER_SHIFT, 3);
        assert_eq!(TRIGGER_MODE_MASK, 0x04);
        assert_eq!(TRIGGER_MODE_SHIFT, 2);
        assert_eq!(OPERATING_MODE_MASK, 0x03);
        assert_eq!(OPERATING_MODE_SHIFT, 0);
        assert_eq!(
            THR_HYST_MASK
                | LP_LN_MASK
                | I2C_GLITCH_FILTER_MASK
                | TRIGGER_MODE_MASK
                | OPERATING_MODE_MASK,
            0xFF,
            "DEVICE_CONFIG_2 masks must cover all 8 bits"
        );
    }

    // TI datasheet SBASAI4 Rev C, Table 7-8
    #[test]
    fn datasheet_xref_sensor_config_1_fields() {
        assert_eq!(SENSOR_CONFIG_1, 0x02);
        assert_eq!(MAG_CH_EN_MASK, 0xF0);
        assert_eq!(MAG_CH_EN_SHIFT, 4);
        assert_eq!(SLEEPTIME_MASK, 0x0F);
        assert_eq!(SLEEPTIME_SHIFT, 0);
        assert_eq!(
            MAG_CH_EN_MASK | SLEEPTIME_MASK,
            0xFF,
            "SENSOR_CONFIG_1 masks must cover all 8 bits"
        );
    }

    // TI datasheet SBASAI4 Rev C, Table 7-9
    #[test]
    fn datasheet_xref_sensor_config_2_fields() {
        assert_eq!(SENSOR_CONFIG_2, 0x03);
        assert_eq!(MAG_THR_DIR_MASK, 0x20);
        assert_eq!(MAG_THR_DIR_SHIFT, 5);
        assert_eq!(MAG_GAIN_CH_MASK, 0x10);
        assert_eq!(MAG_GAIN_CH_SHIFT, 4);
        assert_eq!(ANGLE_EN_MASK, 0x0C);
        assert_eq!(ANGLE_EN_SHIFT, 2);
        assert_eq!(X_Y_RANGE_MASK, 0x02);
        assert_eq!(X_Y_RANGE_SHIFT, 1);
        assert_eq!(Z_RANGE_MASK, 0x01);
        assert_eq!(Z_RANGE_SHIFT, 0);
        assert_eq!(
            MAG_THR_DIR_MASK | MAG_GAIN_CH_MASK | ANGLE_EN_MASK | X_Y_RANGE_MASK | Z_RANGE_MASK,
            0x3F,
            "SENSOR_CONFIG_2 used bits must be 0x3F (bits [7:6] reserved)"
        );
    }

    // TI datasheet SBASAI4 Rev C, Table 7-13
    #[test]
    fn datasheet_xref_t_config_fields() {
        assert_eq!(T_CONFIG, 0x07);
        assert_eq!(T_THR_CONFIG_MASK, 0xFE);
        assert_eq!(T_THR_CONFIG_SHIFT, 1);
        assert_eq!(T_CH_EN_MASK, 0x01);
        assert_eq!(T_CH_EN_SHIFT, 0);
        assert_eq!(
            T_THR_CONFIG_MASK | T_CH_EN_MASK,
            0xFF,
            "T_CONFIG masks must cover all 8 bits"
        );
    }

    // TI datasheet SBASAI4 Rev C, Table 7-14
    #[test]
    fn datasheet_xref_int_config_1_fields() {
        assert_eq!(INT_CONFIG_1, 0x08);
        assert_eq!(RSLT_INT_MASK, 0x80);
        assert_eq!(RSLT_INT_SHIFT, 7);
        assert_eq!(THRESHOLD_INT_MASK, 0x40);
        assert_eq!(THRESHOLD_INT_SHIFT, 6);
        assert_eq!(INT_STATE_MASK, 0x20);
        assert_eq!(INT_STATE_SHIFT, 5);
        assert_eq!(INT_MODE_MASK, 0x1C);
        assert_eq!(INT_MODE_SHIFT, 2);
        assert_eq!(MASK_INTB_MASK, 0x01);
        assert_eq!(MASK_INTB_SHIFT, 0);
        assert_eq!(
            RSLT_INT_MASK | THRESHOLD_INT_MASK | INT_STATE_MASK | INT_MODE_MASK | MASK_INTB_MASK,
            0xFD,
            "INT_CONFIG_1 used bits must be 0xFD (bit 1 reserved)"
        );
    }

    // TI datasheet SBASAI4 Rev C, Table 7-18
    #[test]
    fn datasheet_xref_i2c_address_fields() {
        assert_eq!(I2C_ADDRESS, 0x0C);
        assert_eq!(I2C_ADDRESS_BITS_MASK, 0xFE);
        assert_eq!(I2C_ADDRESS_BITS_SHIFT, 1);
        assert_eq!(I2C_ADDRESS_UPDATE_EN_MASK, 0x01);
        assert_eq!(I2C_ADDRESS_UPDATE_EN_SHIFT, 0);
        assert_eq!(
            I2C_ADDRESS_BITS_MASK | I2C_ADDRESS_UPDATE_EN_MASK,
            0xFF,
            "I2C_ADDRESS masks must cover all 8 bits"
        );
    }

    // TI datasheet SBASAI4 Rev C, Table 7-21
    #[test]
    fn datasheet_xref_device_status_bit_positions() {
        use crate::types::DeviceStatus;

        assert_eq!(DEVICE_STATUS, 0x1C);

        // Bit 0 = VCC_UV
        let s = DeviceStatus::from_register(1 << 0);
        assert!(s.vcc_undervoltage);
        assert!(!s.otp_crc_error);
        assert!(!s.int_error);
        assert!(!s.oscillator_error);
        assert!(!s.int_pin_active);

        // Bit 1 = OTP_CRC
        let s = DeviceStatus::from_register(1 << 1);
        assert!(!s.vcc_undervoltage);
        assert!(s.otp_crc_error);
        assert!(!s.int_error);
        assert!(!s.oscillator_error);
        assert!(!s.int_pin_active);

        // Bit 2 = INT_ERR
        let s = DeviceStatus::from_register(1 << 2);
        assert!(!s.vcc_undervoltage);
        assert!(!s.otp_crc_error);
        assert!(s.int_error);
        assert!(!s.oscillator_error);
        assert!(!s.int_pin_active);

        // Bit 3 = OSC_ERR
        let s = DeviceStatus::from_register(1 << 3);
        assert!(!s.vcc_undervoltage);
        assert!(!s.otp_crc_error);
        assert!(!s.int_error);
        assert!(s.oscillator_error);
        assert!(!s.int_pin_active);

        // Bit 4 = INTB_RB
        let s = DeviceStatus::from_register(1 << 4);
        assert!(!s.vcc_undervoltage);
        assert!(!s.otp_crc_error);
        assert!(!s.int_error);
        assert!(!s.oscillator_error);
        assert!(s.int_pin_active);
    }

    // TI datasheet SBASAI4 Rev C, Tables 7-6 through 7-18
    #[test]
    fn datasheet_xref_no_mask_overlaps() {
        /// Assert that no two masks in the slice share any bits.
        fn assert_pairwise_disjoint(register_name: &str, masks: &[u8]) {
            for i in 0..masks.len() {
                for j in (i + 1)..masks.len() {
                    assert_eq!(
                        masks[i] & masks[j],
                        0,
                        "{register_name}: masks 0x{:02X} and 0x{:02X} overlap (0x{:02X})",
                        masks[i],
                        masks[j],
                        masks[i] & masks[j],
                    );
                }
            }
        }

        assert_pairwise_disjoint(
            "DEVICE_CONFIG_1",
            &[CRC_EN_MASK, MAG_TEMPCO_MASK, CONV_AVG_MASK, I2C_RD_MASK],
        );
        assert_pairwise_disjoint(
            "DEVICE_CONFIG_2",
            &[
                THR_HYST_MASK,
                LP_LN_MASK,
                I2C_GLITCH_FILTER_MASK,
                TRIGGER_MODE_MASK,
                OPERATING_MODE_MASK,
            ],
        );
        assert_pairwise_disjoint("SENSOR_CONFIG_1", &[MAG_CH_EN_MASK, SLEEPTIME_MASK]);
        assert_pairwise_disjoint(
            "SENSOR_CONFIG_2",
            &[
                MAG_THR_DIR_MASK,
                MAG_GAIN_CH_MASK,
                ANGLE_EN_MASK,
                X_Y_RANGE_MASK,
                Z_RANGE_MASK,
            ],
        );
        assert_pairwise_disjoint("T_CONFIG", &[T_THR_CONFIG_MASK, T_CH_EN_MASK]);
        assert_pairwise_disjoint(
            "INT_CONFIG_1",
            &[
                RSLT_INT_MASK,
                THRESHOLD_INT_MASK,
                INT_STATE_MASK,
                INT_MODE_MASK,
                MASK_INTB_MASK,
            ],
        );
        assert_pairwise_disjoint(
            "I2C_ADDRESS",
            &[I2C_ADDRESS_BITS_MASK, I2C_ADDRESS_UPDATE_EN_MASK],
        );
    }

    // -----------------------------------------------------------------------
    // Edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn extract_from_all_ones() {
        // Every multi-bit field extracted from 0xFF should equal its field_max.
        assert_eq!(extract(0xFF, CONV_AVG_MASK, CONV_AVG_SHIFT), 0x07);
        assert_eq!(extract(0xFF, MAG_CH_EN_MASK, MAG_CH_EN_SHIFT), 0x0F);
        assert_eq!(extract(0xFF, THR_HYST_MASK, THR_HYST_SHIFT), 0x07);
        assert_eq!(extract(0xFF, INT_MODE_MASK, INT_MODE_SHIFT), 0x07);
        assert_eq!(extract(0xFF, ANGLE_EN_MASK, ANGLE_EN_SHIFT), 0x03);
        assert_eq!(extract(0xFF, T_THR_CONFIG_MASK, T_THR_CONFIG_SHIFT), 0x7F);
        assert_eq!(
            extract(0xFF, I2C_ADDRESS_BITS_MASK, I2C_ADDRESS_BITS_SHIFT),
            0x7F
        );
    }

    #[test]
    fn insert_into_zero() {
        // Inserting max value into 0x00 should produce only the field bits set.
        assert_eq!(
            insert(0x00, CONV_AVG_MASK, CONV_AVG_SHIFT, 0x07),
            0b0001_1100
        );
        assert_eq!(
            insert(0x00, MAG_CH_EN_MASK, MAG_CH_EN_SHIFT, 0x0F),
            0b1111_0000
        );
        assert_eq!(
            insert(0x00, OPERATING_MODE_MASK, OPERATING_MODE_SHIFT, 0x03),
            0b0000_0011
        );
        assert_eq!(
            insert(0x00, INT_MODE_MASK, INT_MODE_SHIFT, 0x07),
            0b0001_1100
        );
    }

    #[test]
    fn insert_preserves_other_bits() {
        // Setting CONV_AVG in a register that already has CRC_EN and I2C_RD set.
        let existing: u8 = 0b1000_0011; // CRC_EN=1, I2C_RD=3
        let updated = insert(existing, CONV_AVG_MASK, CONV_AVG_SHIFT, 0x05);
        assert_eq!(updated, 0b1001_0111);
        assert_eq!(extract(updated, CRC_EN_MASK, CRC_EN_SHIFT), 1);
        assert_eq!(extract(updated, CONV_AVG_MASK, CONV_AVG_SHIFT), 5);
        assert_eq!(extract(updated, I2C_RD_MASK, I2C_RD_SHIFT), 3);
    }

    // -----------------------------------------------------------------------
    // Manufacturer ID
    // -----------------------------------------------------------------------

    #[test]
    fn manufacturer_id_is_ascii_ti() {
        assert_eq!(MANUFACTURER_ID_EXPECTED, 0x5449);
        // MSB = 'T' (0x54), LSB = 'I' (0x49)
        assert_eq!((MANUFACTURER_ID_EXPECTED >> 8) as u8, b'T');
        assert_eq!((MANUFACTURER_ID_EXPECTED & 0xFF) as u8, b'I');
    }

    // -----------------------------------------------------------------------
    // Register address coverage — all 29 registers present and unique
    // -----------------------------------------------------------------------

    #[test]
    fn all_29_register_addresses_are_unique() {
        let addrs: [u8; 29] = [
            DEVICE_CONFIG_1,     // 0x00
            DEVICE_CONFIG_2,     // 0x01
            SENSOR_CONFIG_1,     // 0x02
            SENSOR_CONFIG_2,     // 0x03
            X_THR_CONFIG,        // 0x04
            Y_THR_CONFIG,        // 0x05
            Z_THR_CONFIG,        // 0x06
            T_CONFIG,            // 0x07
            INT_CONFIG_1,        // 0x08
            MAG_GAIN_CONFIG,     // 0x09
            MAG_OFFSET_CONFIG_1, // 0x0A
            MAG_OFFSET_CONFIG_2, // 0x0B
            I2C_ADDRESS,         // 0x0C
            DEVICE_ID,           // 0x0D
            MANUFACTURER_ID_LSB, // 0x0E
            0x0F,                // MANUFACTURER_ID_MSB (burst read, no const)
            T_MSB_RESULT,        // 0x10
            0x11,                // T_LSB_RESULT (burst read, no const)
            X_MSB_RESULT,        // 0x12
            0x13,                // X_LSB_RESULT (burst read)
            0x14,                // Y_MSB_RESULT (burst read)
            0x15,                // Y_LSB_RESULT (burst read)
            0x16,                // Z_MSB_RESULT (burst read)
            0x17,                // Z_LSB_RESULT (burst read)
            CONV_STATUS,         // 0x18
            ANGLE_RESULT_MSB,    // 0x19
            0x1A,                // ANGLE_RESULT_LSB (burst read)
            MAGNITUDE_RESULT,    // 0x1B
            DEVICE_STATUS,       // 0x1C
        ];

        // Verify sequential addresses 0x00..=0x1C
        for (i, &addr) in addrs.iter().enumerate() {
            assert_eq!(
                addr, i as u8,
                "register at index {i} should have address 0x{:02X} but got 0x{addr:02X}",
                i
            );
        }
    }
}
