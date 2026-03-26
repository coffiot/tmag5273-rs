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
pub(crate) const X_MSB_RESULT: u8 = 0x12;
// 0x13–0x17 X_LSB, Y_MSB/LSB, Z_MSB/LSB — implicit via 6-byte burst from X_MSB

// ---------------------------------------------------------------------------
// 0x18 — CONV_STATUS (read-only)
// ---------------------------------------------------------------------------
pub(crate) const CONV_STATUS: u8 = 0x18;

// CONV_STATUS field masks live in types.rs (ConversionStatus::from_reg)
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

// DEVICE_STATUS field masks live in types.rs (DeviceStatus::from_reg)

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // Helper round-trip: insert then extract must return the original value.
    // -----------------------------------------------------------------------

    fn round_trip(mask: u8, shift: u8, max_value: u8) {
        for v in 0..=max_value {
            let register = insert(0x00, mask, shift, v);
            let got = extract(register, mask, shift);
            assert_eq!(
                got, v,
                "round-trip failed for value {v} (mask=0x{mask:02X}, shift={shift})"
            );
        }
    }

    /// Maximum value that fits in the field defined by mask and shift.
    const fn field_max(mask: u8, shift: u8) -> u8 {
        mask >> shift
    }

    // -- DEVICE_CONFIG_1 (0x00) ---------------------------------------------

    #[test]
    fn round_trip_crc_en() {
        round_trip(
            CRC_EN_MASK,
            CRC_EN_SHIFT,
            field_max(CRC_EN_MASK, CRC_EN_SHIFT),
        );
    }

    #[test]
    fn round_trip_mag_tempco() {
        round_trip(
            MAG_TEMPCO_MASK,
            MAG_TEMPCO_SHIFT,
            field_max(MAG_TEMPCO_MASK, MAG_TEMPCO_SHIFT),
        );
    }

    #[test]
    fn round_trip_conv_avg() {
        round_trip(
            CONV_AVG_MASK,
            CONV_AVG_SHIFT,
            field_max(CONV_AVG_MASK, CONV_AVG_SHIFT),
        );
    }

    #[test]
    fn round_trip_i2c_rd() {
        round_trip(
            I2C_RD_MASK,
            I2C_RD_SHIFT,
            field_max(I2C_RD_MASK, I2C_RD_SHIFT),
        );
    }

    // -- DEVICE_CONFIG_2 (0x01) ---------------------------------------------

    #[test]
    fn round_trip_thr_hyst() {
        round_trip(
            THR_HYST_MASK,
            THR_HYST_SHIFT,
            field_max(THR_HYST_MASK, THR_HYST_SHIFT),
        );
    }

    #[test]
    fn round_trip_lp_ln() {
        round_trip(LP_LN_MASK, LP_LN_SHIFT, field_max(LP_LN_MASK, LP_LN_SHIFT));
    }

    #[test]
    fn round_trip_i2c_glitch_filter() {
        round_trip(
            I2C_GLITCH_FILTER_MASK,
            I2C_GLITCH_FILTER_SHIFT,
            field_max(I2C_GLITCH_FILTER_MASK, I2C_GLITCH_FILTER_SHIFT),
        );
    }

    #[test]
    fn round_trip_trigger_mode() {
        round_trip(
            TRIGGER_MODE_MASK,
            TRIGGER_MODE_SHIFT,
            field_max(TRIGGER_MODE_MASK, TRIGGER_MODE_SHIFT),
        );
    }

    #[test]
    fn round_trip_operating_mode() {
        round_trip(
            OPERATING_MODE_MASK,
            OPERATING_MODE_SHIFT,
            field_max(OPERATING_MODE_MASK, OPERATING_MODE_SHIFT),
        );
    }

    // -- SENSOR_CONFIG_1 (0x02) ---------------------------------------------

    #[test]
    fn round_trip_mag_ch_en() {
        round_trip(
            MAG_CH_EN_MASK,
            MAG_CH_EN_SHIFT,
            field_max(MAG_CH_EN_MASK, MAG_CH_EN_SHIFT),
        );
    }

    #[test]
    fn round_trip_sleeptime() {
        round_trip(
            SLEEPTIME_MASK,
            SLEEPTIME_SHIFT,
            field_max(SLEEPTIME_MASK, SLEEPTIME_SHIFT),
        );
    }

    // -- SENSOR_CONFIG_2 (0x03) ---------------------------------------------

    #[test]
    fn round_trip_mag_thr_dir() {
        round_trip(
            MAG_THR_DIR_MASK,
            MAG_THR_DIR_SHIFT,
            field_max(MAG_THR_DIR_MASK, MAG_THR_DIR_SHIFT),
        );
    }

    #[test]
    fn round_trip_mag_gain_ch() {
        round_trip(
            MAG_GAIN_CH_MASK,
            MAG_GAIN_CH_SHIFT,
            field_max(MAG_GAIN_CH_MASK, MAG_GAIN_CH_SHIFT),
        );
    }

    #[test]
    fn round_trip_angle_en() {
        round_trip(
            ANGLE_EN_MASK,
            ANGLE_EN_SHIFT,
            field_max(ANGLE_EN_MASK, ANGLE_EN_SHIFT),
        );
    }

    #[test]
    fn round_trip_x_y_range() {
        round_trip(
            X_Y_RANGE_MASK,
            X_Y_RANGE_SHIFT,
            field_max(X_Y_RANGE_MASK, X_Y_RANGE_SHIFT),
        );
    }

    #[test]
    fn round_trip_z_range() {
        round_trip(
            Z_RANGE_MASK,
            Z_RANGE_SHIFT,
            field_max(Z_RANGE_MASK, Z_RANGE_SHIFT),
        );
    }

    // -- T_CONFIG (0x07) ----------------------------------------------------

    #[test]
    fn round_trip_t_thr_config() {
        round_trip(
            T_THR_CONFIG_MASK,
            T_THR_CONFIG_SHIFT,
            field_max(T_THR_CONFIG_MASK, T_THR_CONFIG_SHIFT),
        );
    }

    #[test]
    fn round_trip_t_ch_en() {
        round_trip(
            T_CH_EN_MASK,
            T_CH_EN_SHIFT,
            field_max(T_CH_EN_MASK, T_CH_EN_SHIFT),
        );
    }

    // -- INT_CONFIG_1 (0x08) ------------------------------------------------

    #[test]
    fn round_trip_rslt_int() {
        round_trip(
            RSLT_INT_MASK,
            RSLT_INT_SHIFT,
            field_max(RSLT_INT_MASK, RSLT_INT_SHIFT),
        );
    }

    #[test]
    fn round_trip_threshold_int() {
        round_trip(
            THRESHOLD_INT_MASK,
            THRESHOLD_INT_SHIFT,
            field_max(THRESHOLD_INT_MASK, THRESHOLD_INT_SHIFT),
        );
    }

    #[test]
    fn round_trip_int_state() {
        round_trip(
            INT_STATE_MASK,
            INT_STATE_SHIFT,
            field_max(INT_STATE_MASK, INT_STATE_SHIFT),
        );
    }

    #[test]
    fn round_trip_int_mode() {
        round_trip(
            INT_MODE_MASK,
            INT_MODE_SHIFT,
            field_max(INT_MODE_MASK, INT_MODE_SHIFT),
        );
    }

    #[test]
    fn round_trip_mask_intb() {
        round_trip(
            MASK_INTB_MASK,
            MASK_INTB_SHIFT,
            field_max(MASK_INTB_MASK, MASK_INTB_SHIFT),
        );
    }

    // -- I2C_ADDRESS (0x0C) -------------------------------------------------

    #[test]
    fn round_trip_i2c_address_bits() {
        round_trip(
            I2C_ADDRESS_BITS_MASK,
            I2C_ADDRESS_BITS_SHIFT,
            field_max(I2C_ADDRESS_BITS_MASK, I2C_ADDRESS_BITS_SHIFT),
        );
    }

    #[test]
    fn round_trip_i2c_address_update_en() {
        round_trip(
            I2C_ADDRESS_UPDATE_EN_MASK,
            I2C_ADDRESS_UPDATE_EN_SHIFT,
            field_max(I2C_ADDRESS_UPDATE_EN_MASK, I2C_ADDRESS_UPDATE_EN_SHIFT),
        );
    }

    // -- DEVICE_ID (0x0D) ---------------------------------------------------

    #[test]
    fn round_trip_version() {
        round_trip(
            VERSION_MASK,
            VERSION_SHIFT,
            field_max(VERSION_MASK, VERSION_SHIFT),
        );
    }

    // -- CONV_STATUS (0x18) -------------------------------------------------

    #[test]
    fn round_trip_result_status() {
        round_trip(
            RESULT_STATUS_MASK,
            RESULT_STATUS_SHIFT,
            field_max(RESULT_STATUS_MASK, RESULT_STATUS_SHIFT),
        );
    }

    // CONV_STATUS field masks (SET_COUNT, POR, DIAG_STATUS) and
    // DEVICE_STATUS field masks (INTB_RB, OSC_ER, INT_ER, OTP_CRC_ER, VCC_UV_ER)
    // are tested in types.rs via ConversionStatus::from_reg() and DeviceStatus::from_reg().

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
