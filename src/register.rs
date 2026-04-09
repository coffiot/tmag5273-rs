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
/// Bit 7 of the register address — OR into address on a read to trigger a new conversion
/// simultaneously (standby/trigger mode). Datasheet §7.5.1.1.
pub(crate) const CONVERSION_START_BIT: u8 = 0x80;

// ---------------------------------------------------------------------------
// 0x00 — DEVICE_CONFIG_1
// ---------------------------------------------------------------------------
/// Device configuration register 1 address (0x00). Controls CRC, temp compensation,
/// conversion averaging, and I2C read mode.
pub(crate) const DEVICE_CONFIG_1: u8 = 0x00;

/// Bit mask for `CRC_EN` field in `DEVICE_CONFIG_1` — enables I2C CRC-8 appended to reads.
pub(crate) const CRC_EN_MASK: u8 = 0b1000_0000;
/// Bit shift for `CRC_EN` field in `DEVICE_CONFIG_1`.
pub(crate) const CRC_EN_SHIFT: u8 = 7;

/// Bit mask for `MAG_TEMPCO` field in `DEVICE_CONFIG_1` — selects magnetic temperature
/// compensation coefficient (None / NdBFe / Ceramic).
pub(crate) const MAG_TEMPCO_MASK: u8 = 0b0110_0000;
/// Bit shift for `MAG_TEMPCO` field in `DEVICE_CONFIG_1`.
pub(crate) const MAG_TEMPCO_SHIFT: u8 = 5;

/// Bit mask for `CONV_AVG` field in `DEVICE_CONFIG_1` — sets conversion averaging (1×–32×).
pub(crate) const CONV_AVG_MASK: u8 = 0b0001_1100;
/// Bit shift for `CONV_AVG` field in `DEVICE_CONFIG_1`.
pub(crate) const CONV_AVG_SHIFT: u8 = 2;

/// Bit mask for `I2C_RD` field in `DEVICE_CONFIG_1` — selects I2C read frame format
/// (Standard / 1-byte 16-bit / 1-byte 8-bit).
pub(crate) const I2C_RD_MASK: u8 = 0b0000_0011;
/// Bit shift for `I2C_RD` field in `DEVICE_CONFIG_1`.
pub(crate) const I2C_RD_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x01 — DEVICE_CONFIG_2
// ---------------------------------------------------------------------------
/// Device configuration register 2 address (0x01). Controls threshold hysteresis,
/// power mode, glitch filter, trigger mode, and operating mode.
pub(crate) const DEVICE_CONFIG_2: u8 = 0x01;

/// Bit mask for `THR_HYST` field in `DEVICE_CONFIG_2` — magnetic threshold hysteresis
/// (LimitCross / SymmetricBand 1–7 LSB).
pub(crate) const THR_HYST_MASK: u8 = 0b1110_0000;
/// Bit shift for `THR_HYST` field in `DEVICE_CONFIG_2`.
pub(crate) const THR_HYST_SHIFT: u8 = 5;

/// Bit mask for `LP_LN` field in `DEVICE_CONFIG_2` — selects Low-Power vs Low-Noise mode.
pub(crate) const LP_LN_MASK: u8 = 0b0001_0000;
/// Bit shift for `LP_LN` field in `DEVICE_CONFIG_2`.
pub(crate) const LP_LN_SHIFT: u8 = 4;

/// Bit mask for `I2C_GLITCH_FILTER` field in `DEVICE_CONFIG_2` — enables I2C glitch filter.
pub(crate) const I2C_GLITCH_FILTER_MASK: u8 = 0b0000_1000;
/// Bit shift for `I2C_GLITCH_FILTER` field in `DEVICE_CONFIG_2`.
pub(crate) const I2C_GLITCH_FILTER_SHIFT: u8 = 3;

/// Bit mask for `TRIGGER_MODE` field in `DEVICE_CONFIG_2` — selects conversion trigger
/// source (I2C command vs INT pin).
pub(crate) const TRIGGER_MODE_MASK: u8 = 0b0000_0100;
/// Bit shift for `TRIGGER_MODE` field in `DEVICE_CONFIG_2`.
pub(crate) const TRIGGER_MODE_SHIFT: u8 = 2;

/// Bit mask for `OPERATING_MODE` field in `DEVICE_CONFIG_2` — selects Standby / Sleep /
/// Continuous / Wake-up-and-Sleep mode.
pub(crate) const OPERATING_MODE_MASK: u8 = 0b0000_0011;
/// Bit shift for `OPERATING_MODE` field in `DEVICE_CONFIG_2`.
pub(crate) const OPERATING_MODE_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x02 — SENSOR_CONFIG_1
// ---------------------------------------------------------------------------
/// Sensor configuration register 1 address (0x02). Controls magnetic channel selection
/// and wake-up/sleep duration.
pub(crate) const SENSOR_CONFIG_1: u8 = 0x02;

/// Bit mask for `MAG_CH_EN` field in `SENSOR_CONFIG_1` — selects which magnetic axes
/// to measure (Off / X / Y / XY / Z / ZX / YZ / XYZ / pseudo-simultaneous pairs).
pub(crate) const MAG_CH_EN_MASK: u8 = 0b1111_0000;
/// Bit shift for `MAG_CH_EN` field in `SENSOR_CONFIG_1`.
pub(crate) const MAG_CH_EN_SHIFT: u8 = 4;

/// Bit mask for `SLEEPTIME` field in `SENSOR_CONFIG_1` — sets sleep interval in
/// wake-up-and-sleep mode (1 ms–1000 ms).
pub(crate) const SLEEPTIME_MASK: u8 = 0b0000_1111;
/// Bit shift for `SLEEPTIME` field in `SENSOR_CONFIG_1`.
pub(crate) const SLEEPTIME_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x03 — SENSOR_CONFIG_2
// ---------------------------------------------------------------------------
/// Sensor configuration register 2 address (0x03). Controls threshold crossing count,
/// direction, gain channel, angle calculation, and magnetic range.
pub(crate) const SENSOR_CONFIG_2: u8 = 0x03;

/// Bit mask for the RESERVED bit in `SENSOR_CONFIG_2` (bit 7). Must not be written.
/// Datasheet §8.1.4.
#[allow(dead_code)]
pub(crate) const SENSOR_CONFIG_2_RESERVED_MASK: u8 = 0b1000_0000;

/// Bit mask for `THRX_COUNT` field in `SENSOR_CONFIG_2` — number of consecutive threshold
/// crossings required to assert INT (1 or 4).
pub(crate) const THRX_COUNT_MASK: u8 = 0b0100_0000;
/// Bit shift for `THRX_COUNT` field in `SENSOR_CONFIG_2`.
pub(crate) const THRX_COUNT_SHIFT: u8 = 6;

/// Bit mask for `MAG_THR_DIR` field in `SENSOR_CONFIG_2` — selects threshold crossing
/// direction (Above / Below).
pub(crate) const MAG_THR_DIR_MASK: u8 = 0b0010_0000;
/// Bit shift for `MAG_THR_DIR` field in `SENSOR_CONFIG_2`.
pub(crate) const MAG_THR_DIR_SHIFT: u8 = 5;

/// Bit mask for `MAG_GAIN_CH` field in `SENSOR_CONFIG_2` — selects which axis receives
/// the user gain correction (First / Second measurement in pseudo-simultaneous pairs).
pub(crate) const MAG_GAIN_CH_MASK: u8 = 0b0001_0000;
/// Bit shift for `MAG_GAIN_CH` field in `SENSOR_CONFIG_2`.
pub(crate) const MAG_GAIN_CH_SHIFT: u8 = 4;

/// Bit mask for `ANGLE_EN` field in `SENSOR_CONFIG_2` — enables CORDIC angle calculation
/// and selects axis pair (None / XY / YZ / XZ).
pub(crate) const ANGLE_EN_MASK: u8 = 0b0000_1100;
/// Bit shift for `ANGLE_EN` field in `SENSOR_CONFIG_2`.
pub(crate) const ANGLE_EN_SHIFT: u8 = 2;

/// Bit mask for `X_Y_RANGE` field in `SENSOR_CONFIG_2` — selects XY magnetic range
/// (Low / High). Range values differ by device variant (x1 vs x2).
pub(crate) const X_Y_RANGE_MASK: u8 = 0b0000_0010;
/// Bit shift for `X_Y_RANGE` field in `SENSOR_CONFIG_2`.
pub(crate) const X_Y_RANGE_SHIFT: u8 = 1;

/// Bit mask for `Z_RANGE` field in `SENSOR_CONFIG_2` — selects Z magnetic range
/// (Low / High). Range values differ by device variant (x1 vs x2).
pub(crate) const Z_RANGE_MASK: u8 = 0b0000_0001;
/// Bit shift for `Z_RANGE` field in `SENSOR_CONFIG_2`.
pub(crate) const Z_RANGE_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x04 — X_THR_CONFIG
// ---------------------------------------------------------------------------
/// X-axis magnetic threshold configuration register address (0x04).
pub(crate) const X_THR_CONFIG: u8 = 0x04;

// ---------------------------------------------------------------------------
// 0x05 — Y_THR_CONFIG
// ---------------------------------------------------------------------------
/// Y-axis magnetic threshold configuration register address (0x05).
pub(crate) const Y_THR_CONFIG: u8 = 0x05;

// ---------------------------------------------------------------------------
// 0x06 — Z_THR_CONFIG
// ---------------------------------------------------------------------------
/// Z-axis magnetic threshold configuration register address (0x06).
pub(crate) const Z_THR_CONFIG: u8 = 0x06;

// ---------------------------------------------------------------------------
// 0x07 — T_CONFIG
// ---------------------------------------------------------------------------
/// Temperature configuration register address (0x07). Holds 7-bit temperature
/// threshold (bits [7:1]) and temperature channel enable (bit 0).
pub(crate) const T_CONFIG: u8 = 0x07;

/// Bit mask for the 7-bit temperature threshold field in `T_CONFIG` (bits [7:1]).
pub(crate) const T_THR_CONFIG_MASK: u8 = 0b1111_1110;
/// Bit shift for the temperature threshold field in `T_CONFIG`.
pub(crate) const T_THR_CONFIG_SHIFT: u8 = 1;

/// Minimum valid active `T_THR_CONFIG` code: 0x1A (≈ −41 °C).
/// Datasheet §8.1.8: "threshold codes: −41C = 1Ah".
pub(crate) const T_THR_CONFIG_MIN: u8 = 0x1A;

/// Maximum valid active `T_THR_CONFIG` code: 0x34 (≈ 170 °C).
/// Datasheet §8.1.8: "threshold codes: 170C = 34h".
pub(crate) const T_THR_CONFIG_MAX: u8 = 0x34;

/// Bit mask for `T_CH_EN` in `T_CONFIG` — enables the temperature measurement channel.
pub(crate) const T_CH_EN_MASK: u8 = 0b0000_0001;
/// Bit shift for `T_CH_EN` in `T_CONFIG`.
pub(crate) const T_CH_EN_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x08 — INT_CONFIG_1
// ---------------------------------------------------------------------------
/// Interrupt configuration register 1 address (0x08). Controls INT pin mode,
/// trigger sources, pin behavior (latched/pulsed), and INT masking.
pub(crate) const INT_CONFIG_1: u8 = 0x08;

/// Bit mask for the RESERVED bit in `INT_CONFIG_1` (bit 1). Must not be written.
/// Datasheet §8.1.9.
#[allow(dead_code)]
pub(crate) const INT_CONFIG_1_RESERVED_MASK: u8 = 0b0000_0010;

/// Bit mask for `RSLT_INT` in `INT_CONFIG_1` — asserts INT on conversion complete.
pub(crate) const RSLT_INT_MASK: u8 = 0b1000_0000;
/// Bit shift for `RSLT_INT` in `INT_CONFIG_1`.
pub(crate) const RSLT_INT_SHIFT: u8 = 7;

/// Bit mask for `THRSLD_INT` in `INT_CONFIG_1` — asserts INT on threshold crossing.
pub(crate) const THRSLD_INT_MASK: u8 = 0b0100_0000;
/// Bit shift for `THRSLD_INT` in `INT_CONFIG_1`.
pub(crate) const THRSLD_INT_SHIFT: u8 = 6;

/// Bit mask for `INT_STATE` in `INT_CONFIG_1` — selects INT pin behavior
/// (0 = Latched, 1 = 10 µs pulse).
pub(crate) const INT_STATE_MASK: u8 = 0b0010_0000;
/// Bit shift for `INT_STATE` in `INT_CONFIG_1`.
pub(crate) const INT_STATE_SHIFT: u8 = 5;

/// Bit mask for `INT_MODE` field in `INT_CONFIG_1` — selects INT routing
/// (None / Through INT / Through INT except I2C busy / Through SCL / Through SCL except I2C busy).
pub(crate) const INT_MODE_MASK: u8 = 0b0001_1100;
/// Bit shift for `INT_MODE` field in `INT_CONFIG_1`.
pub(crate) const INT_MODE_SHIFT: u8 = 2;

/// Bit mask for `MASK_INTB` in `INT_CONFIG_1` — masks the INT pin output when set.
pub(crate) const MASK_INTB_MASK: u8 = 0b0000_0001;
/// Bit shift for `MASK_INTB` in `INT_CONFIG_1`.
pub(crate) const MASK_INTB_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x09 — MAG_GAIN_CONFIG
// ---------------------------------------------------------------------------
/// Magnetic gain configuration register address (0x09). Holds the 8-bit user
/// gain correction coefficient applied to the selected axis pair.
pub(crate) const MAG_GAIN_CONFIG: u8 = 0x09;

/// `GAIN_VALUE = 0` is interpreted as gain = 1.0 by hardware.
/// Datasheet §8.1.10: "Gain value of 0 is interpreted as 1."
#[allow(dead_code)]
pub(crate) const GAIN_VALUE_IDENTITY: u8 = 0x00;

/// Bit mask for `GAIN_VALUE` field in `MAG_GAIN_CONFIG` — full 8-bit gain coefficient.
/// Datasheet §8.1.10.
#[allow(dead_code)]
pub(crate) const GAIN_VALUE_MASK: u8 = 0xFF;
/// Bit shift for `GAIN_VALUE` field in `MAG_GAIN_CONFIG`.
#[allow(dead_code)]
pub(crate) const GAIN_VALUE_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x0A — MAG_OFFSET_CONFIG_1
// ---------------------------------------------------------------------------
/// Magnetic offset configuration register 1 address (0x0A). Holds the 8-bit
/// signed offset correction for the first axis in the selected measurement pair.
pub(crate) const MAG_OFFSET_CONFIG_1: u8 = 0x0A;

/// Bit mask for `OFFSET_VALUE_1ST` field in `MAG_OFFSET_CONFIG_1` — full 8-bit offset.
/// Datasheet §8.1.11.
#[allow(dead_code)]
pub(crate) const OFFSET_VALUE_1ST_MASK: u8 = 0xFF;
/// Bit shift for `OFFSET_VALUE_1ST` field in `MAG_OFFSET_CONFIG_1`.
#[allow(dead_code)]
pub(crate) const OFFSET_VALUE_1ST_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x0B — MAG_OFFSET_CONFIG_2
// ---------------------------------------------------------------------------
/// Magnetic offset configuration register 2 address (0x0B). Holds the 8-bit
/// signed offset correction for the second axis in the selected measurement pair.
pub(crate) const MAG_OFFSET_CONFIG_2: u8 = 0x0B;

/// Bit mask for `OFFSET_VALUE_2ND` field in `MAG_OFFSET_CONFIG_2` — full 8-bit offset.
/// Datasheet §8.1.12.
#[allow(dead_code)]
pub(crate) const OFFSET_VALUE_2ND_MASK: u8 = 0xFF;
/// Bit shift for `OFFSET_VALUE_2ND` field in `MAG_OFFSET_CONFIG_2`.
#[allow(dead_code)]
pub(crate) const OFFSET_VALUE_2ND_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x0C — I2C_ADDRESS (read-only)
// ---------------------------------------------------------------------------
/// I2C address register address (0x0C). Contains the 7-bit I2C address in bits [7:1]
/// and the address-update-enable flag in bit 0. Write-protected until bit 0 is set.
pub(crate) const I2C_ADDRESS: u8 = 0x0C;

/// Bit mask for the 7-bit I2C address field in `I2C_ADDRESS` register (bits [7:1]).
pub(crate) const I2C_ADDRESS_BITS_MASK: u8 = 0b1111_1110;
/// Bit shift for the I2C address field in `I2C_ADDRESS` register.
pub(crate) const I2C_ADDRESS_BITS_SHIFT: u8 = 1;

/// Bit mask for `I2C_ADDRESS_UPDATE_EN` in `I2C_ADDRESS` — must be set to 1 before
/// writing a new I2C address; self-clears after the address is latched.
pub(crate) const I2C_ADDRESS_UPDATE_EN_MASK: u8 = 0b0000_0001;
/// Bit shift for `I2C_ADDRESS_UPDATE_EN` in `I2C_ADDRESS`.
pub(crate) const I2C_ADDRESS_UPDATE_EN_SHIFT: u8 = 0;

/// Hardware reset value for `I2C_ADDRESS`: `0x6A` (default address 0x35 in bits [7:1]).
/// Datasheet §8.1.13: "Reset = 6Ah".
#[allow(dead_code)]
pub(crate) const I2C_ADDRESS_RESET: u8 = 0x6A;

/// Lowest usable 7-bit I2C address. Addresses 0x00–0x07 are reserved by
/// the I2C specification.
pub(crate) const I2C_ADDRESS_MIN: u8 = 0x08;

/// Highest usable 7-bit I2C address. Addresses 0x78–0x7F are reserved by
/// the I2C specification.
pub(crate) const I2C_ADDRESS_MAX: u8 = 0x77;

/// Lower 7 bits mask (`0b0111_1111`): isolates bits [6:0], clears bit 7.
///
/// The datasheet uses this bit width in two register families:
///
/// - **`THR_HYST = 1h`** (§7.4.1, DEVICE_CONFIG_2 §8.1.2): *"Takes the
///   7 LSB bits of the x_THR_CONFIG register to create two opposite
///   magnetic thresholds of equal magnitude."*
///
/// - **`I2C_ADDRESS`** (§8.1.13): *"There are 7-bits to select 128
///   different addresses."* The `embedded-hal` I2C trait uses 7-bit
///   addresses; this mask strips any stray bit 7 from user-supplied values.
pub(crate) const LSB_7BIT_MASK: u8 = 0x7F;

// ---------------------------------------------------------------------------
// 0x0D — DEVICE_ID (read-only)
// ---------------------------------------------------------------------------
/// Device ID register address (0x0D). Bits [1:0] hold the version code that
/// identifies the device variant (A1/A2 = 0, B1/B2 = 1, C1/C2 = 2, D1/D2 = 3).
pub(crate) const DEVICE_ID: u8 = 0x0D;

/// Bit mask for the `VER` field in `DEVICE_ID` (bits [1:0]).
pub(crate) const VER_MASK: u8 = 0b0000_0011;
/// Bit shift for the `VER` field in `DEVICE_ID`.
pub(crate) const VER_SHIFT: u8 = 0;

/// Bit mask for the RESERVED bits in `DEVICE_ID` (bits [7:2]). Must not be written.
/// Datasheet §8.1.14.
#[allow(dead_code)]
pub(crate) const DEVICE_ID_RESERVED_MASK: u8 = 0b1111_1100;

/// `VER` value for x1 variants (±40/±80 mT range).
/// Datasheet §8.1.14: "1h = ±40-mT and ±80-mT range".
pub(crate) const VER_V1: u8 = 0x01;

/// `VER` value for x2 variants (±133/±266 mT range).
/// Datasheet §8.1.14: "2h = ±133-mT and ±266-mT range".
pub(crate) const VER_V2: u8 = 0x02;

// ---------------------------------------------------------------------------
// 0x0E — MANUFACTURER_ID_LSB (read-only)
// ---------------------------------------------------------------------------
/// Manufacturer ID LSB register address (0x0E). Two-byte burst read from here
/// returns [LSB=0x49, MSB=0x54], forming the 16-bit ASCII code for "TI" (0x5449).
pub(crate) const MANUFACTURER_ID_LSB: u8 = 0x0E;
/// Manufacturer ID MSB register address (0x0F). Always read as the second byte of a
/// 2-byte burst starting at [`MANUFACTURER_ID_LSB`] — never addressed directly.
#[allow(dead_code)]
pub(crate) const MANUFACTURER_ID_MSB: u8 = 0x0F;

/// Expected 16-bit manufacturer ID: ASCII "TI" = 0x5449.
pub(crate) const MANUFACTURER_ID_EXPECTED: u16 = 0x5449;

// ---------------------------------------------------------------------------
// 0x10 — T_MSB_RESULT (read-only)
// ---------------------------------------------------------------------------
/// Temperature result MSB register address (0x10). A 2-byte burst read from here
/// returns [T_MSB, T_LSB] (0x10–0x11); the 16-bit raw value maps to °C.
pub(crate) const T_MSB_RESULT: u8 = 0x10;
/// Temperature result LSB register address (0x11). Always read as the second byte of a
/// 2-byte burst starting at [`T_MSB_RESULT`] — never addressed directly.
#[allow(dead_code)]
pub(crate) const T_LSB_RESULT: u8 = 0x11;

// ---------------------------------------------------------------------------
// 0x12 — X_MSB_RESULT (read-only, burst reads X/Y/Z from here)
// ---------------------------------------------------------------------------
/// X-axis result MSB register address (0x12). A 6-byte burst read from here
/// returns [X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB] (0x12–0x17).
#[cfg_attr(feature = "crc", allow(dead_code))]
pub(crate) const X_MSB_RESULT: u8 = 0x12;
/// X-axis result LSB register address (0x13). Always read as part of the 6-byte
/// burst starting at [`X_MSB_RESULT`] — never addressed directly.
#[allow(dead_code)]
pub(crate) const X_LSB_RESULT: u8 = 0x13;
/// Y-axis result MSB register address (0x14). Always read as part of the 6-byte
/// burst starting at [`X_MSB_RESULT`] — never addressed directly.
#[allow(dead_code)]
pub(crate) const Y_MSB_RESULT: u8 = 0x14;
/// Y-axis result LSB register address (0x15). Always read as part of the 6-byte
/// burst starting at [`X_MSB_RESULT`] — never addressed directly.
#[allow(dead_code)]
pub(crate) const Y_LSB_RESULT: u8 = 0x15;
/// Z-axis result MSB register address (0x16). Always read as part of the 6-byte
/// burst starting at [`X_MSB_RESULT`] — never addressed directly.
#[allow(dead_code)]
pub(crate) const Z_MSB_RESULT: u8 = 0x16;
/// Z-axis result LSB register address (0x17). Always read as part of the 6-byte
/// burst starting at [`X_MSB_RESULT`] — never addressed directly.
#[allow(dead_code)]
pub(crate) const Z_LSB_RESULT: u8 = 0x17;

// ---------------------------------------------------------------------------
// 0x18 — CONV_STATUS (read-only)
// ---------------------------------------------------------------------------
/// Conversion status register address (0x18). Contains data-ready flag, diagnostic
/// fail flag, channel result flags, and POR (power-on-reset) status.
pub(crate) const CONV_STATUS: u8 = 0x18;

/// Bit mask for `SET_COUNT` field in `CONV_STATUS` — 3-bit rolling counter incremented
/// each time a new conversion data set is produced.
pub(crate) const SET_COUNT_MASK: u8 = 0b1110_0000;
/// Bit shift for `SET_COUNT` field in `CONV_STATUS`.
pub(crate) const SET_COUNT_SHIFT: u8 = 5;

/// Bit mask for `POR` field in `CONV_STATUS` — power-on-reset occurred flag (R/W1CP).
/// Set by hardware on power-up; cleared by writing 1.
pub(crate) const POR_MASK: u8 = 0b0001_0000;
/// Bit shift for `POR` field in `CONV_STATUS`.
pub(crate) const POR_SHIFT: u8 = 4;

/// Bit mask for `DIAG_STATUS` field in `CONV_STATUS` — diagnostic self-test failure
/// (VCC undervoltage, memory CRC, INT̄ error, or clock error detected).
pub(crate) const DIAG_STATUS_MASK: u8 = 0b0000_0010;
/// Bit shift for `DIAG_STATUS` field in `CONV_STATUS`.
pub(crate) const DIAG_STATUS_SHIFT: u8 = 1;

/// Bit mask for `RESULT_STATUS` in `CONV_STATUS` — set when a new conversion result
/// is available (data ready).
pub(crate) const RESULT_STATUS_MASK: u8 = 0b0000_0001;
/// Bit shift for `RESULT_STATUS` in `CONV_STATUS`.
pub(crate) const RESULT_STATUS_SHIFT: u8 = 0;

/// Bit mask for the RESERVED bits in `CONV_STATUS` (bits [3:2]). Read-only, always 0.
/// Datasheet §8.1.25.
#[allow(dead_code)]
pub(crate) const CONV_STATUS_RESERVED_MASK: u8 = 0b0000_1100;

/// Hardware reset value for `CONV_STATUS`: `0x10` (POR flag set on power-up).
/// Datasheet §8.1.25: "Reset = 10h".
#[allow(dead_code)]
pub(crate) const CONV_STATUS_RESET: u8 = 0x10;

// ---------------------------------------------------------------------------
// 0x19 — ANGLE_RESULT_MSB (read-only)
// ---------------------------------------------------------------------------
/// Angle result MSB register address (0x19). A 2-byte burst read from here
/// returns [ANGLE_MSB, ANGLE_LSB] (0x19–0x1A); the 9.4 fixed-point value is in degrees
/// (9-bit integer 0–360, 4-bit fraction ×1/16°, 3 MSBs always zero). Datasheet §6.5.2.3.
pub(crate) const ANGLE_RESULT_MSB: u8 = 0x19;
/// Angle result LSB register address (0x1A). Always read as the second byte of a
/// 2-byte (or 3-byte CRC) burst starting at [`ANGLE_RESULT_MSB`] — never addressed directly.
#[allow(dead_code)]
pub(crate) const ANGLE_RESULT_LSB: u8 = 0x1A;

// ---------------------------------------------------------------------------
// 0x1B — MAGNITUDE_RESULT (read-only, used in CRC path of read_angle)
// ---------------------------------------------------------------------------
/// Magnitude result register address (0x1B). Holds the 8-bit CORDIC vector magnitude.
/// Only read in the CRC code path (3-byte burst: ANGLE_MSB, ANGLE_LSB, MAGNITUDE).
#[cfg_attr(not(feature = "crc"), allow(dead_code))]
pub(crate) const MAGNITUDE_RESULT: u8 = 0x1B;

// ---------------------------------------------------------------------------
// 0x1C — DEVICE_STATUS (read-only)
// ---------------------------------------------------------------------------
/// Device status register address (0x1C). Contains fault flags: VCC undervoltage,
/// oscillator error, OTP CRC error, and INT pin error. Write 1 to clear latched bits.
pub(crate) const DEVICE_STATUS: u8 = 0x1C;

/// Bit mask for `INTB_RB` field in `DEVICE_STATUS` — INT̄ pin readback; 1 = inactive (high).
pub(crate) const INTB_RB_MASK: u8 = 0b0001_0000;
/// Bit shift for `INTB_RB` field in `DEVICE_STATUS`.
pub(crate) const INTB_RB_SHIFT: u8 = 4;

/// Bit mask for `OSC_ER` field in `DEVICE_STATUS` — oscillator error detected (R/W1CP).
pub(crate) const OSC_ER_MASK: u8 = 0b0000_1000;
/// Bit shift for `OSC_ER` field in `DEVICE_STATUS`.
pub(crate) const OSC_ER_SHIFT: u8 = 3;

/// Bit mask for `INT_ER` field in `DEVICE_STATUS` — INT̄ pin error detected (R/W1CP).
pub(crate) const INT_ER_MASK: u8 = 0b0000_0100;
/// Bit shift for `INT_ER` field in `DEVICE_STATUS`.
pub(crate) const INT_ER_SHIFT: u8 = 2;

/// Bit mask for `OTP_CRC_ER` field in `DEVICE_STATUS` — OTP CRC error detected (R/W1CP).
pub(crate) const OTP_CRC_ER_MASK: u8 = 0b0000_0010;
/// Bit shift for `OTP_CRC_ER` field in `DEVICE_STATUS`.
pub(crate) const OTP_CRC_ER_SHIFT: u8 = 1;

/// Bit mask for `VCC_UV_ER` field in `DEVICE_STATUS` — VCC undervoltage detected (R/W1CP).
pub(crate) const VCC_UV_ER_MASK: u8 = 0b0000_0001;
/// Bit shift for `VCC_UV_ER` field in `DEVICE_STATUS`.
pub(crate) const VCC_UV_ER_SHIFT: u8 = 0;

/// Bit mask for the RESERVED bits in `DEVICE_STATUS` (bits [7:5]). Read-only, always 0.
/// Datasheet §8.1.29.
#[allow(dead_code)]
pub(crate) const DEVICE_STATUS_RESERVED_MASK: u8 = 0b1110_0000;

/// Hardware reset value for `DEVICE_STATUS`: `0x10` (INTB_RB high at power-up).
/// Datasheet §8.1.29: "Reset = 10h".
#[allow(dead_code)]
pub(crate) const DEVICE_STATUS_RESET: u8 = 0x10;

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
            (THRX_COUNT_MASK, THRX_COUNT_SHIFT, "THRX_COUNT"),
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
            (THRSLD_INT_MASK, THRSLD_INT_SHIFT, "THRSLD_INT"),
            (INT_STATE_MASK, INT_STATE_SHIFT, "INT_STATE"),
            (INT_MODE_MASK, INT_MODE_SHIFT, "INT_MODE"),
            (MASK_INTB_MASK, MASK_INTB_SHIFT, "MASK_INTB"),
            // MAG_GAIN_CONFIG (0x09)
            (GAIN_VALUE_MASK, GAIN_VALUE_SHIFT, "GAIN_VALUE"),
            // MAG_OFFSET_CONFIG_1 (0x0A)
            (
                OFFSET_VALUE_1ST_MASK,
                OFFSET_VALUE_1ST_SHIFT,
                "OFFSET_VALUE_1ST",
            ),
            // MAG_OFFSET_CONFIG_2 (0x0B)
            (
                OFFSET_VALUE_2ND_MASK,
                OFFSET_VALUE_2ND_SHIFT,
                "OFFSET_VALUE_2ND",
            ),
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
            (VER_MASK, VER_SHIFT, "VER"),
            // CONV_STATUS (0x18)
            (SET_COUNT_MASK, SET_COUNT_SHIFT, "SET_COUNT"),
            (POR_MASK, POR_SHIFT, "POR"),
            (DIAG_STATUS_MASK, DIAG_STATUS_SHIFT, "DIAG_STATUS"),
            (RESULT_STATUS_MASK, RESULT_STATUS_SHIFT, "RESULT_STATUS"),
            // DEVICE_STATUS (0x1C)
            (INTB_RB_MASK, INTB_RB_SHIFT, "INTB_RB"),
            (OSC_ER_MASK, OSC_ER_SHIFT, "OSC_ER"),
            (INT_ER_MASK, INT_ER_SHIFT, "INT_ER"),
            (OTP_CRC_ER_MASK, OTP_CRC_ER_SHIFT, "OTP_CRC_ER"),
            (VCC_UV_ER_MASK, VCC_UV_ER_SHIFT, "VCC_UV_ER"),
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
            (VER_MASK, 0x03, "VER_MASK = GENMASK(1,0)"),
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
        assert_eq!(THRX_COUNT_MASK, 0x40);
        assert_eq!(THRX_COUNT_SHIFT, 6);
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
            THRX_COUNT_MASK
                | MAG_THR_DIR_MASK
                | MAG_GAIN_CH_MASK
                | ANGLE_EN_MASK
                | X_Y_RANGE_MASK
                | Z_RANGE_MASK
                | SENSOR_CONFIG_2_RESERVED_MASK,
            0xFF,
            "SENSOR_CONFIG_2 masks must cover all 8 bits"
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
        assert_eq!(THRSLD_INT_MASK, 0x40);
        assert_eq!(THRSLD_INT_SHIFT, 6);
        assert_eq!(INT_STATE_MASK, 0x20);
        assert_eq!(INT_STATE_SHIFT, 5);
        assert_eq!(INT_MODE_MASK, 0x1C);
        assert_eq!(INT_MODE_SHIFT, 2);
        assert_eq!(MASK_INTB_MASK, 0x01);
        assert_eq!(MASK_INTB_SHIFT, 0);
        assert_eq!(
            RSLT_INT_MASK
                | THRSLD_INT_MASK
                | INT_STATE_MASK
                | INT_MODE_MASK
                | MASK_INTB_MASK
                | INT_CONFIG_1_RESERVED_MASK,
            0xFF,
            "INT_CONFIG_1 masks must cover all 8 bits"
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

    // TI datasheet SBASAI4 Rev C, Table 7-19
    #[test]
    fn datasheet_xref_device_id_fields() {
        assert_eq!(DEVICE_ID, 0x0D);
        assert_eq!(VER_MASK, 0x03);
        assert_eq!(VER_SHIFT, 0);
        assert_eq!(DEVICE_ID_RESERVED_MASK, 0xFC);
        assert_eq!(
            VER_MASK | DEVICE_ID_RESERVED_MASK,
            0xFF,
            "DEVICE_ID masks must cover all 8 bits"
        );
    }

    // TI datasheet SBASAI4 Rev C, Tables 7-15 through 7-17
    #[test]
    fn datasheet_xref_mag_gain_offset_fields() {
        assert_eq!(MAG_GAIN_CONFIG, 0x09);
        assert_eq!(GAIN_VALUE_MASK, 0xFF);
        assert_eq!(GAIN_VALUE_SHIFT, 0);
        assert_eq!(MAG_OFFSET_CONFIG_1, 0x0A);
        assert_eq!(OFFSET_VALUE_1ST_MASK, 0xFF);
        assert_eq!(OFFSET_VALUE_1ST_SHIFT, 0);
        assert_eq!(MAG_OFFSET_CONFIG_2, 0x0B);
        assert_eq!(OFFSET_VALUE_2ND_MASK, 0xFF);
        assert_eq!(OFFSET_VALUE_2ND_SHIFT, 0);
    }

    // TI datasheet SBASAI4 Rev C, Table 7-24
    #[test]
    fn datasheet_xref_conv_status_fields() {
        assert_eq!(CONV_STATUS, 0x18);
        assert_eq!(SET_COUNT_MASK, 0xE0);
        assert_eq!(SET_COUNT_SHIFT, 5);
        assert_eq!(POR_MASK, 0x10);
        assert_eq!(POR_SHIFT, 4);
        assert_eq!(DIAG_STATUS_MASK, 0x02);
        assert_eq!(DIAG_STATUS_SHIFT, 1);
        assert_eq!(RESULT_STATUS_MASK, 0x01);
        assert_eq!(RESULT_STATUS_SHIFT, 0);
        assert_eq!(
            SET_COUNT_MASK
                | POR_MASK
                | DIAG_STATUS_MASK
                | RESULT_STATUS_MASK
                | CONV_STATUS_RESERVED_MASK,
            0xFF,
            "CONV_STATUS masks must cover all 8 bits"
        );
    }

    // TI datasheet SBASAI4 Rev C, Table 7-29
    #[test]
    fn datasheet_xref_device_status_fields() {
        assert_eq!(DEVICE_STATUS, 0x1C);
        assert_eq!(INTB_RB_MASK, 0x10);
        assert_eq!(INTB_RB_SHIFT, 4);
        assert_eq!(OSC_ER_MASK, 0x08);
        assert_eq!(OSC_ER_SHIFT, 3);
        assert_eq!(INT_ER_MASK, 0x04);
        assert_eq!(INT_ER_SHIFT, 2);
        assert_eq!(OTP_CRC_ER_MASK, 0x02);
        assert_eq!(OTP_CRC_ER_SHIFT, 1);
        assert_eq!(VCC_UV_ER_MASK, 0x01);
        assert_eq!(VCC_UV_ER_SHIFT, 0);
        assert_eq!(
            INTB_RB_MASK
                | OSC_ER_MASK
                | INT_ER_MASK
                | OTP_CRC_ER_MASK
                | VCC_UV_ER_MASK
                | DEVICE_STATUS_RESERVED_MASK,
            0xFF,
            "DEVICE_STATUS masks must cover all 8 bits"
        );
    }

    // TI datasheet SBASAI4 Rev C, §8.1.25 and §8.1.29 — reset values
    #[test]
    fn datasheet_xref_reset_values() {
        // CONV_STATUS reset = 0x10 → POR bit set
        assert_eq!(CONV_STATUS_RESET, 0x10);
        assert_eq!(
            CONV_STATUS_RESET & POR_MASK,
            POR_MASK,
            "POR must be set at reset"
        );
        assert_eq!(
            CONV_STATUS_RESET & !POR_MASK & !CONV_STATUS_RESERVED_MASK,
            0,
            "only POR should be set in CONV_STATUS reset"
        );

        // DEVICE_STATUS reset = 0x10 → INTB_RB high
        assert_eq!(DEVICE_STATUS_RESET, 0x10);
        assert_eq!(
            DEVICE_STATUS_RESET & INTB_RB_MASK,
            INTB_RB_MASK,
            "INTB_RB must be high at reset"
        );
        assert_eq!(
            DEVICE_STATUS_RESET & !INTB_RB_MASK & !DEVICE_STATUS_RESERVED_MASK,
            0,
            "only INTB_RB should be set in DEVICE_STATUS reset"
        );
    }

    // TI datasheet SBASAI4 Rev C, Table 7-21
    #[test]
    fn datasheet_xref_device_status_bit_positions() {
        use crate::types::DeviceStatus;

        assert_eq!(DEVICE_STATUS, 0x1C);

        // Bit 0 = VCC_UV
        let s = DeviceStatus::from_register(1 << 0);
        assert!(s.vcc_undervoltage_error);
        assert!(!s.otp_crc_error);
        assert!(!s.int_error);
        assert!(!s.oscillator_error);
        assert!(!s.int_bar_readback_high);

        // Bit 1 = OTP_CRC
        let s = DeviceStatus::from_register(1 << 1);
        assert!(!s.vcc_undervoltage_error);
        assert!(s.otp_crc_error);
        assert!(!s.int_error);
        assert!(!s.oscillator_error);
        assert!(!s.int_bar_readback_high);

        // Bit 2 = INT_ERR
        let s = DeviceStatus::from_register(1 << 2);
        assert!(!s.vcc_undervoltage_error);
        assert!(!s.otp_crc_error);
        assert!(s.int_error);
        assert!(!s.oscillator_error);
        assert!(!s.int_bar_readback_high);

        // Bit 3 = OSC_ERR
        let s = DeviceStatus::from_register(1 << 3);
        assert!(!s.vcc_undervoltage_error);
        assert!(!s.otp_crc_error);
        assert!(!s.int_error);
        assert!(s.oscillator_error);
        assert!(!s.int_bar_readback_high);

        // Bit 4 = INTB_RB
        let s = DeviceStatus::from_register(1 << 4);
        assert!(!s.vcc_undervoltage_error);
        assert!(!s.otp_crc_error);
        assert!(!s.int_error);
        assert!(!s.oscillator_error);
        assert!(s.int_bar_readback_high);
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
                THRX_COUNT_MASK,
                MAG_THR_DIR_MASK,
                MAG_GAIN_CH_MASK,
                ANGLE_EN_MASK,
                X_Y_RANGE_MASK,
                Z_RANGE_MASK,
                SENSOR_CONFIG_2_RESERVED_MASK,
            ],
        );
        assert_pairwise_disjoint("T_CONFIG", &[T_THR_CONFIG_MASK, T_CH_EN_MASK]);
        assert_pairwise_disjoint(
            "INT_CONFIG_1",
            &[
                RSLT_INT_MASK,
                THRSLD_INT_MASK,
                INT_STATE_MASK,
                INT_MODE_MASK,
                MASK_INTB_MASK,
                INT_CONFIG_1_RESERVED_MASK,
            ],
        );
        assert_pairwise_disjoint("MAG_GAIN_CONFIG", &[GAIN_VALUE_MASK]);
        assert_pairwise_disjoint("MAG_OFFSET_CONFIG_1", &[OFFSET_VALUE_1ST_MASK]);
        assert_pairwise_disjoint("MAG_OFFSET_CONFIG_2", &[OFFSET_VALUE_2ND_MASK]);
        assert_pairwise_disjoint(
            "I2C_ADDRESS",
            &[I2C_ADDRESS_BITS_MASK, I2C_ADDRESS_UPDATE_EN_MASK],
        );
        assert_pairwise_disjoint("DEVICE_ID", &[VER_MASK, DEVICE_ID_RESERVED_MASK]);
        assert_pairwise_disjoint(
            "CONV_STATUS",
            &[
                SET_COUNT_MASK,
                POR_MASK,
                DIAG_STATUS_MASK,
                RESULT_STATUS_MASK,
                CONV_STATUS_RESERVED_MASK,
            ],
        );
        assert_pairwise_disjoint(
            "DEVICE_STATUS",
            &[
                INTB_RB_MASK,
                OSC_ER_MASK,
                INT_ER_MASK,
                OTP_CRC_ER_MASK,
                VCC_UV_ER_MASK,
                DEVICE_STATUS_RESERVED_MASK,
            ],
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
