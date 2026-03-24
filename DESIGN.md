# Design Decisions: Rust vs SparkFun C++

This document explains where our Rust TMAG5273 driver differs from the [SparkFun Arduino C++ library](https://github.com/sparkfun/SparkFun_TMAG5273_Arduino_Library) and why each design choice produces safer, faster, or more correct code.

## 1. Burst Reads vs Individual Register Reads

**SparkFun:** `getXData()`, `getYData()`, `getZData()` each issue separate I2C transactions. Reading all three axes plus the range register = 6+ I2C round-trips. No atomicity guarantee — a new conversion can complete between reads.

**Rust:** `read_magnetic()` issues a single 6-byte burst read from `X_MSB_RESULT` (0x12). The TMAG5273 latches all result registers when the MSB is read, guaranteeing all values come from the same conversion. Range is cached at `init()` — zero additional I2C reads.

**Impact:** 6x fewer I2C transactions. Atomic data. No stale-range reads.

## 2. Type-Safe Enums vs Raw u8

**SparkFun:** Configuration uses `#define` constants and `uint8_t` parameters. `setConvAvg(0xFF)` compiles and silently writes garbage to the register.

**Rust:** All configuration values are `#[repr(u8)]` enums with exhaustive `TryFrom<u8>`. `ConfigBuilder::conversion_average(ConversionAverage::X8)` only accepts valid variants. Invalid values are a compile error, not a runtime surprise.

**Impact:** Entire class of invalid-configuration bugs eliminated at compile time.

## 3. Result<T, E> vs int8_t Return Codes

**SparkFun:** Functions return `int8_t` where -1, 0, and 1 have inconsistent meanings across methods. `getXData()` returns `0.0` on failure — indistinguishable from a valid zero-field reading.

**Rust:** All fallible operations return `Result<T, Error<I2C::Error>>` with 8 typed error variants carrying context (register address, expected/got values, CRC bytes). The compiler enforces error handling via `#[must_use]`.

**Impact:** No silent failures. Every error path is explicit and informative.

## 4. Builder Pattern vs Imperative Setters

**SparkFun:** 25+ individual `set*()` calls, order-dependent, no consistency validation. Enabling angle calculation without two magnetic channels silently produces garbage.

**Rust:** `ConfigBuilder` collects all settings, then `build()` validates cross-field constraints (angle requires 2+ channels, wake-up-and-sleep requires sleep duration). Invalid combinations return `Err` before any register write.

**Impact:** Configuration is validated atomically. No partial-state bugs.

## 5. Typestate: Compile-Time State Machine

**SparkFun:** `getXData()` can be called before `begin()` — reads from an uninitialized I2C bus, returns garbage silently.

**Rust:** `Tmag5273<I2C, Unconfigured>` only exposes `init()`. Measurement methods exist only on `Tmag5273<I2C, Configured>`. Calling `read_magnetic()` before `init()` is a compile error. Zero runtime cost — `PhantomData` is erased.

**Impact:** Impossible to read from an uninitialized sensor.

## 6. InitError: Bus Recovery on Failure

**SparkFun:** If `begin()` fails, the I2C bus reference is leaked inside the object. The caller cannot reuse the bus for another device.

**Rust:** `init()` returns `InitError<I2C> { error, i2c }` on failure — the I2C bus is returned to the caller. This follows the `ads1x1x` crate pattern standard in the embedded-hal ecosystem.

**Impact:** Failed initialization doesn't consume the bus. Caller can retry or use it elsewhere.

## 7. Ownership vs Global State

**SparkFun:** Uses Arduino's global `Wire` singleton. Multiple `TMAG5273` instances share the same bus with no compile-time coordination.

**Rust:** The driver takes ownership of the I2C bus via move semantics. Only one reference exists at a time. For shared buses, consumers use `embedded-hal-bus` `RefCellDevice`/`MutexDevice` — the sharing strategy is explicit, not implicit.

**Impact:** No accidental bus contention. Sharing is opt-in and visible.

## 8. Precomputed Reciprocals

**SparkFun:** `calculateMagneticField()` divides by 32768.0 on every conversion call.

**Rust:** `INV_32768`, `TADC_RES_INV`, `ANGLE_FRAC_SCALE` are compile-time `const f32` reciprocals. Conversions use multiplication only — no runtime division.

**Impact:** Faster on FPU-less targets. Same speed on FPU targets (compiler often optimizes this anyway, but we don't rely on it).

## 9. CRC: Dual-Layer Control

**SparkFun:** `setCRCMode()` toggles the sensor's CRC register at runtime. CRC validation code is always compiled in.

**Rust:** Two independent layers:
- **Cargo feature `crc`** — controls whether CRC validation code is compiled. Without it, the driver reads 2 bytes (no CRC overhead).
- **`set_crc_enabled()`** — controls the sensor's hardware CRC generation register.

When `crc` is off, the CRC-8 function and 3-byte read paths are completely eliminated from the binary.

**Impact:** Zero-cost when CRC is not needed. Full protection when it is.

## 10. Read Mode Safety Guard

**SparkFun:** `setReadMode()` changes the sensor's I2C response format. If the mode is changed to 1-byte, `getXData()` still reads 2 bytes — silently returning wrong data.

**Rust:** `set_read_mode()` stores the mode in the driver struct. `read_magnetic()`, `read_temperature()`, and `read_angle()` check `self.read_mode != Standard` and return `Error::NonStandardReadMode` instead of silently misinterpreting the response.

**Impact:** Impossible to get corrupt data from a mode mismatch.

## Summary

| Aspect | SparkFun | Rust | Improvement |
|--------|----------|------|-------------|
| I2C reads per XYZ | 6+ transactions | 1 burst | 6x fewer |
| Invalid config values | Runtime garbage | Compile error | Eliminated |
| Error handling | Magic int8_t | Typed Result<T,E> | Explicit |
| Read before init | Silent garbage | Compile error | Impossible |
| Bus on init failure | Leaked | Returned | Recoverable |
| CRC code size | Always compiled | Feature-gated | Zero-cost |
| Division in hot path | Runtime /32768 | Const reciprocal | Faster |
| Mode mismatch | Silent corruption | Error returned | Safe |
