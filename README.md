# tmag5273

Platform-agnostic `no_std` driver for the [TI TMAG5273](https://www.ti.com/product/TMAG5273)
3-axis linear Hall-effect sensor, built on [`embedded-hal`](https://docs.rs/embedded-hal/1.0) 1.0 traits.

## Supported Variants

All eight TMAG5273 variants are supported:

| Variant | Default I2C Address | Sensitivity (Low / High)              |
| ------- | ------------------- | ------------------------------------- |
| A1 / A2 | `0x35`              | v1: ±40 / ±80 mT · v2: ±133 / ±266 mT |
| B1 / B2 | `0x22`              | v1: ±40 / ±80 mT · v2: ±133 / ±266 mT |
| C1 / C2 | `0x78`              | v1: ±40 / ±80 mT · v2: ±133 / ±266 mT |
| D1 / D2 | `0x44`              | v1: ±40 / ±80 mT · v2: ±133 / ±266 mT |

The letter (A–D) determines the factory-default I2C address.
The number (1/2) determines the full-scale sensitivity range.

## Add to Your Project

```toml
[dependencies]
tmag5273 = "0.1"

# Optional features:
# tmag5273 = { version = "0.1", features = ["crc"] }
# tmag5273 = { version = "0.1", features = ["defmt"] }
```

## Quick Start

```rust,no_run
use tmag5273::{Tmag5273, DeviceVariant, ConfigBuilder};

// 1. Create driver (no I2C activity yet — address derived from variant)
let sensor = Tmag5273::new(i2c, DeviceVariant::B1);

// 2. Build a validated configuration (defaults: continuous, XYZ, temp enabled, ±80 mT)
let config = ConfigBuilder::new().build()?;

// 3. Initialize: verifies manufacturer ID, applies config → Configured state
let mut sensor = sensor.init(&config)?;

// 4. Read magnetic field (all enabled axes)
let reading = sensor.read_magnetic()?;
if let Some(x) = reading.x {
    // x.0 is in millitesla (f32)
}

// 5. Read temperature
let temp = sensor.read_temperature()?;
// temp.0 is degrees Celsius (f32)

// 6. Coherent T+XYZ burst read (single 8-byte I2C transaction)
let all = sensor.read_all()?;
```

## Cargo Commands

```bash
# Run all unit tests (no hardware required)
cargo test -p tmag5273

# Run tests with the CRC feature enabled
cargo test -p tmag5273 --features crc

# Run tests with all features
cargo test -p tmag5273 --all-features

# Build docs locally (with feature annotations)
cargo doc -p tmag5273 --all-features --open

# Check the crate compiles for a bare-metal target
cargo check -p tmag5273 --target thumbv7em-none-eabihf
```

## Configuration

Use `ConfigBuilder` to construct a validated `Config`:

```rust,no_run
use tmag5273::{
    ConfigBuilder, OperatingMode, MagneticChannel, AngleCalculation,
    ConversionAverage, Range, PowerMode, SleepDuration, TempCoefficient,
};

let config = ConfigBuilder::new()
    .operating_mode(OperatingMode::ContinuousMeasure) // default
    .magnetic_channels(MagneticChannel::XYZ)          // default
    .temperature_enabled(true)                         // default
    .angle_calculation(AngleCalculation::None)         // default
    .conversion_average(ConversionAverage::X4)
    .xy_range(Range::High)                             // default: ±80 mT (v1)
    .z_range(Range::High)
    .power_mode(PowerMode::LowNoise)
    .temp_coefficient(TempCoefficient::NdBFe)
    .build()?;
```

`build()` validates cross-field constraints before writing any register:

- `AngleCalculation` ≠ `None` requires at least two magnetic channels.
- `OperatingMode::WakeUpAndSleep` requires `SleepDuration` > 1 ms.

### Defaults (match SparkFun `begin()`)

| Field                  | Default             |
| ---------------------- | ------------------- |
| `operating_mode`       | `ContinuousMeasure` |
| `magnetic_channels`    | `XYZ`               |
| `temperature_enabled`  | `true`              |
| `angle_calculation`    | `None`              |
| `conversion_average`   | `X1` (no averaging) |
| `xy_range` / `z_range` | `High` (±80 mT v1)  |
| `power_mode`           | `LowActiveCurrent`  |
| `glitch_filter`        | `true`              |
| `temp_coefficient`     | `None`              |

### Conversion Time

Use `ConversionAverage::conversion_time()` to query the expected conversion
duration (useful for polling delays or scheduling):

```rust,no_run
use tmag5273::{ConversionAverage, MagneticChannel, Micros};

let t: Micros = ConversionAverage::X8.conversion_time(MagneticChannel::XYZ, true);
// t.0 is microseconds
```

## Reading Data

### Magnetic Field

```rust,no_run
let reading = sensor.read_magnetic()?;
// reading.x, reading.y, reading.z are Option<MilliTesla>
// None for disabled channels
```

### Temperature

```rust,no_run
let temp: tmag5273::Celsius = sensor.read_temperature()?;
// temp.0 is degrees Celsius
```

### Angle (CORDIC Engine)

Requires `AngleCalculation` ≠ `None` in the config.

```rust,no_run
use tmag5273::{ConfigBuilder, AngleCalculation, MagneticChannel};

let config = ConfigBuilder::new()
    .angle_calculation(AngleCalculation::XY)
    .magnetic_channels(MagneticChannel::XY)
    .build()?;

let mut sensor = sensor.init(&config)?;
let angle = sensor.read_angle()?;
// angle.angle.0 is degrees (0–360, f32)
// angle.magnitude is 0–255 (unitless field magnitude)
```

### Coherent Burst Read

`read_all()` performs a single 8-byte I2C burst read (0x10–0x17), guaranteeing
all values come from the same conversion cycle — no risk of mixing data from
different cycles.

```rust,no_run
let reading: tmag5273::SensorReading = sensor.read_all()?;
// reading.temperature: Option<Celsius>
// reading.magnetic:    MagneticReading { x, y, z: Option<MilliTesla> }
```

## Standby / Trigger Mode

In standby mode the device measures only when triggered:

```rust,no_run
use tmag5273::{ConfigBuilder, OperatingMode};

let config = ConfigBuilder::new()
    .operating_mode(OperatingMode::Standby)
    .build()
    ?;
let mut sensor = sensor.init(&config)?;

// Trigger a conversion, then poll for completion
sensor.trigger_conversion()?;
let status = sensor.wait_for_conversion()?;

// Now safe to read
let reading = sensor.read_magnetic()?;
```

## Runtime Controls

| Method                                       | Description                                     |
| -------------------------------------------- | ----------------------------------------------- |
| `set_mode(OperatingMode)`                    | Switch operating mode at runtime                |
| `trigger_conversion()`                       | Trigger one conversion (standby / trigger mode) |
| `wait_for_conversion()` → `ConversionStatus` | Blocking poll with bounded retry                |
| `is_conversion_complete()` → `bool`          | Non-blocking check                              |
| `read_status()` → `DeviceStatus`             | Read fault-flag register                        |
| `check_diagnostics()` → `DeviceStatus`       | Lazy diagnostic read (skips status if no fault) |
| `is_interrupt_active()` → `bool`             | Read INT pin state                              |
| `set_crc_enabled(bool)`                      | Toggle sensor-side CRC-8 generation             |
| `get_crc_enabled()` → `bool`                 | Read sensor-side CRC setting                    |
| `set_read_mode(I2cReadMode)`                 | Change I2C response format                      |
| `get_read_mode()` → `I2cReadMode`            | Read current response format                    |

## Advanced

### Threshold Interrupts

```rust,no_run
use tmag5273::{ThresholdConfig, ThresholdDirection, InterruptConfig, InterruptMode, IntPinBehavior};

sensor.set_thresholds(&ThresholdConfig {
    x: 50, y: 50, z: 50, temperature: 0,
    direction: ThresholdDirection::Above,
    hysteresis: 1,
})?;

sensor.set_interrupt(&InterruptConfig {
    on_conversion_complete: false,
    on_threshold_crossing: true,
    mode: InterruptMode::IntPin,
    pin_behavior: IntPinBehavior::Pulse10us,
    mask: false,
})?;
```

### Gain / Offset Calibration

```rust,no_run
use tmag5273::{CalibrationConfig, GainChannel};

sensor.set_calibration(&CalibrationConfig {
    gain: 128,
    offset_1: 0,
    offset_2: 0,
    gain_channel: GainChannel::X,
})?;
```

### Wake-Up-and-Sleep Mode

Implements TI datasheet section 8.2.1.2 with TI-recommended settings:
threshold interrupt on INT pin, 10 µs pulse, hysteresis = 1.

```rust,no_run
// Set thresholds first, then activate wake-and-sleep
sensor.set_thresholds(&threshold_config)?;
sensor.configure_wake_and_sleep()?;
```

### Runtime I2C Address Change

```rust,no_run
// Change address and verify the device responds at the new address
sensor.change_address(0x30)?;
// Note: resets to factory default on power cycle
```

## Error Handling

`Error<E>` is generic over the I2C bus error type:

| Variant                                    | Cause                                                                   |
| ------------------------------------------ | ----------------------------------------------------------------------- |
| `I2c(E)`                                   | I2C bus error                                                           |
| `InvalidManufacturerId(u16)`               | Device ID is not `0x5449` (ASCII "TI")                                  |
| `VersionMismatch { expected, got }`        | Device version does not match variant                                   |
| `CrcMismatch { expected, computed }`       | CRC-8 validation failed (`crc` feature)                                 |
| `ConversionTimeout`                        | Conversion not ready within poll limit                                  |
| `AngleRequiresTwoChannels`                 | Angle config without two axes enabled                                   |
| `SleepDurationRequired`                    | Wake-sleep mode without sleep duration > 1 ms                           |
| `AddressChangeFailed { new_address }`      | Device did not respond at new address                                   |
| `InvalidRegisterValue { register, value }` | Unknown enum value in register                                          |
| `NonStandardReadMode { mode }`             | High-level read called in non-Standard I2C mode                         |
| `TemperatureDisabled`                      | `read_temperature()` called with temperature channel disabled in config |

`InitError<I2C>` wraps `Error` together with the I2C bus so callers can
recover the peripheral on initialization failure.

```rust,no_run
match sensor.init(&config) {
    Ok(s)   => { /* use s */ }
    Err(e)  => {
        // e.error: Error<I2C::Error>
        // e.i2c:  I2C bus — returned for reuse
    }
}
```

## Shared I2C Bus

The driver takes ownership of the I2C bus. For a shared bus use
[`embedded-hal-bus`](https://docs.rs/embedded-hal-bus):

```rust,no_run
use embedded_hal_bus::i2c::RefCellDevice;
use core::cell::RefCell;

let bus = RefCell::new(i2c);
let sensor_a = Tmag5273::new(RefCellDevice::new(&bus), DeviceVariant::A1);
let sensor_b = Tmag5273::new(RefCellDevice::new(&bus), DeviceVariant::C1);
```

## Features

| Feature | Description                                                                                                                                                |
| ------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `crc`   | Enables CRC-8 validation on I2C register-pair reads. Incompatible with burst reads — `read_magnetic()` and `read_all()` always use burst mode without CRC. |
| `defmt` | Enables `defmt::Format` derives on all public types (requires `defmt` 1.0).                                                                                |

Both features are off by default.

## Design

| Property                  | Detail                                                                                               |
| ------------------------- | ---------------------------------------------------------------------------------------------------- |
| `no_std`                  | Zero allocations — no heap dependency                                                                |
| `#![forbid(unsafe_code)]` | No unsafe blocks in this crate                                                                       |
| Typestate                 | `Unconfigured` → `Configured` enforced at compile time via `PhantomData`                             |
| Builder config            | Cross-field validation at build time, not at register write                                          |
| `InitError`               | Returns the I2C bus on init failure for downstream recovery                                          |
| Newtypes                  | `MilliTesla`, `Celsius`, `Degrees`, `Micros` prevent unit confusion                                  |
| Burst reads               | `read_magnetic()` and `read_all()` use single I2C transactions for data coherence                    |
| Const reciprocals         | Temperature / magnetic / angle conversions multiply by `const f32` reciprocals — no runtime division |

See [`DESIGN.md`](DESIGN.md) for a detailed comparison with the SparkFun Arduino library.

## Minimum Supported Rust Version

Rust **1.93** (edition 2024).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.
