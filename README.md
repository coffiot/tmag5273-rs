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
tmag5273 = "3"

# Optional features:
# tmag5273 = { version = "3", features = ["crc"] }
# tmag5273 = { version = "3", features = ["defmt"] }
# tmag5273 = { version = "3", features = ["libm"] }
```

## Quick Start

```rust,no_run
use tmag5273::{Tmag5273, ConfigBuilder};

// 1. Plug-and-play: scan the I2C bus, auto-detect variant and address
let sensor = Tmag5273::scan(i2c).expect("no TMAG5273 found");

// 2. Build a validated configuration (defaults: continuous, XYZ, temp enabled)
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

If you already know the variant, you can skip the scan:

```rust,no_run
use tmag5273::{Tmag5273, DeviceVariant, ConfigBuilder};

let sensor = Tmag5273::new(i2c, DeviceVariant::B1);
```

## Features

| Feature | Description |
| ------- | ----------- |
| `crc`   | Enables CRC-8 validation on all I2C reads. Burst reads use 4-byte block reads with per-block CRC. |
| `defmt` | Enables `defmt::Format` derives on all public types (requires `defmt` 1.0). |
| `libm`  | Enables software angle computation (`plane_angles`, `magnitude_3d`), axis calibration (`AxisCalibrator`), and related types. Pure Rust, ~30KB. |

All features are off by default.

## Configuration

Use `ConfigBuilder` to construct a validated `Config`:

```rust,no_run
use tmag5273::{
    ConfigBuilder, OperatingMode, MagneticChannel, AngleEnable,
    ConversionAverage, Range, PowerNoiseMode, MagneticTempCoefficient,
};

let config = ConfigBuilder::new()
    .operating_mode(OperatingMode::ContinuousMeasure) // default
    .magnetic_channels_enabled(MagneticChannel::XYZ)  // default
    .temp_channel_enabled(true)                       // default
    .angle_enabled(AngleEnable::None)                 // default
    .conversion_average(ConversionAverage::X4)
    .xy_range(Range::High)                            // default: ±80 mT (v1)
    .z_range(Range::High)
    .power_noise_mode(PowerNoiseMode::LowNoise)        // default: LowActiveCurrent
    .magnetic_temp_coefficient(MagneticTempCoefficient::NdBFe)
    .build()?;
```

`build()` validates cross-field constraints before writing any register:

- `AngleEnable` ≠ `None` requires at least two magnetic channels.
- `OperatingMode::WakeUpAndSleep` requires `SleepTime` ≥ conversion time.
- `AngleEnable::YZ` or `AngleEnable::XZ` requires matching XY and Z ranges.

### Defaults (match SparkFun `begin()`)

| Field                       | Default             |
| --------------------------- | ------------------- |
| `operating_mode`            | `ContinuousMeasure` |
| `magnetic_channels_enabled` | `XYZ`               |
| `temp_channel_enabled`      | `true`              |
| `angle_enabled`             | `None`              |
| `conversion_average`        | `X1` (no averaging) |
| `xy_range` / `z_range`      | `High` (±80 mT v1)  |
| `power_noise_mode`          | `LowActiveCurrent`  |
| `i2c_glitch_filter`         | `true`              |
| `magnetic_temp_coefficient` | `None`              |

### Conversion Time

Use `ConversionAverage::conversion_time()` to query the expected conversion
duration (useful for polling delays or scheduling):

```rust,no_run
use tmag5273::{ConversionAverage, MagneticChannel, MicrosIsr};

let t: MicrosIsr = ConversionAverage::X8.conversion_time(MagneticChannel::XYZ, true);
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

### Angle (Hardware CORDIC Engine)

Requires `AngleEnable` ≠ `None` in the config.

```rust,no_run
use tmag5273::{ConfigBuilder, AngleEnable, MagneticChannel};

let config = ConfigBuilder::new()
    .angle_enabled(AngleEnable::XY)
    .magnetic_channels_enabled(MagneticChannel::XYX) // pseudo-simultaneous
    .build()?;

let mut sensor = sensor.init(&config)?;
let angle = sensor.read_angle()?;
// angle.angle.0 is degrees (0–360, f32)
// angle.magnitude.0 is millitesla (CordicMagnitude)
```

### Software Angle Computation (`libm` feature)

Compute angle and magnitude for all three canonical planes from any
`MagneticReading` — no hardware CORDIC configuration needed:

```rust,no_run
let reading = sensor.read_magnetic()?;
let planes = reading.plane_angles();

// Each plane is Option — None if axes missing or zero-magnitude field
if let Some(xy) = planes.xy {
    // xy.angle.0 — degrees (0–360), matching hardware CORDIC convention
    // xy.magnitude.0 — millitesla
    // xy.plane — PlaneAxis::XY
}

// Get the plane with the strongest signal
if let Some(dominant) = planes.dominant() {
    // dominant.plane tells you which axis pair has the strongest field
}

// 3-axis software magnitude
if let Some(mag) = reading.magnitude_3d() {
    // mag.0 — millitesla, sqrt(x² + y² + z²)
}
```

### Auto-Axis Calibration (`libm` feature)

Automatically detect the optimal hardware angle axis pair by analyzing
magnetic field variance during magnet rotation:

```rust,no_run
use tmag5273::{AxisCalibrator, ConfigBuilder};

// 1. Collect samples during magnet rotation
let mut cal = AxisCalibrator::default();
for _ in 0..50 {
    let reading = sensor.read_magnetic()?;
    cal.update(&reading);
    delay.delay_ms(100);
}

// 2. Get the recommended hardware configuration
if let Some(rec) = cal.recommend() {
    // rec.plane — the detected rotation plane (XY, YZ, or XZ)
    // rec.angle_enable — for ConfigBuilder::angle_enabled()
    // rec.magnetic_channel — for ConfigBuilder::magnetic_channels_enabled()
    // rec.variances — per-axis variance for transparency

    // 3. Apply to hardware
    let config = ConfigBuilder::new()
        .angle_enabled(rec.angle_enable)
        .magnetic_channels_enabled(rec.magnetic_channel)
        .build()?;

    let mut sensor = sensor.init(&config)?;
    let angle = sensor.read_angle()?; // hardware CORDIC, optimal axis pair
}
```

The calibrator uses Welford's online algorithm for numerically stable
variance on f32. Fixed-size, zero heap allocation.

### Rotation Tracking (RPM / IPI)

`RotationTracker<const POLES_COUNT: u8, M: TrackingMode>` is a const-generic
rotation tracker with two algorithm modes selected at compile time:

| Mode | Pole counts | Input | Returns from `update()` |
| --- | --- | --- | --- |
| `Cordic` | **2 only** | `Degrees` from CORDIC engine | `Option<SignedDegrees>` (signed delta) |
| `ZeroCrossing` | 2, 3, or 4 | `MilliTesla` (raw axis sample) | `Option<MicrosIsr>` (IPI on crossing) |

`POLES_COUNT` and `M` are checked at compile time — invalid combinations
(e.g. `RotationTracker::<4, Cordic>`) fail to compile.

**CORDIC mode** — diametrically magnetized 2-pole magnets only (TI SBAA463A §3.2).
Multi-pole magnets produce phantom RPM and undercounting:

```rust,no_run
use tmag5273::{CordicTracker, MicrosIsr};

let mut tracker = CordicTracker::new(); // = RotationTracker::<2, Cordic>::new()

loop {
    let angle = sensor.read_angle()?;
    let elapsed = MicrosIsr(/* time since last update */);
    let _delta = tracker.update(angle.angle, elapsed);

    if let Some(rpm) = tracker.rpm() {
        // rpm.0 — revolutions per minute (f32)
    }
}
```

**Zero-crossing mode** — required for multi-pole ring magnets (Gicar 4-pole,
3-pole). Uses a Schmitt trigger on a single raw axis with hysteresis to reject
noise. Returns the inter-pulse interval (IPI) when a crossing is detected:

```rust,no_run
use tmag5273::{RotationTracker, ZeroCrossing, MilliTesla, MicrosIsr};

// 4-pole Gicar ring magnet, H ≈ 10% of peak-to-peak swing
let mut tracker = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(0.8));

loop {
    let reading = sensor.read_magnetic()?;
    let elapsed = MicrosIsr(/* time since last update */);

    if let Some(x) = reading.x {
        if let Some(ipi) = tracker.update(x, elapsed) {
            // ipi.0 — microseconds between this and the previous pole transition
        }
    }

    if let Some(rpm) = tracker.rpm() {
        // average RPM since construction or last reset()
    }
}
```

Both modes share the same query API: `rpm()`, `cumulative_revolutions()`,
`reset()`, `max_abs_delta()`. Sizes: `Cordic` 24 bytes, `ZeroCrossing` 20 bytes.

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
    .build()?;
let mut sensor = sensor.init(&config)?;

// Trigger a conversion, then poll for completion
sensor.trigger_conversion()?;
let status = sensor.wait_for_conversion()?;

// Now safe to read
let reading = sensor.read_magnetic()?;
```

## Runtime Controls

| Method | Description |
| --- | --- |
| `set_mode(OperatingMode)` | Switch operating mode at runtime |
| `trigger_conversion()` | Trigger one conversion (standby / trigger mode) |
| `wait_for_conversion()` → `ConversionStatus` | Blocking poll with bounded retry |
| `is_conversion_complete()` → `bool` | Non-blocking check |
| `read_status()` → `DeviceStatus` | Read fault-flag register |
| `check_diag()` → `DeviceStatus` | Lazy diagnostic read (skips status if no fault) |
| `is_interrupt_active()` → `bool` | Read INT̅ pin state via `INTB_RB` register readback |
| `set_diagnostics(Diagnostics)` | Set per-sample diagnostic checking policy at runtime |
| `diagnostics()` → `Diagnostics` | Read current diagnostic policy |
| `set_crc_enabled(bool)` | Toggle sensor-side CRC-8 generation |
| `get_crc_enabled()` → `bool` | Read sensor-side CRC setting |
| `set_read_mode(I2cReadMode)` | Change I2C response format |
| `get_read_mode()` → `I2cReadMode` | Read current response format |

## Per-Sample Diagnostics

Opt-in per-sample `CONV_STATUS.DIAG_STATUS` checking on every measurement.
Set at runtime via `set_diagnostics()` — no sensor re-initialization needed:

```rust,no_run
use tmag5273::Diagnostics;

// Halt on fault — returns Error::DiagnosticFailure(DeviceStatus)
sensor.set_diagnostics(Diagnostics::Halt);

// Warn on fault — emits defmt::warn! but reads succeed (requires `defmt` feature)
sensor.set_diagnostics(Diagnostics::Warn);

// Ignore faults — for noisy environments (EMI, motor controllers)
sensor.set_diagnostics(Diagnostics::Ignore);

// Default — no overhead, no extra I2C transactions
sensor.set_diagnostics(Diagnostics::Off);
```

Manual `check_diag()` works independently of this setting.

## Advanced

### Threshold Interrupts

```rust,no_run
use tmag5273::{
    ThresholdConfig, MagneticThresholdDirection, TempThresholdConfig,
    InterruptConfig, InterruptMode, InterruptState, ThresholdHysteresis,
};

sensor.set_thresholds(&ThresholdConfig {
    x: 50, y: 50, z: 50,
    temperature: TempThresholdConfig::DISABLED,
    direction: MagneticThresholdDirection::Above,
    hysteresis: ThresholdHysteresis::LimitCross,
    ..Default::default()
})?;

sensor.set_interrupt(&InterruptConfig {
    on_conversion_complete: false,
    on_threshold_crossing: true,
    mode: InterruptMode::ThroughInt,
    pin_behavior: InterruptState::Pulse10us,
    ..Default::default()
})?;
```

### Gain / Offset Calibration

```rust,no_run
use tmag5273::{CalibrationConfig, MagneticGainChannel};

sensor.set_calibration(&CalibrationConfig {
    gain: 128,
    offset_1: 0,
    offset_2: 0,
    gain_channel: MagneticGainChannel::First,
    ..Default::default()
})?;
```

### Wake-Up-and-Sleep Mode

Implements TI datasheet section 8.2.1.2 with TI-recommended settings:
threshold interrupt on INT pin, 10 µs pulse.

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

| Variant | Cause |
| --- | --- |
| `I2c(E)` | I2C bus error |
| `InvalidManufacturerId(u16)` | Device ID is not `0x5449` (ASCII "TI") |
| `VersionMismatch { expected, got }` | Device version does not match variant |
| `CrcMismatch { expected, computed }` | CRC-8 validation failed (`crc` feature) |
| `ConversionTimeout` | Conversion not ready within poll limit |
| `AngleNotEnabled` | `read_angle()` called without angle config |
| `InvalidRegisterValue { register, value }` | Unknown enum value in register |
| `NonStandardReadMode { mode }` | High-level read called in non-Standard I2C mode |
| `TempDisabled` | `read_temperature()` with temperature channel disabled |
| `DiagnosticFailure(DeviceStatus)` | Sensor diagnostic fault (`Diagnostics::Halt`) |
| `CrcFeatureRequired` | Sensor CRC enabled without `crc` Cargo feature |
| `AddressChangeFailed { old, new }` | Device did not respond at new address |

`ConfigError` (from `ConfigBuilder::build()`):

| Variant | Cause |
| --- | --- |
| `AngleRequiresTwoChannels` | Angle enabled with < 2 magnetic channels |
| `SleepShorterThanConversion` | Wake-sleep mode with sleep < conversion time |
| `IntPinTriggerRequiresStandby` | INT pin trigger in non-standby mode |
| `AngleMixedRanges` | YZ/XZ angle with mismatched XY and Z ranges |

`InitError<I2C, D>` wraps `Error` together with the I2C bus and delay so
callers can recover peripherals on initialization failure.

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

## Cargo Commands

```bash
# Unit tests (no hardware required)
cargo test -p tmag5273                     # 349 tests (default features)
cargo test -p tmag5273 --features libm     # 450 tests (+ angle/calibrator/rotation)
cargo test -p tmag5273 --features crc      # 341 tests (+ CRC read paths)

# Build docs with feature annotations
cargo doc -p tmag5273 --all-features --open

# Verify bare-metal compilation
cargo check -p tmag5273 --target thumbv7em-none-eabihf
```

## Design

| Property | Detail |
| --- | --- |
| `no_std` | Zero allocations — no heap dependency |
| `#![forbid(unsafe_code)]` | No unsafe blocks in this crate |
| Typestate | `Unconfigured` → `Configured` enforced at compile time via `PhantomData` |
| Builder config | Cross-field validation at build time, not at register write |
| `InitError` | Returns the I2C bus on init failure for downstream recovery |
| Newtypes | `MilliTesla`, `Celsius`, `Degrees`, `MicrosIsr`, `CordicMagnitude` prevent unit confusion |
| Burst reads | `read_magnetic()` and `read_all()` use single I2C transactions for data coherence |
| Datasheet parity | Type names, field names, and constants trace to TI SBASAI4 Rev C sections |
| Welford variance | `AxisCalibrator` uses numerically stable online algorithm for f32 (`libm` feature) |

## Minimum Supported Rust Version

Rust **1.94** (edition 2024).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.
