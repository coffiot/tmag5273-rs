# TMAG5273 Crate — Public API Reference

> `no_std` · `#![forbid(unsafe_code)]` · MSRV 1.94 · `embedded-hal` 1.0

**~175 public items** across 7 modules. Three layers: **L1** registers, **L2** core driver, **L3** feature-gated computation.

---

## 1. Lifecycle — Typestate Machine

```text
                  ┌─────────────┐
          scan()  │             │  detect()
        ────────▶ │ Unconfigured │ ◀────────
          new()   │             │
                  └──────┬──────┘
                         │ init(&Config)
                         ▼
                  ┌─────────────┐
                  │  Configured  │ ─── measurement + control methods
                  └──────┬──────┘
                         │ into_parts() / into_inner()
                         ▼
                    (I2C, Delay)    ─── bus recovery
```

### Constructors (`Unconfigured`)

| Method | Signature | I2C? | Purpose |
|--------|-----------|------|---------|
| `scan` | `fn(I2C) → Result<Self, (Error, I2C)>` | Yes | Try all 4 known addresses, auto-detect variant |
| `detect` | `fn(I2C, u8) → Result<Self, (Error, I2C)>` | Yes | Auto-detect variant at known address |
| `new` | `fn(I2C, DeviceVariant) → Self` | No | Explicit variant, no bus activity |
| `new_with_delay` | `fn(I2C, DeviceVariant, D) → Self` | No | With real `DelayNs` backend |
| `new_with_address` | `fn(I2C, DeviceVariant, u8) → Self` | No | Override default address |

### Initialization

| Method | Signature | Notes |
|--------|-----------|-------|
| `init` | `fn(self, &Config) → Result<Tmag5273<Configured>, InitError>` | Consumes `Unconfigured`, validates + writes registers |

### Teardown

| Method | Returns | Use case |
|--------|---------|----------|
| `into_parts` | `(I2C, D)` | Recover bus + delay for reinit |
| `into_inner` | `I2C` | Recover bus only |

---

## 2. Configuration

### ConfigBuilder (const builder pattern)

```rust
let config = ConfigBuilder::new()
    .operating_mode(OperatingMode::Standby)
    .conversion_average(ConversionAverage::X1)
    .magnetic_channels_enabled(MagneticChannel::XYZ)
    .angle_enabled(AngleEnable::XY)
    .build()?;  // → Result<Config, ConfigError>
```

| Builder method | Type | Default |
|----------------|------|---------|
| `operating_mode` | `OperatingMode` | `ContinuousMeasure` |
| `magnetic_channels_enabled` | `MagneticChannel` | `XYZ` |
| `temp_channel_enabled` | `bool` | `true` |
| `angle_enabled` | `AngleEnable` | `None` |
| `conversion_average` | `ConversionAverage` | `X32` |
| `xy_range` / `z_range` | `Range` | `Low` (±40 mT A/B, ±80 mT C/D) |
| `power_noise_mode` | `PowerNoiseMode` | `LowActiveCurrent` |
| `sleep_time` | `SleepTime` | `Ms1` |
| `trigger_mode` | `TriggerMode` | `I2cCommand` |
| `i2c_glitch_filter` | `bool` | `true` |
| `magnetic_temp_coefficient` | `MagneticTempCoefficient` | `None` |

**Cross-field validation** (`build()` errors):

| `ConfigError` | Condition |
|----------------|-----------|
| `AngleRequiresTwoChannels` | `AngleEnable` ≠ `None` but less than 2 axes enabled |
| `SleepShorterThanConversion` | `WakeUpAndSleep` mode with sleep < conversion time |
| `IntPinTriggerRequiresStandby` | INT trigger mode but not in Standby |
| `AngleMixedRanges` | YZ/XZ angle with mismatched XY/Z ranges |

### Runtime Configuration (Configured state)

**Write:**

| Method | Parameter | Notes |
|--------|-----------|-------|
| `set_mode` | `OperatingMode` | Change mode post-init |
| `set_thresholds` | `&ThresholdConfig` | X/Y/Z codes, hysteresis, direction |
| `set_interrupt` | `&InterruptConfig` | INT routing, latch vs pulse |
| `set_calibration` | `&CalibrationConfig` | Gain + offset registers |
| `set_crc_enabled` | `bool` | Toggle CRC on reads |
| `set_read_mode` | `I2cReadMode` | Burst read format |
| `set_diagnostics` | `Diagnostics` | Off / Halt / Warn / Ignore |
| `change_address` | `u8` | Persistent I2C address change |
| `sleep` | — | Enter low-power sleep |
| `configure_wake_and_sleep` | — | Convenience for WakeUpAndSleep setup |

**Read (every `set_*` has a matching `get_*`):**

`get_operating_mode`, `get_conversion_average`, `get_magnetic_channels_enabled`, `get_angle_enabled`, `get_xy_range`, `get_z_range`, `get_power_noise_mode`, `get_sleep_time`, `get_trigger_mode`, `get_i2c_glitch_filter`, `get_magnetic_temp_coefficient`, `get_crc_enabled`, `get_read_mode`, `get_threshold_hysteresis`, `get_threshold_crossing_count`, `get_threshold_direction`, `get_x_threshold`, `get_y_threshold`, `get_z_threshold`, `get_gain_channel`, `get_magnetic_gain`, `get_magnetic_offset_1`, `get_magnetic_offset_2`, `get_temp_channel_enabled`, `get_temp_threshold`, `get_interrupt`, `get_interrupt_mode`, `get_interrupt_on_conversion`, `get_interrupt_on_threshold`, `get_interrupt_pin_behavior`, `get_interrupt_mask`

---

## 3. Measurement (Configured state)

### Core reads

| Method | Returns | Channels | Notes |
|--------|---------|----------|-------|
| `read_magnetic` | `MagneticReading` | X/Y/Z as `Option<MilliTesla>` | Based on enabled channels |
| `read_temperature` | `Celsius` | — | Error if temp disabled |
| `read_angle` | `AngleReading` | CORDIC angle + magnitude | Error if angle not enabled |
| `read_all` | `SensorReading` | Atomic temp + magnetic | Single burst read |

### Conversion control

| Method | Returns | Use case |
|--------|---------|----------|
| `trigger_conversion` | `()` | Start new conversion (Standby mode) |
| `wait_for_conversion` | `ConversionStatus` | Polls until ready |
| `is_conversion_complete` | `bool` | Non-blocking check |
| `conversion_time` | `MicrosIsr` | Expected conversion duration |

### Status & diagnostics

| Method | Returns |
|--------|---------|
| `read_status` | `DeviceStatus` (VCC, OTP, INT, OSC errors) |
| `check_diag` | `DeviceStatus` (with diagnostic policy applied) |
| `is_interrupt_active` | `bool` |
| `read_conversion_status` | `ConversionStatus` (set_count, POR, diag, ready) |

---

## 4. Physical Types (newtypes)

| Type | Inner | Ops | Purpose |
|------|-------|-----|---------|
| `MilliTesla(f32)` | f32 | `+`, `−`, `Neg`, `PartialOrd`, `abs()`, `From→f32` | Magnetic flux density |
| `CordicMagnitude(f32)` | f32 | `PartialOrd`, `From→f32` | 2-axis CORDIC magnitude |
| `Celsius(f32)` | f32 | `PartialOrd`, `From→f32` | Temperature |
| `Degrees(f32)` | f32 | `MIN`=0°, `MAX`=360°, `is_valid()`, `Sub→SignedDegrees`, `From→f32` | Absolute angle |
| `SignedDegrees(f32)` | f32 | `MIN`=−360°, `MAX`=360°, `abs()`, `From→f32` | Angular delta |
| `Rpm(f32)` | f32 | `PartialOrd`, `From→f32` | Rotational speed in rev/min |
| `MicrosIsr(u32)` | u32 | `Display`, `PartialOrd`, `to_millis()→f32`, `to_seconds()→f32`, `From→u32` | Duration in µs |
| `MicrosRange { min, max }` | MicrosIsr | — | Duration bounds |

---

## 5. Enums (sensor configuration)

### Operating modes

| Enum | Variants |
|------|----------|
| `OperatingMode` | `Standby`, `Sleep`, `ContinuousMeasure`, `WakeUpAndSleep` |
| `ConversionAverage` | `X1`, `X2`, `X4`, `X8`, `X16`, `X32` — methods: `samples_per_channel()`, `conversion_time()` |
| `SleepTime` | `Ms1`…`Ms20000` — method: `as_micros_isr()` |
| `PowerNoiseMode` | `LowActiveCurrent`, `LowNoise` |
| `TriggerMode` | `I2cCommand`, `IntPin` |

### Channel selection

| Enum | Variants |
|------|----------|
| `MagneticChannel` | `Off`, `X`, `Y`, `XY`, `Z`, `ZX`, `YZ`, `XYZ`, `XYX`, `YXY`, `YZY`, `XZX` |
| `AngleEnable` | `None`, `XY`, `YZ`, `XZ` |
| `Range` | `Low` (±40/±80 mT), `High` (±80/±266 mT) |
| `Axis` | `X`, `Y`, `Z` — software convenience type |

### Interrupts & thresholds

| Enum | Variants |
|------|----------|
| `InterruptMode` | `None`, `ThroughInt`, `ThroughIntExceptI2cBusy`, `ThroughScl`, `ThroughSclExceptI2cBusy` |
| `InterruptState` | `Latched`, `Pulse10us` |
| `ThresholdCrossingCount` | `One`, `Four` |
| `MagneticThresholdDirection` | `Above`, `Below` |
| `ThresholdHysteresis` | `LimitCross`, `SymmetricBand` |

### Device identity

| Enum | Key methods |
|------|-------------|
| `DeviceVariant` | `A1`–`D2`, `KNOWN_ADDRESSES`, `from_address_version()`, `default_address()`, `version()`, `range_xy()`, `range_z()` |
| `Diagnostics` | `Off`, `Halt`, `Warn`, `Ignore` |
| `PoleCount` | `Two`, `Three`, `Four` — method: `count()` |

---

## 6. Rotation Tracking (`src/rotation.rs`)

```text
RotationTracker<const POLES_COUNT: u8, M: TrackingMode>
├── M = Cordic        (POLES_COUNT must be 2)    — CORDIC angle input
└── M = ZeroCrossing  (POLES_COUNT ∈ {2,3,4})    — Schmitt trigger on raw mT
```

### CORDIC mode (`RotationTracker<2, Cordic>` / `CordicTracker`)

| Method | Signature | Notes |
|--------|-----------|-------|
| `new` | `fn() → Self` | 2-pole only (compile-time enforced) |
| `update` | `fn(&mut, Degrees, MicrosIsr) → Option<SignedDegrees>` | Feed CORDIC angle + elapsed |
| `accumulated_electrical_angle` | `fn(&self) → f32` | Total accumulated |
| `angular_velocity_dps` | `fn(&self) → Option<f32>` | Instantaneous deg/s |

### Zero-crossing mode (`RotationTracker<N, ZeroCrossing>`)

| Method | Signature | Notes |
|--------|-----------|-------|
| `new` | `fn(MilliTesla) → Self` | Hysteresis for Schmitt trigger |
| `update` | `fn(&mut, MilliTesla, MicrosIsr)` | Feed axis value + elapsed |
| `crossings` | `fn(&self) → u32` | Raw zero-crossing count |

### Shared methods (both modes)

| Method | Returns | Notes |
|--------|---------|-------|
| `poles` | `PoleCount` | Strongly-typed pole count |
| `rpm` | `Option<Rpm>` | Mechanical RPM (None if insufficient data) |
| `cumulative_revolutions` | `f32` | Total mechanical revolutions |
| `max_abs_delta` | `Option<SignedDegrees>` | Largest single-step delta (Nyquist diagnostic) |
| `reset` | `()` | Clear accumulated state |

---

## 7. Angle & Calibration (`src/angle.rs`) — requires `libm`

### MagneticReading extensions

| Method | Signature | Notes |
|--------|-----------|-------|
| `magnitude_3d` | `fn(&self) → Option<MilliTesla>` | `√(x²+y²+z²)`, requires all 3 axes |
| `plane_angles` | `fn(&self) → PlaneAngles` | Software `atan2` on XY/YZ/XZ planes |

### PlaneAngles / PlaneAngle

```rust
PlaneAngles { xy: Option<PlaneAngle>, yz: Option<PlaneAngle>, xz: Option<PlaneAngle> }
PlaneAngle  { angle: Degrees, magnitude: MilliTesla, plane: PlaneAxis }
```

- `PlaneAngles::dominant() → Option<PlaneAngle>` — plane with largest magnitude
- Indexable: `plane_angles[PlaneAxis::XY]`
- `PlaneAxis` converts to `MagneticChannel` and `AngleEnable` via `From`

### AxisCalibrator — auto-detect optimal axis/plane

```rust
let mut cal = AxisCalibrator::default();
for _ in 0..500 { cal.update(&sensor.read_magnetic()?); }

// For rotating magnets (variance-based):
let rec = cal.recommend();
// For static fields (mean-based):
let rec = cal.recommend_static();
```

| Field on `AxisRecommendation` | Type | Purpose |
|-------------------------------|------|---------|
| `plane` | `PlaneAxis` | Best 2-axis plane |
| `angle_enable` | `AngleEnable` | Matching angle config |
| `channel` | `MagneticChannel` | Matching channel config (may be pseudo-simultaneous XYX/YXY) |
| `variances` | `[Option<f32>; 3]` | Per-axis variance [X, Y, Z] |
| `abs_means` | `[Option<f32>; 3]` | Per-axis absolute mean [X, Y, Z] |

### Welford — online mean/variance accumulator

| Method | Returns |
|--------|---------|
| `update(f32)` | — |
| `count()` | `u32` |
| `mean()` | `Option<f32>` (≥2 samples) |
| `variance()` / `sample_variance()` | `Option<f32>` |
| `stddev()` / `sample_stddev()` | `Option<f32>` |
| `abs_mean()` | `Option<f32>` |

---

## 8. Errors

### `Error<E>` (generic over I2C error)

| Variant | Cause |
|---------|-------|
| `I2c(E)` | Bus error (transparent via `From<E>`) |
| `InvalidManufacturerId(u16)` | Not a TI device |
| `VersionMismatch { expected, got }` | Wrong device variant |
| `CrcMismatch { expected, computed }` | CRC-8 validation failed (`crc` feature) |
| `ConversionTimeout` | `wait_for_conversion` timed out |
| `AngleNotEnabled` | `read_angle()` called without angle config |
| `TempDisabled` | `read_temperature()` called without temp channel |
| `CrcFeatureRequired` | CRC enable attempted without `crc` feature |
| `AddressChangeFailed { old, new }` | I2C address change verification failed |
| `InvalidRegisterValue { register, value }` | Unexpected register decode |
| `NonStandardReadMode { mode }` | Unrecognized read mode |
| `DiagnosticFailure(DeviceStatus)` | Sensor self-test failed |

### `ConfigError` (non-generic)

Builder validation failures — see §2 table above.

### `InitError<I2C, D>` (`#[non_exhaustive]`)

| Field | Type | Purpose |
|-------|------|---------|
| `error` | `Error<I2C::Error>` | What failed |
| `i2c` | `I2C` | Recoverable bus |
| `delay` | `D` | Recoverable delay |

---

## 9. Feature Gates

| Feature | Default | Enables |
|---------|---------|---------|
| `libm` | off | `magnitude_3d()`, `plane_angles()`, `AxisCalibrator`, `Welford`, `PlaneAxis`, `PlaneAngle`, `PlaneAngles` |
| `crc` | off | CRC-8 validation on I2C reads (incompatible with burst reads) |
| `defmt` | off | `defmt::Format` derives on all public types |

---

## 10. Constants

| Constant | Type | Value | Purpose |
|----------|------|-------|---------|
| `WAKE_DELAY` | `MicrosRange` | 80–200 µs | Post-sleep wake-up delay |
| `WAKE_RETRY_DELAY` | `MicrosIsr` | 5000 µs | Retry delay after NACK on wake |
| `DeviceVariant::KNOWN_ADDRESSES` | `[u8; 4]` | `[0x22, 0x46, 0x6A, 0x35]` | All possible I2C addresses |
| `TempThresholdConfig::DISABLED` | `Self` | `0x00` | Disabled temperature threshold |
| `AxisCalibrator::MIN_CALIBRATION_SAMPLES` | `u32` | `8` | Minimum for `recommend()` |
