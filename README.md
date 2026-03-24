# tmag5273

Platform-agnostic `no_std` driver for the [TI TMAG5273](https://www.ti.com/product/TMAG5273)
3-axis linear Hall-effect sensor, built on [`embedded-hal`](https://docs.rs/embedded-hal/1.0) 1.0 traits.

## Supported Variants

All eight TMAG5273 variants are supported:

| Variant | I2C Address | Sensitivity |
|---------|-------------|-------------|
| A1 / A2 | 0x35 | v1: ±40/80 mT, v2: ±133/266 mT |
| B1 / B2 | 0x22 | v1: ±40/80 mT, v2: ±133/266 mT |
| C1 / C2 | 0x78 | v1: ±40/80 mT, v2: ±133/266 mT |
| D1 / D2 | 0x44 | v1: ±40/80 mT, v2: ±133/266 mT |

## Usage

```rust,no_run
use tmag5273::{Tmag5273, DeviceVariant, ConfigBuilder};

// Create driver (works with any embedded-hal 1.0 I2C bus)
let sensor = Tmag5273::new(i2c, DeviceVariant::B1);

// Configure with builder (defaults match SparkFun begin())
let config = ConfigBuilder::new().build().unwrap();
let mut sensor = sensor.init(&config).unwrap();

// Read magnetic field
let reading = sensor.read_magnetic().unwrap();
if let Some(x) = reading.x {
    // x.0 is the value in millitesla
}

// Read temperature
let temp = sensor.read_temperature().unwrap();
// temp.0 is degrees Celsius

// Read angle (requires 2-axis channel config)
let angle = sensor.read_angle().unwrap();
// angle.angle.0 is degrees, angle.magnitude is 0-255
```

## Features

| Feature | Description |
|---------|-------------|
| `crc` | Enables CRC-8 validation on I2C reads (disables burst reads) |
| `defmt` | Enables `defmt::Format` derives on all public types |

## Capabilities

- Magnetic field reading (X, Y, Z axes — any combination)
- Temperature measurement
- Angle calculation (CORDIC engine, any two-axis pair)
- Operating mode control (Standby, Sleep, Continuous, Wake-up-and-Sleep)
- Threshold and interrupt configuration
- Gain/offset calibration
- Runtime I2C address change with verification
- CRC-8 read validation (feature-gated)

## Design

- **`no_std`** + `#![forbid(unsafe_code)]` — zero allocations, no heap
- **Typestate** — `Unconfigured` vs `Configured` enforced at compile time
- **Builder config** — validated at build time, not at register write
- **`InitError`** — returns the I2C bus on init failure for recovery
- **Newtypes** — `MilliTesla`, `Celsius`, `Degrees` prevent unit confusion

## Minimum Supported Rust Version

Rust 1.93 (edition 2024).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.
