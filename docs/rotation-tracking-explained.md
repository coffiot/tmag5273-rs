# Rotation Tracking: Crossings, Revolutions, and IPI

A practical guide to how the TMAG5273 driver measures rotation from a
multi-pole magnet assembly — explained for people who don't think in
Tesla units every day.

## Your Hardware Setup

A small tube (~1 cm diameter) with 4 magnets glued at 90° intervals,
alternating polarity (N-S-N-S). This mimics a Gicar flowmeter impeller.
The TMAG5273 Hall-effect sensor sits beside the tube.

```
        Looking at the tube end-on:

              N
              |
         S ---+--- S
              |
              N

        4 magnets, 90° apart, alternating polarity
```

## What the Sensor Sees

As the tube spins, each magnet passes the sensor in turn. The sensor
measures magnetic field strength in milliTesla (mT) — positive when a
North pole faces it, negative when South faces it.

One full revolution produces a wave that crosses zero 4 times:

```
Field (mT)
  +8 |    N         N         N         N
     |   / \       / \       / \       / \
   0 |--/---\-----/---\-----/---\-----/---\--
     | /     \   /     \   /     \   /     \
  -8 |/   S   \ /   S   \ /   S   \ /   S
     +---------------------------------------- time
     |<-- one revolution -->|
     |  4 zero-crossings    |
```

## Crossings

A **crossing** is when the magnetic field flips polarity — the signal
passes through zero. The driver uses a Schmitt trigger with a dead band
(hysteresis ±H) to avoid counting noise as crossings:

```
Field (mT)
  +8 |    N
     |   / \
  +H |--/---\---------  must cross +H to register "going up"
   0 | /     \
  -H |/-------\-------  must cross -H to register "going down"
  -8 |     S
        ^         ^
     crossing  crossing
       #1        #2
```

Values inside the dead band (-H to +H) are ignored. This prevents
sensor noise from being mistaken for real magnet transitions.

The driver wraps this count in a `Crossings(u32)` newtype — use `.0`
to access the raw count.

**Your 4-magnet tube produces 4 crossings per revolution:**

```
One full rotation (N-S-N-S arrangement):

  N passes sensor  →  field rises to +peak
                   →  crossing #1 (field falls through zero, N→S transition)
  S passes sensor  →  field drops to -peak
                   →  crossing #2 (field rises through zero, S→N transition)
  N passes sensor  →  field rises to +peak
                   →  crossing #3 (field falls through zero, N→S transition)
  S passes sensor  →  field drops to -peak
                   →  crossing #4 (field rises through zero, S→N transition)
```

Each crossing happens at the **boundary between adjacent magnets**,
not when a magnet is directly facing the sensor. The zero-crossing
is where the field flips sign — halfway between a North and a South.

## Revolutions

Simple division — crossings divided by pole count:

```
revolutions = crossings / poles
```

| Crossings | Poles | Revolutions |
|-----------|-------|-------------|
| 4         | 4     | 1.0         |
| 49        | 4     | 12.25       |
| 100       | 4     | 25.0        |

## RPM (Revolutions Per Minute)

How fast the tube is spinning:

```
RPM = revolutions / elapsed_seconds * 60

Example:
  49 crossings in 0.297 seconds
  revolutions = 49 / 4 = 12.25
  RPM = 12.25 / 0.297 * 60 = 2475 RPM
```

## IPI (Inter-Pulse Interval)

The time between consecutive crossings, measured in microseconds (µs).
This is the most granular timing data the driver provides.

```
Field
  +8 |     N              N              N
     |    / \            / \            / \
   0 |---/---\----------/---\----------/---\---
     |  /     \        /     \        /
  -8 | /   S   \      /   S   \      /
     |  ^        ^     ^        ^
     |  cross    cross  cross    cross
     |  #1       #2     #3       #4
     |
     |  |--IPI 1--|--IPI 2--|--IPI 3--|
     |   6000 µs   5800 µs   6200 µs
```

### Key rules

- **N crossings produce N-1 IPIs.** The first crossing has nothing
  before it, so there is no interval to measure. This is the "N-1
  convention" used consistently by both tmag5273 and coffiot-core.

- **IPI includes the triggering sample's elapsed time.** The crossing
  happened during that sample period, so its time is part of the interval.

- **Shorter IPI = faster rotation.** Longer IPI = slower.

### IPI to RPM conversion

```
One crossing every IPI microseconds.
One revolution = 4 crossings (for 4 poles).
One revolution takes 4 * IPI microseconds.

RPM = 60,000,000 / (IPI_us * poles)

Example:
  IPI = 6000 µs
  RPM = 60,000,000 / (6000 * 4) = 2500 RPM
```

### Quick reference table (4-pole magnet)

| IPI (µs) | Crossings/sec | RPM   | Flow speed    |
|----------|--------------|-------|---------------|
| 3,333    | 300 Hz       | 4,500 | Very fast     |
| 4,286    | 233 Hz       | 3,500 | Fast          |
| 6,000    | 167 Hz       | 2,500 | Medium        |
| 10,000   | 100 Hz       | 1,500 | Slow          |
| 30,000   | 33 Hz        | 500   | Very slow     |

## Why IPI Matters for Flow Measurement

In a real coffee machine, water pushes the Gicar impeller. Faster water
= faster spin = shorter IPI. The coffiot-core `FlowDetector` collects
every IPI from a single espresso shot into a buffer:

```
Shot timeline:
  pump starts → water flows → impeller spins → IPIs recorded

  IPI buffer: [8000, 6500, 5200, 4800, 4600, 4500, 4500, ...]
               slow   ↓    ↓    ↓     ↓    steady flow
              (pump   (flow stabilizes)
              ramp-up)
```

This per-pulse resolution lets the system detect:

- **Flow ramp-up** — IPIs decreasing as pump reaches pressure
- **Steady flow** — IPIs roughly constant
- **Channeling** — sudden IPI changes mid-shot (uneven extraction)
- **Pump stall** — IPIs suddenly increasing or stopping

Aggregate RPM would blur these details. Individual IPIs preserve them.

## What Else Can Be Measured

The raw data from the sensor is: **field strength (mT) + elapsed time
(µs) per sample**. From crossings and IPIs, several derived metrics
are possible:

### Currently implemented

| Metric | Source | Unit | What it tells you |
|--------|--------|------|-------------------|
| **Crossings** | Schmitt trigger count | `Crossings(u32)` | Total pole transitions since start/reset |
| **Revolutions** | crossings / poles | f32 | Total mechanical rotations |
| **RPM** | revolutions / elapsed * 60 | f32 | Average rotational speed |
| **IPI** | elapsed between crossings | µs (u32) | Per-pulse instantaneous timing |

### Derivable from IPIs (not yet implemented)

| Metric | How to compute | What it tells you |
|--------|---------------|-------------------|
| **Instantaneous RPM** | `60e6 / (IPI * poles)` per crossing | Speed at each pulse — not just the average |
| **Flow rate (mL/s)** | `RPM * calibration_factor` | Volume of water passing through (needs calibration with known volume) |
| **Total volume (mL)** | sum of per-IPI flow increments | How much water flowed during the entire shot |
| **Acceleration (RPM/s)** | `(RPM_n - RPM_n-1) / IPI_n` | How fast the impeller is speeding up or slowing down |
| **Jitter / variance** | standard deviation of IPIs | Smoothness of rotation — high jitter may indicate air bubbles or channeling |
| **Silence duration** | gap with no crossings | Time since last pulse — detects flow stop, dripping, or shot end |
| **Pulse frequency (Hz)** | `1e6 / IPI` | Electrical frequency of crossings — useful for Nyquist checks |

### Derivable from raw field strength (not yet implemented)

| Metric | How to compute | What it tells you |
|--------|---------------|-------------------|
| **Peak amplitude** | max |field| per half-cycle | Magnet distance from sensor — changes if impeller shifts |
| **Waveform symmetry** | compare positive vs negative half-cycles | Magnet alignment quality — asymmetry means uneven magnet spacing |
| **Signal-to-noise ratio** | peak amplitude / noise floor | Sensor reliability — low SNR means crossings may be unreliable |
| **Temperature** | TMAG5273 built-in temp sensor | Ambient temperature near the sensor (available separately from field) |

### The measurement hierarchy

```
Raw sensor data (mT + µs per sample)
  │
  ├── Crossings (count)
  │     │
  │     ├── Revolutions = crossings / poles
  │     │     │
  │     │     └── RPM = revolutions / time * 60  (average)
  │     │
  │     └── IPI (µs between crossings)
  │           │
  │           ├── Instantaneous RPM = 60e6 / (IPI * poles)
  │           ├── Flow rate = f(instantaneous RPM, calibration)
  │           ├── Total volume = sum of flow increments
  │           ├── Acceleration = delta RPM / delta time
  │           ├── Jitter = stdev(IPIs)
  │           └── Silence = gap since last crossing
  │
  └── Field amplitude (mT)
        │
        ├── Peak tracking (magnet distance)
        ├── Waveform symmetry (magnet alignment)
        └── SNR (signal quality)
```

The current driver gives you everything above the dashed line
(crossings, revolutions, RPM, IPI). The downstream consumer
(coffiot-core / prototype firmware) computes the rest.

## How It Works in Code

The driver's `ZeroCrossing::update()` method is called once per sensor
reading. It returns the IPI when a crossing is detected:

```
Every ~1 ms (sensor sample loop):
  read magnetic field from TMAG5273
  call tracker.update(field_mT, elapsed_us)
    → None        (no crossing this sample)
    → None        (first crossing — no prior to measure from)
    → Some(6000)  (second crossing — 6000 µs since the first)
    → None        (no crossing)
    → None        (no crossing)
    → Some(5800)  (third crossing — 5800 µs since the second)
    ...
```

The driver emits one IPI at a time. The caller (prototype firmware or
coffiot-core) collects them into a buffer for the complete shot.

## Glossary

| Term | Meaning |
|------|---------|
| **Crossing** | Magnetic field flips polarity past the Schmitt trigger threshold |
| **Pole** | One magnet in the assembly (your tube has 4 poles) |
| **Revolution** | One full rotation of the tube = `poles` crossings |
| **RPM** | Revolutions per minute — rotational speed |
| **IPI** | Inter-Pulse Interval — microseconds between consecutive crossings |
| **Hysteresis (H)** | Dead band around zero that rejects noise (typically 10% of signal swing) |
| **MicrosIsr** | The `u32` microsecond type used by both tmag5273 and coffiot-core |
| **Schmitt trigger** | Logic that requires the signal to cross +H and -H (not just zero) to count |
| **N-1 convention** | N crossings produce N-1 intervals (first crossing has no predecessor) |

## Further Reading

- `src/rotation.rs` — implementation with full doc comments
- `docs/solutions/best-practices/ipi-cross-repo-integration-2026-04-07.md` — type mapping between tmag5273 and coffiot-core
- `docs/solutions/hardware-validation/cordic-invalid-for-multi-pole-magnets-2026-04-02.md` — why CORDIC doesn't work with 4-pole magnets
