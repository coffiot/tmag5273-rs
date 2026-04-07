//! Rotation tracking for magnet assemblies with two independent algorithms.
//!
//! Provides [`RotationTracker<POLES_COUNT, M>`] — a const-generic, stateful
//! accumulator that counts revolutions and estimates angular velocity.
//! The const parameter `POLES_COUNT` is the number of magnet poles
//! (2, 3, or 4 for zero-crossing; exactly 2 for CORDIC).
//! The type parameter `M` selects the tracking algorithm:
//!
//! | Mode | Input | When to use |
//! |------|-------|------------|
//! | [`Cordic`] | [`Degrees`] | 2-pole diametrically magnetized magnets only (TI SBAA463A §3.2) |
//! | [`ZeroCrossing`] | [`MilliTesla`] | Any supported pole count; real Gicar flowmeters and multi-pole ring magnets |
//!
//! # Choosing a Mode
//!
//! **Use [`Cordic`]** if and only if your magnet assembly is a single
//! diametrically magnetized cylinder (2 poles, N on one hemisphere, S on
//! the other). The TMAG5273 CORDIC engine produces a clean 0–360° angle
//! signal for this geometry. `RotationTracker::<2, Cordic>::new()` enforces
//! this at compile time — it is only constructible for `POLES_COUNT = 2`.
//!
//! **Use [`ZeroCrossing`]** for all other cases, including 4-pole Gicar
//! ring magnets (and similar flowmeter impellers), 2-pole through 4-pole assemblies,
//! or any situation where CORDIC outputs are unreliable. Zero-crossing
//! with Schmitt trigger hysteresis works for any pole count and is the
//! production-proven method for Hall-latch based flow metering. See:
//! `docs/solutions/hardware-validation/cordic-invalid-for-multi-pole-magnets-2026-04-02.md`
//!
//! # Quick Start
//!
//! ```
//! use tmag5273::{RotationTracker, Cordic, ZeroCrossing, Degrees, MicrosIsr, MilliTesla, Rpm};
//!
//! // --- 2-pole diametric magnet: CORDIC mode ---
//! let mut cordic = RotationTracker::<2, Cordic>::new();
//! let _ = cordic.update(Degrees(45.0), MicrosIsr(2000));
//! let _ = cordic.update(Degrees(135.0), MicrosIsr(2000));
//! if let Some(rpm) = cordic.rpm() {
//!     let _ = rpm; // physical RPM
//! }
//!
//! // --- 4-pole ring magnet: zero-crossing mode ---
//! // H = 0.8 mT (10% of typical 8 mT swing)
//! let mut zc = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(0.8));
//! // update() returns Option<MicrosIsr> — the inter-pulse interval (IPI)
//! // when a crossing is detected. First crossing returns None.
//! let _no_crossing = zc.update(MilliTesla(5.2), MicrosIsr(300));
//! let _first_cross = zc.update(MilliTesla(-4.1), MicrosIsr(300)); // None (first)
//! if let Some(ipi) = zc.update(MilliTesla(5.2), MicrosIsr(500)) {
//!     let _raw_us: u32 = ipi.0; // unwrap for coffiot-core's ipis_us buffer
//! }
//! if let Some(rpm) = zc.rpm() {
//!     let _ = rpm;
//! }
//! ```
//!
//! # Pole Count and Revolutions
//!
//! Both modes report **mechanical (physical)** revolutions and RPM:
//!
//! - `Cordic`: pole_pairs = `POLES_COUNT / 2` (integer). For `POLES_COUNT = 2` the
//!   electrical and mechanical quantities are identical.
//! - `ZeroCrossing`: one revolution = `POLES_COUNT` zero-crossings. A 4-pole
//!   ring magnet produces 4 polarity flips per mechanical revolution.
//!
//! # Precision (CORDIC mode)
//!
//! Cumulative rotation is stored as a split accumulator: `i32` revolution
//! counter + `f32` fractional_angle degrees in \[0, 360). This avoids the
//! catastrophic precision loss of a bare `f32` accumulator (which loses
//! 1° resolution beyond ~46k revolutions). The split representation
//! supports ±2 billion revolutions with sub-degree precision at all times.

use core::mem::size_of;

use crate::types::{Crossings, Degrees, MicrosIsr, MilliTesla, PoleCount, Rpm, SignedDegrees};

// ---------------------------------------------------------------------------
// Sealed-trait infrastructure
// ---------------------------------------------------------------------------

mod private {
    use crate::{PoleCount, Rpm, SignedDegrees};

    /// Internal sealed supertrait carrying the common algorithm interface.
    /// Not accessible outside this module — external code cannot implement
    /// `TrackingMode` for custom types.
    pub trait Sealed {
        /// Instantaneous or average physical RPM. `None` if not enough data.
        fn rpm(&self, poles: PoleCount) -> Option<Rpm>;

        /// Total mechanical revolutions since construction or last reset.
        fn cumulative_revolutions(&self, poles: PoleCount) -> f32;

        /// Clear all accumulated state. The next `update()` starts fresh.
        fn reset(&mut self);

        /// Largest absolute angle delta observed (CORDIC Nyquist diagnostic).
        ///
        /// Returns `None` for [`ZeroCrossing`](super::ZeroCrossing) (concept
        /// does not apply) and before any delta has been computed.
        fn max_abs_delta(&self) -> Option<SignedDegrees>;
    }
}

// ---------------------------------------------------------------------------
// Public mode markers
// ---------------------------------------------------------------------------

/// CORDIC-based angle tracking mode.
///
/// Accepts [`Degrees`] input from the TMAG5273 CORDIC engine and uses
/// a shortest-path delta accumulator. **Only valid for 2-pole diametrically
/// magnetized magnets** (TI SBAA463A §3.2). Using CORDIC with multi-pole
/// ring magnets produces phantom RPM and severe undercounting.
///
/// `RotationTracker::<2, Cordic>::new()` is the only constructor —
/// attempting `RotationTracker::<4, Cordic>::new()` is a **compile error**:
///
/// ```compile_fail
/// use tmag5273::{RotationTracker, Cordic};
/// let _ = RotationTracker::<4, Cordic>::new(); // error[E0599]: no function `new` found
/// ```
///
/// # Size
///
/// `size_of::<RotationTracker<2, Cordic>>() == size_of::<Cordic>() == 24 bytes`
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Cordic {
    /// Previous angle sample in degrees. `None` before the first `update()`.
    previous_angle: Option<Degrees>,

    /// Complete electrical revolutions (positive = forward, negative = reverse).
    revolutions: i32,

    /// Fractional angle within the current electrical revolution.
    /// Always in [Degrees::MIN, Degrees::MAX) after normalization.
    fractional_angle: Degrees,

    /// Most recent angular velocity in electrical degrees/second.
    ///
    /// `f32::NAN` before the second sample or when elapsed = 0.
    last_velocity_dps: f32,

    /// Largest absolute angle delta observed across all `update()` calls.
    ///
    /// Stored as `f32` with `NaN` = not yet computed (same pattern as
    /// `last_velocity_dps`). Exposed as `Option<SignedDegrees>` at the
    /// public boundary. Values near 180° indicate under-sampling.
    max_abs_delta: f32,
}

/// Zero-crossing Schmitt trigger tracking mode.
///
/// Accepts raw magnetic field values and counts pole transitions using
/// hysteresis to reject noise. Works for **any supported pole count**
/// (2, 3, or 4).
/// This is the production-proven method for Gicar flowmeters and other
/// multi-pole Hall-latch applications.
///
/// # Schmitt Trigger
///
/// A crossing is counted only when the signal transitions from below `-H`
/// to above `+H` (or vice versa). Noise within the `±H` dead band is
/// ignored. Set `H` to approximately 10% of the signal's peak-to-peak swing.
///
/// # Size
///
/// `size_of::<RotationTracker<4, ZeroCrossing>>() == size_of::<ZeroCrossing>() == 20 bytes`
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ZeroCrossing {
    /// Schmitt trigger threshold.
    ///
    /// A crossing is counted when the signal moves from below `-hysteresis`
    /// to above `+hysteresis` (or the reverse). Typical: 10% of signal swing.
    hysteresis: MilliTesla,

    /// Total zero-crossings counted since construction or last reset.
    crossings: Crossings,

    /// Total elapsed time accumulated via `update()` calls.
    ///
    /// Used for average RPM computation. Saturates at `MicrosIsr(u32::MAX)`
    /// (~71 minutes) — reset the tracker for long-running sessions.
    elapsed: MicrosIsr,

    /// Elapsed time accumulated since the previous zero-crossing.
    ///
    /// When a crossing is detected, `update()` returns `Some(ipi)` with
    /// this value (the inter-pulse interval) and resets it to zero.
    /// The first crossing returns `None` (N pulses produce N-1 intervals).
    /// Saturating addition — caps at `MicrosIsr(u32::MAX)`.
    ipi: MicrosIsr,

    /// Current Schmitt trigger state.
    ///
    /// `None` = no sample received yet (next `update()` initializes state
    /// without counting a crossing). `Some(true)` = last confirmed level
    /// was HIGH (above `+H`). `Some(false)` = LOW (below `-H`).
    ///
    /// `Option<bool>` is 1 byte via niche optimization (3 valid bit
    /// patterns: None, Some(false), Some(true)).
    state_high: Option<bool>,
}

// ---------------------------------------------------------------------------
// Sealed trait implementations
// ---------------------------------------------------------------------------

impl private::Sealed for Cordic {
    fn rpm(&self, poles: PoleCount) -> Option<Rpm> {
        debug_assert!(
            poles == PoleCount::Two,
            "Cordic RPM math is only valid for POLES_COUNT=2 (diametrically magnetized magnet)"
        );
        if self.last_velocity_dps.is_nan() {
            return None;
        }
        let pole_pairs = (poles.count() / 2) as f32;
        let rpm_raw = self.last_velocity_dps / (Degrees::MAX.0 * pole_pairs) * 60.0;
        Some(Rpm(rpm_raw))
    }

    fn cumulative_revolutions(&self, poles: PoleCount) -> f32 {
        debug_assert!(
            poles == PoleCount::Two,
            "Cordic cumulative_revolutions is only valid for POLES_COUNT=2 (diametrically magnetized magnet)"
        );
        let pole_pairs = (poles.count() / 2) as f32;
        let revolutions = (self.revolutions as f32) + self.fractional_angle.0 / Degrees::MAX.0;
        revolutions / pole_pairs
    }

    fn reset(&mut self) {
        self.previous_angle = None;
        self.revolutions = 0;
        self.fractional_angle = Degrees::MIN;
        self.last_velocity_dps = f32::NAN;
        self.max_abs_delta = f32::NAN;
    }

    fn max_abs_delta(&self) -> Option<SignedDegrees> {
        if self.max_abs_delta.is_nan() {
            None
        } else {
            Some(SignedDegrees(self.max_abs_delta))
        }
    }
}

impl private::Sealed for ZeroCrossing {
    fn rpm(&self, poles: PoleCount) -> Option<Rpm> {
        if self.crossings == Crossings(0) || self.elapsed.0 == 0 {
            return None;
        }
        let revolutions = self.crossings.0 as f32 / poles.count() as f32;
        Some(Rpm(revolutions / self.elapsed.to_seconds() * 60.0))
    }

    fn cumulative_revolutions(&self, poles: PoleCount) -> f32 {
        self.crossings.0 as f32 / poles.count() as f32
    }

    fn reset(&mut self) {
        self.crossings = Crossings(0);
        self.elapsed = MicrosIsr(0);
        self.ipi = MicrosIsr(0);
        self.state_high = None;
        // hysteresis is intentionally preserved — it is a constructor
        // parameter, not transient measurement state.
    }

    fn max_abs_delta(&self) -> Option<SignedDegrees> {
        // Not applicable to zero-crossing tracking.
        None
    }
}

// ---------------------------------------------------------------------------
// Public tracking mode trait
// ---------------------------------------------------------------------------

/// Marker trait for rotation tracking algorithm selection.
///
/// Sealed — only [`Cordic`] and [`ZeroCrossing`] implement this trait.
/// External code cannot add implementations.
pub trait TrackingMode: private::Sealed {}
impl TrackingMode for Cordic {}
impl TrackingMode for ZeroCrossing {}

// ---------------------------------------------------------------------------
// Core struct
// ---------------------------------------------------------------------------

/// Rotation tracker with compile-time pole count and algorithm selection.
///
/// `POLES_COUNT` is the total number of magnet poles (2, 3, or 4). `M` is the
/// tracking algorithm — [`Cordic`] or [`ZeroCrossing`].
///
/// See the [module documentation](self) for a mode selection guide.
///
/// # Constructors
///
/// | Expression | When available |
/// |-----------|----------------|
/// | `RotationTracker::<2, Cordic>::new()` | CORDIC, 2-pole only |
/// | `RotationTracker::<N, ZeroCrossing>::new(H)` | Zero-crossing, N ∈ {2, 3, 4} |
///
/// # Thread Safety
///
/// Not `Sync` — designed for single-threaded embedded use.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RotationTracker<const POLES_COUNT: u8, M: TrackingMode> {
    mode: M,
}

// ---------------------------------------------------------------------------
// Shared methods available on all RotationTracker<POLES_COUNT, M>
// ---------------------------------------------------------------------------

impl<const POLES_COUNT: u8, M: TrackingMode> RotationTracker<POLES_COUNT, M> {
    /// Returns the strongly typed pole count for this tracker.
    #[inline]
    pub const fn poles(&self) -> PoleCount {
        match POLES_COUNT {
            2 => PoleCount::Two,
            3 => PoleCount::Three,
            4 => PoleCount::Four,
            _ => panic!("POLES_COUNT must be 2, 3, or 4"),
        }
    }

    /// Instantaneous physical (mechanical) RPM.
    ///
    /// - **CORDIC mode**: computed from the last velocity measurement.
    ///   Returns `None` before the second `update()` or when the most
    ///   recent `elapsed` was `MicrosIsr(0)`.
    /// - **Zero-crossing mode**: computed as average RPM over all elapsed
    ///   time since construction or last [`reset()`](Self::reset).
    ///   Returns `None` when no crossings have been recorded yet.
    pub fn rpm(&self) -> Option<Rpm> {
        self.mode.rpm(self.poles())
    }

    /// Total cumulative mechanical revolutions since construction or last reset.
    ///
    /// - **CORDIC**: `electrical_revolutions / pole_pairs`
    /// - **Zero-crossing**: `crossings / POLES_COUNT`
    ///
    /// The return value is a `f32` to preserve sub-revolution precision.
    pub fn cumulative_revolutions(&self) -> f32 {
        self.mode.cumulative_revolutions(self.poles())
    }

    /// Resets all accumulated state. The next `update()` acts as a fresh
    /// first sample.
    ///
    /// The [`hysteresis`] parameter of a `ZeroCrossing` tracker is
    /// preserved across resets — only measurement state is cleared.
    ///
    /// [`hysteresis`]: ZeroCrossing
    pub fn reset(&mut self) {
        self.mode.reset();
    }

    /// Largest absolute angle delta observed across all `update()` calls.
    ///
    /// **CORDIC mode only** — values near 180° indicate under-sampling
    /// (Nyquist ambiguity). Returns `None` for [`ZeroCrossing`] where
    /// this concept does not apply, and before any delta has been computed
    /// (i.e., before the second `update()` call).
    ///
    /// Typical healthy values at 2500 Hz sample rate:
    /// - 1000 RPM (1 pole pair): ~2.4° max delta
    /// - 5000 RPM (1 pole pair): ~12° max delta
    pub fn max_abs_delta(&self) -> Option<SignedDegrees> {
        self.mode.max_abs_delta()
    }
}

// ---------------------------------------------------------------------------
// CORDIC-specific constructors (POLES_COUNT = 2 only)
// ---------------------------------------------------------------------------

impl Default for RotationTracker<2, Cordic> {
    fn default() -> Self {
        Self::new()
    }
}

impl RotationTracker<2, Cordic> {
    /// Creates a new CORDIC rotation tracker.
    ///
    /// Only constructible for `POLES_COUNT = 2` — diametrically magnetized
    /// 2-pole magnets. Attempting `RotationTracker::<4, Cordic>::new()`
    /// is a compile error (no such method exists).
    ///
    /// # Examples
    ///
    /// ```
    /// use tmag5273::{RotationTracker, Cordic};
    ///
    /// let tracker = RotationTracker::<2, Cordic>::new();
    /// ```
    pub fn new() -> Self {
        Self {
            mode: Cordic {
                previous_angle: None,
                revolutions: 0,
                fractional_angle: Degrees::MIN,
                last_velocity_dps: f32::NAN,
                max_abs_delta: f32::NAN,
            },
        }
    }
}

// ---------------------------------------------------------------------------
// CORDIC-specific update and query methods (POLES = 2 only)
// ---------------------------------------------------------------------------

impl RotationTracker<2, Cordic> {
    /// Feed an angle reading and elapsed time since the previous call.
    ///
    /// Returns the signed angular delta for this step as [`SignedDegrees`],
    /// or `None` on the first call (no previous angle) or when the input is
    /// invalid (NaN / out-of-range). `Some(SignedDegrees(0.0))` means the
    /// sensor was genuinely stationary — distinguishable from `None`.
    ///
    /// # Wraparound
    ///
    /// The delta uses the shortest-path convention: 350°→10° yields +20°,
    /// not −340°. A delta of exactly +180° is treated as forward rotation
    /// (Nyquist ambiguity limit — direction is genuinely indeterminate).
    ///
    /// # Velocity
    ///
    /// If `elapsed` is `MicrosIsr(0)`, the velocity is set to unavailable.
    /// Division by zero is never silently defaulted.
    ///
    /// # Non-finite Input
    ///
    /// NaN and out-of-range angle values are rejected. No state is modified
    /// and `None` is returned. This prevents infinite loops in the
    /// `fractional_angle` normalization step.
    pub fn update(&mut self, angle: Degrees, elapsed: MicrosIsr) -> Option<SignedDegrees> {
        // Guard: reject non-finite / out-of-range input. NaN/Inf would corrupt
        // the accumulator and infinite-loop the fractional_angle normalization.
        if !angle.is_valid() {
            defmt_warn!("update: angle {}° is non-finite, ignoring sample", angle.0);
            return None;
        }

        let Some(previous_angle) = self.mode.previous_angle else {
            // First sample — establish baseline. No delta computed.
            self.mode.previous_angle = Some(angle);
            self.mode.fractional_angle = angle;
            return None;
        };

        // Branch-based shortest-path delta in (-180°, +180°].
        // Avoids `rem_euclid` which has rounding artifacts at the 0/360
        // boundary on f32.
        let mut delta = SignedDegrees(angle.0 - previous_angle.0);

        if delta > SignedDegrees(180.0) {
            delta -= SignedDegrees::MAX;
        } else if delta <= -SignedDegrees(180.0) {
            delta += SignedDegrees::MAX;
        }

        self.mode.previous_angle = Some(angle);

        // Track the largest observed delta for Nyquist aliasing detection.
        let abs_delta = delta.abs();
        if self.mode.max_abs_delta.is_nan() || abs_delta.0 > self.mode.max_abs_delta {
            self.mode.max_abs_delta = abs_delta.0;
        }

        // Accumulate into split representation (rev counter + fractional_angle).
        self.mode.fractional_angle += delta;
        while self.mode.fractional_angle >= Degrees::MAX {
            self.mode.revolutions += 1;
            self.mode.fractional_angle -= SignedDegrees::MAX;
        }
        while self.mode.fractional_angle < Degrees::MIN {
            self.mode.revolutions -= 1;
            self.mode.fractional_angle += SignedDegrees::MAX;
        }

        // Compute instantaneous electrical velocity (deg/s).
        let secs = elapsed.to_seconds();
        self.mode.last_velocity_dps = if secs > 0.0 { delta.0 / secs } else { f32::NAN };

        Some(delta)
    }

    /// Total cumulative rotation in electrical degrees.
    ///
    /// For a single pole pair (`POLES_COUNT = 2`), this equals the mechanical angle.
    pub fn accumulated_electrical_angle(&self) -> f32 {
        (self.mode.revolutions as f32) * Degrees::MAX.0 + self.mode.fractional_angle.0
    }

    /// Instantaneous angular velocity in electrical degrees per second.
    ///
    /// Returns `None` before the second `update()` call or when the most
    /// recent `elapsed` was `MicrosIsr(0)`.
    pub fn angular_velocity_dps(&self) -> Option<f32> {
        if self.mode.last_velocity_dps.is_nan() {
            None
        } else {
            Some(self.mode.last_velocity_dps)
        }
    }
}

// ---------------------------------------------------------------------------
// Zero-crossing constructor and update (generic over POLES_COUNT)
// ---------------------------------------------------------------------------

impl<const POLES_COUNT: u8> RotationTracker<POLES_COUNT, ZeroCrossing> {
    /// Creates a new zero-crossing tracker with the given Schmitt trigger
    /// threshold.
    ///
    /// # Parameters
    ///
    /// - `hysteresis`: the dead-band half-width in mT. A crossing is counted
    ///   only when the signal moves from below `-hysteresis` to above
    ///   `+hysteresis` (or the reverse). Typical: 10% of signal peak-to-peak
    ///   swing. Values ≤ 0 degenerate to simple sign-change detection.
    ///
    /// # Compile-Time Pole Count Validation
    ///
    /// `POLES_COUNT` must be 2, 3, or 4. Any other value triggers a compile-time
    /// assertion failure:
    ///
    /// ```compile_fail
    /// use tmag5273::{RotationTracker, ZeroCrossing, MilliTesla};
    /// let _ = RotationTracker::<7, ZeroCrossing>::new(MilliTesla(0.5)); // error: POLES_COUNT must be 2, 3, or 4
    /// ```
    ///
    /// # Examples
    ///
    /// ```
    /// use tmag5273::{RotationTracker, ZeroCrossing, MilliTesla};
    ///
    /// // 4-pole Gicar ring magnet, H = 0.8 mT
    /// let tracker = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(0.8));
    /// ```
    pub fn new(hysteresis: MilliTesla) -> Self {
        // Compile-time assertion: only 2 through 4 are valid pole counts
        // for the supported magnet assemblies.
        const {
            assert!(
                POLES_COUNT >= 2 && POLES_COUNT <= 4,
                "POLES_COUNT must be 2, 3, or 4"
            )
        };
        Self {
            mode: ZeroCrossing {
                hysteresis: MilliTesla(hysteresis.0.max(0.0)),
                crossings: Crossings(0),
                elapsed: MicrosIsr(0),
                ipi: MicrosIsr(0),
                state_high: None,
            },
        }
    }

    /// Feed a magnetic field sample and elapsed time since the previous call.
    ///
    /// Returns the inter-pulse interval (IPI) when a crossing is detected,
    /// starting from the **second** crossing onward (N pulses → N-1 intervals,
    /// matching coffiot-core's `ipis_us[..count - 1]` convention). The first
    /// crossing returns `None` because there is no prior crossing to measure
    /// from.
    ///
    /// On the first call, the current value initializes the Schmitt trigger
    /// state. No crossing is counted.
    ///
    /// # Schmitt Trigger Logic
    ///
    /// - If currently HIGH (`value ≥ +H` previously confirmed) and
    ///   `value ≤ -H` → count one crossing, set state LOW.
    /// - If currently LOW (`value ≤ -H` previously confirmed) and
    ///   `value ≥ +H` → count one crossing, set state HIGH.
    /// - Values within `(-H, +H)` are ignored (noise dead band).
    ///
    /// # Return Value
    ///
    /// - `None` — no crossing detected, or this was the first crossing
    /// - `Some(MicrosIsr(n))` — inter-pulse interval in microseconds since
    ///   the previous crossing, including the current sample's `elapsed`
    ///
    /// # Elapsed Accumulation
    ///
    /// `elapsed` is accumulated into an internal [`MicrosIsr`] counter.
    /// Saturating addition is used — the counter stops incrementing at
    /// `MicrosIsr(u32::MAX)` (~71 min). Reset the tracker for long-running sessions.
    pub fn update(&mut self, value: MilliTesla, elapsed: MicrosIsr) -> Option<MicrosIsr> {
        let hysteresis = self.mode.hysteresis;

        // Accumulate IPI and total elapsed before crossing detection,
        // so the triggering sample's elapsed is included in the interval.
        self.mode.ipi = MicrosIsr(self.mode.ipi.0.saturating_add(elapsed.0));
        self.mode.elapsed = MicrosIsr(self.mode.elapsed.0.saturating_add(elapsed.0));

        let crossed = match self.mode.state_high {
            Some(true) => {
                // Currently HIGH — transition when value drops to or below -H.
                if value <= -hysteresis {
                    self.mode.state_high = Some(false);
                    self.mode.crossings = self.mode.crossings.saturating_add(1);
                    true
                } else {
                    false
                }
            }
            Some(false) => {
                // Currently LOW — transition when value rises to or above +H.
                if value >= hysteresis {
                    self.mode.state_high = Some(true);
                    self.mode.crossings = self.mode.crossings.saturating_add(1);
                    true
                } else {
                    false
                }
            }
            None => {
                // First sample: initialize state only if clearly outside the
                // dead band. Values within (-H, +H) leave state as None,
                // deferring initialization to the first unambiguous sample.
                // This prevents ±1 phantom crossings when the first reading
                // falls inside the Schmitt trigger's hysteresis band.
                if value >= hysteresis {
                    self.mode.state_high = Some(true);
                } else if value <= -hysteresis {
                    self.mode.state_high = Some(false);
                }
                false
            }
        };

        if crossed {
            let interval = self.mode.ipi;
            self.mode.ipi = MicrosIsr(0);
            // First crossing (crossings == 1 after increment): no prior
            // crossing to measure from → None. Subsequent: Some(interval).
            if self.mode.crossings > Crossings(1) {
                Some(interval)
            } else {
                None
            }
        } else {
            None
        }
    }

    /// Total zero-crossings counted since construction or last [`reset()`].
    ///
    /// Raw crossing count — divide by `POLES_COUNT` for mechanical revolutions,
    /// or use [`cumulative_revolutions()`] which does this automatically.
    ///
    /// [`reset()`]: RotationTracker::reset
    /// [`cumulative_revolutions()`]: RotationTracker::cumulative_revolutions
    pub fn crossings(&self) -> Crossings {
        self.mode.crossings
    }
}

// ---------------------------------------------------------------------------
// Backward-compatibility type alias
// ---------------------------------------------------------------------------

/// Type alias for the 2-pole CORDIC-based rotation tracker.
///
/// Direct replacement for the previous `RotationTracker` type (before the
/// const-generic redesign). For multi-pole magnets, use
/// `RotationTracker::<N, ZeroCrossing>` instead.
pub type CordicTracker = RotationTracker<2, Cordic>;

// ---------------------------------------------------------------------------
// Size assertions
// ---------------------------------------------------------------------------

// Cordic mode: 5 × f32 + 1 × i32 = 5×4 + 4 = 24 bytes maximum.
// With layout optimization (no pole_pairs u8 field), target is ≤ 24 bytes.
const _: () = assert!(
    size_of::<RotationTracker<2, Cordic>>() <= 24,
    "RotationTracker<2, Cordic> exceeds 24-byte struct size budget"
);

// ZeroCrossing mode: f32 + u32 + u32 + u32 + Option<bool> = 4+4+4+4+1 = 17 bytes,
// aligned to 4 bytes = 20 bytes.
const _: () = assert!(
    size_of::<RotationTracker<4, ZeroCrossing>>() <= 24,
    "RotationTracker<4, ZeroCrossing> exceeds 24-byte struct size budget"
);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn mt(value: f32) -> MilliTesla {
        MilliTesla(value)
    }

    // -----------------------------------------------------------------------
    // CORDIC mode — happy path
    // -----------------------------------------------------------------------

    #[test]
    fn cordic_sequential_angles_accumulate() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees::MIN, MicrosIsr(0));
        t.update(Degrees(90.0), MicrosIsr(10_000));
        t.update(Degrees(180.0), MicrosIsr(10_000));
        t.update(Degrees(270.0), MicrosIsr(10_000));

        let ed = t.accumulated_electrical_angle();
        assert!((ed - 270.0).abs() < 0.01, "expected ~270, got {}", ed);

        // velocity: 90° per 10 ms = 9000 deg/s
        let v = t.angular_velocity_dps().unwrap();
        assert!((v - 9000.0).abs() < 1.0, "expected ~9000, got {}", v);
    }

    #[test]
    fn cordic_full_revolution_single_pole() {
        let mut t = RotationTracker::<2, Cordic>::new();
        let angles = [0.0, 90.0, 180.0, 270.0, 0.0]; // back to 0 = full rev
        for &a in &angles {
            t.update(Degrees(a), MicrosIsr(10_000));
        }
        let rev = t.cumulative_revolutions();
        assert!((rev - 1.0).abs() < 0.01, "expected ~1.0, got {}", rev);
    }

    #[test]
    fn cordic_rpm_single_pole_pair() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees::MIN, MicrosIsr(0));
        // 90° in 1 ms = 90_000 deg/s electrical
        t.update(Degrees(90.0), MicrosIsr(1000));

        let rpm = t.rpm().unwrap();
        // RPM = 90_000 / (360 * 1) * 60 = 90_000 / 360 * 60 = 15_000
        assert!(
            (rpm.0 - 15_000.0).abs() < 1.0,
            "expected 15000, got {}",
            rpm.0
        );
    }

    // -----------------------------------------------------------------------
    // CORDIC mode — wraparound edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn cordic_forward_wraparound_350_to_10() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees(350.0), MicrosIsr(0));
        let delta = t.update(Degrees(10.0), MicrosIsr(1000));

        let d = delta.unwrap();
        assert!(
            (d - SignedDegrees(20.0)).abs() < SignedDegrees(0.01),
            "expected +20, got {:?}",
            d
        );
    }

    #[test]
    fn cordic_backward_wraparound_10_to_350() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees(10.0), MicrosIsr(0));
        let delta = t.update(Degrees(350.0), MicrosIsr(1000));

        let d = delta.unwrap();
        assert!(
            (d - SignedDegrees(-20.0)).abs() < SignedDegrees(0.01),
            "expected -20, got {:?}",
            d
        );
    }

    #[test]
    fn cordic_backward_accumulation_underflow() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees(10.0), MicrosIsr(0));
        t.update(Degrees(350.0), MicrosIsr(1000)); // -20°
        t.update(Degrees(330.0), MicrosIsr(1000)); // -20°
        t.update(Degrees(310.0), MicrosIsr(1000)); // -20°

        // Baseline at 10°, cumulative delta = -60° → rev=-1, frac=310°
        let rev = t.cumulative_revolutions();
        let expected = (-1.0_f32 + 310.0 / Degrees::MAX.0) / 1.0; // pole_pairs = 1
        assert!(
            (rev - expected).abs() < 0.01,
            "expected ~{}, got {}",
            expected,
            rev
        );
        assert!(
            t.mode.fractional_angle >= Degrees::MIN && t.mode.fractional_angle < Degrees::MAX,
            "fractional_angle out of range: {:?}",
            t.mode.fractional_angle
        );
        assert!(
            t.mode.revolutions < 0,
            "revolutions should be negative: {}",
            t.mode.revolutions
        );
    }

    // -----------------------------------------------------------------------
    // CORDIC mode — first sample, zero elapsed, stationary
    // -----------------------------------------------------------------------

    #[test]
    fn cordic_first_update_no_velocity() {
        let mut t = RotationTracker::<2, Cordic>::new();
        let delta = t.update(Degrees(45.0), MicrosIsr(1000));

        assert!(delta.is_none());
        assert!(t.angular_velocity_dps().is_none());
        assert!(t.rpm().is_none());
    }

    #[test]
    fn cordic_zero_elapsed_no_velocity() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees::MIN, MicrosIsr(1000));
        t.update(Degrees(90.0), MicrosIsr(0)); // zero elapsed

        assert!(t.angular_velocity_dps().is_none());
        assert!(t.rpm().is_none());
    }

    #[test]
    fn cordic_identical_angles_zero_delta() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees(180.0), MicrosIsr(0));
        let delta = t.update(Degrees(180.0), MicrosIsr(1000));

        assert_eq!(delta, Some(SignedDegrees(0.0)));
        let v = t.angular_velocity_dps().unwrap();
        assert_eq!(v, 0.0);
    }

    // -----------------------------------------------------------------------
    // CORDIC mode — Nyquist boundary
    // -----------------------------------------------------------------------

    #[test]
    fn cordic_half_turn_treated_as_forward() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees::MIN, MicrosIsr(0));
        let delta = t.update(Degrees(180.0), MicrosIsr(1000));

        let d = delta.unwrap();
        assert!(
            (d - SignedDegrees(180.0)).abs() < SignedDegrees(0.01),
            "expected +180, got {:?}",
            d
        );
    }

    // -----------------------------------------------------------------------
    // CORDIC mode — non-finite input guards
    // -----------------------------------------------------------------------

    #[test]
    fn cordic_nan_input_ignored() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees(90.0), MicrosIsr(0));
        t.update(Degrees(180.0), MicrosIsr(1000));
        let rev_before = t.cumulative_revolutions();
        let max_before = t.max_abs_delta();

        let delta = t.update(Degrees(f32::NAN), MicrosIsr(1000));
        assert!(delta.is_none());
        assert_eq!(t.cumulative_revolutions(), rev_before);
        assert_eq!(
            t.max_abs_delta(),
            max_before,
            "NaN must not corrupt max_abs_delta"
        );
    }

    #[test]
    fn cordic_infinity_input_ignored() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees::MIN, MicrosIsr(0));
        let rev_before = t.cumulative_revolutions();

        let delta = t.update(Degrees(f32::INFINITY), MicrosIsr(1000));
        assert!(delta.is_none());
        assert_eq!(t.cumulative_revolutions(), rev_before);
    }

    #[test]
    fn cordic_neg_infinity_input_ignored() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees(90.0), MicrosIsr(0));

        let delta = t.update(Degrees(f32::NEG_INFINITY), MicrosIsr(1000));
        assert!(delta.is_none());
    }

    // -----------------------------------------------------------------------
    // CORDIC mode — reset
    // -----------------------------------------------------------------------

    #[test]
    fn cordic_reset_clears_all_state() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees::MIN, MicrosIsr(0));
        t.update(Degrees(90.0), MicrosIsr(1000));
        t.update(Degrees(180.0), MicrosIsr(1000));

        t.reset();

        assert_eq!(t.mode.revolutions, 0);
        assert_eq!(t.mode.fractional_angle, Degrees::MIN);
        assert!(t.mode.previous_angle.is_none());
        assert!(t.angular_velocity_dps().is_none());

        // Next update acts as first sample.
        let delta = t.update(Degrees(45.0), MicrosIsr(1000));
        assert!(delta.is_none());
    }

    // -----------------------------------------------------------------------
    // CORDIC mode — precision
    // -----------------------------------------------------------------------

    #[test]
    fn cordic_many_small_increments_no_drift() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees::MIN, MicrosIsr(0));

        let mut angle = 0.0_f32;
        for _ in 0..100_000 {
            angle += 1.0;
            if angle >= Degrees::MAX.0 {
                angle -= Degrees::MAX.0;
            }
            t.update(Degrees(angle), MicrosIsr(100));
        }

        // 100,000° / 360° = 277.777... revolutions
        let expected_rev = 100_000.0 / Degrees::MAX.0;
        let actual_rev = t.cumulative_revolutions();
        let error = (actual_rev - expected_rev).abs();
        assert!(
            error < 0.01,
            "expected ~{}, got {} (error {})",
            expected_rev,
            actual_rev,
            error
        );
    }

    // -----------------------------------------------------------------------
    // CORDIC mode — constructor and Default
    // -----------------------------------------------------------------------

    #[test]
    fn cordic_new_initializes_nan_sentinels() {
        let t = RotationTracker::<2, Cordic>::new();
        assert!(t.mode.previous_angle.is_none());
        assert_eq!(t.mode.revolutions, 0);
        assert_eq!(t.mode.fractional_angle, Degrees::MIN);
        assert!(
            t.mode.last_velocity_dps.is_nan(),
            "last_velocity_dps must be NaN initially"
        );
        assert!(
            t.mode.max_abs_delta.is_nan(),
            "max_abs_delta must be NaN initially"
        );
        // Public API should return None for uninitialized state.
        assert!(t.angular_velocity_dps().is_none());
        assert!(t.max_abs_delta().is_none());
        assert!(t.rpm().is_none());
    }

    #[test]
    fn cordic_default_delegates_to_new() {
        let from_default = RotationTracker::<2, Cordic>::default();
        // Default must produce the same NaN-sentinel state as new().
        assert!(from_default.mode.last_velocity_dps.is_nan());
        assert!(from_default.mode.max_abs_delta.is_nan());
        assert!(from_default.angular_velocity_dps().is_none());
        assert!(from_default.max_abs_delta().is_none());
        assert!(from_default.rpm().is_none());
    }

    // -----------------------------------------------------------------------
    // CORDIC mode — max_abs_delta
    // -----------------------------------------------------------------------

    #[test]
    fn cordic_max_abs_delta_sequential() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees::MIN, MicrosIsr(0));
        t.update(Degrees(90.0), MicrosIsr(10_000));
        t.update(Degrees(180.0), MicrosIsr(10_000));

        let mad = t.max_abs_delta().unwrap();
        assert!(
            (mad - SignedDegrees(90.0)).abs() < SignedDegrees(0.01),
            "expected 90.0, got {:?}",
            mad
        );
    }

    #[test]
    fn cordic_max_abs_delta_reset_clears() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees::MIN, MicrosIsr(0));
        t.update(Degrees(90.0), MicrosIsr(10_000));
        assert!(t.max_abs_delta().is_some());

        t.reset();
        assert_eq!(t.max_abs_delta(), None);
    }

    #[test]
    fn cordic_max_abs_delta_first_update_stays_zero() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees(45.0), MicrosIsr(1000));
        assert_eq!(t.max_abs_delta(), None);
    }

    #[test]
    fn cordic_max_abs_delta_wraparound_uses_shortest_path() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees(350.0), MicrosIsr(0));
        t.update(Degrees(10.0), MicrosIsr(1000));

        // Shortest-path delta: 350→10 = +20° (not 340°)
        let mad = t.max_abs_delta().unwrap();
        assert!(
            (mad - SignedDegrees(20.0)).abs() < SignedDegrees(0.01),
            "expected 20.0, got {:?}",
            mad
        );
    }

    #[test]
    fn cordic_max_abs_delta_near_nyquist() {
        let mut t = RotationTracker::<2, Cordic>::new();
        t.update(Degrees::MIN, MicrosIsr(0));
        t.update(Degrees(179.0), MicrosIsr(1000));

        let mad = t.max_abs_delta().unwrap();
        assert!(
            (mad - SignedDegrees(179.0)).abs() < SignedDegrees(0.01),
            "expected 179.0, got {:?}",
            mad
        );
    }

    // -----------------------------------------------------------------------
    // CORDIC mode — type alias
    // -----------------------------------------------------------------------

    #[test]
    fn cordic_tracker_type_alias_works() {
        let t = CordicTracker::new();
        assert!(t.rpm().is_none());
    }

    // -----------------------------------------------------------------------
    // Zero-crossing mode — happy path
    // -----------------------------------------------------------------------

    #[test]
    fn zc_schmitt_trigger_two_crossings_in_full_cycle() {
        // Feed values: +5, +3, -1, -5, -3, +1, +5 with H=2.0
        // Crossings: state starts HIGH (5 > 0)
        //   +3: within band → no change
        //   -1: within band (-1 > -2) → no change
        //   -5: below -H (-5 < -2) → LOW, crossing #1
        //   -3: already LOW, -3 < -2 so no new crossing
        //   +1: within band → no change
        //   +5: above +H (5 > 2) → HIGH, crossing #2
        // Total: 2 crossings
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(2.0));
        let values = [5.0_f32, 3.0, -1.0, -5.0, -3.0, 1.0, 5.0];
        for &v in &values {
            t.update(mt(v), MicrosIsr(1000));
        }
        assert_eq!(
            t.crossings(),
            Crossings(2),
            "expected 2 crossings from one full cycle, got {:?}",
            t.crossings()
        );
    }

    #[test]
    fn zc_rpm_from_crossings_and_elapsed() {
        // 80 crossings over 0.3s with 4 poles = 80/4/0.3*60 = 4000 RPM
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(0.5));

        // Inject 80 crossings: alternate HIGH-crossing and LOW-crossing signals.
        // First update initializes state (no crossing). Then each pair of
        // opposite-sign values beyond ±H yields one crossing each.
        // To get 80 crossings, we provide 81 alternating samples.
        let mut state_positive = true;
        t.update(
            if state_positive { mt(5.0) } else { mt(-5.0) },
            MicrosIsr(0),
        );
        for _ in 0..80 {
            state_positive = !state_positive;
            let elapsed = 300_000_u32 / 80; // ~3750 µs per crossing
            t.update(
                if state_positive { mt(5.0) } else { mt(-5.0) },
                MicrosIsr(elapsed),
            );
        }

        let crossings = t.crossings();
        assert_eq!(
            crossings,
            Crossings(80),
            "expected 80 crossings, got {:?}",
            crossings
        );

        let rpm = t.rpm().expect("rpm should be Some after crossings");
        // 80 crossings / 4 poles = 20 revolutions / (total_elapsed / 1e6) * 60
        // total_elapsed ≈ 300_000 µs = 0.3s
        // rpm ≈ 20 / 0.3 * 60 = 4000
        assert!(
            (rpm.0 - 4000.0).abs() < 100.0,
            "expected ~4000 RPM, got {}",
            rpm.0
        );
    }

    #[test]
    fn zc_cumulative_revolutions() {
        // 80 crossings with 4 poles = 20 mechanical revolutions
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(0.5));
        // inject 80 crossings
        let mut high = true;
        t.update(mt(5.0), MicrosIsr(0));
        for _ in 0..80 {
            high = !high;
            t.update(if high { mt(5.0) } else { mt(-5.0) }, MicrosIsr(1000));
        }
        let rev = t.cumulative_revolutions();
        assert!(
            (rev - 20.0).abs() < 0.01,
            "expected 20.0 revolutions, got {}",
            rev
        );
    }

    #[test]
    fn zc_reset_clears_state() {
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(0.5));
        t.update(mt(5.0), MicrosIsr(0));
        t.update(mt(-5.0), MicrosIsr(1000));
        assert!(t.crossings() > Crossings(0));

        t.reset();

        assert_eq!(t.crossings(), Crossings(0));
        assert_eq!(t.cumulative_revolutions(), 0.0);
        assert!(t.rpm().is_none());
        assert!(t.mode.state_high.is_none());
        // hysteresis preserved
        assert!((t.mode.hysteresis.0 - 0.5).abs() < 1e-6);
    }

    // -----------------------------------------------------------------------
    // Zero-crossing mode — edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn zc_noise_within_band_counts_no_crossings() {
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(2.0));
        // First sample initializes state (H=2.0, initialized as HIGH)
        t.update(mt(1.5), MicrosIsr(1000));
        // All values stay within (-2.0, +2.0) dead band
        let noise = [1.0_f32, -1.0, 0.5, -0.5, 1.8, -1.8, 0.0];
        for &v in &noise {
            t.update(mt(v), MicrosIsr(1000));
        }
        assert_eq!(
            t.crossings(),
            Crossings(0),
            "noise within dead band must not count crossings"
        );
    }

    #[test]
    fn zc_first_update_initializes_no_crossing() {
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(0.5));
        // Very first update — no crossing counted, just state initialization
        t.update(mt(5.0), MicrosIsr(1000));
        assert_eq!(
            t.crossings(),
            Crossings(0),
            "first update must not count a crossing"
        );
        assert_eq!(t.mode.state_high, Some(true));
    }

    #[test]
    fn zc_zero_hysteresis_simple_sign_change() {
        // H=0: any sign change counts, no dead band
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(0.0));
        t.update(mt(1.0), MicrosIsr(1000)); // initialize HIGH
        t.update(mt(-1.0), MicrosIsr(1000)); // crosses to LOW (1 crossing)
        t.update(mt(1.0), MicrosIsr(1000)); // crosses to HIGH (2 crossings)
        assert_eq!(
            t.crossings(),
            Crossings(2),
            "H=0 should behave as sign-change detection"
        );
    }

    #[test]
    fn zc_zero_hysteresis_detects_zero_crossing_at_exact_zero() {
        // With H=0, a signal passing through exactly 0.0 mT must trigger
        // a state transition — not skip it due to strict comparison.
        let mut t = RotationTracker::<2, ZeroCrossing>::new(MilliTesla(0.0));
        t.update(mt(5.0), MicrosIsr(100)); // init HIGH
        t.update(mt(0.0), MicrosIsr(100)); // exactly 0.0 — should transition to LOW
        assert_eq!(
            t.crossings(),
            Crossings(1),
            "0.0 mT must trigger crossing with H=0"
        );
        t.update(mt(0.0), MicrosIsr(100)); // still at 0.0 — should transition back to HIGH
        assert_eq!(
            t.crossings(),
            Crossings(2),
            "second 0.0 mT must trigger crossing back"
        );
    }

    #[test]
    fn zc_no_rpm_before_crossings() {
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(0.5));
        assert!(t.rpm().is_none(), "rpm() must be None before any crossings");
        t.update(mt(5.0), MicrosIsr(1000)); // first update (no crossing)
        assert!(
            t.rpm().is_none(),
            "rpm() must be None after only first update"
        );
    }

    #[test]
    fn zc_max_abs_delta_returns_zero() {
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(0.5));
        t.update(mt(5.0), MicrosIsr(1000));
        t.update(mt(-5.0), MicrosIsr(1000));
        assert_eq!(
            t.max_abs_delta(),
            None,
            "ZeroCrossing max_abs_delta must always return None"
        );
    }

    #[test]
    fn zc_exposes_strong_pole_count() {
        let t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(0.5));
        assert_eq!(t.poles(), PoleCount::Four);
        assert_eq!(u8::from(t.poles()), 4);
    }

    #[test]
    fn zc_two_pole_works() {
        // 2-pole zero-crossing is valid (per R5)
        let mut t = RotationTracker::<2, ZeroCrossing>::new(MilliTesla(0.5));
        t.update(mt(5.0), MicrosIsr(0));
        t.update(mt(-5.0), MicrosIsr(1000));
        t.update(mt(5.0), MicrosIsr(1000));
        assert_eq!(t.crossings(), Crossings(2));
        // 2 crossings / 2 poles = 1 revolution
        assert!((t.cumulative_revolutions() - 1.0).abs() < 0.01);
    }

    #[test]
    fn zc_three_pole_works() {
        let mut t = RotationTracker::<3, ZeroCrossing>::new(MilliTesla(0.5));
        // 3 crossings / 3 poles = 1 revolution
        t.update(mt(5.0), MicrosIsr(0));
        t.update(mt(-5.0), MicrosIsr(1000));
        t.update(mt(5.0), MicrosIsr(1000));
        t.update(mt(-5.0), MicrosIsr(1000));
        assert_eq!(t.crossings(), Crossings(3));
        assert!((t.cumulative_revolutions() - 1.0).abs() < 0.01);
    }

    #[test]
    fn zc_negative_hysteresis_clamped_to_zero() {
        // H < 0.0 should be clamped to 0.0 (degenerate sign-change detection)
        let t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(-1.0));
        assert_eq!(
            t.mode.hysteresis,
            MilliTesla(0.0),
            "negative H must clamp to MilliTesla(0.0)"
        );
    }

    #[test]
    fn zc_rpm_none_when_all_elapsed_zero() {
        // crossings > 0 but elapsed == MicrosIsr(0) → rpm() must return None
        // (avoids division by zero).
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(0.5));
        t.update(mt(5.0), MicrosIsr(0)); // init state, no elapsed
        t.update(mt(-5.0), MicrosIsr(0)); // crossing #1, still no elapsed
        t.update(mt(5.0), MicrosIsr(0)); // crossing #2, still no elapsed
        assert_eq!(t.crossings(), Crossings(2), "should have 2 crossings");
        assert!(
            t.rpm().is_none(),
            "rpm() must be None when elapsed == MicrosIsr(0) (even with crossings > 0)"
        );
    }

    #[test]
    fn zc_elapsed_saturates_at_max() {
        // Verify elapsed uses saturating_add, not wrapping_add.
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(0.5));
        t.update(mt(5.0), MicrosIsr(u32::MAX - 10));
        t.update(mt(-5.0), MicrosIsr(20)); // would overflow without saturation
        assert_eq!(
            t.mode.elapsed,
            MicrosIsr(u32::MAX),
            "elapsed must saturate at MicrosIsr(u32::MAX), not wrap"
        );
    }

    // -----------------------------------------------------------------------
    // Edge-case tests (review-verified gaps)
    // -----------------------------------------------------------------------

    #[test]
    fn zc_first_sample_in_dead_band_defers_init() {
        // First sample inside (-H, +H) should NOT initialize state_high.
        // Subsequent in-band samples also defer. First out-of-band sample
        // initializes without counting a crossing.
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(0.8));

        // First sample: +0.3 mT, inside dead band (H=0.8)
        t.update(mt(0.3), MicrosIsr(1000));
        assert!(
            t.mode.state_high.is_none(),
            "in-band first sample must not initialize state"
        );
        assert_eq!(t.crossings(), Crossings(0));

        // Second sample: still in-band
        t.update(mt(-0.5), MicrosIsr(1000));
        assert!(
            t.mode.state_high.is_none(),
            "in-band samples keep state as None"
        );
        assert_eq!(t.crossings(), Crossings(0));

        // Third sample: +1.0 mT, clearly above +H → initializes as HIGH
        t.update(mt(1.0), MicrosIsr(1000));
        assert_eq!(t.mode.state_high, Some(true));
        assert_eq!(
            t.crossings(),
            Crossings(0),
            "initialization must not count a crossing"
        );

        // Now a real crossing: drop below -H
        t.update(mt(-1.0), MicrosIsr(1000));
        assert_eq!(t.mode.state_high, Some(false));
        assert_eq!(
            t.crossings(),
            Crossings(1),
            "real crossing after deferred init"
        );
    }

    #[test]
    fn zc_exact_hysteresis_boundary_triggers_crossing() {
        // Values exactly at ±H should trigger transitions (boundary inclusive).
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(2.0));

        // Init with clearly HIGH
        t.update(mt(5.0), MicrosIsr(1000));
        assert_eq!(t.mode.state_high, Some(true));

        // Exact -H: should transition HIGH→LOW
        t.update(mt(-2.0), MicrosIsr(1000));
        assert_eq!(
            t.crossings(),
            Crossings(1),
            "exact -H must trigger HIGH→LOW crossing"
        );
        assert_eq!(t.mode.state_high, Some(false));

        // Exact +H: should transition LOW→HIGH
        t.update(mt(2.0), MicrosIsr(1000));
        assert_eq!(
            t.crossings(),
            Crossings(2),
            "exact +H must trigger LOW→HIGH crossing"
        );
        assert_eq!(t.mode.state_high, Some(true));
    }

    #[test]
    fn cordic_large_raw_delta_normalizes_to_shortest_path() {
        // Raw delta of 350° (from 0°→350°) should normalize to -10° via
        // shortest-path, not +350°.
        let mut t = RotationTracker::<2, Cordic>::new();

        // Seed with 0°
        let delta1 = t.update(Degrees(0.0), MicrosIsr(1000));
        assert!(delta1.is_none(), "first update returns None (no previous)");

        // Jump to 350° — raw delta = +350°, shortest path = -10°
        let delta2 = t.update(Degrees(350.0), MicrosIsr(1000));
        let d = delta2.expect("second update should return Some");
        assert!(
            (d.0 - (-10.0)).abs() < 0.01,
            "350° raw delta should normalize to -10° shortest path, got {}",
            d.0
        );

        // Cumulative: 0° + (-10°) = -10° → fractional should be 350°
        let frac = t.mode.fractional_angle.0;
        assert!(
            (frac - 350.0).abs() < 0.01,
            "fractional angle should be 350° after -10° delta, got {}",
            frac
        );
    }

    // -----------------------------------------------------------------------
    // ZeroCrossing IPI (inter-pulse interval) emission
    // -----------------------------------------------------------------------

    #[test]
    fn zc_ipi_first_crossing_returns_none() {
        // First crossing has no prior crossing — IPI is undefined.
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(2.0));
        assert!(t.update(mt(5.0), MicrosIsr(1000)).is_none()); // init HIGH
        let ipi = t.update(mt(-5.0), MicrosIsr(1000)); // first crossing
        assert!(
            ipi.is_none(),
            "first crossing must return None (N-1 convention)"
        );
        assert_eq!(t.crossings(), Crossings(1));
    }

    #[test]
    fn zc_ipi_second_crossing_returns_interval() {
        // Second crossing emits IPI = elapsed since first crossing.
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(2.0));
        t.update(mt(5.0), MicrosIsr(0)); // init HIGH
        t.update(mt(-5.0), MicrosIsr(1000)); // crossing #1 → None
        let ipi = t.update(mt(5.0), MicrosIsr(2000)); // crossing #2
        assert_eq!(
            ipi,
            Some(MicrosIsr(2000)),
            "IPI should be elapsed since crossing #1"
        );
    }

    #[test]
    fn zc_ipi_accumulates_across_non_crossing_samples() {
        // IPI accumulates elapsed from all samples between crossings.
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(2.0));
        t.update(mt(5.0), MicrosIsr(0)); // init HIGH
        t.update(mt(-5.0), MicrosIsr(100)); // crossing #1 → None
        t.update(mt(-3.0), MicrosIsr(200)); // no crossing, IPI accumulates
        t.update(mt(-4.0), MicrosIsr(300)); // no crossing, IPI accumulates
        let ipi = t.update(mt(5.0), MicrosIsr(400)); // crossing #2
        assert_eq!(
            ipi,
            Some(MicrosIsr(200 + 300 + 400)),
            "IPI must include all elapsed since previous crossing"
        );
    }

    #[test]
    fn zc_ipi_multiple_crossings_emit_correct_intervals() {
        // 4 crossings → 3 IPIs, each with correct accumulated elapsed.
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(2.0));
        t.update(mt(5.0), MicrosIsr(0)); // init HIGH

        let ipi1 = t.update(mt(-5.0), MicrosIsr(100)); // crossing #1
        assert!(ipi1.is_none(), "first crossing → None");

        let ipi2 = t.update(mt(5.0), MicrosIsr(200)); // crossing #2
        assert_eq!(ipi2, Some(MicrosIsr(200)));

        let ipi3 = t.update(mt(-5.0), MicrosIsr(300)); // crossing #3
        assert_eq!(ipi3, Some(MicrosIsr(300)));

        let ipi4 = t.update(mt(5.0), MicrosIsr(400)); // crossing #4
        assert_eq!(ipi4, Some(MicrosIsr(400)));

        assert_eq!(t.crossings(), Crossings(4));
    }

    #[test]
    fn zc_ipi_includes_triggering_sample_elapsed() {
        // The elapsed of the sample that triggers the crossing is part of the IPI.
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(2.0));
        t.update(mt(5.0), MicrosIsr(0)); // init HIGH
        t.update(mt(-5.0), MicrosIsr(500)); // crossing #1
        // No intermediate samples — crossing #2 has only its own elapsed.
        let ipi = t.update(mt(5.0), MicrosIsr(750));
        assert_eq!(ipi, Some(MicrosIsr(750)));
    }

    #[test]
    fn zc_ipi_no_crossing_returns_none() {
        // Values within dead band never produce crossings or IPIs.
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(2.0));
        t.update(mt(5.0), MicrosIsr(1000)); // init HIGH
        assert!(t.update(mt(0.5), MicrosIsr(1000)).is_none());
        assert!(t.update(mt(-0.5), MicrosIsr(1000)).is_none());
        assert!(t.update(mt(1.0), MicrosIsr(1000)).is_none());
        assert_eq!(t.crossings(), Crossings(0));
    }

    #[test]
    fn zc_ipi_zero_elapsed_between_crossings() {
        // MicrosIsr(0) between crossings → Some(MicrosIsr(0)) is valid, not suppressed.
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(2.0));
        t.update(mt(5.0), MicrosIsr(0)); // init
        t.update(mt(-5.0), MicrosIsr(0)); // crossing #1 → None
        let ipi = t.update(mt(5.0), MicrosIsr(0)); // crossing #2
        assert_eq!(
            ipi,
            Some(MicrosIsr(0)),
            "zero-elapsed IPI must not be suppressed"
        );
    }

    #[test]
    fn zc_ipi_saturates_at_u32_max() {
        // Accumulate large elapsed BETWEEN crossings to trigger saturation.
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(2.0));
        t.update(mt(5.0), MicrosIsr(0)); // init
        t.update(mt(-5.0), MicrosIsr(100)); // crossing #1 → None, ipi resets
        t.update(mt(0.0), MicrosIsr(u32::MAX - 10)); // no crossing, ipi accumulates
        let ipi = t.update(mt(5.0), MicrosIsr(20)); // crossing #2, would overflow
        assert_eq!(
            ipi,
            Some(MicrosIsr(u32::MAX)),
            "IPI must saturate at u32::MAX, not wrap"
        );
    }

    #[test]
    fn zc_ipi_reset_makes_next_crossing_first() {
        // After reset(), the next crossing is treated as "first" → returns None.
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(2.0));
        t.update(mt(5.0), MicrosIsr(0));
        t.update(mt(-5.0), MicrosIsr(100)); // crossing #1
        t.update(mt(5.0), MicrosIsr(200)); // crossing #2 → Some

        t.reset();

        t.update(mt(5.0), MicrosIsr(0)); // re-init HIGH
        let ipi = t.update(mt(-5.0), MicrosIsr(500)); // first crossing after reset
        assert!(ipi.is_none(), "first crossing after reset must return None");
        assert_eq!(t.crossings(), Crossings(1));
    }

    #[test]
    fn zc_ipi_raw_u32_passthrough() {
        // IPI .0 yields raw u32 suitable for coffiot-core's ipis_us buffer.
        let mut t = RotationTracker::<4, ZeroCrossing>::new(MilliTesla(2.0));
        t.update(mt(5.0), MicrosIsr(0));
        t.update(mt(-5.0), MicrosIsr(1000)); // crossing #1
        let ipi = t.update(mt(5.0), MicrosIsr(2500)); // crossing #2
        let raw: u32 = ipi.expect("crossing #2 should emit IPI").0;
        assert_eq!(raw, 2500_u32);
    }

    // -----------------------------------------------------------------------
    // Struct size assertions (runtime view of compile-time consts above)
    // -----------------------------------------------------------------------

    #[test]
    fn size_of_cordic_tracker_within_budget() {
        assert!(
            size_of::<RotationTracker<2, Cordic>>() <= 24,
            "Cordic tracker size {} exceeds 24-byte budget",
            size_of::<RotationTracker<2, Cordic>>()
        );
    }

    #[test]
    fn size_of_zero_crossing_tracker_within_budget() {
        assert!(
            size_of::<RotationTracker<4, ZeroCrossing>>() <= 24,
            "ZeroCrossing tracker size {} exceeds 24-byte budget",
            size_of::<RotationTracker<4, ZeroCrossing>>()
        );
    }
}
