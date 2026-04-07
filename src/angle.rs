//! High-level computational extensions (Layer 3 API).
//!
//! This module is the home for feature-gated analysis utilities that build on
//! the core sensor types defined in [`types`](crate::types). The module is
//! gated by `#[cfg(feature = "libm")]` in `lib.rs` — it is only compiled
//! when the `libm` feature is enabled.

use crate::MagneticReading;

use crate::types::{AngleEnable, Degrees, MagneticChannel, MilliTesla, SignedDegrees};

impl MagneticReading {
    /// Software-computed 3-axis magnitude: `sqrt(x² + y² + z²)`.
    ///
    /// Returns `None` unless all three axes are present. If you need the
    /// magnitude from fewer axes, compute it yourself from the fields you
    /// have. Requires the `libm` feature.
    ///
    /// For the hardware-computed 2-axis magnitude from the CORDIC engine,
    /// use [`AngleReading::magnitude`] instead.
    ///
    /// # Examples
    ///
    /// *Requires the `libm` Cargo feature.*
    ///
    /// ```rust
    /// use tmag5273::{MagneticReading, MilliTesla};
    ///
    /// let reading = MagneticReading {
    ///     x: Some(MilliTesla(3.0)),
    ///     y: Some(MilliTesla(4.0)),
    ///     z: Some(MilliTesla(0.0)),
    /// };
    /// let mag = reading.magnitude_3d().unwrap();
    /// assert!((mag.0 - 5.0).abs() < 0.01); // 3-4-5 triangle
    /// ```
    #[inline]
    pub fn magnitude_3d(&self) -> Option<MilliTesla> {
        match (self.x, self.y, self.z) {
            (Some(x), Some(y), Some(z)) => {
                Some(MilliTesla(libm::sqrtf(x.0 * x.0 + y.0 * y.0 + z.0 * z.0)))
            }
            _ => {
                if self.x.is_none() {
                    defmt_debug!("magnitude_3d error: X axis disabled");
                }
                if self.y.is_none() {
                    defmt_debug!("magnitude_3d error: Y axis disabled");
                }
                if self.z.is_none() {
                    defmt_debug!("magnitude_3d error: Z axis disabled");
                }
                None
            }
        }
    }

    /// Compute angle and magnitude for all three canonical planes.
    ///
    /// Each plane requires both constituent axes to be present (`Some`)
    /// **and** not both zero. Missing or zero-magnitude planes are `None`.
    ///
    /// Axis ordering per plane matches the TMAG5273 hardware CORDIC
    /// convention (datasheet §8.1.4, `ANGLE_EN` field):
    /// - XY: `atan2(Y, X)` — `ANGLE_EN = 01b`, X 1st Y 2nd
    /// - YZ: `atan2(Z, Y)` — `ANGLE_EN = 10b`, Y 1st Z 2nd
    /// - XZ: `atan2(Z, X)` — `ANGLE_EN = 11b`, X 1st Z 2nd
    ///
    /// Use [`.dominant()`](PlaneAngles::dominant) on the result to get the
    /// plane with the strongest signal.
    pub fn plane_angles(&self) -> PlaneAngles {
        if self.x.is_none() {
            defmt_debug!("plane_angles: X axis disabled");
        }
        if self.y.is_none() {
            defmt_debug!("plane_angles: Y axis disabled");
        }
        if self.z.is_none() {
            defmt_debug!("plane_angles: Z axis disabled");
        }

        let axes = [self.x, self.y, self.z];
        // (1st_axis_index, 2nd_axis_index, plane) — axis ordering matches
        // datasheet §8.1.4 ANGLE_EN: atan2(2nd, 1st).
        const PLANES: [(usize, usize, PlaneAxis); 3] = [
            (0, 1, PlaneAxis::XY),
            (1, 2, PlaneAxis::YZ),
            (0, 2, PlaneAxis::XZ),
        ];
        let [xy, yz, xz] = PLANES.map(|(i, j, plane)| match (axes[i], axes[j]) {
            (Some(axis_one), Some(axis_two)) => {
                PlaneAngle::from_axes(axis_one.0, axis_two.0, plane)
            }
            _ => None,
        });

        PlaneAngles { xy, yz, xz }
    }
}

/// Maps each plane to its **default** pseudo-simultaneous channel.
///
/// `XY → XYX`, `YZ → YZY`, `XZ → XZX`. This is a convenience for
/// users who do not use the calibrator. [`AxisCalibrator::recommend`]
/// constructs `MagneticChannel` directly and may return `YXY` instead
/// of `XYX` for the XY plane based on per-axis variance.
impl From<PlaneAxis> for MagneticChannel {
    #[inline]
    fn from(plane: PlaneAxis) -> Self {
        match plane {
            PlaneAxis::XY => Self::XYX,
            PlaneAxis::YZ => Self::YZY,
            PlaneAxis::XZ => Self::XZX,
        }
    }
}

/// Canonical measurement plane for 2-axis angle computation.
///
/// Identifies which pair of sensor axes defines the plane. This is a
/// software-only concept — no corresponding register field exists.
/// Use the `From` conversions to obtain the matching [`AngleEnable`]
/// or [`MagneticChannel`] for hardware configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PlaneAxis {
    /// X-Y plane.
    XY = 0,
    /// Y-Z plane.
    YZ = 1,
    /// X-Z plane.
    XZ = 2,
}

impl_try_from_u8!(PlaneAxis {
    0 => XY,
    1 => YZ,
    2 => XZ,
});

impl From<PlaneAxis> for AngleEnable {
    #[inline]
    fn from(plane: PlaneAxis) -> Self {
        match plane {
            PlaneAxis::XY => Self::XY,
            PlaneAxis::YZ => Self::YZ,
            PlaneAxis::XZ => Self::XZ,
        }
    }
}

/// Angle and magnitude for a single canonical plane.
///
/// Produced by [`MagneticReading::plane_angles`]. The `angle` uses the
/// same axis ordering as the TMAG5273 hardware CORDIC (datasheet §8.1.4):
/// `atan2(2nd_axis, 1st_axis)`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[must_use]
pub struct PlaneAngle {
    /// Angle in degrees (0–360), matching hardware CORDIC convention.
    pub angle: Degrees,
    /// 2-axis magnitude: `sqrt(axis1² + axis2²)`.
    pub magnitude: MilliTesla,
    /// Which plane this measurement belongs to.
    pub plane: PlaneAxis,
}

impl PlaneAngle {
    /// Compute from two axis values in mT.
    ///
    /// Axis ordering matches the TMAG5273 hardware CORDIC convention
    /// (datasheet §8.1.4): `atan2(2nd_axis, 1st_axis)`.
    ///
    /// Returns `None` if both axes are exactly zero (angle is physically
    /// undefined with no field vector).
    fn from_axes(first: f32, second: f32, plane: PlaneAxis) -> Option<Self> {
        if first == 0.0 && second == 0.0 {
            return None;
        }
        let magnitude = MilliTesla(libm::sqrtf(first * first + second * second));
        // atan2f returns radians in (-π, π]; convert to degrees in [0, 360).
        let mut angle = Degrees(libm::atan2f(second, first) * (180.0 / core::f32::consts::PI));
        if angle < Degrees::MIN {
            angle += SignedDegrees::MAX;
        }
        Some(Self {
            angle,
            magnitude,
            plane,
        })
    }
}

/// Angle and magnitude results for all three canonical planes.
///
/// Produced by [`MagneticReading::plane_angles`]. Each field is `None`
/// when the required axes are missing or both are exactly zero (angle
/// is physically undefined with no field vector).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[must_use]
pub struct PlaneAngles {
    /// X-Y plane result (requires both X and Y axes).
    pub xy: Option<PlaneAngle>,
    /// Y-Z plane result (requires both Y and Z axes).
    pub yz: Option<PlaneAngle>,
    /// X-Z plane result (requires both X and Z axes).
    pub xz: Option<PlaneAngle>,
}

// let angle = all_angles[PlaneAxis::XY];
impl core::ops::Index<PlaneAxis> for PlaneAngles {
    type Output = Option<PlaneAngle>;

    fn index(&self, axis: PlaneAxis) -> &Self::Output {
        match axis {
            PlaneAxis::XY => &self.xy,
            PlaneAxis::YZ => &self.yz,
            PlaneAxis::XZ => &self.xz,
        }
    }
}

impl PlaneAngles {
    /// Returns the plane with the largest 2-axis magnitude.
    ///
    /// Comparison uses the magnitude value directly — `sqrt` is monotonic
    /// so this is equivalent to sum-of-squares ordering.
    /// Tie-breaking: XY > YZ > XZ (first in field order wins).
    ///
    /// Returns `None` if all three planes are `None`.
    pub fn dominant(&self) -> Option<PlaneAngle> {
        [self.xy, self.yz, self.xz]
            .into_iter()
            .flatten()
            .reduce(|best, candidate| {
                if best.magnitude >= candidate.magnitude {
                    best
                } else {
                    candidate
                }
            })
    }
}

/// Welford's online algorithm for numerically stable mean and variance.
///
/// Accumulates running statistics from a stream of `f32` values with
/// zero heap allocation. Numerically stable even with DC-biased inputs
/// where the naive `(Σx²/n) − (Σx/n)²` formula suffers from
/// catastrophic cancellation on `f32`.
///
/// # Example
///
/// ```
/// use tmag5273::Welford;
///
/// let mut w = Welford::default();
/// for v in [10.0, 10.1, 9.9, 10.0, 10.2] {
///     w.update(v);
/// }
/// assert!(w.mean().is_some());
/// assert!(w.variance().is_some());
/// ```
///
/// See [Welford's online algorithm](https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm).
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Welford {
    count: u32,
    mean: f32,
    mean2: f32,
}

impl Welford {
    /// Feed a new sample. Non-finite values (`NaN`, `±Inf`) are silently
    /// rejected to protect the running accumulators.
    pub fn update(&mut self, value: f32) {
        // Reject non-finite input — NaN/Inf would permanently corrupt
        // the running mean and mean2 accumulators.
        if !value.is_finite() {
            defmt_debug!("Welford:update: value is not finite");
            return;
        }
        // Saturating prevents wrap-to-zero in release builds (~19 days at 2500 Hz).
        // Once capped, further updates negligibly affect mean/variance.
        self.count = self.count.saturating_add(1);
        let delta = value - self.mean;
        self.mean += delta / (self.count as f32);
        let delta2 = value - self.mean;
        self.mean2 += delta * delta2;
    }

    /// Number of finite samples accumulated so far.
    pub fn count(&self) -> u32 {
        self.count
    }

    /// Running mean. Returns `None` if fewer than 2 samples.
    pub fn mean(&self) -> Option<f32> {
        if self.count < 2 {
            return None;
        }
        Some(self.mean)
    }

    /// Population variance (÷N). Returns `None` if fewer than 2 samples.
    pub fn variance(&self) -> Option<f32> {
        if self.count < 2 {
            return None;
        }
        Some(self.mean2 / (self.count as f32))
    }

    /// Sample variance (÷(N−1)). Returns `None` if fewer than 2 samples.
    pub fn sample_variance(&self) -> Option<f32> {
        if self.count < 2 {
            return None;
        }
        Some(self.mean2 / (self.count - 1) as f32)
    }

    /// Population standard deviation. Returns `None` if fewer than 2 samples.
    pub fn stddev(&self) -> Option<f32> {
        self.variance().map(libm::sqrtf)
    }

    /// Sample standard deviation (÷√(N−1)). Returns `None` if fewer than 2 samples.
    pub fn sample_stddev(&self) -> Option<f32> {
        self.sample_variance().map(libm::sqrtf)
    }

    /// Absolute mean value. Returns `None` if fewer than 2 samples.
    /// Used by [`AxisCalibrator::recommend_static`] to rank axes by
    /// DC signal strength rather than variance.
    pub fn abs_mean(&self) -> Option<f32> {
        if self.count < 2 {
            return None;
        }
        Some(libm::fabsf(self.mean))
    }
}

/// Accumulates per-axis running statistics from [`MagneticReading`] samples
/// and recommends the optimal hardware angle configuration.
///
/// # Usage
///
/// 1. Create with [`AxisCalibrator::default()`]
/// 2. Feed samples with [`update()`](AxisCalibrator::update)
/// 3. Call [`recommend()`](AxisCalibrator::recommend) (rotating magnet) or
///    [`recommend_static()`](AxisCalibrator::recommend_static) (stationary magnet)
/// 4. Pass the recommendation to [`ConfigBuilder`](crate::ConfigBuilder)
///
/// ## Rotating magnets — [`recommend()`](AxisCalibrator::recommend)
///
/// Ranks axes by **variance**. The rotation plane axes have sinusoidal
/// signals producing high variance; the perpendicular axis stays near
/// constant. **Nyquist requirement:** sample at ≥ 10× the electrical
/// rotation frequency for clean results.
///
/// ## Static / DC fields — [`recommend_static()`](AxisCalibrator::recommend_static)
///
/// Ranks axes by **absolute mean**. All axes have near-equal ADC noise
/// variance under a static field, but mean magnitudes differ by orders
/// of magnitude. Use this when the magnet is stationary.
#[derive(Debug, Default, Clone)]
pub struct AxisCalibrator {
    /// Per-axis accumulators: [X, Y, Z].
    stats: [Welford; 3],
}

impl AxisCalibrator {
    /// Minimum samples per axis before [`recommend()`](Self::recommend)
    /// can produce a result.
    pub const MIN_CALIBRATION_SAMPLES: u32 = 8;

    /// Ingest one magnetic reading. Axes that are `None` are skipped
    /// without error — partial channel configurations are handled
    /// gracefully.
    pub fn update(&mut self, reading: &MagneticReading) {
        for (stat, axis) in self.stats.iter_mut().zip([reading.x, reading.y, reading.z]) {
            if let Some(mt) = axis {
                stat.update(mt.0);
            }
        }
    }

    /// Recommend the optimal CORDIC plane for a **rotating** magnet based
    /// on per-axis variance. The two axes with the highest variance are
    /// selected as the measurement plane.
    ///
    /// Returns `None` if fewer than 2 axes have at least
    /// [`MIN_CALIBRATION_SAMPLES`](Self::MIN_CALIBRATION_SAMPLES) samples each.
    ///
    /// See [`recommend_static()`](Self::recommend_static) for static/DC fields.
    pub fn recommend(&self) -> Option<AxisRecommendation> {
        self.recommend_inner(Welford::variance)
    }

    /// Recommend the optimal CORDIC plane for a **static (non-rotating)**
    /// magnetic field based on per-axis signal strength (`|mean|`).
    ///
    /// In a static field all axes have near-equal ADC-noise variance, but
    /// the mean magnitudes differ by orders of magnitude, making absolute
    /// mean the correct discriminator.
    ///
    /// Returns `None` under the same conditions as [`recommend()`](Self::recommend).
    pub fn recommend_static(&self) -> Option<AxisRecommendation> {
        self.recommend_inner(Welford::abs_mean)
    }

    /// Shared implementation: qualify axes, score with `scorer`, sort,
    /// derive plane + channel, and populate both `variances` and `abs_means`.
    fn recommend_inner(&self, scorer: fn(&Welford) -> Option<f32>) -> Option<AxisRecommendation> {
        // Count axes with enough samples.
        let qualifying = self
            .stats
            .iter()
            .filter(|s| s.count >= Self::MIN_CALIBRATION_SAMPLES)
            .count();
        if qualifying < 2 {
            return None;
        }

        // Score each axis using the provided scorer function.
        let scores: [Option<f32>; 3] = core::array::from_fn(|i| {
            if self.stats[i].count >= Self::MIN_CALIBRATION_SAMPLES {
                scorer(&self.stats[i])
            } else {
                defmt_debug!(
                    "recommend: axis {} has {} samples (< {}), skipping",
                    i,
                    self.stats[i].count,
                    Self::MIN_CALIBRATION_SAMPLES
                );
                None
            }
        });

        // Build (score, axis_index) pairs for sorting.
        // Tie-breaking: lower index wins (X=0 > Y=1 > Z=2) — achieved
        // by using a stable sort where equal scores preserve order.
        let mut ranked: [(Option<f32>, u8); 3] = core::array::from_fn(|i| (scores[i], i as u8));

        // Sort descending by score. None sorts below any Some.
        // Manual stable sort for 3 elements (no alloc):
        if ranked[1].0 > ranked[0].0 {
            ranked.swap(0, 1);
        }
        if ranked[2].0 > ranked[1].0 {
            ranked.swap(1, 2);
        }
        if ranked[1].0 > ranked[0].0 {
            ranked.swap(0, 1);
        }

        // Two highest-scoring axes → plane.
        let (Some(_), a) = ranked[0] else {
            defmt_debug!("recommend: top-ranked axis has no score");
            return None;
        };
        let (Some(_), b) = ranked[1] else {
            defmt_debug!("recommend: second-ranked axis has no score");
            return None;
        };
        let axes = if a <= b { (a, b) } else { (b, a) };
        let plane = match axes {
            (0, 1) => PlaneAxis::XY,
            (1, 2) => PlaneAxis::YZ,
            (0, 2) => PlaneAxis::XZ,
            _ => unreachable!(),
        };

        let angle_enable = AngleEnable::from(plane);

        // Smart channel selection: for XY, pick the variant that
        // double-samples the higher-scoring axis.
        let channel = match plane {
            PlaneAxis::XY => {
                let x_score = scores[0].expect("X axis qualified but has no score");
                let y_score = scores[1].expect("Y axis qualified but has no score");
                if x_score >= y_score {
                    MagneticChannel::XYX
                } else {
                    MagneticChannel::YXY
                }
            }
            PlaneAxis::YZ => MagneticChannel::YZY,
            PlaneAxis::XZ => MagneticChannel::XZX,
        };

        // Always populate both variances and abs_means for transparency.
        let variances: [Option<f32>; 3] = core::array::from_fn(|i| {
            if self.stats[i].count >= Self::MIN_CALIBRATION_SAMPLES {
                self.stats[i].variance()
            } else {
                None
            }
        });
        let abs_means: [Option<f32>; 3] = core::array::from_fn(|i| {
            if self.stats[i].count >= Self::MIN_CALIBRATION_SAMPLES {
                self.stats[i].abs_mean()
            } else {
                None
            }
        });

        Some(AxisRecommendation {
            plane,
            angle_enable,
            magnetic_channel: channel,
            variances,
            abs_means,
        })
    }
}

/// Recommended hardware angle configuration from [`AxisCalibrator::recommend`].
///
/// Contains the optimal [`AngleEnable`] + [`MagneticChannel`] pair and
/// per-axis variance for transparency.
///
/// **Important:** When `angle_enable` is [`AngleEnable::YZ`] or
/// [`AngleEnable::XZ`], [`ConfigBuilder::build()`](crate::ConfigBuilder::build)
/// requires matching XY and Z ranges — set both to the same [`Range`](crate::Range)
/// or [`ConfigError::AngleMixedRanges`](crate::ConfigError::AngleMixedRanges)
/// will be returned.
///
/// Pseudo-simultaneous modes (XYX, YXY, YZY, XZX) use 3 measurement slots
/// instead of 2, increasing conversion time by ~50%. This may require
/// adjusting [`SleepTime`](crate::SleepTime) in wake-up-and-sleep mode.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[must_use]
pub struct AxisRecommendation {
    /// The selected measurement plane.
    pub plane: PlaneAxis,
    /// Hardware angle calculation mode for [`ConfigBuilder::angle_enabled`](crate::ConfigBuilder::angle_enabled).
    pub angle_enable: AngleEnable,
    /// Pseudo-simultaneous channel for [`ConfigBuilder::magnetic_channels_enabled`](crate::ConfigBuilder::magnetic_channels_enabled).
    pub magnetic_channel: MagneticChannel,
    /// Per-axis population variance in mT²: `[X, Y, Z]`.
    /// `None` for axes with fewer than [`AxisCalibrator::MIN_CALIBRATION_SAMPLES`] samples.
    /// `Some(v)` means the axis both qualified and contributed to the recommendation.
    pub variances: [Option<f32>; 3],
    /// Per-axis absolute mean field strength in mT: `[X, Y, Z]`.
    /// `None` for axes with fewer than [`AxisCalibrator::MIN_CALIBRATION_SAMPLES`] samples.
    /// Populated by both [`AxisCalibrator::recommend`] and
    /// [`AxisCalibrator::recommend_static`] for transparency.
    pub abs_means: [Option<f32>; 3],
}

#[cfg(test)]
mod tests {
    use super::*;

    // -- MagneticReading::magnitude edge cases (requires `libm` feature) -----

    #[test]
    fn magnitude_3d_all_none_returns_none() {
        let r = MagneticReading {
            x: None,
            y: None,
            z: None,
        };
        assert!(r.magnitude_3d().is_none());
    }

    #[test]
    fn magnitude_3d_partial_axes_returns_none() {
        let r = MagneticReading {
            x: Some(MilliTesla(5.0)),
            y: None,
            z: None,
        };
        assert!(r.magnitude_3d().is_none());

        let r = MagneticReading {
            x: Some(MilliTesla(-3.0)),
            y: Some(MilliTesla(-4.0)),
            z: None,
        };
        assert!(r.magnitude_3d().is_none());
    }

    #[test]
    fn magnitude_3d_all_axes() {
        let r = MagneticReading {
            x: Some(MilliTesla(1.0)),
            y: Some(MilliTesla(2.0)),
            z: Some(MilliTesla(2.0)),
        };
        // sqrt(1 + 4 + 4) = 3
        assert!((r.magnitude_3d().unwrap().0 - 3.0).abs() < 0.01);
    }

    #[test]
    fn magnitude_3d_zero_fields() {
        let r = MagneticReading {
            x: Some(MilliTesla(0.0)),
            y: Some(MilliTesla(0.0)),
            z: Some(MilliTesla(0.0)),
        };
        assert_eq!(r.magnitude_3d().unwrap().0, 0.0);
    }

    #[test]
    fn magnitude_3d_negative_fields() {
        let r = MagneticReading {
            x: Some(MilliTesla(-3.0)),
            y: Some(MilliTesla(-4.0)),
            z: Some(MilliTesla(0.0)),
        };
        assert!((r.magnitude_3d().unwrap().0 - 5.0).abs() < 0.01);
    }

    // -- plane_angles() -------------------------------------------------------

    #[test]
    fn plane_angles_3_4_triangle() {
        let r = MagneticReading {
            x: Some(MilliTesla(3.0)),
            y: Some(MilliTesla(4.0)),
            z: Some(MilliTesla(0.0)),
        };
        let pa = r.plane_angles();
        let xy = pa.xy.unwrap();
        // atan2(4, 3) ≈ 53.13°
        assert!((xy.angle.0 - 53.13).abs() < 0.01);
        assert!((xy.magnitude.0 - 5.0).abs() < 0.01);
        assert_eq!(xy.plane, PlaneAxis::XY);
    }

    #[test]
    fn plane_angles_all_three_present() {
        let r = MagneticReading {
            x: Some(MilliTesla(1.0)),
            y: Some(MilliTesla(2.0)),
            z: Some(MilliTesla(3.0)),
        };
        let pa = r.plane_angles();
        assert!(pa.xy.is_some());
        assert!(pa.yz.is_some());
        assert!(pa.xz.is_some());
    }

    #[test]
    fn plane_angles_only_xy_present() {
        let r = MagneticReading {
            x: Some(MilliTesla(1.0)),
            y: Some(MilliTesla(1.0)),
            z: None,
        };
        let pa = r.plane_angles();
        assert!(pa.xy.is_some());
        assert!(pa.yz.is_none());
        assert!(pa.xz.is_none());
    }

    #[test]
    fn plane_angles_only_yz_present() {
        let r = MagneticReading {
            x: None,
            y: Some(MilliTesla(1.0)),
            z: Some(MilliTesla(1.0)),
        };
        let pa = r.plane_angles();
        assert!(pa.xy.is_none());
        assert!(pa.yz.is_some());
        assert!(pa.xz.is_none());
    }

    #[test]
    fn plane_angles_single_axis_all_none() {
        let r = MagneticReading {
            x: Some(MilliTesla(5.0)),
            y: None,
            z: None,
        };
        let pa = r.plane_angles();
        assert!(pa.xy.is_none());
        assert!(pa.yz.is_none());
        assert!(pa.xz.is_none());
    }

    #[test]
    fn plane_angles_all_axes_none() {
        let r = MagneticReading {
            x: None,
            y: None,
            z: None,
        };
        let pa = r.plane_angles();
        assert!(pa.xy.is_none());
        assert!(pa.yz.is_none());
        assert!(pa.xz.is_none());
    }

    #[test]
    fn plane_angles_zero_magnitude_returns_none() {
        let r = MagneticReading {
            x: Some(MilliTesla(0.0)),
            y: Some(MilliTesla(0.0)),
            z: Some(MilliTesla(0.0)),
        };
        let pa = r.plane_angles();
        assert!(pa.xy.is_none());
        assert!(pa.yz.is_none());
        assert!(pa.xz.is_none());
    }

    #[test]
    fn plane_angles_one_axis_zero_other_nonzero() {
        // X=3, Y=0 → XY angle = atan2(0, 3) = 0°, magnitude = 3.0
        let r = MagneticReading {
            x: Some(MilliTesla(3.0)),
            y: Some(MilliTesla(0.0)),
            z: None,
        };
        let xy = r.plane_angles().xy.unwrap();
        assert!((xy.angle.0 - 0.0).abs() < 0.01);
        assert!((xy.magnitude.0 - 3.0).abs() < 0.01);
    }

    #[test]
    fn plane_angles_negative_values_correct_quadrant() {
        // Q2: X=-3, Y=4 → atan2(4, -3) ≈ 126.87°
        let r = MagneticReading {
            x: Some(MilliTesla(-3.0)),
            y: Some(MilliTesla(4.0)),
            z: None,
        };
        let xy = r.plane_angles().xy.unwrap();
        assert!((xy.angle.0 - 126.87).abs() < 0.01);

        // Q3: X=-3, Y=-4 → atan2(-4, -3) ≈ 233.13°
        let r = MagneticReading {
            x: Some(MilliTesla(-3.0)),
            y: Some(MilliTesla(-4.0)),
            z: None,
        };
        let xy = r.plane_angles().xy.unwrap();
        assert!((xy.angle.0 - 233.13).abs() < 0.01);

        // Q4: X=3, Y=-4 → atan2(-4, 3) ≈ 306.87°
        let r = MagneticReading {
            x: Some(MilliTesla(3.0)),
            y: Some(MilliTesla(-4.0)),
            z: None,
        };
        let xy = r.plane_angles().xy.unwrap();
        assert!((xy.angle.0 - 306.87).abs() < 0.01);
    }

    #[test]
    fn plane_angles_range_0_360() {
        // Verify negative atan2 result normalizes to 0-360
        // X=0, Y=-1 → atan2(-1, 0) = -90° → normalized to 270°
        let r = MagneticReading {
            x: Some(MilliTesla(0.0)),
            y: Some(MilliTesla(-1.0)),
            z: None,
        };
        let xy = r.plane_angles().xy.unwrap();
        assert!((xy.angle.0 - 270.0).abs() < 0.01);
    }

    #[test]
    fn plane_angles_datasheet_parity_xy_atan2_y_x() {
        // TMAG5273 CORDIC for XY: ANGLE_EN=01b, "X 1st Y 2nd"
        // = atan2(Y, X). Verify our software matches.
        // X=3, Y=4 → atan2(4, 3) ≈ 53.1301°
        let r = MagneticReading {
            x: Some(MilliTesla(3.0)),
            y: Some(MilliTesla(4.0)),
            z: None,
        };
        let xy = r.plane_angles().xy.unwrap();
        let expected = libm::atan2f(4.0, 3.0) * (180.0 / core::f32::consts::PI);
        assert!((xy.angle.0 - expected).abs() < 0.01);
    }

    #[test]
    fn plane_angles_datasheet_parity_yz_atan2_z_y() {
        // TMAG5273 CORDIC for YZ: ANGLE_EN=10b, "Y 1st Z 2nd"
        // = atan2(Z, Y). Y=3, Z=4 → atan2(4, 3) ≈ 53.1301°
        let r = MagneticReading {
            x: None,
            y: Some(MilliTesla(3.0)),
            z: Some(MilliTesla(4.0)),
        };
        let yz = r.plane_angles().yz.unwrap();
        let expected = libm::atan2f(4.0, 3.0) * (180.0 / core::f32::consts::PI);
        assert!((yz.angle.0 - expected).abs() < 0.01);
        assert_eq!(yz.plane, PlaneAxis::YZ);
    }

    #[test]
    fn plane_angles_datasheet_parity_xz_atan2_z_x() {
        // TMAG5273 CORDIC for XZ: ANGLE_EN=11b, "X 1st Z 2nd"
        // = atan2(Z, X). X=3, Z=4 → atan2(4, 3) ≈ 53.1301°
        let r = MagneticReading {
            x: Some(MilliTesla(3.0)),
            y: None,
            z: Some(MilliTesla(4.0)),
        };
        let xz = r.plane_angles().xz.unwrap();
        let expected = libm::atan2f(4.0, 3.0) * (180.0 / core::f32::consts::PI);
        assert!((xz.angle.0 - expected).abs() < 0.01);
        assert_eq!(xz.plane, PlaneAxis::XZ);
    }

    // -- dominant() -----------------------------------------------------------

    #[test]
    fn dominant_picks_largest_magnitude() {
        let r = MagneticReading {
            x: Some(MilliTesla(1.0)),
            y: Some(MilliTesla(2.0)),
            z: Some(MilliTesla(10.0)),
        };
        let d = r.plane_angles().dominant().unwrap();
        // YZ has magnitude sqrt(4+100) ≈ 10.20, largest
        assert_eq!(d.plane, PlaneAxis::YZ);
    }

    #[test]
    fn dominant_single_plane_returns_it() {
        let r = MagneticReading {
            x: Some(MilliTesla(3.0)),
            y: Some(MilliTesla(4.0)),
            z: None,
        };
        let d = r.plane_angles().dominant().unwrap();
        assert_eq!(d.plane, PlaneAxis::XY);
    }

    #[test]
    fn dominant_equal_magnitude_prefers_xy() {
        // All axes equal → all planes have same magnitude → XY wins (first)
        let r = MagneticReading {
            x: Some(MilliTesla(1.0)),
            y: Some(MilliTesla(1.0)),
            z: Some(MilliTesla(1.0)),
        };
        let d = r.plane_angles().dominant().unwrap();
        assert_eq!(d.plane, PlaneAxis::XY);
    }

    #[test]
    fn dominant_all_none_returns_none() {
        let r = MagneticReading {
            x: None,
            y: None,
            z: None,
        };
        assert!(r.plane_angles().dominant().is_none());
    }

    // -- AxisCalibrator -------------------------------------------------------

    #[test]
    fn calibrator_known_values() {
        let mut cal = AxisCalibrator::default();
        for i in 0..10 {
            cal.update(&MagneticReading {
                x: Some(MilliTesla(i as f32)),
                y: Some(MilliTesla(10.0)),
                z: Some(MilliTesla(-5.0)),
            });
        }
        assert_eq!(cal.stats[0].count, 10);
        assert_eq!(cal.stats[1].count, 10);
        assert_eq!(cal.stats[2].count, 10);
        // X: 0..9 → mean = 4.5
        assert!((cal.stats[0].mean - 4.5).abs() < 1e-6);
        // Y: constant 10.0 → variance ≈ 0
        assert!((cal.stats[1].variance().unwrap()).abs() < 1e-6);
    }

    #[test]
    fn calibrator_skips_none_axes() {
        let mut cal = AxisCalibrator::default();
        for _ in 0..5 {
            cal.update(&MagneticReading {
                x: Some(MilliTesla(1.0)),
                y: None,
                z: Some(MilliTesla(2.0)),
            });
        }
        assert_eq!(cal.stats[0].count, 5);
        assert_eq!(cal.stats[1].count, 0); // Y never updated
        assert_eq!(cal.stats[2].count, 5);
    }

    #[test]
    fn calibrator_single_sample_variance_none() {
        let mut cal = AxisCalibrator::default();
        cal.update(&MagneticReading {
            x: Some(MilliTesla(5.0)),
            y: Some(MilliTesla(5.0)),
            z: Some(MilliTesla(5.0)),
        });
        assert!(cal.stats[0].variance().is_none());
    }

    #[test]
    fn calibrator_two_identical_samples_zero_variance() {
        let mut cal = AxisCalibrator::default();
        for _ in 0..2 {
            cal.update(&MagneticReading {
                x: Some(MilliTesla(3.0)),
                y: None,
                z: None,
            });
        }
        assert!((cal.stats[0].variance().unwrap() - 0.0).abs() < 1e-6);
    }

    #[test]
    fn calibrator_all_none_counts_zero() {
        let mut cal = AxisCalibrator::default();
        for _ in 0..10 {
            cal.update(&MagneticReading {
                x: None,
                y: None,
                z: None,
            });
        }
        assert_eq!(cal.stats[0].count, 0);
        assert_eq!(cal.stats[1].count, 0);
        assert_eq!(cal.stats[2].count, 0);
    }

    #[test]
    fn calibrator_welford_known_variance() {
        // [1.0, 3.0] → mean = 2.0, population variance = 1.0
        let mut cal = AxisCalibrator::default();
        cal.update(&MagneticReading {
            x: Some(MilliTesla(1.0)),
            y: None,
            z: None,
        });
        cal.update(&MagneticReading {
            x: Some(MilliTesla(3.0)),
            y: None,
            z: None,
        });
        let var = cal.stats[0].variance().unwrap();
        assert!((var - 1.0).abs() < 1e-6);
    }

    #[test]
    fn calibrator_dc_biased_welford_stability() {
        // DC-biased: [50.0, 51.0, 49.0, 50.5]
        // mean = 50.125, population variance = 0.546875
        let mut cal = AxisCalibrator::default();
        for &v in &[50.0_f32, 51.0, 49.0, 50.5] {
            cal.update(&MagneticReading {
                x: Some(MilliTesla(v)),
                y: None,
                z: None,
            });
        }
        let var = cal.stats[0].variance().unwrap();
        assert!(
            (var - 0.546875).abs() < 1e-4,
            "DC-biased variance: expected ~0.546875, got {var}"
        );
    }

    #[test]
    fn welford_nan_input_ignored() {
        let mut cal = AxisCalibrator::default();
        // Feed 4 valid samples
        for v in [1.0, 2.0, 3.0, 4.0] {
            cal.update(&MagneticReading {
                x: Some(MilliTesla(v)),
                y: None,
                z: None,
            });
        }
        let count_before = cal.stats[0].count;
        let mean_before = cal.stats[0].mean;

        // NaN should be silently ignored — no state corruption.
        cal.update(&MagneticReading {
            x: Some(MilliTesla(f32::NAN)),
            y: None,
            z: None,
        });
        assert_eq!(cal.stats[0].count, count_before);
        assert_eq!(cal.stats[0].mean, mean_before);
    }

    #[test]
    fn welford_infinity_input_ignored() {
        let mut cal = AxisCalibrator::default();
        cal.update(&MagneticReading {
            x: Some(MilliTesla(1.0)),
            y: None,
            z: None,
        });
        let count_before = cal.stats[0].count;

        cal.update(&MagneticReading {
            x: Some(MilliTesla(f32::INFINITY)),
            y: None,
            z: None,
        });
        assert_eq!(cal.stats[0].count, count_before);
    }

    #[test]
    fn welford_count_saturates_at_max() {
        // Verify saturating_add prevents wrap-to-zero after u32::MAX samples.
        let mut w = Welford::default();
        // Feed 3 real samples to establish a meaningful mean
        w.update(10.0);
        w.update(12.0);
        w.update(11.0);
        assert_eq!(w.count, 3);

        // Manually set count to MAX - 1, then feed one more sample
        w.count = u32::MAX - 1;
        w.update(11.0);
        assert_eq!(w.count, u32::MAX, "count should saturate at u32::MAX");

        // One more update — count stays at MAX, accumulator doesn't corrupt
        w.update(11.0);
        assert_eq!(w.count, u32::MAX, "count stays saturated");
        assert!(w.mean.is_finite(), "mean must stay finite after saturation");
        assert!(
            w.mean2.is_finite(),
            "mean2 must stay finite after saturation"
        );

        // Variance should still be computable (not NaN/Inf)
        let v = w.variance();
        assert!(v.is_some(), "variance should be Some after saturation");
        assert!(v.unwrap().is_finite(), "variance must be finite");
    }

    // -- recommend() ----------------------------------------------------------

    fn feed_rotation(cal: &mut AxisCalibrator, samples: u32, x_amp: f32, y_amp: f32, z_const: f32) {
        for i in 0..samples {
            let t = (i as f32) / (samples as f32) * 2.0 * core::f32::consts::PI;
            cal.update(&MagneticReading {
                x: Some(MilliTesla(x_amp * libm::sinf(t))),
                y: Some(MilliTesla(y_amp * libm::cosf(t))),
                z: Some(MilliTesla(z_const)),
            });
        }
    }

    #[test]
    fn recommend_xy_rotation() {
        let mut cal = AxisCalibrator::default();
        // X oscillates ±10, Y oscillates ±5, Z constant
        feed_rotation(&mut cal, 16, 10.0, 5.0, 1.0);
        let rec = cal.recommend().unwrap();
        assert_eq!(rec.plane, PlaneAxis::XY);
        assert_eq!(rec.angle_enable, AngleEnable::XY);
        // X has higher variance → XYX
        assert_eq!(rec.magnetic_channel, MagneticChannel::XYX);
    }

    #[test]
    fn recommend_xy_y_higher_variance() {
        let mut cal = AxisCalibrator::default();
        // Y oscillates more than X
        feed_rotation(&mut cal, 16, 3.0, 10.0, 0.5);
        let rec = cal.recommend().unwrap();
        assert_eq!(rec.plane, PlaneAxis::XY);
        assert_eq!(rec.magnetic_channel, MagneticChannel::YXY);
    }

    #[test]
    fn recommend_yz_rotation() {
        let mut cal = AxisCalibrator::default();
        for i in 0..16 {
            let t = (i as f32) / 16.0 * 2.0 * core::f32::consts::PI;
            cal.update(&MagneticReading {
                x: Some(MilliTesla(0.1)),
                y: Some(MilliTesla(10.0 * libm::sinf(t))),
                z: Some(MilliTesla(8.0 * libm::cosf(t))),
            });
        }
        let rec = cal.recommend().unwrap();
        assert_eq!(rec.plane, PlaneAxis::YZ);
        assert_eq!(rec.magnetic_channel, MagneticChannel::YZY);
    }

    #[test]
    fn recommend_xz_rotation() {
        let mut cal = AxisCalibrator::default();
        for i in 0..16 {
            let t = (i as f32) / 16.0 * 2.0 * core::f32::consts::PI;
            cal.update(&MagneticReading {
                x: Some(MilliTesla(10.0 * libm::sinf(t))),
                y: Some(MilliTesla(0.1)),
                z: Some(MilliTesla(8.0 * libm::cosf(t))),
            });
        }
        let rec = cal.recommend().unwrap();
        assert_eq!(rec.plane, PlaneAxis::XZ);
        assert_eq!(rec.magnetic_channel, MagneticChannel::XZX);
    }

    #[test]
    fn recommend_variances_field_correct() {
        let mut cal = AxisCalibrator::default();
        feed_rotation(&mut cal, 16, 10.0, 5.0, 1.0);
        let rec = cal.recommend().unwrap();
        // All three variances should be Some (all axes had 16 samples)
        assert!(rec.variances[0].is_some());
        assert!(rec.variances[1].is_some());
        assert!(rec.variances[2].is_some());
        // X variance > Y variance > Z variance (Z is constant)
        assert!(rec.variances[0].unwrap() > rec.variances[1].unwrap());
        assert!(rec.variances[1].unwrap() > rec.variances[2].unwrap());
    }

    #[test]
    fn recommend_too_few_samples_returns_none() {
        let mut cal = AxisCalibrator::default();
        // Only 4 samples (below MIN_CALIBRATION_SAMPLES=8)
        feed_rotation(&mut cal, 4, 10.0, 5.0, 1.0);
        assert!(cal.recommend().is_none());
    }

    #[test]
    fn recommend_only_one_axis_returns_none() {
        let mut cal = AxisCalibrator::default();
        for i in 0..16 {
            cal.update(&MagneticReading {
                x: Some(MilliTesla(i as f32)),
                y: None,
                z: None,
            });
        }
        assert!(cal.recommend().is_none());
    }

    #[test]
    fn recommend_two_axes_sufficient() {
        let mut cal = AxisCalibrator::default();
        for i in 0..16 {
            let t = (i as f32) / 16.0 * 2.0 * core::f32::consts::PI;
            cal.update(&MagneticReading {
                x: Some(MilliTesla(10.0 * libm::sinf(t))),
                y: Some(MilliTesla(5.0 * libm::cosf(t))),
                z: None,
            });
        }
        let rec = cal.recommend().unwrap();
        assert_eq!(rec.plane, PlaneAxis::XY);
        assert!(rec.variances[2].is_none()); // Z never observed
    }

    #[test]
    fn recommend_subthreshold_axis_variance_is_none() {
        // Z gets 5 samples — enough for Welford::variance() to return Some
        // (threshold = 2), but below MIN_CALIBRATION_SAMPLES (8). The public
        // `variances` field must report None for it, not leak the raw Welford
        // output. Regression test for the "leaky threshold" fix.
        let mut cal = AxisCalibrator::default();
        for i in 0..16_u32 {
            let t = (i as f32) / 16.0 * 2.0 * core::f32::consts::PI;
            cal.update(&MagneticReading {
                x: Some(MilliTesla(10.0 * libm::sinf(t))),
                y: Some(MilliTesla(5.0 * libm::cosf(t))),
                z: if i < 5 {
                    Some(MilliTesla(i as f32))
                } else {
                    None
                },
            });
        }

        // Precondition: Z has enough samples for Welford but not for the
        // calibrator's qualification threshold.
        assert_eq!(cal.stats[2].count, 5);
        assert!(
            cal.stats[2].variance().is_some(),
            "Welford should return Some for 5 samples"
        );

        let rec = cal.recommend().unwrap();
        assert_eq!(rec.plane, PlaneAxis::XY);
        assert!(rec.variances[0].is_some()); // X qualified (16 samples)
        assert!(rec.variances[1].is_some()); // Y qualified (16 samples)
        assert!(
            rec.variances[2].is_none(),
            "Z has 5 samples (< MIN_CALIBRATION_SAMPLES=8): must be None, not Some"
        );
    }

    #[test]
    fn recommend_isotropic_noise_returns_xy() {
        // All axes equal variance → tie-breaking X>Y>Z → picks X and Y → XY
        let mut cal = AxisCalibrator::default();
        for i in 0..16 {
            let v = (i as f32) * 0.5;
            cal.update(&MagneticReading {
                x: Some(MilliTesla(v)),
                y: Some(MilliTesla(v)),
                z: Some(MilliTesla(v)),
            });
        }
        let rec = cal.recommend().unwrap();
        assert_eq!(rec.plane, PlaneAxis::XY);
    }

    #[test]
    fn recommend_zero_samples_returns_none() {
        let cal = AxisCalibrator::default();
        assert!(cal.recommend().is_none());
    }

    #[test]
    fn recommend_exactly_min_samples_succeeds() {
        let mut cal = AxisCalibrator::default();
        // Exactly MIN_CALIBRATION_SAMPLES (8) on two axes
        for i in 0..AxisCalibrator::MIN_CALIBRATION_SAMPLES {
            let t = (i as f32) / (AxisCalibrator::MIN_CALIBRATION_SAMPLES as f32)
                * 2.0
                * core::f32::consts::PI;
            cal.update(&MagneticReading {
                x: Some(MilliTesla(10.0 * libm::sinf(t))),
                y: Some(MilliTesla(5.0 * libm::cosf(t))),
                z: None,
            });
        }
        assert!(cal.recommend().is_some());
    }

    #[test]
    fn recommend_one_below_min_samples_returns_none() {
        let mut cal = AxisCalibrator::default();
        // One fewer than MIN_CALIBRATION_SAMPLES (7)
        for i in 0..(AxisCalibrator::MIN_CALIBRATION_SAMPLES - 1) {
            let t = (i as f32) / 7.0 * 2.0 * core::f32::consts::PI;
            cal.update(&MagneticReading {
                x: Some(MilliTesla(10.0 * libm::sinf(t))),
                y: Some(MilliTesla(5.0 * libm::cosf(t))),
                z: None,
            });
        }
        assert!(cal.recommend().is_none());
    }

    #[test]
    fn recommend_output_accepted_by_config_builder() {
        use crate::config::ConfigBuilder;
        use crate::types::Range;

        let mut cal = AxisCalibrator::default();
        feed_rotation(&mut cal, 16, 10.0, 5.0, 1.0);
        let rec = cal.recommend().unwrap();

        // XY recommendation — ranges don't need to match
        let result = ConfigBuilder::new()
            .angle_enabled(rec.angle_enable)
            .magnetic_channels_enabled(rec.magnetic_channel)
            .build();
        assert!(
            result.is_ok(),
            "ConfigBuilder rejected XY recommendation: {result:?}"
        );

        // Now test YZ — needs matching ranges
        let mut cal = AxisCalibrator::default();
        for i in 0..16 {
            let t = (i as f32) / 16.0 * 2.0 * core::f32::consts::PI;
            cal.update(&MagneticReading {
                x: Some(MilliTesla(0.1)),
                y: Some(MilliTesla(10.0 * libm::sinf(t))),
                z: Some(MilliTesla(8.0 * libm::cosf(t))),
            });
        }
        let rec = cal.recommend().unwrap();
        assert_eq!(rec.plane, PlaneAxis::YZ);
        let result = ConfigBuilder::new()
            .angle_enabled(rec.angle_enable)
            .magnetic_channels_enabled(rec.magnetic_channel)
            .xy_range(Range::High)
            .z_range(Range::High)
            .build();
        assert!(
            result.is_ok(),
            "ConfigBuilder rejected YZ recommendation: {result:?}"
        );
    }

    // -- recommend_static() ---------------------------------------------------

    #[test]
    fn recommend_static_picks_strongest_axes() {
        // Static field: Z=10 mT, X=2 mT, Y=1 mT → XZ plane
        let mut cal = AxisCalibrator::default();
        for _ in 0..16 {
            cal.update(&MagneticReading {
                x: Some(MilliTesla(2.0)),
                y: Some(MilliTesla(1.0)),
                z: Some(MilliTesla(10.0)),
            });
        }
        let rec = cal.recommend_static().unwrap();
        assert_eq!(rec.plane, PlaneAxis::XZ);
        assert_eq!(rec.magnetic_channel, MagneticChannel::XZX);
    }

    #[test]
    fn recommend_static_negative_field_uses_abs() {
        // Negative field: Z=-10 mT, X=-2 mT, Y=0.5 mT → XZ plane
        let mut cal = AxisCalibrator::default();
        for _ in 0..16 {
            cal.update(&MagneticReading {
                x: Some(MilliTesla(-2.0)),
                y: Some(MilliTesla(0.5)),
                z: Some(MilliTesla(-10.0)),
            });
        }
        let rec = cal.recommend_static().unwrap();
        assert_eq!(rec.plane, PlaneAxis::XZ);
    }

    #[test]
    fn recommend_static_yz_when_y_z_strongest() {
        let mut cal = AxisCalibrator::default();
        for _ in 0..16 {
            cal.update(&MagneticReading {
                x: Some(MilliTesla(0.1)),
                y: Some(MilliTesla(8.0)),
                z: Some(MilliTesla(5.0)),
            });
        }
        let rec = cal.recommend_static().unwrap();
        assert_eq!(rec.plane, PlaneAxis::YZ);
    }

    #[test]
    fn recommend_static_equal_means_defaults_xy() {
        // Equal means on all axes → tie-break X > Y > Z → XY
        let mut cal = AxisCalibrator::default();
        for _ in 0..16 {
            cal.update(&MagneticReading {
                x: Some(MilliTesla(5.0)),
                y: Some(MilliTesla(5.0)),
                z: Some(MilliTesla(5.0)),
            });
        }
        let rec = cal.recommend_static().unwrap();
        assert_eq!(rec.plane, PlaneAxis::XY);
    }

    #[test]
    fn recommend_static_abs_means_populated() {
        let mut cal = AxisCalibrator::default();
        for _ in 0..16 {
            cal.update(&MagneticReading {
                x: Some(MilliTesla(-3.0)),
                y: Some(MilliTesla(7.0)),
                z: Some(MilliTesla(0.0)),
            });
        }
        let rec = cal.recommend_static().unwrap();
        assert!((rec.abs_means[0].unwrap() - 3.0).abs() < 0.01);
        assert!((rec.abs_means[1].unwrap() - 7.0).abs() < 0.01);
        assert!((rec.abs_means[2].unwrap() - 0.0).abs() < 0.01);
    }

    #[test]
    fn recommend_static_too_few_samples_returns_none() {
        let cal = AxisCalibrator::default();
        assert!(cal.recommend_static().is_none());
    }

    #[test]
    fn recommend_static_output_accepted_by_config_builder() {
        use crate::config::ConfigBuilder;
        use crate::types::Range;

        let mut cal = AxisCalibrator::default();
        for _ in 0..16 {
            cal.update(&MagneticReading {
                x: Some(MilliTesla(1.0)),
                y: Some(MilliTesla(5.0)),
                z: Some(MilliTesla(10.0)),
            });
        }
        let rec = cal.recommend_static().unwrap();
        assert_eq!(rec.plane, PlaneAxis::YZ);
        let result = ConfigBuilder::new()
            .angle_enabled(rec.angle_enable)
            .magnetic_channels_enabled(rec.magnetic_channel)
            .xy_range(Range::High)
            .z_range(Range::High)
            .build();
        assert!(
            result.is_ok(),
            "ConfigBuilder rejected recommend_static YZ: {result:?}"
        );
    }

    #[test]
    fn recommend_variance_still_works_after_refactor() {
        // Characterization: existing recommend() produces same result as before
        let mut cal = AxisCalibrator::default();
        feed_rotation(&mut cal, 16, 10.0, 5.0, 1.0);
        let rec = cal.recommend().unwrap();
        assert_eq!(rec.plane, PlaneAxis::XY);
        assert_eq!(rec.magnetic_channel, MagneticChannel::XYX);
        // abs_means should also be populated
        assert!(rec.abs_means[0].is_some());
        assert!(rec.abs_means[1].is_some());
        assert!(rec.abs_means[2].is_some());
    }

    // -- PlaneAxis conversions ------------------------------------------------

    #[test]
    fn axis_plane_to_angle_enable() {
        assert_eq!(AngleEnable::from(PlaneAxis::XY), AngleEnable::XY);
        assert_eq!(AngleEnable::from(PlaneAxis::YZ), AngleEnable::YZ);
        assert_eq!(AngleEnable::from(PlaneAxis::XZ), AngleEnable::XZ);
    }

    #[test]
    fn axis_plane_to_magnetic_channel() {
        assert_eq!(MagneticChannel::from(PlaneAxis::XY), MagneticChannel::XYX);
        assert_eq!(MagneticChannel::from(PlaneAxis::YZ), MagneticChannel::YZY);
        assert_eq!(MagneticChannel::from(PlaneAxis::XZ), MagneticChannel::XZX);
    }

    #[test]
    fn plane_angle_fields_accessible() {
        let pa = PlaneAngle {
            angle: Degrees(45.0),
            magnitude: MilliTesla(10.0),
            plane: PlaneAxis::XY,
        };
        assert_eq!(pa.angle.0, 45.0);
        assert_eq!(pa.magnitude.0, 10.0);
        assert_eq!(pa.plane, PlaneAxis::XY);
    }
}
