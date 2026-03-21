"""
tests/motion/test_trapezoidal_profile.py
=========================================

Unit tests for :class:`~motion.trapezoidal_profile.TrapezoidalProfile`.

Coverage
--------
* Construction — valid parameters, all errors collected and raised together.
* Full trapezoidal profile — phase boundaries, kinematics, duration.
* Triangular profile — short distance, reduced peak speed.
* ``get_speed_at()`` — each phase, boundaries, clamping outside range.
* ``total_duration()`` — analytical formula.
* ``to_segments()`` — count, signed distances sum to total, speed bounds,
  invalid segment_length rejection.
* ``to_dict()`` / ``from_dict()`` — round-trip, missing keys, wrong type.
* ``fits_in_arduino_buffer()`` / ``segment_count()`` — helper methods.
* ``__repr__`` — smoke test.
"""

from __future__ import annotations

import math
from typing import Final, Union

import pytest

from motion.trapezoidal_profile import TrapezoidalProfile, _MIN_SEGMENT_SPEED_MM_S
from motion.velocity_profile import DEFAULT_SEGMENT_LENGTH_MM, MoveSegment

# ---------------------------------------------------------------------------
# Shared test constants
# ---------------------------------------------------------------------------

# A standard full trapezoidal profile used across many tests.
# At 10 mm/s with 50 mm/s² accel, d_accel = 10²/(2*50) = 1 mm.
# Total distance = 100 mm >> 2 mm, so cruise phase exists.
SPEED:    Final[float] = 10.0   # mm/s
ACCEL:    Final[float] = 50.0   # mm/s²
DIST:     Final[float] = 100.0  # mm (descending — negative in real use)

# A short profile where the motor cannot reach SPEED before it must decel.
# d_accel_full = 10²/(2*50) = 1 mm → need >= 2 mm for full trapezoidal.
# Short distance: 1.0 mm < 2.0 mm → triangular.
SHORT_DIST: Final[float] = 1.0   # mm


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_profile(
    speed:    float = SPEED,
    accel:    float = ACCEL,
    distance: float = -DIST,   # negative = descending
) -> TrapezoidalProfile:
    """Construct a :class:`TrapezoidalProfile` with default or overridden values."""
    return TrapezoidalProfile(
        target_speed_mm_s = speed,
        accel_mm_s2       = accel,
        distance_mm       = distance,
    )


# ---------------------------------------------------------------------------
# Construction and validation
# ---------------------------------------------------------------------------

class TestConstruction:
    """Tests for constructor validation and attribute assignment."""

    def test_valid_descending_profile(self) -> None:
        """Constructor succeeds for a standard descending profile."""
        p: TrapezoidalProfile = make_profile(distance=-DIST)
        assert p.target_speed_mm_s == SPEED
        assert p.accel_mm_s2       == ACCEL
        assert p.distance_mm       == -DIST

    def test_valid_ascending_profile(self) -> None:
        """Constructor succeeds for an ascending profile."""
        p: TrapezoidalProfile = make_profile(distance=DIST)
        assert p.distance_mm == DIST

    def test_zero_speed_raises(self) -> None:
        """Zero target speed raises ValueError."""
        with pytest.raises(ValueError, match="target_speed_mm_s"):
            TrapezoidalProfile(target_speed_mm_s=0.0, accel_mm_s2=ACCEL, distance_mm=-DIST)

    def test_negative_speed_raises(self) -> None:
        """Negative target speed raises ValueError."""
        with pytest.raises(ValueError, match="target_speed_mm_s"):
            TrapezoidalProfile(target_speed_mm_s=-5.0, accel_mm_s2=ACCEL, distance_mm=-DIST)

    def test_nan_speed_raises(self) -> None:
        """NaN target speed raises ValueError."""
        with pytest.raises(ValueError, match="target_speed_mm_s"):
            TrapezoidalProfile(target_speed_mm_s=float("nan"), accel_mm_s2=ACCEL, distance_mm=-DIST)

    def test_inf_speed_raises(self) -> None:
        """Infinite target speed raises ValueError."""
        with pytest.raises(ValueError, match="target_speed_mm_s"):
            TrapezoidalProfile(target_speed_mm_s=float("inf"), accel_mm_s2=ACCEL, distance_mm=-DIST)

    def test_zero_accel_raises(self) -> None:
        """Zero acceleration raises ValueError."""
        with pytest.raises(ValueError, match="accel_mm_s2"):
            TrapezoidalProfile(target_speed_mm_s=SPEED, accel_mm_s2=0.0, distance_mm=-DIST)

    def test_negative_accel_raises(self) -> None:
        """Negative acceleration raises ValueError."""
        with pytest.raises(ValueError, match="accel_mm_s2"):
            TrapezoidalProfile(target_speed_mm_s=SPEED, accel_mm_s2=-50.0, distance_mm=-DIST)

    def test_zero_distance_raises(self) -> None:
        """Zero distance raises ValueError."""
        with pytest.raises(ValueError, match="distance_mm"):
            TrapezoidalProfile(target_speed_mm_s=SPEED, accel_mm_s2=ACCEL, distance_mm=0.0)

    def test_nan_distance_raises(self) -> None:
        """NaN distance raises ValueError."""
        with pytest.raises(ValueError, match="distance_mm"):
            TrapezoidalProfile(target_speed_mm_s=SPEED, accel_mm_s2=ACCEL, distance_mm=float("nan"))

    def test_multiple_errors_reported_together(self) -> None:
        """All validation errors are collected and raised in a single ValueError."""
        with pytest.raises(ValueError) as exc_info:
            TrapezoidalProfile(target_speed_mm_s=-1.0, accel_mm_s2=-1.0, distance_mm=0.0)
        message: str = str(exc_info.value)
        assert "target_speed_mm_s" in message
        assert "accel_mm_s2"       in message
        assert "distance_mm"       in message


# ---------------------------------------------------------------------------
# Full trapezoidal profile kinematics
# ---------------------------------------------------------------------------

class TestFullTrapezoidal:
    """Tests for a profile where cruise phase exists."""

    def setup_method(self) -> None:
        """Create a descending full trapezoidal profile for each test."""
        # d_accel = 10²/(2*50) = 1 mm
        # d_constant = 100 - 2 = 98 mm
        # t_accel = 10/50 = 0.2 s
        # t_constant = 98/10 = 9.8 s
        # t_total = 2*0.2 + 9.8 = 10.2 s
        self.p: TrapezoidalProfile = make_profile(distance=-DIST)

    def test_is_not_triangular(self) -> None:
        """Full trapezoidal profile reports is_triangular == False."""
        assert not self.p.is_triangular

    def test_peak_speed_equals_target(self) -> None:
        """Peak speed equals target speed for a full trapezoidal profile."""
        assert self.p.peak_speed_mm_s == pytest.approx(SPEED)

    def test_total_duration(self) -> None:
        """Total duration matches analytical formula: 2*t_accel + t_constant."""
        t_accel: float   = SPEED / ACCEL
        d_accel: float   = SPEED ** 2 / (2.0 * ACCEL)
        t_constant: float = (DIST - 2.0 * d_accel) / SPEED
        expected: float  = 2.0 * t_accel + t_constant
        assert self.p.total_duration() == pytest.approx(expected)

    def test_get_speed_at_zero(self) -> None:
        """Speed at t=0 is 0 mm/s."""
        speed_at_zero: float = self.p.get_speed_at(0.0)
        assert speed_at_zero == pytest.approx(0.0)

    def test_get_speed_at_end_of_accel(self) -> None:
        """Speed at end of accel phase equals peak speed."""
        t_accel: float = SPEED / ACCEL
        speed: float   = self.p.get_speed_at(t_accel)
        assert speed == pytest.approx(SPEED)

    def test_get_speed_during_cruise(self) -> None:
        """Speed during cruise phase equals peak speed."""
        t_mid: float = self.p.total_duration() / 2.0
        speed: float = self.p.get_speed_at(t_mid)
        assert speed == pytest.approx(SPEED)

    def test_get_speed_at_end_of_decel(self) -> None:
        """Speed at end of decel phase (total_duration) is 0 mm/s."""
        speed: float = self.p.get_speed_at(self.p.total_duration())
        assert speed == pytest.approx(0.0, abs=1e-9)

    def test_get_speed_beyond_end_clamped(self) -> None:
        """Speed beyond total_duration is clamped to 0."""
        speed: float = self.p.get_speed_at(self.p.total_duration() + 10.0)
        assert speed == pytest.approx(0.0)

    def test_get_speed_before_start_clamped(self) -> None:
        """Speed for negative t is clamped to 0."""
        speed: float = self.p.get_speed_at(-1.0)
        assert speed == pytest.approx(0.0)

    def test_get_speed_mid_accel(self) -> None:
        """Speed at half the accel time follows v = a*t."""
        t_accel: float  = SPEED / ACCEL
        t_half: float   = t_accel / 2.0
        expected: float = ACCEL * t_half
        speed: float    = self.p.get_speed_at(t_half)
        assert speed == pytest.approx(expected)


# ---------------------------------------------------------------------------
# Triangular profile kinematics
# ---------------------------------------------------------------------------

class TestTriangularProfile:
    """Tests for a profile where total distance is too short for cruise phase."""

    def setup_method(self) -> None:
        """Create a triangular profile (1 mm < 2*d_accel_full = 2 mm)."""
        # peak_speed = sqrt(50 * 1) = sqrt(50) ≈ 7.071 mm/s
        # t_accel = peak_speed / 50 ≈ 0.1414 s
        # total_duration = 2 * t_accel ≈ 0.2828 s
        self.p: TrapezoidalProfile = TrapezoidalProfile(
            target_speed_mm_s = SPEED,
            accel_mm_s2       = ACCEL,
            distance_mm       = -SHORT_DIST,
        )
        self.expected_peak: float = math.sqrt(ACCEL * SHORT_DIST)

    def test_is_triangular(self) -> None:
        """Short-distance profile reports is_triangular == True."""
        assert self.p.is_triangular

    def test_peak_speed_less_than_target(self) -> None:
        """Peak speed is less than target speed for a triangular profile."""
        assert self.p.peak_speed_mm_s < SPEED
        assert self.p.peak_speed_mm_s == pytest.approx(self.expected_peak)

    def test_total_duration_triangular(self) -> None:
        """Duration is 2 * peak_speed / accel for a triangular profile."""
        expected: float = 2.0 * self.expected_peak / ACCEL
        assert self.p.total_duration() == pytest.approx(expected)

    def test_get_speed_at_peak_time(self) -> None:
        """Speed at half total_duration equals peak speed."""
        t_peak: float = self.p.total_duration() / 2.0
        speed: float  = self.p.get_speed_at(t_peak)
        assert speed == pytest.approx(self.expected_peak)

    def test_get_speed_at_end_is_zero(self) -> None:
        """Speed at total_duration is 0 for a triangular profile."""
        speed: float = self.p.get_speed_at(self.p.total_duration())
        assert speed == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# to_segments()
# ---------------------------------------------------------------------------

class TestToSegments:
    """Tests for the analytical segment discretisation."""

    def setup_method(self) -> None:
        """Create standard descending full trapezoidal profile."""
        self.p: TrapezoidalProfile = make_profile(distance=-DIST)

    def test_segments_not_empty(self) -> None:
        """to_segments() returns a non-empty list."""
        segs: list[MoveSegment] = self.p.to_segments()
        assert len(segs) > 0

    def test_segment_distances_sum_to_total(self) -> None:
        """Absolute sum of segment distances equals total profile distance."""
        segs: list[MoveSegment] = self.p.to_segments(0.5)
        total: float = sum(abs(s.distance_mm) for s in segs)
        assert total == pytest.approx(DIST, rel=1e-6)

    def test_all_segments_descending(self) -> None:
        """All segments have negative distance for a descending profile."""
        segs: list[MoveSegment] = self.p.to_segments()
        seg: MoveSegment
        for seg in segs:
            assert seg.distance_mm < 0.0

    def test_all_segments_ascending(self) -> None:
        """All segments have positive distance for an ascending profile."""
        p_up: TrapezoidalProfile = make_profile(distance=DIST)
        segs: list[MoveSegment]  = p_up.to_segments()
        seg: MoveSegment
        for seg in segs:
            assert seg.distance_mm > 0.0

    def test_all_segment_speeds_positive(self) -> None:
        """All segment speeds are > 0."""
        segs: list[MoveSegment] = self.p.to_segments()
        seg: MoveSegment
        for seg in segs:
            assert seg.speed_mm_s > 0.0

    def test_all_segment_speeds_at_most_peak(self) -> None:
        """No segment speed exceeds the profile peak speed."""
        segs: list[MoveSegment] = self.p.to_segments()
        seg: MoveSegment
        for seg in segs:
            assert seg.speed_mm_s <= self.p.peak_speed_mm_s + 1e-9

    def test_segment_count_increases_with_finer_resolution(self) -> None:
        """Finer segment_length_mm produces more segments."""
        coarse: int = len(self.p.to_segments(2.0))
        fine:   int = len(self.p.to_segments(0.5))
        assert fine > coarse

    def test_invalid_segment_length_zero_raises(self) -> None:
        """segment_length_mm = 0 raises ValueError."""
        with pytest.raises(ValueError, match="segment_length_mm"):
            self.p.to_segments(0.0)

    def test_invalid_segment_length_negative_raises(self) -> None:
        """Negative segment_length_mm raises ValueError."""
        with pytest.raises(ValueError, match="segment_length_mm"):
            self.p.to_segments(-1.0)

    def test_invalid_segment_length_nan_raises(self) -> None:
        """NaN segment_length_mm raises ValueError."""
        with pytest.raises(ValueError, match="segment_length_mm"):
            self.p.to_segments(float("nan"))

    def test_single_segment_short_distance(self) -> None:
        """A very short distance produces exactly one segment."""
        p: TrapezoidalProfile = TrapezoidalProfile(
            target_speed_mm_s=10.0, accel_mm_s2=50.0, distance_mm=-0.3
        )
        segs: list[MoveSegment] = p.to_segments(segment_length_mm=0.5)
        assert len(segs) == 1

    def test_min_speed_floor_applied(self) -> None:
        """No segment speed is below _MIN_SEGMENT_SPEED_MM_S."""
        segs: list[MoveSegment] = self.p.to_segments()
        seg: MoveSegment
        for seg in segs:
            assert seg.speed_mm_s >= _MIN_SEGMENT_SPEED_MM_S


# ---------------------------------------------------------------------------
# to_dict() and from_dict()
# ---------------------------------------------------------------------------

class TestSerialisationRoundTrip:
    """Tests for JSON serialisation and reconstruction."""

    def test_to_dict_contains_required_keys(self) -> None:
        """to_dict() returns a dict with all required keys."""
        d: dict[str, object] = make_profile().to_dict()
        assert "type"               in d
        assert "target_speed_mm_s"  in d
        assert "accel_mm_s2"        in d
        assert "distance_mm"        in d

    def test_to_dict_type_is_trapezoidal(self) -> None:
        """to_dict() sets type to 'trapezoidal'."""
        d: dict[str, object] = make_profile().to_dict()
        assert d["type"] == "trapezoidal"

    def test_round_trip_preserves_parameters(self) -> None:
        """from_dict(to_dict(p)) produces an equivalent profile."""
        original: TrapezoidalProfile = make_profile(distance=-75.0)
        restored: TrapezoidalProfile = TrapezoidalProfile.from_dict(original.to_dict())

        assert restored.target_speed_mm_s == pytest.approx(original.target_speed_mm_s)
        assert restored.accel_mm_s2       == pytest.approx(original.accel_mm_s2)
        assert restored.distance_mm       == pytest.approx(original.distance_mm)

    def test_round_trip_preserves_duration(self) -> None:
        """Restored profile has the same total_duration as the original."""
        original: TrapezoidalProfile = make_profile()
        restored: TrapezoidalProfile = TrapezoidalProfile.from_dict(original.to_dict())
        assert restored.total_duration() == pytest.approx(original.total_duration())

    def test_from_dict_missing_speed_raises(self) -> None:
        """from_dict() with missing 'target_speed_mm_s' raises ValueError."""
        bad: dict[str, object] = {"accel_mm_s2": ACCEL, "distance_mm": -DIST}
        with pytest.raises(ValueError, match="target_speed_mm_s"):
            TrapezoidalProfile.from_dict(bad)

    def test_from_dict_missing_accel_raises(self) -> None:
        """from_dict() with missing 'accel_mm_s2' raises ValueError."""
        bad: dict[str, object] = {"target_speed_mm_s": SPEED, "distance_mm": -DIST}
        with pytest.raises(ValueError, match="accel_mm_s2"):
            TrapezoidalProfile.from_dict(bad)

    def test_from_dict_missing_distance_raises(self) -> None:
        """from_dict() with missing 'distance_mm' raises ValueError."""
        bad: dict[str, object] = {"target_speed_mm_s": SPEED, "accel_mm_s2": ACCEL}
        with pytest.raises(ValueError, match="distance_mm"):
            TrapezoidalProfile.from_dict(bad)

    def test_from_dict_invalid_value_raises(self) -> None:
        """from_dict() with invalid value (zero distance) raises ValueError."""
        bad: dict[str, object] = {
            "target_speed_mm_s": SPEED, "accel_mm_s2": ACCEL, "distance_mm": 0.0
        }
        with pytest.raises(ValueError):
            TrapezoidalProfile.from_dict(bad)


# ---------------------------------------------------------------------------
# Buffer and count helpers
# ---------------------------------------------------------------------------

class TestBufferHelpers:
    """Tests for fits_in_arduino_buffer() and segment_count()."""

    def test_segment_count_matches_to_segments_length(self) -> None:
        """segment_count() returns the same value as len(to_segments())."""
        p: TrapezoidalProfile   = make_profile()
        seg_len: float          = 0.5
        expected: int           = len(p.to_segments(seg_len))
        assert p.segment_count(seg_len) == expected

    def test_fits_in_buffer_coarse_resolution(self) -> None:
        """A 100 mm profile with 2 mm segments fits in the 64-segment buffer."""
        p: TrapezoidalProfile = make_profile()
        assert p.fits_in_arduino_buffer(segment_length_mm=2.0)

    def test_does_not_fit_fine_resolution(self) -> None:
        """A 100 mm profile with 0.5 mm segments exceeds the 64-segment buffer."""
        p: TrapezoidalProfile = make_profile()
        # 100 mm / 0.5 mm = 200 segments > 64
        assert not p.fits_in_arduino_buffer(segment_length_mm=0.5)

    def test_custom_buffer_size(self) -> None:
        """fits_in_arduino_buffer() respects a custom buffer_size argument."""
        p: TrapezoidalProfile = make_profile()
        count: int = p.segment_count(0.5)
        assert p.fits_in_arduino_buffer(0.5, buffer_size=count)
        assert not p.fits_in_arduino_buffer(0.5, buffer_size=count - 1)


# ---------------------------------------------------------------------------
# __repr__
# ---------------------------------------------------------------------------

class TestRepr:
    """Smoke tests for __repr__."""

    def test_repr_contains_key_info(self) -> None:
        """__repr__ includes speed, accel, distance, and shape."""
        p: TrapezoidalProfile = make_profile()
        r: str = repr(p)
        assert "TrapezoidalProfile" in r
        assert "trapezoidal"        in r

    def test_repr_triangular_label(self) -> None:
        """__repr__ labels a triangular profile correctly."""
        p: TrapezoidalProfile = TrapezoidalProfile(
            target_speed_mm_s=10.0, accel_mm_s2=50.0, distance_mm=-1.0
        )
        assert "triangular" in repr(p)
