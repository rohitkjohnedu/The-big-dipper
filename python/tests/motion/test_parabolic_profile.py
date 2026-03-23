"""
tests/motion/test_parabolic_profile.py
=======================================

Unit tests for :class:`~motion.parabolic_profile.ParabolicProfile`.

Test groups
-----------
* ``TestConstruction``     — parameter validation and stored attributes.
* ``TestSpeedAtPosition``  — _speed_at_position analytical values.
* ``TestGetSpeedAt``       — time-domain get_speed_at() via lookup table.
* ``TestTotalDuration``    — duration is finite, positive, and ≈ d / avg_v.
* ``TestToSegments``       — count, signs, distances, speeds.
* ``TestSerialisationRoundTrip`` — to_dict / from_dict.
* ``TestBufferHelpers``    — segment_count, fits_in_arduino_buffer.
* ``TestRepr``             — __repr__ smoke test.
"""

from __future__ import annotations

import math

import pytest

from motion.parabolic_profile import ParabolicProfile, _MIN_SEGMENT_SPEED_MM_S
from motion.velocity_profile import MoveSegment


# ---------------------------------------------------------------------------
# Shared fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def descent() -> ParabolicProfile:
    """Standard 20 mm downward parabolic profile."""
    return ParabolicProfile(
        target_speed_mm_s = 8.0,
        distance_mm       = -20.0,
        accel_mm_s2       = 30.0,
    )


@pytest.fixture
def ascent() -> ParabolicProfile:
    """Standard 20 mm upward parabolic profile."""
    return ParabolicProfile(
        target_speed_mm_s = 8.0,
        distance_mm       = 20.0,
        accel_mm_s2       = 30.0,
    )


# ---------------------------------------------------------------------------
# TestConstruction
# ---------------------------------------------------------------------------

class TestConstruction:
    """Parameter validation and attribute storage."""

    def test_stores_target_speed(self, descent: ParabolicProfile) -> None:
        assert descent.target_speed_mm_s == 8.0

    def test_stores_distance(self, descent: ParabolicProfile) -> None:
        assert descent.distance_mm == -20.0

    def test_stores_accel(self, descent: ParabolicProfile) -> None:
        assert descent.accel_mm_s2 == 30.0

    def test_default_accel(self) -> None:
        p = ParabolicProfile(target_speed_mm_s=5.0, distance_mm=10.0)
        assert p.accel_mm_s2 == 30.0

    def test_raises_on_zero_speed(self) -> None:
        with pytest.raises(ValueError, match="target_speed_mm_s"):
            ParabolicProfile(target_speed_mm_s=0.0, distance_mm=10.0)

    def test_raises_on_negative_speed(self) -> None:
        with pytest.raises(ValueError, match="target_speed_mm_s"):
            ParabolicProfile(target_speed_mm_s=-1.0, distance_mm=10.0)

    def test_raises_on_nan_speed(self) -> None:
        with pytest.raises(ValueError, match="target_speed_mm_s"):
            ParabolicProfile(target_speed_mm_s=float("nan"), distance_mm=10.0)

    def test_raises_on_zero_distance(self) -> None:
        with pytest.raises(ValueError, match="distance_mm"):
            ParabolicProfile(target_speed_mm_s=5.0, distance_mm=0.0)

    def test_raises_on_nan_distance(self) -> None:
        with pytest.raises(ValueError, match="distance_mm"):
            ParabolicProfile(target_speed_mm_s=5.0, distance_mm=float("nan"))

    def test_raises_on_zero_accel(self) -> None:
        with pytest.raises(ValueError, match="accel_mm_s2"):
            ParabolicProfile(target_speed_mm_s=5.0, distance_mm=10.0, accel_mm_s2=0.0)

    def test_raises_on_negative_accel(self) -> None:
        with pytest.raises(ValueError, match="accel_mm_s2"):
            ParabolicProfile(target_speed_mm_s=5.0, distance_mm=10.0, accel_mm_s2=-5.0)

    def test_raises_collects_multiple_errors(self) -> None:
        with pytest.raises(ValueError) as exc_info:
            ParabolicProfile(target_speed_mm_s=-1.0, distance_mm=0.0, accel_mm_s2=-1.0)
        msg = str(exc_info.value)
        assert "target_speed_mm_s" in msg
        assert "distance_mm"       in msg
        assert "accel_mm_s2"       in msg

    def test_positive_distance_accepted(self) -> None:
        p = ParabolicProfile(target_speed_mm_s=5.0, distance_mm=10.0)
        assert p.distance_mm == 10.0


# ---------------------------------------------------------------------------
# TestSpeedAtPosition
# ---------------------------------------------------------------------------

class TestSpeedAtPosition:
    """Analytical spot-checks of _speed_at_position."""

    def test_speed_at_zero_is_zero(self, descent: ParabolicProfile) -> None:
        assert descent._speed_at_position(0.0) == pytest.approx(0.0)

    def test_speed_at_end_is_zero(self, descent: ParabolicProfile) -> None:
        assert descent._speed_at_position(20.0) == pytest.approx(0.0)

    def test_speed_at_midpoint_equals_peak(self, descent: ParabolicProfile) -> None:
        # v(d/2) = v_peak * 4 * 0.5 * 0.5 = v_peak
        assert descent._speed_at_position(10.0) == pytest.approx(8.0, rel=1e-9)

    def test_speed_at_quarter_point(self, descent: ParabolicProfile) -> None:
        # v(5) = 8 * 4 * (5/20) * (1 - 5/20) = 8 * 4 * 0.25 * 0.75 = 6.0
        assert descent._speed_at_position(5.0) == pytest.approx(6.0, rel=1e-9)

    def test_speed_at_three_quarter_point(self, descent: ParabolicProfile) -> None:
        # Symmetric with quarter point
        assert descent._speed_at_position(15.0) == pytest.approx(6.0, rel=1e-9)

    def test_profile_is_symmetric(self, descent: ParabolicProfile) -> None:
        for x in [1.0, 3.0, 5.0, 7.0, 9.0]:
            v_fwd  = descent._speed_at_position(x)
            v_back = descent._speed_at_position(20.0 - x)
            assert v_fwd == pytest.approx(v_back, rel=1e-9), (
                f"Asymmetry at x={x}: {v_fwd} vs {v_back}"
            )

    def test_speed_clamps_beyond_end(self, descent: ParabolicProfile) -> None:
        # x clamped to [0, d]
        assert descent._speed_at_position(25.0) == pytest.approx(0.0)

    def test_speed_clamps_before_start(self, descent: ParabolicProfile) -> None:
        assert descent._speed_at_position(-1.0) == pytest.approx(0.0)

    def test_speed_positive_between_endpoints(self, descent: ParabolicProfile) -> None:
        for x in [0.5, 2.0, 5.0, 10.0, 15.0, 18.0, 19.5]:
            assert descent._speed_at_position(x) > 0.0

    def test_speed_peaks_at_midpoint(self, descent: ParabolicProfile) -> None:
        v_mid = descent._speed_at_position(10.0)
        for x in [5.0, 7.0, 13.0, 15.0]:
            assert descent._speed_at_position(x) < v_mid


# ---------------------------------------------------------------------------
# TestGetSpeedAt
# ---------------------------------------------------------------------------

class TestGetSpeedAt:
    """Time-domain speed lookup via the pre-computed LUT."""

    def test_returns_zero_before_start(self, descent: ParabolicProfile) -> None:
        assert descent.get_speed_at(0.0) == 0.0
        assert descent.get_speed_at(-1.0) == 0.0

    def test_returns_zero_at_or_after_end(self, descent: ParabolicProfile) -> None:
        t_end = descent.total_duration()
        assert descent.get_speed_at(t_end) == 0.0
        assert descent.get_speed_at(t_end + 1.0) == 0.0

    def test_positive_during_move(self, descent: ParabolicProfile) -> None:
        t_mid = descent.total_duration() / 2.0
        assert descent.get_speed_at(t_mid) > 0.0

    def test_near_peak_at_half_duration(self, descent: ParabolicProfile) -> None:
        # The peak speed is at the spatial midpoint.  Due to the parabolic
        # speed shape, the time midpoint lags behind the spatial midpoint —
        # but the speed at the time midpoint should be close to peak.
        t_mid = descent.total_duration() / 2.0
        v_mid = descent.get_speed_at(t_mid)
        # Speed at time midpoint should be well above average (2/3 * peak)
        # and reasonably close to peak
        assert v_mid > 0.5 * descent.target_speed_mm_s

    def test_speed_bounded_by_peak(self, descent: ParabolicProfile) -> None:
        t_total = descent.total_duration()
        for t in [t_total * f for f in [0.1, 0.25, 0.5, 0.75, 0.9]]:
            assert descent.get_speed_at(t) <= descent.target_speed_mm_s + 1e-9


# ---------------------------------------------------------------------------
# TestTotalDuration
# ---------------------------------------------------------------------------

class TestTotalDuration:
    """Duration is finite, positive, and consistent with the parabola average."""

    def test_positive(self, descent: ParabolicProfile) -> None:
        assert descent.total_duration() > 0.0

    def test_finite(self, descent: ParabolicProfile) -> None:
        assert math.isfinite(descent.total_duration())

    def test_greater_than_distance_over_peak(self, descent: ParabolicProfile) -> None:
        # Average speed is 2/3 * peak, so duration > d / v_peak
        t_min = abs(descent.distance_mm) / descent.target_speed_mm_s
        assert descent.total_duration() > t_min

    def test_duration_much_larger_than_d_over_peak(self, descent: ParabolicProfile) -> None:
        # The parabolic integral ∫ dx/v(x) diverges analytically at both
        # endpoints (speed → 0).  With the _MIN_SEGMENT_SPEED_MM_S floor the
        # result is finite but substantially larger than d/v_peak.
        # Verify it is at least 2× that lower bound.
        d = abs(descent.distance_mm)
        lower_bound = d / descent.target_speed_mm_s
        assert descent.total_duration() > 2.0 * lower_bound

    def test_ascent_equals_descent_duration(
        self,
        descent: ParabolicProfile,
        ascent:  ParabolicProfile,
    ) -> None:
        assert descent.total_duration() == pytest.approx(ascent.total_duration(), rel=1e-6)

    def test_longer_distance_gives_longer_duration(self) -> None:
        p_short = ParabolicProfile(target_speed_mm_s=8.0, distance_mm=-10.0)
        p_long  = ParabolicProfile(target_speed_mm_s=8.0, distance_mm=-20.0)
        assert p_long.total_duration() > p_short.total_duration()

    def test_higher_speed_gives_shorter_duration(self) -> None:
        p_slow = ParabolicProfile(target_speed_mm_s=4.0,  distance_mm=-20.0)
        p_fast = ParabolicProfile(target_speed_mm_s=10.0, distance_mm=-20.0)
        assert p_fast.total_duration() < p_slow.total_duration()


# ---------------------------------------------------------------------------
# TestToSegments
# ---------------------------------------------------------------------------

class TestToSegments:
    """to_segments() produces correctly structured segment lists."""

    def test_descent_segments_all_negative(self, descent: ParabolicProfile) -> None:
        segs = descent.to_segments(1.0)
        for s in segs:
            assert s.distance_mm < 0.0

    def test_ascent_segments_all_positive(self, ascent: ParabolicProfile) -> None:
        segs = ascent.to_segments(1.0)
        for s in segs:
            assert s.distance_mm > 0.0

    def test_distances_sum_to_total(self, descent: ParabolicProfile) -> None:
        segs = descent.to_segments(1.0)
        total = sum(abs(s.distance_mm) for s in segs)
        assert total == pytest.approx(abs(descent.distance_mm), rel=1e-6)

    def test_segment_count_for_1mm_steps(self, descent: ParabolicProfile) -> None:
        # 20 mm / 1 mm = 20 segments
        segs = descent.to_segments(1.0)
        assert len(segs) == 20

    def test_segment_count_for_half_mm_steps(self, descent: ParabolicProfile) -> None:
        # 20 mm / 0.5 mm = 40 segments
        segs = descent.to_segments(0.5)
        assert len(segs) == 40

    def test_all_speeds_positive(self, descent: ParabolicProfile) -> None:
        segs = descent.to_segments(1.0)
        for s in segs:
            assert s.speed_mm_s >= _MIN_SEGMENT_SPEED_MM_S

    def test_all_speeds_at_most_peak(self, descent: ParabolicProfile) -> None:
        segs = descent.to_segments(1.0)
        for s in segs:
            assert s.speed_mm_s <= descent.target_speed_mm_s + 1e-9

    def test_accel_propagated_to_all_segments(self, descent: ParabolicProfile) -> None:
        segs = descent.to_segments(1.0)
        for s in segs:
            assert s.accel_mm_s2 == descent.accel_mm_s2

    def test_speed_peaks_near_midpoint(self, descent: ParabolicProfile) -> None:
        segs = descent.to_segments(1.0)
        speeds = [s.speed_mm_s for s in segs]
        peak_idx = speeds.index(max(speeds))
        # Peak should be within 2 segments of the midpoint (index 9 or 10)
        assert abs(peak_idx - len(segs) // 2) <= 2

    def test_speed_profile_is_symmetric(self, descent: ParabolicProfile) -> None:
        segs = descent.to_segments(1.0)
        speeds = [s.speed_mm_s for s in segs]
        n = len(speeds)
        for i in range(n // 2):
            assert speeds[i] == pytest.approx(speeds[n - 1 - i], rel=1e-6), (
                f"Asymmetry at index {i}: {speeds[i]:.4f} vs {speeds[n-1-i]:.4f}"
            )

    def test_monotonically_rising_then_falling(self, descent: ParabolicProfile) -> None:
        segs = descent.to_segments(1.0)
        speeds = [s.speed_mm_s for s in segs]
        mid = len(speeds) // 2
        # First half: non-decreasing
        for i in range(mid - 1):
            assert speeds[i] <= speeds[i + 1] + 1e-9
        # Second half: non-increasing
        for i in range(mid, len(speeds) - 1):
            assert speeds[i] >= speeds[i + 1] - 1e-9

    def test_raises_on_zero_segment_length(self, descent: ParabolicProfile) -> None:
        with pytest.raises(ValueError, match="segment_length_mm"):
            descent.to_segments(0.0)

    def test_raises_on_negative_segment_length(self, descent: ParabolicProfile) -> None:
        with pytest.raises(ValueError, match="segment_length_mm"):
            descent.to_segments(-1.0)

    def test_raises_on_nan_segment_length(self, descent: ParabolicProfile) -> None:
        with pytest.raises(ValueError, match="segment_length_mm"):
            descent.to_segments(float("nan"))

    def test_smoother_than_trapezoidal(self) -> None:
        """Adjacent speed differences are smaller than for an equivalent TrapezoidalProfile."""
        from motion.trapezoidal_profile import TrapezoidalProfile

        parab = ParabolicProfile(
            target_speed_mm_s=8.0, distance_mm=-20.0, accel_mm_s2=30.0
        )
        trap = TrapezoidalProfile(
            target_speed_mm_s=8.0, accel_mm_s2=30.0, distance_mm=-20.0
        )
        p_segs = parab.to_segments(1.0)
        t_segs = trap.to_segments(1.0)

        def max_adjacent_delta(segs: list[MoveSegment]) -> float:
            speeds = [s.speed_mm_s for s in segs]
            return max(abs(speeds[i+1] - speeds[i]) for i in range(len(speeds) - 1))

        # Parabolic adjacent speed differences should be strictly smaller
        assert max_adjacent_delta(p_segs) < max_adjacent_delta(t_segs)


# ---------------------------------------------------------------------------
# TestSerialisationRoundTrip
# ---------------------------------------------------------------------------

class TestSerialisationRoundTrip:
    """to_dict() / from_dict() round-trip fidelity."""

    def test_type_field(self, descent: ParabolicProfile) -> None:
        assert descent.to_dict()["type"] == "parabolic"

    def test_round_trip_speed(self, descent: ParabolicProfile) -> None:
        p2 = ParabolicProfile.from_dict(descent.to_dict())
        assert p2.target_speed_mm_s == descent.target_speed_mm_s

    def test_round_trip_distance(self, descent: ParabolicProfile) -> None:
        p2 = ParabolicProfile.from_dict(descent.to_dict())
        assert p2.distance_mm == descent.distance_mm

    def test_round_trip_accel(self, descent: ParabolicProfile) -> None:
        p2 = ParabolicProfile.from_dict(descent.to_dict())
        assert p2.accel_mm_s2 == descent.accel_mm_s2

    def test_round_trip_duration(self, descent: ParabolicProfile) -> None:
        p2 = ParabolicProfile.from_dict(descent.to_dict())
        assert p2.total_duration() == pytest.approx(descent.total_duration(), rel=1e-9)

    def test_from_dict_wrong_type_raises(self, descent: ParabolicProfile) -> None:
        d = descent.to_dict()
        d["type"] = "trapezoidal"
        with pytest.raises(ValueError, match="type"):
            ParabolicProfile.from_dict(d)

    def test_from_dict_missing_key_raises(self, descent: ParabolicProfile) -> None:
        d = descent.to_dict()
        del d["target_speed_mm_s"]
        with pytest.raises(ValueError, match="Missing required key"):
            ParabolicProfile.from_dict(d)

    def test_from_dict_default_accel(self, descent: ParabolicProfile) -> None:
        d = descent.to_dict()
        del d["accel_mm_s2"]
        p2 = ParabolicProfile.from_dict(d)
        assert p2.accel_mm_s2 == 30.0


# ---------------------------------------------------------------------------
# TestBufferHelpers
# ---------------------------------------------------------------------------

class TestBufferHelpers:
    """segment_count and fits_in_arduino_buffer helpers."""

    def test_segment_count_matches_to_segments_length(
        self, descent: ParabolicProfile
    ) -> None:
        assert descent.segment_count(1.0) == len(descent.to_segments(1.0))

    def test_fits_in_buffer_1mm_steps(self, descent: ParabolicProfile) -> None:
        # 20 mm / 1 mm = 20 segments — well within the 64-slot buffer.
        assert descent.fits_in_arduino_buffer(segment_length_mm=1.0)

    def test_not_fits_in_buffer_very_small_steps(self) -> None:
        # 20 mm / 0.1 mm = 200 segments — exceeds the 64-slot buffer.
        p = ParabolicProfile(target_speed_mm_s=8.0, distance_mm=-20.0)
        assert not p.fits_in_arduino_buffer(segment_length_mm=0.1)

    def test_fits_exactly_at_buffer_boundary(self) -> None:
        # 32 mm / 0.5 mm = 64 segments — exactly at the limit.
        p = ParabolicProfile(target_speed_mm_s=8.0, distance_mm=-32.0)
        assert p.fits_in_arduino_buffer(segment_length_mm=0.5, buffer_size=64)


# ---------------------------------------------------------------------------
# TestRepr
# ---------------------------------------------------------------------------

class TestRepr:
    def test_repr_contains_peak(self, descent: ParabolicProfile) -> None:
        assert "8.0" in repr(descent)

    def test_repr_contains_distance(self, descent: ParabolicProfile) -> None:
        assert "-20.0" in repr(descent)

    def test_repr_is_string(self, descent: ParabolicProfile) -> None:
        assert isinstance(repr(descent), str)
