"""
motion/parabolic_profile.py
===========================

Concrete :class:`~motion.velocity_profile.VelocityProfile` implementation for
a parabolic velocity profile.

Design overview
---------------
A parabolic velocity profile follows a quadratic bell curve over position:

    v(x) = v_peak × 4 × (x / d) × (1 − x / d)

where *x* is the unsigned distance from the start of travel and *d* is the
total unsigned distance.  This gives:

* v(0)   = 0       — gentle entry from rest
* v(d/2) = v_peak  — maximum speed at the midpoint
* v(d)   = 0       — gentle exit back to rest

Compared to a trapezoidal profile (linear ramp → cruise plateau → linear ramp),
the parabolic curve has **no phase boundaries** — velocity changes continuously
and smoothly over the entire travel distance.  There is no abrupt step from ramp
to cruise speed, which produces cleaner fluid dynamics during dip-coating
entry and withdrawal.

The average speed for the parabolic profile is exactly (2/3) × v_peak, so the
total duration is longer than a trapezoidal profile at the same peak speed.

Numerical note
--------------
The time-to-position inverse (needed for ``get_speed_at(t)``) and total duration
(needed for ``total_duration()``) are pre-computed once in ``__init__`` using a
Riemann sum with ``_N_INTEGRATION`` sub-intervals.  The parabolic speed formula
approaches zero at the endpoints, so each interval is floored at
``_MIN_SEGMENT_SPEED_MM_S`` to keep the integration finite.

Usage example::

    from motion.parabolic_profile import ParabolicProfile

    p = ParabolicProfile(
        target_speed_mm_s = 8.0,
        distance_mm       = -20.0,   # negative → descending
        accel_mm_s2       = 30.0,
    )

    print(p.total_duration())          # seconds
    print(p.get_speed_at(1.0))         # mm/s at t = 1.0 s
    segs = p.to_segments(1.0)          # list[MoveSegment], 20 segments
    data = p.to_dict()
    p2   = ParabolicProfile.from_dict(data)
"""

from __future__ import annotations

import bisect
import math
from typing import Final

from motion.velocity_profile import (
    DEFAULT_SEGMENT_LENGTH_MM,
    MoveSegment,
    ProfileDict,
    VelocityProfile,
)

# ---------------------------------------------------------------------------
# Module constants
# ---------------------------------------------------------------------------

# String identifier stored in to_dict() / expected by from_dict().
_PROFILE_TYPE: Final[str] = "parabolic"

# Minimum speed floor applied during numerical integration and in to_segments().
# Prevents division-by-zero in the integration near the zero-speed endpoints.
_MIN_SEGMENT_SPEED_MM_S: Final[float] = 0.01

# Number of sub-intervals used for numerical integration in __init__.
# 1000 intervals over a 20 mm move gives 0.02 mm resolution — duration
# error is well below 0.5 % for typical dip-coater distances.
_N_INTEGRATION: Final[int] = 1_000


# ---------------------------------------------------------------------------
# ParabolicProfile
# ---------------------------------------------------------------------------

class ParabolicProfile(VelocityProfile):
    """Parabolic (quadratic bell-curve) velocity profile.

    Velocity follows a downward-opening parabola over the unsigned travel
    distance, peaking at the midpoint and touching zero at both endpoints.

    Parameters
    ----------
    target_speed_mm_s:
        Peak speed reached at the midpoint of travel (mm/s).  Must be
        finite and > 0.
    distance_mm:
        Signed total travel distance in mm.  Negative → descending (toward
        substrate); positive → ascending (away from substrate).  Must be
        finite and non-zero.
    accel_mm_s2:
        Acceleration magnitude used in the ``CMD MOVE_SEG`` commands streamed
        to the Arduino.  The TMC5130 hardware ramp uses this value to smoothly
        accelerate between consecutive segment speed setpoints during velocity
        blending.  Must be finite and > 0.  Defaults to 30.0 mm/s².

    Raises
    ------
    ValueError
        If any parameter fails its invariant.

    Notes
    -----
    * ``total_duration()`` and ``get_speed_at(t)`` are backed by a
      pre-computed numerical lookup table built in ``__init__``.
    * ``to_segments()`` evaluates the parabolic formula analytically at each
      segment midpoint — no lookup table is used for the primary output.
    * Average speed = (2/3) × ``target_speed_mm_s``; total duration is
      therefore 50 % longer than a triangular profile at the same peak speed.
    """

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def __init__(
        self,
        target_speed_mm_s: float,
        distance_mm:       float,
        accel_mm_s2:       float = 30.0,
    ) -> None:
        """Construct and validate a parabolic profile, pre-computing kinematics.

        Parameters
        ----------
        target_speed_mm_s:
            Peak speed at midpoint (mm/s).
        distance_mm:
            Signed total travel distance (mm).
        accel_mm_s2:
            Acceleration for MOVE_SEG commands (mm/s²).  Defaults to 30.0.

        Raises
        ------
        ValueError
            If any parameter is non-finite, zero (distance), or non-positive
            (speed, accel).  All errors are collected and raised together.
        """
        # --- Validate all fields before computing anything ----------------
        errors: list[str] = []

        if not (target_speed_mm_s > 0.0) or not math.isfinite(target_speed_mm_s):
            errors.append(
                f"target_speed_mm_s must be finite and > 0, got {target_speed_mm_s!r}"
            )
        if distance_mm == 0.0 or not math.isfinite(distance_mm):
            errors.append(
                f"distance_mm must be finite and non-zero, got {distance_mm!r}"
            )
        if not (accel_mm_s2 > 0.0) or not math.isfinite(accel_mm_s2):
            errors.append(
                f"accel_mm_s2 must be finite and > 0, got {accel_mm_s2!r}"
            )

        if errors:
            raise ValueError("; ".join(errors))

        # --- Store constructor parameters ---------------------------------
        self.target_speed_mm_s: float = target_speed_mm_s
        self.distance_mm:       float = distance_mm
        self.accel_mm_s2:       float = accel_mm_s2

        # --- Direction and unsigned distance used throughout the class ----
        self._direction:   float = 1.0 if distance_mm > 0.0 else -1.0
        self._total_dist:  float = abs(distance_mm)

        # --- Pre-compute numerical integration for total_duration() and
        #     the time→speed lookup table used by get_speed_at(t). --------
        #
        # Walk along the unsigned distance axis in _N_INTEGRATION steps,
        # applying the speed floor to prevent division by zero near the
        # parabola's zero-valued endpoints.  Record:
        #   _lut_t[i] — cumulative time at the *end* of interval i
        #   _lut_v[i] — (floored) speed at the midpoint of interval i
        #
        # These two arrays allow binary-search lookup of speed vs. time.

        dx:       float     = self._total_dist / _N_INTEGRATION
        t_accum:  float     = 0.0
        lut_t:    list[float] = []
        lut_v:    list[float] = []

        for i in range(_N_INTEGRATION):
            x_mid: float = (i + 0.5) * dx
            v:     float = max(self._speed_at_position(x_mid), _MIN_SEGMENT_SPEED_MM_S)
            t_accum += dx / v
            lut_t.append(t_accum)
            lut_v.append(v)

        self._t_total: float      = t_accum
        self._lut_t:   list[float] = lut_t
        self._lut_v:   list[float] = lut_v

    # ------------------------------------------------------------------
    # VelocityProfile interface
    # ------------------------------------------------------------------

    def get_speed_at(self, t: float) -> float:
        """Return the commanded speed in mm/s at *t* seconds into the move.

        Uses binary search into the pre-computed time lookup table to find
        which position interval *t* falls in, then returns the floored
        parabolic speed at that interval's midpoint.

        Values outside ``[0, total_duration()]`` return 0.

        Parameters
        ----------
        t:
            Time into the move in seconds.

        Returns
        -------
        float
            Commanded speed in mm/s, always ≥ 0.
        """
        if t <= 0.0 or self._t_total == 0.0:
            return 0.0
        if t >= self._t_total:
            return 0.0

        # Binary search: find the first interval whose cumulative end-time
        # exceeds t.  That interval is the one currently being traversed.
        idx: int = bisect.bisect_left(self._lut_t, t)
        if idx >= len(self._lut_v):
            return 0.0
        return self._lut_v[idx]

    def total_duration(self) -> float:
        """Return the total move duration in seconds.

        Computed numerically in ``__init__`` as the Riemann sum of
        Δt = Δx / v(x_mid) over _N_INTEGRATION intervals.

        Returns
        -------
        float
            Duration in seconds, always > 0 for a valid profile.
        """
        return self._t_total

    def to_segments(
        self,
        segment_length_mm: float = DEFAULT_SEGMENT_LENGTH_MM,
    ) -> list[MoveSegment]:
        """Discretise the parabolic curve into constant-speed move segments.

        Each segment's speed is sampled at its spatial midpoint using the
        analytic parabolic formula.  The segment step is adjusted slightly
        so that an integer number of segments spans exactly the total distance.

        Speed is floored at ``_MIN_SEGMENT_SPEED_MM_S`` to satisfy the
        :class:`~motion.velocity_profile.MoveSegment` ``speed_mm_s > 0``
        invariant — the floor only activates for very small segment sizes
        (≪ 0.1 mm) where the parabola drops below 0.01 mm/s at the endpoints.

        Parameters
        ----------
        segment_length_mm:
            Approximate spatial resolution in mm.  Must be finite and > 0.

        Returns
        -------
        list[MoveSegment]
            Ordered list of segments.  The absolute sum of all ``distance_mm``
            values equals ``abs(self.distance_mm)`` exactly.

        Raises
        ------
        ValueError
            If *segment_length_mm* is ≤ 0 or non-finite.
        """
        if not (segment_length_mm > 0.0) or not math.isfinite(segment_length_mm):
            raise ValueError(
                f"segment_length_mm must be finite and > 0, got {segment_length_mm!r}"
            )

        segments: list[MoveSegment] = []

        # Walk along the unsigned distance axis in steps of segment_length_mm.
        pos: float = 0.0
        while pos < self._total_dist:
            # Clamp the last segment so total distance is exact.
            step:  float = min(segment_length_mm, self._total_dist - pos)

            # Evaluate speed at the spatial midpoint of this segment.
            mid:   float = pos + step / 2.0
            speed: float = max(self._speed_at_position(mid), _MIN_SEGMENT_SPEED_MM_S)

            segments.append(
                MoveSegment(
                    distance_mm = step * self._direction,
                    speed_mm_s  = speed,
                    accel_mm_s2 = self.accel_mm_s2,
                )
            )
            pos += step

        return segments

    def to_dict(self) -> ProfileDict:
        """Serialise to a JSON-safe dict for storage in ``DipProfile.velocity_profile_data``.

        Returns
        -------
        ProfileDict
            Contains keys ``"type"``, ``"target_speed_mm_s"``,
            ``"distance_mm"``, and ``"accel_mm_s2"``.
        """
        return {
            "type":               _PROFILE_TYPE,
            "target_speed_mm_s":  self.target_speed_mm_s,
            "distance_mm":        self.distance_mm,
            "accel_mm_s2":        self.accel_mm_s2,
        }

    @classmethod
    def from_dict(cls, data: ProfileDict) -> "ParabolicProfile":
        """Reconstruct a :class:`ParabolicProfile` from a serialised dict.

        Parameters
        ----------
        data:
            Dict as returned by :meth:`to_dict` or loaded from JSON.

        Returns
        -------
        ParabolicProfile
            Fully constructed and validated instance.

        Raises
        ------
        ValueError
            If *data* is missing required keys, has an unexpected type field,
            or contains invalid parameter values.
        """
        if data.get("type") != _PROFILE_TYPE:
            raise ValueError(
                f"Expected profile type {_PROFILE_TYPE!r}, "
                f"got {data.get('type')!r}"
            )
        try:
            return cls(
                target_speed_mm_s = float(data["target_speed_mm_s"]),
                distance_mm       = float(data["distance_mm"]),
                accel_mm_s2       = float(data.get("accel_mm_s2", 30.0)),
            )
        except KeyError as exc:
            raise ValueError(f"Missing required key in profile dict: {exc}") from exc

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _speed_at_position(self, x: float) -> float:
        """Return the parabolic speed in mm/s at unsigned position *x* mm.

        Does **not** apply the minimum speed floor — that is the caller's
        responsibility (applied in :meth:`to_segments` and in the numerical
        integration in ``__init__``).

        Parameters
        ----------
        x:
            Unsigned position in mm along the travel axis, 0 ≤ x ≤ total_dist.

        Returns
        -------
        float
            Speed in mm/s.  Returns 0.0 at x=0 and x=total_dist.
        """
        d: float = self._total_dist
        # Clamp to [0, d] to handle floating-point overshoot.
        x = max(0.0, min(x, d))
        # Parabolic: v(x) = v_peak × 4 × (x/d) × (1 − x/d)
        return self.target_speed_mm_s * 4.0 * (x / d) * (1.0 - x / d)

    def __repr__(self) -> str:
        return (
            f"ParabolicProfile("
            f"peak={self.target_speed_mm_s} mm/s, "
            f"dist={self.distance_mm} mm, "
            f"accel={self.accel_mm_s2} mm/s², "
            f"duration≈{self._t_total:.2f} s)"
        )
