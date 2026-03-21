"""
motion/trapezoidal_profile.py
==============================

Concrete :class:`~motion.velocity_profile.VelocityProfile` implementation for
a standard trapezoidal (or triangular) velocity profile.

Design overview
---------------
A trapezoidal profile has three phases:

1. **Acceleration** — motor ramps from 0 to *peak_speed* at *accel_mm_s²*.
2. **Cruise** — motor runs at constant *peak_speed*.
3. **Deceleration** — motor ramps back to 0 at *accel_mm_s²*.

If the total distance is too short for the motor to reach *target_speed_mm_s*
before it must begin decelerating, the cruise phase is omitted and the profile
degenerates to a **triangular** shape.  The peak speed is then::

    peak_speed = sqrt(accel_mm_s2 * total_distance)

All kinematic quantities are pre-computed once in ``__init__`` and stored as
private attributes so that repeated calls to :meth:`get_speed_at`,
:meth:`total_duration`, and :meth:`to_segments` are O(1) or O(n_segments).

Usage example::

    from motion.trapezoidal_profile import TrapezoidalProfile

    p = TrapezoidalProfile(
        target_speed_mm_s=10.0,
        accel_mm_s2=50.0,
        distance_mm=-100.0,   # negative → descending
    )

    print(p.total_duration())          # seconds
    print(p.get_speed_at(0.5))         # mm/s at t = 0.5 s
    segs = p.to_segments(0.5)          # list[MoveSegment]
    data = p.to_dict()                 # serialise for DipProfile.velocity_profile_data
    p2   = TrapezoidalProfile.from_dict(data)  # reconstruct
"""

from __future__ import annotations

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
_PROFILE_TYPE: Final[str] = "trapezoidal"

# Minimum speed used as a floor in to_segments() to avoid a zero-speed
# MoveSegment at the very start or end of the profile where kinematic speed
# rounds to zero due to floating-point.
_MIN_SEGMENT_SPEED_MM_S: Final[float] = 0.01


# ---------------------------------------------------------------------------
# TrapezoidalProfile
# ---------------------------------------------------------------------------

class TrapezoidalProfile(VelocityProfile):
    """Standard trapezoidal (or triangular) velocity profile.

    All kinematic quantities are computed analytically from three constructor
    parameters.  No iterative solvers or numerical integration are used.

    Parameters
    ----------
    target_speed_mm_s:
        Desired cruise speed in mm/s.  Must be finite and > 0.  The motor may
        not reach this speed if *distance_mm* is too short — see the triangular
        profile note above.
    accel_mm_s2:
        Acceleration (and deceleration) magnitude in mm/s².  Must be finite
        and > 0.
    distance_mm:
        Signed total travel distance in mm.  Negative → descending (toward
        substrate); positive → ascending (away from substrate).  Must be
        finite and non-zero.

    Raises
    ------
    ValueError
        If any parameter fails its invariant.

    Attributes
    ----------
    target_speed_mm_s:
        Requested cruise speed (may not be reached for short distances).
    accel_mm_s2:
        Acceleration / deceleration magnitude.
    distance_mm:
        Signed travel distance as supplied to the constructor.

    Notes
    -----
    * :meth:`to_segments` uses signed ``distance_mm`` to produce correctly
      directed :class:`~motion.velocity_profile.MoveSegment` objects.
    * :meth:`to_dict` / :meth:`from_dict` round-trip losslessly through JSON.
    """

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def __init__(
        self,
        target_speed_mm_s: float,
        accel_mm_s2:       float,
        distance_mm:       float,
    ) -> None:
        """Construct and validate a trapezoidal profile, pre-computing kinematics.

        Parameters
        ----------
        target_speed_mm_s:
            Desired cruise speed in mm/s.
        accel_mm_s2:
            Acceleration magnitude in mm/s².
        distance_mm:
            Signed total travel distance in mm.

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
        if not (accel_mm_s2 > 0.0) or not math.isfinite(accel_mm_s2):
            errors.append(
                f"accel_mm_s2 must be finite and > 0, got {accel_mm_s2!r}"
            )
        if distance_mm == 0.0 or not math.isfinite(distance_mm):
            errors.append(
                f"distance_mm must be finite and non-zero, got {distance_mm!r}"
            )

        if errors:
            raise ValueError("; ".join(errors))

        # --- Store constructor parameters ---------------------------------
        self.target_speed_mm_s: float = target_speed_mm_s
        self.accel_mm_s2:       float = accel_mm_s2
        self.distance_mm:       float = distance_mm

        # --- Pre-compute kinematic quantities -----------------------------
        # Direction multiplier (+1 ascending, -1 descending) and total
        # unsigned distance used throughout the kinematic calculations.
        self._direction:    float = 1.0 if distance_mm > 0.0 else -1.0
        self._total_dist:   float = abs(distance_mm)

        # Unsigned distance required to accelerate from 0 to target_speed.
        d_accel_full: float = (target_speed_mm_s ** 2) / (2.0 * accel_mm_s2)

        if self._total_dist >= 2.0 * d_accel_full:
            # ---- Full trapezoidal — motor reaches target speed -----------
            self._peak_speed:   float = target_speed_mm_s
            self._d_accel:      float = d_accel_full
            self._d_constant:   float = self._total_dist - 2.0 * d_accel_full
        else:
            # ---- Triangular — too short to reach target speed ------------
            # Peak speed is the maximum achievable given the distance.
            self._peak_speed  = math.sqrt(accel_mm_s2 * self._total_dist)
            self._d_accel     = self._total_dist / 2.0
            self._d_constant  = 0.0

        # Decel distance mirrors accel distance.
        self._d_decel: float = self._d_accel

        # Pre-compute phase boundary times for get_speed_at().
        self._t_accel_end: float = self._peak_speed / accel_mm_s2
        self._t_const_end: float = (
            self._t_accel_end
            + (self._d_constant / self._peak_speed if self._peak_speed > 0.0 else 0.0)
        )
        self._t_total: float = self._t_const_end + self._t_accel_end

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def peak_speed_mm_s(self) -> float:
        """Actual peak speed reached during the profile in mm/s.

        Equal to *target_speed_mm_s* for a full trapezoidal profile, or less
        than it for a triangular profile (insufficient distance).
        """
        return self._peak_speed

    @property
    def is_triangular(self) -> bool:
        """``True`` if the profile is triangular (never reaches target speed)."""
        return self._d_constant == 0.0

    # ------------------------------------------------------------------
    # VelocityProfile interface
    # ------------------------------------------------------------------

    def get_speed_at(self, t: float) -> float:
        """Return the commanded speed in mm/s at *t* seconds into the move.

        Uses exact kinematic equations for each phase:

        * Accel  (0 ≤ t < t_accel_end): ``v = accel * t``
        * Cruise (t_accel_end ≤ t < t_const_end): ``v = peak_speed``
        * Decel  (t_const_end ≤ t ≤ t_total): ``v = peak_speed - accel * (t - t_const_end)``

        Values outside ``[0, total_duration()]`` are clamped to 0.

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

        if t < self._t_accel_end:
            # Acceleration phase
            return self.accel_mm_s2 * t

        if t < self._t_const_end:
            # Cruise phase
            return self._peak_speed

        if t <= self._t_total:
            # Deceleration phase
            return max(0.0, self._peak_speed - self.accel_mm_s2 * (t - self._t_const_end))

        # Beyond end of move
        return 0.0

    def total_duration(self) -> float:
        """Return the total move duration in seconds.

        Returns
        -------
        float
            Duration in seconds, always > 0.
        """
        return self._t_total

    def to_segments(
        self,
        segment_length_mm: float = DEFAULT_SEGMENT_LENGTH_MM,
    ) -> list[MoveSegment]:
        """Discretise the profile analytically into constant-speed move segments.

        Rather than sampling the velocity curve in time (as a spline profile
        would), the speed at each spatial position is computed exactly using
        kinematic equations:

        * Accel phase:  ``v(x) = sqrt(2 * accel * x)``
        * Cruise phase: ``v(x) = peak_speed``
        * Decel phase:  ``v(x) = sqrt(2 * accel * (total_dist - x))``

        The speed at the **midpoint** of each segment is used as that
        segment's constant speed.  Segments at the very start or end where
        kinematic speed rounds to nearly zero are floored to
        ``_MIN_SEGMENT_SPEED_MM_S`` to satisfy the :class:`MoveSegment`
        ``speed_mm_s > 0`` invariant.

        Parameters
        ----------
        segment_length_mm:
            Spatial resolution in mm.  Must be > 0.

        Returns
        -------
        list[MoveSegment]
            Ordered list of segments.  The signed sum of all ``distance_mm``
            values equals ``self.distance_mm`` to within one segment length.

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
            step: float = min(segment_length_mm, self._total_dist - pos)

            # Compute speed at the spatial midpoint of this segment.
            mid: float  = pos + step / 2.0
            speed: float = self._speed_at_position(mid)

            # Floor to avoid zero-speed segments at profile endpoints.
            speed = max(speed, _MIN_SEGMENT_SPEED_MM_S)

            segments.append(
                MoveSegment(
                    distance_mm  = step * self._direction,
                    speed_mm_s   = speed,
                    accel_mm_s2  = self.accel_mm_s2,
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
            ``"accel_mm_s2"``, and ``"distance_mm"``.
        """
        return {
            "type":               _PROFILE_TYPE,
            "target_speed_mm_s":  self.target_speed_mm_s,
            "accel_mm_s2":        self.accel_mm_s2,
            "distance_mm":        self.distance_mm,
        }

    @classmethod
    def from_dict(cls, data: ProfileDict) -> TrapezoidalProfile:
        """Reconstruct a :class:`TrapezoidalProfile` from a serialised dict.

        Parameters
        ----------
        data:
            Dict as returned by :meth:`to_dict` or loaded from JSON.

        Returns
        -------
        TrapezoidalProfile
            Fully constructed and validated instance.

        Raises
        ------
        ValueError
            If *data* is missing required keys or contains invalid values.
        """
        try:
            return cls(
                target_speed_mm_s = float(data["target_speed_mm_s"]),
                accel_mm_s2       = float(data["accel_mm_s2"]),
                distance_mm       = float(data["distance_mm"]),
            )
        except KeyError as exc:
            raise ValueError(f"Missing required key in profile dict: {exc}") from exc

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _speed_at_position(self, x: float) -> float:
        """Return the kinematic speed in mm/s at unsigned position *x* mm.

        Uses exact analytical equations for each phase.  The decel boundary
        is measured from the end of the profile.

        Parameters
        ----------
        x:
            Unsigned position in mm along the travel axis, 0 ≤ x ≤ total_dist.

        Returns
        -------
        float
            Speed in mm/s, always ≥ 0.
        """
        # Decel phase starts at this unsigned position.
        decel_start: float = self._total_dist - self._d_decel

        if x <= self._d_accel:
            # Acceleration phase: v = sqrt(2 * a * x)
            return math.sqrt(2.0 * self.accel_mm_s2 * x) if x > 0.0 else 0.0

        if x < decel_start:
            # Cruise phase
            return self._peak_speed

        # Deceleration phase: v = sqrt(2 * a * remaining)
        remaining: float = self._total_dist - x
        return math.sqrt(2.0 * self.accel_mm_s2 * remaining) if remaining > 0.0 else 0.0

    def __repr__(self) -> str:
        shape: str = "triangular" if self.is_triangular else "trapezoidal"
        return (
            f"TrapezoidalProfile("
            f"target_speed={self.target_speed_mm_s} mm/s, "
            f"accel={self.accel_mm_s2} mm/s², "
            f"distance={self.distance_mm} mm, "
            f"peak={self._peak_speed:.3f} mm/s, "
            f"shape={shape})"
        )
