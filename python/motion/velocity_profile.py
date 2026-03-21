"""
motion/velocity_profile.py
==========================

Abstract base class and shared data types for all velocity profile
implementations used in the dip coater.

Design overview
---------------
Python is responsible for all motion mathematics.  The Arduino executes
simple move primitives (``MOVE_SEG`` / ``DWELL_SEG``) and knows nothing
about profile shapes.  Every concrete profile must implement this interface
so that :class:`~core.command_interface.CommandInterface` can treat
trapezoidal, segmented, and spline profiles uniformly.

The two public types defined here are:

* :class:`MoveSegment` — a single constant-speed move step ready to be
  streamed to the Arduino as a ``CMD MOVE_SEG`` command.
* :class:`VelocityProfile` — ABC that all profile classes must subclass.

Usage example::

    from motion.trapezoidal_profile import TrapezoidalProfile

    profile = TrapezoidalProfile(
        target_speed_mm_s=10.0,
        accel_mm_s2=50.0,
        distance_mm=100.0,
    )

    print(profile.total_duration())       # → float, seconds
    print(profile.get_speed_at(0.5))      # → float, mm/s at t=0.5 s
    segments = profile.to_segments(0.5)   # → list[MoveSegment]
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Final, TypeAlias

# ---------------------------------------------------------------------------
# Type aliases
# ---------------------------------------------------------------------------

# Serialised form used by to_dict() / from_dict().  Always a flat
# string-keyed dict so it round-trips through JSON without loss.
ProfileDict: TypeAlias = dict[str, Any]

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Default spatial resolution used when discretising a continuous profile into
# move segments.  0.5 mm gives smooth motion on the uStepperS32 while keeping
# segment count well within the Arduino's MOVE_SEG_BUFFER_SIZE (64 segments).
DEFAULT_SEGMENT_LENGTH_MM: Final[float] = 0.5


# ---------------------------------------------------------------------------
# MoveSegment
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class MoveSegment:
    """A single constant-speed move primitive ready for streaming to the Arduino.

    Each :class:`MoveSegment` maps directly to one ``CMD MOVE_SEG`` command::

        CMD MOVE_SEG <distance_mm> <speed_mm_s> <accel_mm_s2>

    The ``distance_mm`` value is signed:

    * **Negative** → move toward the substrate (descending / dipping).
    * **Positive** → move away from the substrate (ascending / withdrawing).

    Attributes
    ----------
    distance_mm:
        Displacement in millimetres.  Positive = upward, negative = downward.
        Must not be zero.
    speed_mm_s:
        Target traversal speed in mm/s.  Must be strictly positive — the sign
        of ``distance_mm`` determines direction, not this field.
    accel_mm_s2:
        Acceleration (and deceleration) magnitude used for this segment in
        mm/s².  Must be strictly positive.

    Raises
    ------
    ValueError
        If any field fails its invariant (see :meth:`__post_init__`).

    Examples
    --------
    >>> seg = MoveSegment(distance_mm=-10.0, speed_mm_s=5.0, accel_mm_s2=50.0)
    >>> seg.distance_mm
    -10.0
    """

    distance_mm:  float
    speed_mm_s:   float
    accel_mm_s2:  float

    def __post_init__(self) -> None:
        """Validate field invariants immediately after construction.

        Raises
        ------
        ValueError
            If ``distance_mm`` is zero, ``speed_mm_s`` is not strictly
            positive, or ``accel_mm_s2`` is not strictly positive.
        """
        # Collect all errors before raising so the caller sees them all at once.
        errors: list[str] = []

        if not (self.distance_mm != 0.0) or not math.isfinite(self.distance_mm):
            errors.append(
                f"distance_mm must be a finite non-zero value, got {self.distance_mm!r}"
            )
        if not (self.speed_mm_s > 0.0) or not math.isfinite(self.speed_mm_s):
            errors.append(
                f"speed_mm_s must be finite and > 0, got {self.speed_mm_s!r}"
            )
        if not (self.accel_mm_s2 > 0.0) or not math.isfinite(self.accel_mm_s2):
            errors.append(
                f"accel_mm_s2 must be finite and > 0, got {self.accel_mm_s2!r}"
            )

        if errors:
            raise ValueError("; ".join(errors))


# ---------------------------------------------------------------------------
# VelocityProfile — abstract base class
# ---------------------------------------------------------------------------

class VelocityProfile(ABC):
    """Abstract base class for all dip coater velocity profiles.

    Concrete subclasses must implement five methods:

    * :meth:`get_speed_at` — instantaneous commanded speed at a given time.
    * :meth:`total_duration` — total move duration in seconds.
    * :meth:`to_segments` — discretise the profile into :class:`MoveSegment`
      objects ready for streaming to the Arduino.
    * :meth:`to_dict` — serialise the profile parameters to a plain dict for
      storage in :attr:`~core.profile.DipProfile.velocity_profile_data`.
    * :meth:`from_dict` — reconstruct a profile instance from that dict.

    All concrete profiles are interchangeable at call sites that accept
    ``VelocityProfile`` — :class:`~core.command_interface.CommandInterface`
    uses ``isinstance`` checks only to decide whether to take the fast
    ``CMD RUN_PROFILE`` shortcut (trapezoidal) or stream segments.

    Notes
    -----
    * ``get_speed_at`` returns a *magnitude* (always ≥ 0).  Direction is
      encoded in the sign of ``distance_mm`` in each :class:`MoveSegment`.
    * ``to_segments`` is the primary output for the Arduino streaming path.
      The segment count must not exceed ``MOVE_SEG_BUFFER_SIZE`` (64) on the
      Arduino.  Callers should assert this before sending.
    """

    # ------------------------------------------------------------------
    # Abstract interface
    # ------------------------------------------------------------------

    @abstractmethod
    def get_speed_at(self, t: float) -> float:
        """Return the commanded speed in mm/s at *t* seconds into the move.

        The returned value is always non-negative — direction is not encoded
        here.  Values outside ``[0, total_duration()]`` are clamped by each
        concrete implementation.

        Parameters
        ----------
        t:
            Time into the move in seconds.  May be negative or greater than
            :meth:`total_duration`; implementations must handle gracefully.

        Returns
        -------
        float
            Commanded speed in mm/s, always ≥ 0.
        """

    @abstractmethod
    def total_duration(self) -> float:
        """Return the total move duration in seconds.

        Returns
        -------
        float
            Duration in seconds, always > 0 for a valid profile.
        """

    @abstractmethod
    def to_segments(
        self,
        segment_length_mm: float = DEFAULT_SEGMENT_LENGTH_MM,
    ) -> list[MoveSegment]:
        """Discretise the profile into constant-speed :class:`MoveSegment` objects.

        The list is ready to be streamed to the Arduino as a sequence of
        ``CMD MOVE_SEG`` commands via
        :meth:`~core.command_interface.CommandInterface`.

        Parameters
        ----------
        segment_length_mm:
            Spatial resolution of the discretisation in mm.  Smaller values
            produce smoother motion at the cost of more segments.  Defaults to
            :data:`DEFAULT_SEGMENT_LENGTH_MM` (0.5 mm).  Must be > 0.

        Returns
        -------
        list[MoveSegment]
            Ordered list of move segments.  The sum of ``|distance_mm|`` over
            all segments equals the total profile distance to within one
            segment length.

        Notes
        -----
        The Arduino's ``MOVE_SEG_BUFFER_SIZE`` is 64 segments.  Callers are
        responsible for asserting the returned list length before sending.
        Trapezoidal profiles compute this analytically; spline profiles sample
        the scipy interpolant at each spatial step.
        """

    @abstractmethod
    def to_dict(self) -> ProfileDict:
        """Serialise the profile parameters to a JSON-safe dict.

        The returned dict is stored verbatim in
        :attr:`~core.profile.DipProfile.velocity_profile_data` and round-trips
        through ``json.dumps`` / ``json.loads`` without loss.

        Returns
        -------
        ProfileDict
            All parameters needed to reconstruct this profile via
            :meth:`from_dict`.  Must include at least a ``"type"`` key
            whose value matches the profile's class identifier string.
        """

    @classmethod
    @abstractmethod
    def from_dict(cls, data: ProfileDict) -> VelocityProfile:
        """Reconstruct a profile instance from a serialised dict.

        This is the inverse of :meth:`to_dict`.  Implementations must validate
        all required keys and raise :exc:`ValueError` on missing or malformed
        data so callers get a clear error message rather than a ``KeyError``.

        Parameters
        ----------
        data:
            Dict as returned by :meth:`to_dict` or loaded from JSON.

        Returns
        -------
        VelocityProfile
            A fully constructed and validated profile instance.

        Raises
        ------
        ValueError
            If *data* is missing required keys or contains invalid values.
        """

    # ------------------------------------------------------------------
    # Concrete helpers (available to all subclasses)
    # ------------------------------------------------------------------

    def segment_count(
        self,
        segment_length_mm: float = DEFAULT_SEGMENT_LENGTH_MM,
    ) -> int:
        """Return the number of segments :meth:`to_segments` would produce.

        This is a convenience wrapper so callers can check the count before
        calling :meth:`to_segments`, avoiding an unnecessary full computation.
        The default implementation simply calls :meth:`to_segments` and counts;
        subclasses may override with an analytical formula.

        Parameters
        ----------
        segment_length_mm:
            Passed through to :meth:`to_segments`.

        Returns
        -------
        int
            Number of :class:`MoveSegment` objects that would be produced.
        """
        return len(self.to_segments(segment_length_mm))

    def fits_in_arduino_buffer(
        self,
        segment_length_mm: float = DEFAULT_SEGMENT_LENGTH_MM,
        buffer_size: int = 64,
    ) -> bool:
        """Return ``True`` if the segment count fits in the Arduino buffer.

        Parameters
        ----------
        segment_length_mm:
            Spatial resolution passed to :meth:`segment_count`.
        buffer_size:
            Maximum number of segments the Arduino can hold.  Default is 64,
            matching ``MOVE_SEG_BUFFER_SIZE`` in the firmware.

        Returns
        -------
        bool
            ``True`` if ``segment_count(segment_length_mm) <= buffer_size``.
        """
        return self.segment_count(segment_length_mm) <= buffer_size
