"""
motion/spline_profile.py
========================

Stub implementation of :class:`~motion.velocity_profile.VelocityProfile` for
spline-based velocity profiles.

All methods raise :exc:`NotImplementedError`.  The stub exists so that:

* ``from motion.spline_profile import SplineProfile`` never causes an
  :exc:`ImportError`.
* The ``"spline"`` branch in
  :meth:`~core.command_interface.CommandInterface.run` is wired up and ready
  for the future profile editor tab (Step 23 in the build order).
* Tests that verify the ``"spline"`` dispatch path get a clear, descriptive
  error rather than an ``AttributeError`` or silent skip.

Future implementation notes
---------------------------
The spline profile will accept a list of (position_mm, speed_mm_s) waypoints
and fit a smooth interpolant (e.g. scipy ``CubicSpline``) through them.
``to_segments()`` will sample the interpolant at ``segment_length_mm``
intervals and produce a :class:`~motion.velocity_profile.MoveSegment` list
ready for streaming to the Arduino.

The profile editor tab (``ui/tabs/profile_editor_tab.py``) will draw the
waypoints on a canvas and save them into
``DipProfile.velocity_profile_data["waypoints"]``.
"""

from __future__ import annotations

from typing import Any

from motion.velocity_profile import (
    DEFAULT_SEGMENT_LENGTH_MM,
    MoveSegment,
    ProfileDict,
    VelocityProfile,
)

_NOT_IMPLEMENTED_MSG = (
    "SplineProfile is not yet implemented — "
    "use the profile editor tab to create custom velocity profiles."
)


class SplineProfile(VelocityProfile):
    """Stub spline velocity profile.

    All methods raise :exc:`NotImplementedError` with a descriptive message.
    The class signature is complete so imports and isinstance checks work
    correctly throughout the codebase.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        raise NotImplementedError(_NOT_IMPLEMENTED_MSG)

    def get_speed_at(self, t: float) -> float:
        raise NotImplementedError(_NOT_IMPLEMENTED_MSG)

    def total_duration(self) -> float:
        raise NotImplementedError(_NOT_IMPLEMENTED_MSG)

    def to_segments(
        self,
        segment_length_mm: float = DEFAULT_SEGMENT_LENGTH_MM,
    ) -> list[MoveSegment]:
        raise NotImplementedError(_NOT_IMPLEMENTED_MSG)

    def to_dict(self) -> ProfileDict:
        raise NotImplementedError(_NOT_IMPLEMENTED_MSG)

    @classmethod
    def from_dict(cls, data: ProfileDict) -> "SplineProfile":
        raise NotImplementedError(_NOT_IMPLEMENTED_MSG)
