"""
core/profile.py
===============

``DipProfile`` dataclass and JSON persistence helpers.

A ``DipProfile`` holds every parameter needed to execute one dip-coating run.
Profiles are saved as human-readable JSON files in the ``profiles/`` directory
and can be reloaded at any time without losing any information.

Three velocity profile types are supported:

``"trapezoidal"``
    Fixed-shape profile: descend at ``dip_speed_mm_s``, dwell at the bottom,
    ascend at ``withdraw_speed_mm_s``, dwell at the top, repeat ``n_dips``
    times.  Executed on the Arduino with a single ``CMD RUN_PROFILE`` command.
    ``velocity_profile_data`` is left empty ``{}`` for this type.

``"segmented"``
    An explicit, ordered list of move and dwell steps defined by the user in
    the profile editor.  Each step maps directly to one ``CMD MOVE_SEG`` or
    ``CMD DWELL_SEG`` command.  Repetition is baked into the list — add the
    pattern *N* times if *N* dips are desired.  Steps are stored inside
    ``velocity_profile_data["segments"]``.

``"spline"``
    Smooth scipy-interpolated velocity curve (not yet implemented — stub only).
    ``velocity_profile_data`` will hold the spline waypoints when implemented.

JSON segment format for ``"segmented"`` profiles::

    {
      "velocity_profile_data": {
        "segments": [
          {"type": "move",  "distance_mm": -30.0, "speed_mm_s": 5.0,
           "accel_mm_s2": 50.0, "comment": "slow descent into solution"},
          {"type": "dwell", "duration_ms": 2000,
           "comment": "soak"},
          {"type": "move",  "distance_mm": 50.0,  "speed_mm_s": 15.0,
           "accel_mm_s2": 50.0, "comment": "fast withdraw"}
        ]
      }
    }

The ``comment`` field is optional, stored verbatim in JSON, and ignored
during execution — it exists solely for human readability.
"""

import json
import logging
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Final

# Module-level logger — messages appear under "core.profile".
log: Final[logging.Logger] = logging.getLogger(__name__)

# Set of accepted velocity_profile_type strings.
_VALID_PROFILE_TYPES: Final[frozenset[str]] = frozenset({
    "trapezoidal",
    "segmented",
    "spline",
})


# ---------------------------------------------------------------------------
# DipProfile dataclass
# ---------------------------------------------------------------------------

@dataclass
class DipProfile:
    """
    All parameters required to execute one dip-coating run.

    Required fields must be supplied at construction time.  Optional fields
    have sensible defaults.  ``__post_init__`` stamps ``created_at`` and runs
    full field validation so an invalid profile can never be constructed.

    Attributes:
        name:                  Human-readable profile name, used as the display
                               label in the UI and as the default filename stem
                               when saving.  Must not be empty or whitespace.
        dip_speed_mm_s:        Descent speed (mm/s).  Must be > 0.
        withdraw_speed_mm_s:   Ascent/withdrawal speed (mm/s).  Must be > 0.
        accel_mm_s2:           Acceleration and deceleration ramp rate (mm/s²).
                               Must be > 0.
        dip_depth_mm:          Distance to travel below the starting position
                               (mm, positive value).  Must be > 0.
        dwell_bottom_ms:       Time to hold position at the bottom of each dip
                               (ms).  May be 0 for no pause.
        dwell_top_ms:          Time to hold position at the top between dips
                               (ms).  May be 0 for no pause.
        n_dips:                Number of dip cycles.  Must be >= 1.
        notes:                 Free-text operator notes stored in the JSON file.
                               Not used during execution.
        created_at:            ISO-8601 UTC timestamp set automatically when the
                               profile is first constructed.  Preserved on
                               load/save round-trips.
        velocity_profile_type: Selects the motion strategy.  One of
                               ``"trapezoidal"`` | ``"segmented"`` | ``"spline"``.
        velocity_profile_data: Type-specific payload stored as a nested dict.
                               Empty for trapezoidal; contains ``"segments"``
                               list for segmented; spline waypoints (future).
    """

    # --- Required fields (no default — must be supplied at construction) -----
    name:                str
    dip_speed_mm_s:      float
    withdraw_speed_mm_s: float
    accel_mm_s2:         float
    dip_depth_mm:        float
    dwell_bottom_ms:     int
    dwell_top_ms:        int
    n_dips:              int

    # --- Optional fields (have defaults) -------------------------------------
    notes:                 str            = ""
    created_at:            str            = ""
    velocity_profile_type: str            = "trapezoidal"
    velocity_profile_data: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """
        Called automatically by the dataclass machinery after ``__init__``.

        Stamps ``created_at`` with the current UTC time when constructing a
        new profile in code (i.e. when the field was not supplied or is empty).
        Profiles loaded from JSON preserve their original timestamp.

        Then runs ``_validate()`` to reject any out-of-range field values
        before the object is returned to the caller.
        """
        # Stamp creation time only if not already set (e.g. loaded from JSON).
        if not self.created_at:
            self.created_at = datetime.now(timezone.utc).isoformat()

        # Validate all fields — raises ValueError on the first problem found.
        _validate(self)


# ---------------------------------------------------------------------------
# Persistence helpers
# ---------------------------------------------------------------------------

def save_profile(profile: DipProfile, path: str | Path) -> None:
    """
    Serialise a ``DipProfile`` to a JSON file.

    The output is pretty-printed with a 2-space indent so it is easy to read
    and diff in version control.  All dataclass fields are included, meaning
    a ``load_profile`` → ``save_profile`` round-trip is lossless.

    Args:
        profile: The ``DipProfile`` instance to serialise.
        path:    Destination file path.  The parent directory must already
                 exist; this function does not create missing directories.

    Raises:
        OSError: If the file cannot be written (permissions, disk full, etc.).
    """
    path = Path(path)

    # ``asdict`` recursively converts the dataclass (and any nested dataclasses)
    # to plain dicts/lists suitable for json.dump.
    with path.open("w", encoding="utf-8") as fh:
        json.dump(asdict(profile), fh, indent=2)

    log.debug("profile saved → %s", path)


def load_profile(path: str | Path) -> DipProfile:
    """
    Deserialise a ``DipProfile`` from a JSON file.

    Performs explicit type coercion on all numeric fields so that a JSON file
    edited by hand (where integers may be stored as floats or vice-versa)
    still loads correctly.  Unknown extra keys in the JSON are silently ignored
    so that profiles saved by a newer version of the application can still be
    read by an older one.

    Args:
        path: Path to a JSON file previously written by ``save_profile()``.

    Returns:
        A fully validated ``DipProfile`` instance.

    Raises:
        FileNotFoundError:   If ``path`` does not exist.
        json.JSONDecodeError: If the file is not valid JSON.
        ValueError:          If a required field is missing or a value fails
                             validation (e.g. negative speed).
        KeyError:            If a required top-level field is absent from the
                             JSON object (wrapped in ``ValueError`` with a
                             descriptive message).
    """
    path = Path(path)

    # Open and decode the JSON file.
    with path.open(encoding="utf-8") as fh:
        data: dict[str, Any] = json.load(fh)

    # Extract and coerce all fields.  Using explicit float()/int() calls means
    # a JSON integer like 10 is accepted where a float is expected, and vice versa.
    try:
        profile = DipProfile(
            name                  = str(data["name"]),
            dip_speed_mm_s        = float(data["dip_speed_mm_s"]),
            withdraw_speed_mm_s   = float(data["withdraw_speed_mm_s"]),
            accel_mm_s2           = float(data["accel_mm_s2"]),
            dip_depth_mm          = float(data["dip_depth_mm"]),
            dwell_bottom_ms       = int(data["dwell_bottom_ms"]),
            dwell_top_ms          = int(data["dwell_top_ms"]),
            n_dips                = int(data["n_dips"]),
            # Optional fields fall back to safe defaults if absent.
            notes                 = str(data.get("notes", "")),
            created_at            = str(data.get("created_at", "")),
            velocity_profile_type = str(data.get("velocity_profile_type", "trapezoidal")),
            velocity_profile_data = dict(data.get("velocity_profile_data", {})),
        )
    except KeyError as exc:
        raise ValueError(
            f"Profile file {path} is missing required field: {exc}"
        ) from exc

    log.debug("profile loaded ← %s", path)
    return profile


def list_profiles(directory: str | Path) -> list[DipProfile]:
    """
    Load every valid JSON profile from a directory.

    Files that fail to parse are logged as warnings and skipped — a single
    corrupt or incompatible file does not prevent the rest from loading.
    The returned list is sorted alphabetically by filename so the ordering
    is deterministic and independent of filesystem order.

    Args:
        directory: Path to the ``profiles/`` directory to scan.

    Returns:
        A list of ``DipProfile`` instances, sorted by filename (ascending).
        Returns an empty list if the directory contains no ``*.json`` files
        or if all files fail validation.
    """
    directory = Path(directory)
    profiles: list[DipProfile] = []

    # glob("*.json") returns files in filesystem order — sort for determinism.
    for path in sorted(directory.glob("*.json")):
        try:
            profiles.append(load_profile(path))
        except Exception as exc:
            # Log and skip — one bad file must not block the others.
            log.warning("skipping %s: %s", path.name, exc)

    return profiles


# ---------------------------------------------------------------------------
# Internal validation helper
# ---------------------------------------------------------------------------

def _validate(p: DipProfile) -> None:
    """
    Validate all fields of a ``DipProfile`` and raise ``ValueError`` if any
    are out of range.

    Collects *all* errors before raising so the caller sees the full list of
    problems in one exception rather than fixing them one at a time.

    The ``!(x > 0)`` pattern is used for positive-value checks instead of
    ``x <= 0`` because ``!(NaN > 0)`` correctly evaluates to ``True``,
    catching float("nan") inputs that ``x <= 0`` would silently pass.

    Args:
        p: The ``DipProfile`` instance to validate.

    Raises:
        ValueError: If one or more fields are invalid.  The message contains
                    a semicolon-separated list of all violations found.
    """
    errors: list[str] = []

    # --- Name ----------------------------------------------------------------
    if not p.name or not p.name.strip():
        errors.append("name must not be empty or whitespace")

    # --- Motion parameters (must be strictly positive) -----------------------
    if not (p.dip_speed_mm_s > 0):
        errors.append(f"dip_speed_mm_s must be > 0, got {p.dip_speed_mm_s}")
    if not (p.withdraw_speed_mm_s > 0):
        errors.append(f"withdraw_speed_mm_s must be > 0, got {p.withdraw_speed_mm_s}")
    if not (p.accel_mm_s2 > 0):
        errors.append(f"accel_mm_s2 must be > 0, got {p.accel_mm_s2}")
    if not (p.dip_depth_mm > 0):
        errors.append(f"dip_depth_mm must be > 0, got {p.dip_depth_mm}")

    # --- Dwell times (zero is valid — means no pause) ------------------------
    if p.dwell_bottom_ms < 0:
        errors.append(f"dwell_bottom_ms must be >= 0, got {p.dwell_bottom_ms}")
    if p.dwell_top_ms < 0:
        errors.append(f"dwell_top_ms must be >= 0, got {p.dwell_top_ms}")

    # --- Number of dips (at least one) ---------------------------------------
    if p.n_dips < 1:
        errors.append(f"n_dips must be >= 1, got {p.n_dips}")

    # --- Velocity profile type -----------------------------------------------
    if p.velocity_profile_type not in _VALID_PROFILE_TYPES:
        errors.append(
            f"velocity_profile_type must be one of {sorted(_VALID_PROFILE_TYPES)}, "
            f"got {p.velocity_profile_type!r}"
        )

    # --- Raise if any errors were found --------------------------------------
    if errors:
        raise ValueError("Invalid profile: " + "; ".join(errors))
