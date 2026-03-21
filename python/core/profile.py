"""
core/profile.py

DipProfile dataclass and JSON persistence helpers.

A DipProfile stores all parameters needed to execute one dip-coating run.
Profiles are saved as human-readable JSON files in the profiles/ directory
and can be loaded back into the application at any time.
"""

import json
import logging
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

log = logging.getLogger(__name__)


@dataclass
class DipProfile:
    """All parameters for one dip-coating run."""
    name:                  str
    dip_speed_mm_s:        float
    withdraw_speed_mm_s:   float
    accel_mm_s2:           float
    dip_depth_mm:          float
    dwell_bottom_ms:       int
    dwell_top_ms:          int
    n_dips:                int
    notes:                 str  = ""
    created_at:            str  = ""
    velocity_profile_type: str  = "trapezoidal"   # "trapezoidal" | "spline"
    velocity_profile_data: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        # Stamp creation time if not supplied (e.g. when constructing in code).
        if not self.created_at:
            self.created_at = datetime.now(timezone.utc).isoformat()
        _validate(self)


def save_profile(profile: DipProfile, path: str | Path) -> None:
    """
    Serialise a DipProfile to a JSON file.

    Args:
        profile: The profile to save.
        path:    Destination file path.  Parent directories must exist.
    """
    path = Path(path)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(asdict(profile), fh, indent=2)
    log.debug("profile saved to %s", path)


def load_profile(path: str | Path) -> DipProfile:
    """
    Load a DipProfile from a JSON file.

    Args:
        path: Path to a JSON file previously written by save_profile().

    Returns:
        DipProfile instance.

    Raises:
        ValueError:  If required fields are missing or values are invalid.
        FileNotFoundError: If the file does not exist.
        json.JSONDecodeError: If the file is not valid JSON.
    """
    path = Path(path)
    with path.open(encoding="utf-8") as fh:
        data = json.load(fh)

    # Pull required fields — raise KeyError (descriptive) if any are absent.
    try:
        profile = DipProfile(
            name                  = data["name"],
            dip_speed_mm_s        = float(data["dip_speed_mm_s"]),
            withdraw_speed_mm_s   = float(data["withdraw_speed_mm_s"]),
            accel_mm_s2           = float(data["accel_mm_s2"]),
            dip_depth_mm          = float(data["dip_depth_mm"]),
            dwell_bottom_ms       = int(data["dwell_bottom_ms"]),
            dwell_top_ms          = int(data["dwell_top_ms"]),
            n_dips                = int(data["n_dips"]),
            notes                 = data.get("notes", ""),
            created_at            = data.get("created_at", ""),
            velocity_profile_type = data.get("velocity_profile_type", "trapezoidal"),
            velocity_profile_data = data.get("velocity_profile_data", {}),
        )
    except KeyError as exc:
        raise ValueError(f"Profile file {path} is missing required field: {exc}") from exc

    log.debug("profile loaded from %s", path)
    return profile


def list_profiles(directory: str | Path) -> list[DipProfile]:
    """
    Load all valid JSON profiles from a directory.

    Files that fail to parse are logged as warnings and skipped — one bad
    file does not prevent the rest from loading.

    Args:
        directory: Path to the profiles/ directory.

    Returns:
        List of DipProfile instances sorted by name.
    """
    directory = Path(directory)
    profiles = []
    for path in sorted(directory.glob("*.json")):
        try:
            profiles.append(load_profile(path))
        except Exception as exc:
            log.warning("skipping %s: %s", path.name, exc)
    return profiles


# ---------------------------------------------------------------------------
# Internal validation
# ---------------------------------------------------------------------------

def _validate(p: DipProfile) -> None:
    """Raise ValueError if any profile field is out of range."""
    errors = []

    if not p.name or not p.name.strip():
        errors.append("name must not be empty")
    if not (p.dip_speed_mm_s > 0):
        errors.append(f"dip_speed_mm_s must be > 0, got {p.dip_speed_mm_s}")
    if not (p.withdraw_speed_mm_s > 0):
        errors.append(f"withdraw_speed_mm_s must be > 0, got {p.withdraw_speed_mm_s}")
    if not (p.accel_mm_s2 > 0):
        errors.append(f"accel_mm_s2 must be > 0, got {p.accel_mm_s2}")
    if not (p.dip_depth_mm > 0):
        errors.append(f"dip_depth_mm must be > 0, got {p.dip_depth_mm}")
    if p.dwell_bottom_ms < 0:
        errors.append(f"dwell_bottom_ms must be >= 0, got {p.dwell_bottom_ms}")
    if p.dwell_top_ms < 0:
        errors.append(f"dwell_top_ms must be >= 0, got {p.dwell_top_ms}")
    if p.n_dips < 1:
        errors.append(f"n_dips must be >= 1, got {p.n_dips}")
    if p.velocity_profile_type not in ("trapezoidal", "segmented", "spline"):
        errors.append(f"velocity_profile_type must be 'trapezoidal', 'segmented', or 'spline', got {p.velocity_profile_type!r}")

    if errors:
        raise ValueError("Invalid profile: " + "; ".join(errors))
