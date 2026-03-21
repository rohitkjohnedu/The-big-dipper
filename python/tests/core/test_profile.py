"""
tests/core/test_profile.py
===========================

Tests for ``core/profile.py`` — ``DipProfile`` dataclass and JSON persistence.

Covers:
- Construction with valid and invalid field values.
- Automatic ``created_at`` stamping and preservation on round-trip.
- ``save_profile`` / ``load_profile`` round-trip fidelity.
- Error handling: missing fields, invalid JSON, file not found.
- ``list_profiles``: ordering, skipping corrupt files, empty directory.

All tests run without hardware — no serial port required.
"""

import json
from pathlib import Path
from typing import Any, Final, Union

import pytest

from core.profile import DipProfile, save_profile, load_profile, list_profiles


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def make_profile(**kwargs: Any) -> DipProfile:
    """
    Construct a valid ``DipProfile`` with sensible defaults.

    Any keyword argument overrides the corresponding default, allowing each
    test to change only the field it cares about.

    Args:
        **kwargs: Field overrides passed directly to ``DipProfile``.

    Returns:
        A fully validated ``DipProfile`` instance.
    """
    defaults: dict[str, Any] = dict(
        name                = "test",
        dip_speed_mm_s      = 10.0,
        withdraw_speed_mm_s = 15.0,
        accel_mm_s2         = 50.0,
        dip_depth_mm        = 100.0,
        dwell_bottom_ms     = 1000,
        dwell_top_ms        = 500,
        n_dips              = 3,
    )
    defaults.update(kwargs)
    return DipProfile(**defaults)


# ---------------------------------------------------------------------------
# Construction and validation
# ---------------------------------------------------------------------------

class TestConstruction:
    """Verify DipProfile construction and field defaults."""

    def test_valid_profile_constructs(self) -> None:
        """A profile with all required fields in range must construct without error."""
        p: DipProfile = make_profile()
        assert p.name == "test"

    def test_created_at_auto_stamped(self) -> None:
        """created_at must be set automatically when not supplied."""
        p: DipProfile = make_profile()
        assert p.created_at != ""

    def test_created_at_preserved_if_supplied(self) -> None:
        """An explicit created_at value must survive construction unchanged."""
        timestamp: str = "2026-01-01T00:00:00+00:00"
        p: DipProfile = make_profile(created_at=timestamp)
        assert p.created_at == timestamp

    def test_default_velocity_profile_type(self) -> None:
        """velocity_profile_type must default to 'trapezoidal'."""
        expected_type: str = "trapezoidal"
        assert make_profile().velocity_profile_type == expected_type

    def test_segmented_velocity_profile_type(self) -> None:
        """velocity_profile_type = 'segmented' must be accepted."""
        p: DipProfile = make_profile(velocity_profile_type="segmented")
        assert p.velocity_profile_type == "segmented"

    def test_spline_velocity_profile_type(self) -> None:
        """velocity_profile_type = 'spline' must be accepted."""
        p: DipProfile = make_profile(velocity_profile_type="spline")
        assert p.velocity_profile_type == "spline"

    def test_zero_dwell_allowed(self) -> None:
        """dwell times of 0 ms are valid — they mean no pause at that position."""
        p: DipProfile = make_profile(dwell_bottom_ms=0, dwell_top_ms=0)
        assert p.dwell_bottom_ms == 0
        assert p.dwell_top_ms == 0


class TestValidation:
    """Verify that out-of-range field values are rejected at construction time."""

    # Each tuple is (field_name, bad_value) — the bad value must raise ValueError.
    _INVALID_CASES: Final[list[tuple[str, Union[float, int]]]] = [
        ("dip_speed_mm_s",      0.0),    # zero speed not allowed
        ("dip_speed_mm_s",      -5.0),   # negative speed not allowed
        ("withdraw_speed_mm_s", 0.0),    # zero withdraw speed not allowed
        ("accel_mm_s2",         0.0),    # zero acceleration not allowed
        ("dip_depth_mm",        0.0),    # zero depth not allowed
        ("dip_depth_mm",        -10.0),  # negative depth not allowed
        ("n_dips",              0),      # at least one dip required
        ("n_dips",              -1),     # negative dips not allowed
        ("dwell_bottom_ms",     -1),     # negative dwell time not allowed
        ("dwell_top_ms",        -1),     # negative dwell time not allowed
    ]

    @pytest.mark.parametrize("field,bad_value", _INVALID_CASES)
    def test_invalid_field_raises(self, field: str, bad_value: Union[float, int]) -> None:
        """Each out-of-range value must raise ValueError."""
        with pytest.raises(ValueError):
            make_profile(**{field: bad_value})

    def test_empty_name_raises(self) -> None:
        """An empty name string must raise ValueError."""
        with pytest.raises(ValueError):
            make_profile(name="")

    def test_whitespace_name_raises(self) -> None:
        """A name containing only whitespace must raise ValueError."""
        with pytest.raises(ValueError):
            make_profile(name="   ")

    def test_invalid_velocity_profile_type_raises(self) -> None:
        """An unrecognised velocity_profile_type string must raise ValueError."""
        with pytest.raises(ValueError):
            make_profile(velocity_profile_type="cubic")


# ---------------------------------------------------------------------------
# Save / load round-trip
# ---------------------------------------------------------------------------

class TestSaveLoad:
    """Verify that save_profile / load_profile preserve all field values exactly."""

    def test_round_trip(self, tmp_path: Path) -> None:
        """Every field must survive a save → load round-trip without modification."""
        p: DipProfile = make_profile()
        path: Path = tmp_path / "profile.json"
        save_profile(p, path)
        loaded: DipProfile = load_profile(path)

        # Explicit typed intermediates allow pytest.approx to receive a known float.
        expected_dip_speed: float = p.dip_speed_mm_s
        expected_withdraw:  float = p.withdraw_speed_mm_s
        expected_accel:     float = p.accel_mm_s2
        expected_depth:     float = p.dip_depth_mm

        assert loaded.name                  == p.name
        assert loaded.dip_speed_mm_s        == pytest.approx(expected_dip_speed)
        assert loaded.withdraw_speed_mm_s   == pytest.approx(expected_withdraw)
        assert loaded.accel_mm_s2           == pytest.approx(expected_accel)
        assert loaded.dip_depth_mm          == pytest.approx(expected_depth)
        assert loaded.dwell_bottom_ms       == p.dwell_bottom_ms
        assert loaded.dwell_top_ms          == p.dwell_top_ms
        assert loaded.n_dips                == p.n_dips
        assert loaded.notes                 == p.notes
        assert loaded.created_at            == p.created_at
        assert loaded.velocity_profile_type == p.velocity_profile_type

    def test_saved_file_is_valid_json(self, tmp_path: Path) -> None:
        """The saved file must be parseable by the standard json module."""
        path: Path = tmp_path / "profile.json"
        save_profile(make_profile(), path)
        with path.open() as fh:
            data: dict[str, Any] = json.load(fh)
        expected_name: str = "test"
        assert data["name"] == expected_name

    def test_file_not_found_raises(self, tmp_path: Path) -> None:
        """Loading a non-existent file must raise FileNotFoundError."""
        missing_path: Path = tmp_path / "nonexistent.json"
        with pytest.raises(FileNotFoundError):
            load_profile(missing_path)

    def test_missing_required_field_raises(self, tmp_path: Path) -> None:
        """A JSON file missing required fields must raise ValueError or KeyError."""
        path: Path = tmp_path / "bad.json"
        incomplete: dict[str, Any] = {"name": "x", "dip_speed_mm_s": 10}
        path.write_text(json.dumps(incomplete), encoding="utf-8")
        with pytest.raises((ValueError, KeyError)):
            load_profile(path)

    def test_invalid_json_raises(self, tmp_path: Path) -> None:
        """A file containing invalid JSON must raise json.JSONDecodeError."""
        path: Path = tmp_path / "bad.json"
        path.write_text("not json {{{", encoding="utf-8")
        with pytest.raises(json.JSONDecodeError):
            load_profile(path)

    def test_default_profile_loads(self) -> None:
        """
        The committed profiles/default.json must always load without error.

        This test acts as a regression guard — any accidental corruption of
        default.json will be caught here before it reaches production.
        """
        path: Path = Path(__file__).parent.parent.parent / "profiles" / "default.json"
        p: DipProfile = load_profile(path)
        expected_name: str = "default"
        assert p.name == expected_name


# ---------------------------------------------------------------------------
# list_profiles
# ---------------------------------------------------------------------------

class TestListProfiles:
    """Verify list_profiles scanning, ordering, and fault tolerance."""

    def test_returns_all_valid_profiles(self, tmp_path: Path) -> None:
        """All valid JSON files in the directory must be returned."""
        names: list[str] = ["alpha", "beta", "gamma"]
        for name in names:
            save_profile(make_profile(name=name), tmp_path / f"{name}.json")

        profiles: list[DipProfile] = list_profiles(tmp_path)
        returned_names: list[str] = [p.name for p in profiles]

        assert len(profiles) == 3
        assert returned_names == names

    def test_skips_invalid_files(self, tmp_path: Path) -> None:
        """A corrupt JSON file must be skipped; valid files must still be returned."""
        save_profile(make_profile(name="good"), tmp_path / "good.json")
        (tmp_path / "bad.json").write_text("not json", encoding="utf-8")

        profiles: list[DipProfile] = list_profiles(tmp_path)
        expected_count: int = 1
        expected_name: str = "good"

        assert len(profiles) == expected_count
        assert profiles[0].name == expected_name

    def test_empty_directory_returns_empty_list(self, tmp_path: Path) -> None:
        """A directory with no JSON files must return an empty list."""
        result: list[DipProfile] = list_profiles(tmp_path)
        assert result == []

    def test_sorted_by_filename(self, tmp_path: Path) -> None:
        """Profiles must be returned sorted alphabetically by filename."""
        unsorted_names: list[str] = ["zebra", "apple", "mango"]
        for name in unsorted_names:
            save_profile(make_profile(name=name), tmp_path / f"{name}.json")

        profiles: list[DipProfile] = list_profiles(tmp_path)
        returned_names: list[str] = [p.name for p in profiles]
        expected_names: list[str] = ["apple", "mango", "zebra"]

        assert returned_names == expected_names
