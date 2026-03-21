"""
tests/core/test_profile.py

Tests for core/profile.py — DipProfile dataclass and JSON persistence.
All tests run without hardware.
"""

import json
import pytest
from pathlib import Path
from core.profile import DipProfile, save_profile, load_profile, list_profiles


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_profile(**kwargs) -> DipProfile:
    defaults = dict(
        name                  = "test",
        dip_speed_mm_s        = 10.0,
        withdraw_speed_mm_s   = 15.0,
        accel_mm_s2           = 50.0,
        dip_depth_mm          = 100.0,
        dwell_bottom_ms       = 1000,
        dwell_top_ms          = 500,
        n_dips                = 3,
    )
    defaults.update(kwargs)
    return DipProfile(**defaults)


# ---------------------------------------------------------------------------
# Construction and validation
# ---------------------------------------------------------------------------

class TestConstruction:
    def test_valid_profile_constructs(self):
        p = make_profile()
        assert p.name == "test"

    def test_created_at_auto_stamped(self):
        p = make_profile()
        assert p.created_at != ""

    def test_created_at_preserved_if_supplied(self):
        p = make_profile(created_at="2026-01-01T00:00:00+00:00")
        assert p.created_at == "2026-01-01T00:00:00+00:00"

    def test_default_velocity_profile_type(self):
        assert make_profile().velocity_profile_type == "trapezoidal"

    def test_spline_velocity_profile_type(self):
        p = make_profile(velocity_profile_type="spline")
        assert p.velocity_profile_type == "spline"

    def test_zero_dwell_allowed(self):
        # dwell times of 0 are valid — no pause at top or bottom
        make_profile(dwell_bottom_ms=0, dwell_top_ms=0)


class TestValidation:
    @pytest.mark.parametrize("field,bad_value", [
        ("dip_speed_mm_s",      0.0),
        ("dip_speed_mm_s",      -5.0),
        ("withdraw_speed_mm_s", 0.0),
        ("accel_mm_s2",         0.0),
        ("dip_depth_mm",        0.0),
        ("dip_depth_mm",        -10.0),
        ("n_dips",              0),
        ("n_dips",              -1),
        ("dwell_bottom_ms",     -1),
        ("dwell_top_ms",        -1),
    ])
    def test_invalid_field_raises(self, field, bad_value):
        with pytest.raises(ValueError):
            make_profile(**{field: bad_value})

    def test_empty_name_raises(self):
        with pytest.raises(ValueError):
            make_profile(name="")

    def test_whitespace_name_raises(self):
        with pytest.raises(ValueError):
            make_profile(name="   ")

    def test_invalid_velocity_profile_type_raises(self):
        with pytest.raises(ValueError):
            make_profile(velocity_profile_type="cubic")


# ---------------------------------------------------------------------------
# Save / load round-trip
# ---------------------------------------------------------------------------

class TestSaveLoad:
    def test_round_trip(self, tmp_path):
        p = make_profile()
        path = tmp_path / "profile.json"
        save_profile(p, path)
        loaded = load_profile(path)
        assert loaded.name                  == p.name
        assert loaded.dip_speed_mm_s        == pytest.approx(p.dip_speed_mm_s)
        assert loaded.withdraw_speed_mm_s   == pytest.approx(p.withdraw_speed_mm_s)
        assert loaded.accel_mm_s2           == pytest.approx(p.accel_mm_s2)
        assert loaded.dip_depth_mm          == pytest.approx(p.dip_depth_mm)
        assert loaded.dwell_bottom_ms       == p.dwell_bottom_ms
        assert loaded.dwell_top_ms          == p.dwell_top_ms
        assert loaded.n_dips                == p.n_dips
        assert loaded.notes                 == p.notes
        assert loaded.created_at            == p.created_at
        assert loaded.velocity_profile_type == p.velocity_profile_type

    def test_saved_file_is_valid_json(self, tmp_path):
        path = tmp_path / "profile.json"
        save_profile(make_profile(), path)
        with path.open() as fh:
            data = json.load(fh)
        assert data["name"] == "test"

    def test_file_not_found_raises(self, tmp_path):
        with pytest.raises(FileNotFoundError):
            load_profile(tmp_path / "nonexistent.json")

    def test_missing_required_field_raises(self, tmp_path):
        path = tmp_path / "bad.json"
        path.write_text(json.dumps({"name": "x", "dip_speed_mm_s": 10}), encoding="utf-8")
        with pytest.raises((ValueError, KeyError)):
            load_profile(path)

    def test_invalid_json_raises(self, tmp_path):
        path = tmp_path / "bad.json"
        path.write_text("not json {{{", encoding="utf-8")
        with pytest.raises(json.JSONDecodeError):
            load_profile(path)

    def test_default_profile_loads(self):
        # Verifies the committed default.json is always valid
        path = Path(__file__).parent.parent.parent / "profiles" / "default.json"
        p = load_profile(path)
        assert p.name == "default"


# ---------------------------------------------------------------------------
# list_profiles
# ---------------------------------------------------------------------------

class TestListProfiles:
    def test_returns_all_valid_profiles(self, tmp_path):
        for name in ("alpha", "beta", "gamma"):
            save_profile(make_profile(name=name), tmp_path / f"{name}.json")
        profiles = list_profiles(tmp_path)
        assert len(profiles) == 3
        assert [p.name for p in profiles] == ["alpha", "beta", "gamma"]

    def test_skips_invalid_files(self, tmp_path):
        save_profile(make_profile(name="good"), tmp_path / "good.json")
        (tmp_path / "bad.json").write_text("not json", encoding="utf-8")
        profiles = list_profiles(tmp_path)
        assert len(profiles) == 1
        assert profiles[0].name == "good"

    def test_empty_directory_returns_empty_list(self, tmp_path):
        assert list_profiles(tmp_path) == []

    def test_sorted_by_filename(self, tmp_path):
        for name in ("zebra", "apple", "mango"):
            save_profile(make_profile(name=name), tmp_path / f"{name}.json")
        profiles = list_profiles(tmp_path)
        assert [p.name for p in profiles] == ["apple", "mango", "zebra"]
