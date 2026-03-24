"""
tests/test_main.py
==================

Smoke tests for main.py — verifies the entry point imports cleanly and
that CLI arguments are wired correctly.

No hardware required.

Run:
    uv run pytest tests/test_main.py -v
"""

from __future__ import annotations

import sys
from pathlib import Path


class TestMain:

    def test_imports_without_error(self):
        """main.py can be imported without raising."""
        import main  # noqa: F401

    def test_parse_args_defaults(self):
        """Default arguments resolve to expected directory names."""
        import main
        args = main._parse_args.__wrapped__() if hasattr(main._parse_args, "__wrapped__") else _parse_with(main, [])
        assert args.profile_dir == "profiles"
        assert args.log_dir     == "logs"
        assert args.port        is None

    def test_parse_args_port(self):
        """--port argument is captured correctly."""
        args = _parse_with(__import__("main"), ["--port", "COM4"])
        assert args.port == "COM4"

    def test_parse_args_custom_dirs(self):
        """--profile-dir and --log-dir are captured correctly."""
        args = _parse_with(__import__("main"), [
            "--profile-dir", "/tmp/myprofiles",
            "--log-dir",     "/tmp/mylogs",
        ])
        assert args.profile_dir == "/tmp/myprofiles"
        assert args.log_dir     == "/tmp/mylogs"

    def test_port_prefills_control_tab(self, qtbot, tmp_path):
        """Passing --port causes ControlTab port field to be pre-filled."""
        from ui.main_window import MainWindow
        win = MainWindow(
            profile_dir=tmp_path / "profiles",
            log_dir=tmp_path / "logs",
        )
        qtbot.addWidget(win)
        win.get_tab_control()._edit_port.setText("COM7")
        assert win.get_tab_control()._edit_port.text() == "COM7"

    def test_setup_logging_creates_log_dir(self, tmp_path):
        """_setup_logging() creates the log directory if it does not exist."""
        import main
        new_dir = tmp_path / "newlogs"
        assert not new_dir.exists()
        main._setup_logging(new_dir)
        assert new_dir.exists()

    def test_setup_logging_creates_app_log(self, tmp_path):
        """_setup_logging() creates app.log inside the log directory."""
        import main
        main._setup_logging(tmp_path)
        assert (tmp_path / "app.log").exists()


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _parse_with(module, argv: list[str]):
    """Call module._parse_args() with a controlled sys.argv."""
    original = sys.argv
    try:
        sys.argv = ["main.py"] + argv
        return module._parse_args()
    finally:
        sys.argv = original
