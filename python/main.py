"""
main.py
=======

Entry point for the Dip Coater Control application.

Usage
-----
::

    cd python

    # Default directories (profiles/ and logs/ relative to working directory):
    uv run python main.py

    # Pre-fill the port field so the operator just clicks Connect:
    uv run python main.py --port COM4

    # Override storage directories:
    uv run python main.py --profile-dir D:/profiles --log-dir D:/logs

Options
-------
--port PORT
    Pre-fill the serial port field in the Control tab (e.g. COM4).
    The operator must still click Connect — this just saves typing.

--profile-dir PATH
    Directory for JSON profile files.  Created if absent.
    Default: ``profiles/`` relative to the working directory.

--log-dir PATH
    Directory for CSV telemetry recordings and the application log.
    Created if absent.
    Default: ``logs/`` relative to the working directory.
"""

from __future__ import annotations

import argparse
import logging
import logging.handlers
import sys
from pathlib import Path

from PyQt6.QtWidgets import QApplication

from ui.main_window import MainWindow


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Dip Coater Control — PyQt6 desktop application"
    )
    parser.add_argument(
        "--port", default=None,
        metavar="PORT",
        help="Pre-fill the serial port field (e.g. COM4).  "
             "The operator must still click Connect.",
    )
    parser.add_argument(
        "--profile-dir", default="profiles",
        metavar="PATH",
        help="Directory for JSON profile files (default: profiles/)",
    )
    parser.add_argument(
        "--log-dir", default="logs",
        metavar="PATH",
        help="Directory for CSV recordings and app log (default: logs/)",
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Logging setup
# ---------------------------------------------------------------------------

def _setup_logging(log_dir: Path) -> None:
    """Configure root logger: INFO to console, DEBUG to rotating file."""
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file: Path = log_dir / "app.log"

    root: logging.Logger = logging.getLogger()
    root.setLevel(logging.DEBUG)

    # Console handler — INFO and above
    console: logging.StreamHandler = logging.StreamHandler(sys.stdout)
    console.setLevel(logging.INFO)
    console.setFormatter(logging.Formatter("%(levelname)-8s %(name)s: %(message)s"))
    root.addHandler(console)

    # Rotating file handler — DEBUG and above, 5 × 1 MB
    file_handler: logging.handlers.RotatingFileHandler = logging.handlers.RotatingFileHandler(
        log_file, maxBytes=1_048_576, backupCount=5, encoding="utf-8"
    )
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(
        logging.Formatter("%(asctime)s %(levelname)-8s %(name)s: %(message)s")
    )
    root.addHandler(file_handler)

    logging.getLogger(__name__).info(
        "logging started — file: %s", log_file
    )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    args: argparse.Namespace = _parse_args()

    profile_dir: Path = Path(args.profile_dir)
    log_dir:     Path = Path(args.log_dir)

    _setup_logging(log_dir)

    app: QApplication = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setApplicationName("Dip Coater Control")
    app.setOrganizationName("LMTS")

    win: MainWindow = MainWindow(profile_dir=profile_dir, log_dir=log_dir)

    # Pre-fill port field if --port was supplied
    if args.port:
        win.get_tab_control()._edit_port.setText(args.port)

    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
