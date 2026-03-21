"""
tests/conftest.py
=================

Session-level pytest configuration and shared fixtures.

Hardware fixtures
-----------------
All hardware fixtures are **session-scoped** — the serial port is opened once
per ``pytest`` invocation and shared across every test that uses it.  This
avoids the 2-second Arduino reset delay on every test.

If the serial port cannot be opened (hardware not connected), every test that
depends on a hardware fixture is automatically **skipped** with a clear reason
message.  All non-hardware unit tests continue to run normally.

Usage
-----
Run only unit tests (default, no hardware needed)::

    uv run pytest

Run everything including hardware tests (Arduino must be on COM4)::

    uv run pytest -m hardware

Use a different port::

    uv run pytest -m hardware --hw-port COM3

Hardware tests can also be included in a full run::

    uv run pytest --hw-port COM4
"""

from __future__ import annotations

import queue
import time
from typing import Generator, Optional

import pytest

from core.command_interface import CommandInterface, CommandError
from core.data_recorder import DataRecorder
from core.serial_manager import SerialManager
from core.telemetry_parser import TelemetryFrame


# ---------------------------------------------------------------------------
# CLI options
# ---------------------------------------------------------------------------

def pytest_addoption(parser: pytest.Parser) -> None:
    """Register custom command-line options for hardware tests."""
    parser.addoption(
        "--hw-port",
        default="COM4",
        help="Serial port for hardware tests (default: COM4)",
    )
    parser.addoption(
        "--hw-baud",
        default=115200,
        type=int,
        help="Baud rate for hardware tests (default: 115200)",
    )


# ---------------------------------------------------------------------------
# Session-scoped hardware fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def hw_port(request: pytest.FixtureRequest) -> str:
    """Return the serial port name supplied via ``--hw-port``."""
    port: str = request.config.getoption("--hw-port")
    return port


@pytest.fixture(scope="session")
def hw_manager(hw_port: str, request: pytest.FixtureRequest) -> Generator[SerialManager, None, None]:
    """Open a SerialManager on the hardware port for the test session.

    Skips all dependent tests automatically if the port cannot be opened.
    The manager is stopped and the port closed after the session ends.
    """
    baud: int = request.config.getoption("--hw-baud")
    mgr: SerialManager = SerialManager(port=hw_port, baud=baud)
    try:
        mgr.start()
    except Exception as exc:
        pytest.skip(f"Hardware not available on {hw_port}: {exc}")

    yield mgr

    mgr.stop()


@pytest.fixture(scope="session")
def hw_ci(hw_manager: SerialManager) -> CommandInterface:
    """Return a CommandInterface wired to the session hardware manager."""
    return CommandInterface(hw_manager)


# ---------------------------------------------------------------------------
# Hardware helper functions (used by hardware test modules)
# ---------------------------------------------------------------------------

def hw_flush_queue(mgr: SerialManager) -> None:
    """Discard all frames currently sitting in the telemetry queue.

    Call this after sending a command so that subsequent waits only see
    frames produced after the command was processed by the Arduino.
    """
    while True:
        try:
            mgr.telem_queue.get_nowait()
        except queue.Empty:
            break


def hw_wait_for_state_transition(
    mgr: SerialManager,
    leave: str,
    arrive: str,
    timeout_leave_s: float = 5.0,
    timeout_arrive_s: float = 60.0,
    recorder: Optional[DataRecorder] = None,
) -> bool:
    """Wait for the Arduino to leave *leave* state then reach *arrive* state.

    Phase 1: waits up to *timeout_leave_s* for state != *leave*.
    Phase 2: waits up to *timeout_arrive_s* for state == *arrive*.

    If *recorder* is provided every frame seen in both phases is passed to
    ``recorder.record()`` so that telemetry is captured during the run.

    Returns True if both phases succeed, False on timeout.
    """
    # Phase 1 — leave current state
    deadline: float = time.monotonic() + timeout_leave_s
    left: bool = False
    while time.monotonic() < deadline:
        try:
            frame: TelemetryFrame = mgr.telem_queue.get(
                timeout=min(deadline - time.monotonic(), 0.2)
            )
            if recorder is not None:
                recorder.record(frame)
            if frame.state != leave:
                left = True
                break
        except queue.Empty:
            pass

    if not left:
        return False

    # Phase 2 — arrive at target state
    deadline = time.monotonic() + timeout_arrive_s
    while time.monotonic() < deadline:
        try:
            frame = mgr.telem_queue.get(
                timeout=min(deadline - time.monotonic(), 1.0)
            )
            if recorder is not None:
                recorder.record(frame)
            if frame.state == arrive:
                return True
        except queue.Empty:
            pass

    return False


def hw_current_state(mgr: SerialManager, timeout_s: float = 3.0) -> str:
    """Return the Arduino's current state from the next telemetry frame.

    Returns an empty string if no frame arrives within *timeout_s*.
    """
    try:
        frame: TelemetryFrame = mgr.telem_queue.get(timeout=timeout_s)
        return frame.state
    except queue.Empty:
        return ""
