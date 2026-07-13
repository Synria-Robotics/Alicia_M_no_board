"""Detect Linux serial device paths for Damiao USB2CAN adapters."""

from __future__ import annotations


def is_dm_serial_channel(channel: str) -> bool:
    """Return True if *channel* should use ``Controller.from_dm_serial``."""
    path = str(channel).strip()
    return path.startswith("/dev/tty") or path.startswith("/dev/serial/")
