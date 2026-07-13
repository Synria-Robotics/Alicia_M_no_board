"""Tests for serial channel path detection."""

from __future__ import annotations

import unittest

from arx_d_can.hardware.serial_channel import is_dm_serial_channel


class SerialChannelTests(unittest.TestCase):
    def test_tty_paths_are_serial(self) -> None:
        self.assertTrue(is_dm_serial_channel("/dev/ttyACM2"))
        self.assertTrue(is_dm_serial_channel("/dev/ttyUSB0"))

    def test_serial_symlinks_are_serial(self) -> None:
        self.assertTrue(
            is_dm_serial_channel(
                "/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00"
            )
        )
        self.assertTrue(
            is_dm_serial_channel(
                "/dev/serial/by-path/pci-0000:46:00.3-usb-0:2.2.4:1.0"
            )
        )

    def test_can_interface_is_not_serial(self) -> None:
        self.assertFalse(is_dm_serial_channel("can0"))
        self.assertFalse(is_dm_serial_channel("vcan0"))


if __name__ == "__main__":
    unittest.main()
