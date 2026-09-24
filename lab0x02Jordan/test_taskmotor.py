"""PC-only behavioral checks: python test_taskmotor.py (no board required)."""
import importlib.util
from pathlib import Path
import unittest
import time

import runpy
from unittest.mock import MagicMock, patch

# Stand-ins for the hardware modules.
fake_pyb = MagicMock()
fake_driver = MagicMock()
fake_encoder = MagicMock()

with patch.dict("sys.modules", {
    "pyb": fake_pyb,
    "driver": fake_driver,
    "encoder": fake_encoder,
}):
    # Regular computer Python doesn't provide these MicroPython functions.
    with patch("time.ticks_us", return_value=0, create=True), \
         patch("time.ticks_diff", return_value=0, create=True):

        namespace = runpy.run_path("taskmotor.py")
        Task1 = namespace["Task1"]

        # Execute only the constructor.
        task = Task1()

        # Check the starting state.
        assert task.state == Task1.S0_PREPARE

        # Check that a Motor object was requested.
        fake_driver.Motor.assert_called_once()

        # Check that the task stored the returned motor object.
        assert task.right_motor is fake_driver.Motor.return_value

        # Check the PWM channel setup.
        timer = fake_pyb.Timer.return_value
        timer.channel.assert_called_once_with(
            1,
            mode=fake_pyb.Timer.PWM,
            pin=fake_pyb.Pin.cpu.A8,
            pulse_width_percent=0,
        )

print("Initialization test passed")
