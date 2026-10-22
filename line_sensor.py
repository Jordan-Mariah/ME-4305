"""owns the inputs from the adc and provides read_averages(), set_calibration(calibration),
read_weights(), read_centroid()"""

"""13-channel analog line-sensor driver.

Channel numbers, ADC pins, and positions must use the same order.
Positions are in millimeters, positive toward Romi's left.

Calibration and normal sensing use the same acquisition method.
"""

from array import array
from math import isfinite
from pyb import ADC, Pin

from line_calibration import validate_calibration

#aligned to the ROMI Pin configuration
SENSOR_PINS = (
    Pin.cpu.A0, Pin.cpu.A1, Pin.cpu.A4, Pin.cpu.A6, Pin.cpu.A7,
    Pin.cpu.B0, Pin.cpu.B1, Pin.cpu.C0, Pin.cpu.C1, Pin.cpu.C2,
    Pin.cpu.C3, Pin.cpu.C4, Pin.cpu.C5,
)
SENSOR_CHANNELS = tuple(range(1, 14))
# Assumes channel 1 on right of ROMI but verify
# reverse position signs for if channel 1 is on left of Romi
SENSOR_POSITIONS_MM = tuple(range(-24, 25, 4))
# check ST-LINK USART bridge routing before using PA2.
# =scale battery voltage to ADC range with a suitable divider.
BATTERY_SENSE_PIN = Pin.cpu.A2  # CN10-35; no ADC instantiated here

CALIBRATION_PATH = "line_sensor.json"

class LineSensor:
    CHANNEL_COUNT = 13

    def __init__(
        self,
        pins,
        channels,
        positions_mm,
        timer,
        sample_count=10,
        sample_rate_hz=5000,
        min_span=10,
    ):
        """
        pins:
            the driver reader order has 13 ADC-capable pin objects channels:
            They then correspond to printed sensor numbers, 1 through 13.
        positions_mm: to lateral positions which are positive toward Romi's left.
        timer:A free pyb.Timer supporting timed ADC acquisition.
        This driver configures its frequency.
        sample_count:Number of samples averaged per channel.
        sample_rate_hz:Samples per second PER CHANNEL; at most 5000.
        min_span:Minimum acceptable black-minus-white ADC difference.
        """
        self.channels = tuple(channels)
        self.positions_mm = tuple(float(p) for p in positions_mm)
        pins = tuple(pins)

        if not (
            len(pins)
            == len(self.channels)
            == len(self.positions_mm)
            == self.CHANNEL_COUNT
        ):
            raise ValueError("Provide exactly 13 pins, channels, and positions")

        if sorted(self.channels) != list(range(1, 14)):
            raise ValueError("Channels must contain numbers 1 through 13")

        if not all(isfinite(p) for p in self.positions_mm):
            raise ValueError("Sensor positions must be finite")

        if not isinstance(sample_count, int) or sample_count <= 0:
            raise ValueError("Sample count must be a positive integer")

        if not 0 < sample_rate_hz <= 5000:
            raise ValueError("Sample rate must be greater than 0 and at most 5000 Hz")

        if not 0 < min_span <= 4095:
            raise ValueError("Invalid minimum calibration span")

        self.sample_count = sample_count
        self.sample_rate_hz = sample_rate_hz
        self.min_span = min_span

        self.adcs = tuple(ADC(pin) for pin in pins)

        # Allocate once; each acquisition overwrites these buffers.
        self.buffers = tuple(
            array("H", [0] * sample_count)
            for _ in range(self.CHANNEL_COUNT)
        )

        # Reused floating-point output arrays.
        self.averages = array("f", [0.0] * self.CHANNEL_COUNT)
        self.weights = array("f", [0.0] * self.CHANNEL_COUNT)

        self.timer = timer
        self.timer.init(freq=sample_rate_hz)

        self.white = None
        self.black = None
        self.spans = None
        self.total_weight = 0.0

    def read_averages(self):
        """Average all channels.
        Array is reused then rewritten
        Copy it immediately if the readings must be retained.
        """
        if not ADC.read_timed_multi(
            self.adcs, self.buffers, self.timer
        ):
            raise RuntimeError("ADC acquisition overrun")

        #cal one average adc reading for each of the 13 sensors
        for i, buffer in enumerate(self.buffers):
            self.averages[i] = sum(buffer) / self.sample_count

        return self.averages

    def set_calibration(self, calibration):
        """Validate and copy measured black/white ADC endpoints"""
        validate_calibration(
            calibration,
            self.channels,
            self.min_span,
        )

        # Use ordinary floating-point values for subtraction.
        white = tuple(float(v) for v in calibration["white"])
        black = tuple(float(v) for v in calibration["black"])
        spans = tuple(
            b - w for w, b in zip(white, black)
        )

        self.white = white
        self.black = black
        self.spans = spans

    def read_weights(self):
        """get readings and return blackness weights
        White is a weight of about 0 and then black is approximately 1
        The array returned is reused on each next call
        """
        if self.spans is None:
            raise RuntimeError("Apply calibration before normalizing")

        readings = self.read_averages()
        total = 0.0

        for i in range(self.CHANNEL_COUNT):
            weight = (
                (readings[i] - self.white[i]) / self.spans[i]
            )

            if weight < 0.0:
                weight = 0.0
            elif weight > 1.0:
                weight = 1.0

            self.weights[i] = weight
            total += weight

        self.total_weight = total
        return self.weights

    def read_centroid(self):
        """Acquire and return line position in mm.

        Negative: line on Romi's right.
        Positive: line on Romi's left.
        None: zero total weight, so position is undefined.
        """
        weights = self.read_weights()

        if self.total_weight == 0.0:
            return None

        weighted_position = 0.0

        for position, weight in zip(self.positions_mm, weights):
            weighted_position += position * weight

        return weighted_position / self.total_weight
