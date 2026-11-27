"""Preliminary wiring test. Run separately from main.py."""

from pyb import ADC, Pin
from time import sleep_ms
from line_sensor import SENSOR_PINS, SENSOR_CHANNELS

# Hold both motor drivers disabled.
left_sleep = Pin(Pin.cpu.A10, Pin.OUT_PP, value=0)
right_sleep = Pin(Pin.cpu.B4, Pin.OUT_PP, value=0)

adcs = tuple(ADC(pin) for pin in SENSOR_PINS)

try:
    while True:
        readings = []

        for adc in adcs:
            # Average 10 ordinary ADC readings for this wiring check.
            average = sum(adc.read() for _ in range(10)) / 10
            readings.append(average)

        print(" ".join(
            "{}:{:.0f}".format(channel, reading)
            for channel, reading in zip(SENSOR_CHANNELS, readings)
        ))

        sleep_ms(500)

finally:
    left_sleep.low()
    right_sleep.low()