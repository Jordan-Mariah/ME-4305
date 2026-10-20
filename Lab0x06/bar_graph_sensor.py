# infrared sensor test
#should be run through putty and used for help with calibration 

from pyb import USB_VCP, delay, Pin
from line_sensor import LineSensor
from pyb import USB_VCP, delay, Pin, Timer
from line_sensor import (
    LineSensor,
    SENSOR_PINS,
    SENSOR_CHANNELS,
    SENSOR_POSITIONS_MM,
)

def make_frame(values, channels, full_scale=4095, width=36):
    if len(values) != 13 or len(channels) != 13:
        raise ValueError("Expected all 13 channel readings")
    lines = ["Romi ADC readings (0-4095); q to quit", "Ch  Reading  Bar"]
    for channel, value in zip(channels, values):
        size = int(width * min(full_scale, max(0, value)) / full_scale)
        lines.append("{:2d}  {:7.1f}  |{}{}|".format(
            channel, value, "#" * size, " " * (width - size)))
    # Cursor home and clear each row; fixed-width bars replace the old frame.
    frame = "\x1b[H"
    for row in lines:
        frame += "\x1b[2K" + row + "\r\n"
    return frame


def plot_adc(read_values, channels, refresh_ms=100):
    vcp = USB_VCP()
    vcp.write("\x1b[2J\x1b[H")  # one time clear screen
    try:
        while True:
            if vcp.any():
                command = vcp.read()
                if command and (b"q" in command or b"Q" in command):
                    break
            vcp.write(make_frame(read_values(), channels))
            delay(refresh_ms)  # Display based on the rate of refresh
    except KeyboardInterrupt:
        pass
    finally:
        vcp.write("\r\nSensor display stopped.\r\n")


def main():
    # Keep both motor drivers disabled.
    left_sleep = Pin(Pin.cpu.A10, Pin.OUT_PP, value=0)
    right_sleep = Pin(Pin.cpu.B4, Pin.OUT_PP, value=0)

    sensor_timer = Timer(6)

    line_sensor = LineSensor(
        pins=SENSOR_PINS,
        channels=SENSOR_CHANNELS,
        positions_mm=SENSOR_POSITIONS_MM,
        timer=sensor_timer,
        sample_count=10,
        sample_rate_hz=5000,
        min_span=10,
    )

    try:
        # Pass the method this way plot_adc can call it every refresh cycle
        read_values = line_sensor.read_averages
        plot_adc(read_values, SENSOR_CHANNELS)

    finally:
        left_sleep.low()
        right_sleep.low()
        sensor_timer.deinit()

if __name__ == "__main__":
    main()
