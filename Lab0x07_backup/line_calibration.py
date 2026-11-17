"""Save and load line-sensor calibration before the scheduler starts.

Keep motors disabled during calibration. Startup code must handle file errors.
Here we store all of the 13 printed sensor numbers in driver order.
These are stored with the measured black and white ADC averages (0-4095) in that 
same order. Convert ulab arrays to ordinary lists with .tolist() before saving. 
JSON reloads arrays as lists.
"""

import json


def validate_calibration(calibration, channels, min_span, adc_max=4095):
    white, black = calibration["white"], calibration["black"]
    if len(channels) != 13 or list(calibration["channels"]) != list(channels):
        raise ValueError("Calibration must match all 13 configured channels")
    if len(white) != 13 or len(black) != 13 or not 0 < min_span <= adc_max:
        raise ValueError("Invalid calibration lengths or minimum span")
    for w, b in zip(white, black):
        if not 0 <= w < b <= adc_max or b - w < min_span:
            raise ValueError("Invalid black/white values or insufficient contrast")
    return calibration


def save_calibration(filename, calibration, channels, min_span, adc_max=4095):
    # Validate before opening the destination, then write the JSON file.
    validate_calibration(calibration, channels, min_span, adc_max)
    with open(filename, "w") as file:
        json.dump(calibration, file)


def load_calibration(filename, channels, min_span, adc_max=4095):
    # Startup code reports errors or repeats calibration.
    with open(filename, "r") as file:
        calibration = json.load(file)
    validate_calibration(calibration, channels, min_span, adc_max)
    return calibration


# Startup usage AFTER measuring all channels over black, then white:
# channels = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
# Reorder channels if your driver reads them in a different order.
# calibration = {
#     "channels": channels,
#     "black": list(black_averages),
#     "white": list(white_averages),
# }
# min_span = 10  # Example only; choose for measured contrast and noise.
# save_calibration("line_calibration.json", calibration, channels, min_span)
# Later, reuse only when sensing conditions are unchanged:
# calibration = load_calibration("line_calibration.json", channels, min_span)
