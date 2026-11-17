"""Validate, save, and load BNO055 calibration profiles."""

import json


def validate_calibration(calibration, expected_config):
    if not isinstance(calibration, dict):
        raise ValueError("Calibration must be a dictionary")

    if calibration.get("version") != 1:
        raise ValueError("The calibration file version is not supported")

    if calibration.get("config") != expected_config:
        raise ValueError("Calibration is not the same as IMU config.")

    profile = calibration.get("profile")

    if not isinstance(profile, list) or len(profile) != 22:
        raise ValueError("The calibration profile needs 22 bytes")

    for value in profile:
        # Use type() to reject booleans, which Python treats as integers
        if type(value) is not int or not 0 <= value <= 255:
            raise ValueError("Profile values are required as integers from 0 to 255")

    return calibration


def save_imu_calibration(filename, calibration, expected_config):
    # Validate before opening the file
    validate_calibration(calibration, expected_config)

    with open(filename, "w") as file:
        json.dump(calibration, file)


def load_imu_calibration(filename, expected_config):
    with open(filename, "r") as file:
        calibration = json.load(file)

    validate_calibration(calibration, expected_config)
    return calibration