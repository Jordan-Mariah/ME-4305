from IMU import BNO055, IMU_I2C_BUS
from main import setup_imu_calibration
from time import sleep_ms
from math import pi

imu = BNO055(bus=IMU_I2C_BUS)
setup_imu_calibration(imu, force=False)

input("Keep Romi level and stationary. Press Enter to begin.")

previous_heading = imu.heading()
relative_heading_deg = 0.0

while True:
    heading_deg = imu.heading()
    yaw_deg_s = imu.read_gyro_z()

    # Handle heading wrapping between 359° and 0°
    change_deg = (heading_deg - previous_heading + 180) % 360 - 180
    relative_heading_deg += change_deg
    previous_heading = heading_deg

    print(
        "Heading deg:", heading_deg,
        "Relative rad:", relative_heading_deg * pi / 180,
        "Yaw rad/s:", yaw_deg_s * pi / 180,
    )

    sleep_ms(200)