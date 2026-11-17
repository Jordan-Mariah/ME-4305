from IMU import BNO055, IMU_I2C_BUS
from main import IMU_CONFIG
from IMU_calibration import load_imu_calibration
from time import sleep_ms, ticks_ms, ticks_diff
from math import pi

imu = BNO055(bus=IMU_I2C_BUS)
# Require an existing valid profile: do not silently recalibrate in this test.
calibration = load_imu_calibration("imu_calibration.json", IMU_CONFIG)
imu.write_calibration_profile(bytes(calibration["profile"]))
print("Saved profile restored:", len(calibration["profile"]), "bytes")
print("Calibration status:", imu.calib_status())

print("Keep Romi level and stationary. Starting in 3 seconds...")
sleep_ms(3000)

previous_heading = imu.heading()
relative_heading_deg = 0.0
start = ticks_ms()
stationary_min = 0.0
stationary_max = 0.0
yaw_sum = 0.0
yaw_peak = 0.0
samples = 0
stationary_reported = False
print("Keep stationary for the next 20 seconds.")
print("time_s,raw_heading_deg,relative_deg,relative_rad,yaw_rad_s,wrap")

while True:
    heading_deg = imu.heading()
    yaw_deg_s = imu.read_gyro_z()

    # Handle heading wrapping between 359° and 0°
    change_deg = (heading_deg - previous_heading + 180) % 360 - 180
    relative_heading_deg += change_deg
    wrapped = abs(heading_deg - previous_heading) > 180
    previous_heading = heading_deg

    elapsed_ms = ticks_diff(ticks_ms(), start)
    if elapsed_ms < 20_000:
        stationary_min = min(stationary_min, relative_heading_deg)
        stationary_max = max(stationary_max, relative_heading_deg)
        yaw_sum += yaw_deg_s
        yaw_peak = max(yaw_peak, abs(yaw_deg_s))
        samples += 1
    elif not stationary_reported:
        print("Stationary heading range (deg):", stationary_max - stationary_min)
        if samples:
            print("Stationary mean yaw (deg/s):", yaw_sum / samples)
        print("Stationary peak absolute yaw (deg/s):", yaw_peak)
        print("Now turn LEFT 90 degrees, pause, then return RIGHT to the start.")
        print("Then turn slowly through a raw 0/360 boundary; look for WRAP.")
        stationary_reported = True

    print("{:.2f},{:.2f},{:.2f},{:.3f},{:.3f},{}".format(
        elapsed_ms / 1000, heading_deg, relative_heading_deg,
        relative_heading_deg * pi / 180, yaw_deg_s * pi / 180,
        "WRAP" if wrapped else "",
    ))

    sleep_ms(200)
