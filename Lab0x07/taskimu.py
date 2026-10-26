from IMU import BNO055
from tshare import Share
from time import ticks_ms
from math import pi


class TaskImu:
    S0_IDLE = 0
    S1_READ_YAW = 1

    #sensor data is a shared diciotnary that we can access via the steering task
    def __init__(self, imu: BNO055, imu_flag: Share,
                 sensor_data: Share):
        self.imu = imu
        self.imu_flag = imu_flag
        self.sensor_data = sensor_data

        self.state = self.S0_IDLE
        self.sensor_data.value["imu"] = None

    def run(self):
        while True:

            if self.state == self.S0_IDLE:
                #when the colection is dissabled there are no valid measurements
                self.sensor_data.value["imu"] = None

                #when the imu flag is set go to the read_yaw state
                if self.imu_flag.value:
                    self.state = self.S1_READ_YAW

            elif self.state == self.S1_READ_YAW:
                #go back to idel if the imu flag is now dissabled
                if not self.imu_flag.value:
                    self.sensor_data.value["imu"] = None
                    self.state = self.S0_IDLE

                else:
                    # Read heading(yaw angle) in degrees and yaw rate in degrees/s
                    heading_deg = self.imu.heading()
                    yaw_deg_s = self.imu.read_gyro_z()

                    # calc yaw rate to radians/s (steering equations)
                    yaw_rad_s = yaw_deg_s * pi / 180

                    # Publish one complete measurement
                    self.sensor_data.value["imu"] = (
                        heading_deg,
                        yaw_rad_s,
                        ticks_ms(),
                    )

            yield self.state