from IMU import BNO055
from tshare import Share
from math import pi


class TaskImu():

    # Task state constants
    S0_IDLE = 0
    S1_READ_YAW = 1


    def __init__(self, imu: BNO055, imu_flag: Share, sensor_data: Share) -> None:
        # Set owned drivers and shares
        self.imu = imu
        self.imu_flag = imu_flag
        self.sensor_data = sensor_data

        # Set default state
        self.state = self.S0_IDLE


    def run(self):

        while True:

            # Default state, wait for signal to read data
            if self.state == self.S0_IDLE:

                # Enable read mode if imu flag raised
                if self.imu_flag.value == True:


                    self.state = self.S1_READ_YAW

            # Reading and data pushing mode
            elif self.state == self.S1_READ_YAW:
                pass

            yield self.state