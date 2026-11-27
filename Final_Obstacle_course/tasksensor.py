from line_sensor import LineSensor
from time import ticks_us, ticks_diff, ticks_add
from tshare import Share


# Define constants

SAMPLE_PER = 50_000

# States
S0_IDLE = 0
S1_GET_CENTROID = 1

class TaskSensor():
    '''Task that handles sensors and shares relavent data'''


    def __init__(self, line_sensor: LineSensor, sensor_flag: Share, sensor_data: Share) -> None:
        self.line_sensor = line_sensor
        self.sensor_flag = sensor_flag
        self.sensor_data = sensor_data # Data dictionary

        self.state = S0_IDLE


    def run(self):
        # TaskSensor generator

        while True:
            # Wait for collection signal
            if self.state == S0_IDLE:
                if self.sensor_flag.value == True:
                    sample_next = ticks_us()
                    self.state = S1_GET_CENTROID


            # Collect and export data every SAMPLE_PER[iod]
            elif self.state == S1_GET_CENTROID:
                if ticks_diff(sample_next, ticks_us()) <= 0:
                    # Period elapsed, collect and export data

                    line_data = self.line_sensor.read_centroid()
                    #copy the sensor attributes into shared dictionary
                    self.sensor_data.value["all_black"] = self.line_sensor.all_black
                    self.sensor_data.value["white_background"] = self.line_sensor.white_background
                    self.sensor_data.value["line_detected"] = self.line_sensor.line_detected
                    self.sensor_data.value["sensor_time_us"] = ticks_us()
                    self.sensor_data.value["line"] = line_data

                    sample_next = ticks_add(ticks_us(), SAMPLE_PER)

                # Check if data collection is dissabled
                if self.sensor_flag.value == False:
                    self.state = S0_IDLE

            yield self.state