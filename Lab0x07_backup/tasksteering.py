from math import pi, isfinite
from time import ticks_ms, ticks_diff

# Initial tuning values, to be verified on your robot.
WHEEL_RADIUS_M = 0.035       # Confirm actual wheel radius.
FORWARD_SPEED_M_S = 0.06
TURN_RATE_RAD_S = 0.30
YAW_KP = 2.0                # Wheel rad/s per chassis rad/s error.
MAX_WHEEL_RAD_S = 4.0       # Replace with actual limit limit.
GYRO_SIGN = 1              # comfirm that a left turn is positive left turn must read positive.

DEPARTURE_DELAY_MS = 2000
LANE_SPACING_M = 0.10
LANE_DIRECTION = 1            #left lane change is 1, and right is -1
MANEUVER_TIMEOUT_MS = 10000   #adjust maybe
LINE_TURN_RATIO = 0.2         #adjust maybe (currently adjusts each angular v by 20%)

#Driving states
FOLLOW_LINE = 0
DEPART = 1
TURN_BACK = 2
FOLLOW_TARGET = 3


class TaskSteering:
    def __init__(self, imu, wheel_requests, run_flag, sensor_data):
        self.imu = imu
        self.wheel_requests = wheel_requests
        self.run_flag = run_flag
        self.sensor_data = sensor_data
        self.state = FOLLOW_LINE
        self.started = False
        self.cleared_line = False
        self.lateral_position = 0.0
        #handling errors for measurements input
        if (not isfinite(WHEEL_RADIUS_M) or WHEEL_RADIUS_M <= 0
                or not isfinite(FORWARD_SPEED_M_S)
                or not isfinite(MAX_WHEEL_RAD_S) or MAX_WHEEL_RAD_S <= 0
                or abs(FORWARD_SPEED_M_S / WHEEL_RADIUS_M) > MAX_WHEEL_RAD_S):
            raise ValueError("Invalid forward speed or wheel-speed limit")

    def run(self):
        while True:
            now = ticks_ms() #current clock
            if not self.run_flag.value: #when no run then make wheel request o and yield to scheduler
                self.wheel_requests.set((0.0, 0.0))
                self.started = False
                yield
                continue

            if not self.started:
                self.started = True #mark initialization true
                self.state = FOLLOW_LINE #strat with line following
                self.run_start = now  #track begining of run
                self.previous_time = now  #used for next update interval
                self.cleared_line = False #have the sensors los the line
                self.lateral_position = 0.0 #horizontal distance from line

            dt = ticks_diff(now, self.previous_time) / 1000 #elapsed time in seconds
            self.previous_time = now
            line_position = self.sensor_data.value["line"] #line centroid

            #lane change specific
            if self.state in (DEPART, TURN_BACK):
                heading = GYRO_SIGN * self.imu.heading() * pi / 180 #heading in radians
                psi = (heading - self.heading_start + pi) % (2 * pi) - pi # heading angle (psi) compared to the departure position
                #speed is the average of the left and right angular velocities multiplied by wheel rad
                speed = WHEEL_RADIUS_M * (self.sensor_data.value["left_velocity"]
                    + self.sensor_data.value["right_velocity"]
                ) / 2
                self.lateral_position += speed * psi * dt #get estimate of horizontal distance in meters
                if line_position is None: #remember even when QTR detects no line
                    self.cleared_line = True
                #if the manuver take longer than its max time clear run flag and stop ROMI
                if ticks_diff(now, self.maneuver_start) >= MANEUVER_TIMEOUT_MS: 
                    self.run_flag.set(False)
                    self.wheel_requests.set((0.0, 0.0))
                    yield
                    continue

            #line following state (adapted from lab 0x06 previously in TaskMotor)
            if self.state == FOLLOW_LINE:
                requests = line_wheel_requests(line_position)
                #after we follow the line until the departure time
                if ticks_diff(now, self.run_start) >= DEPARTURE_DELAY_MS:
                    self.maneuver_start = now #track the time of departure
                    self.heading_start = GYRO_SIGN * self.imu.heading() * pi / 180 #initial heading in radians
                    self.state = DEPART #enter the state to move off of the line
            #impliment turn dirrection requested
            elif self.state == DEPART:
                requests = yaw_wheel_requests(
                    self.imu,
                    LANE_DIRECTION * TURN_RATE_RAD_S
                )
                #reverse when latteral displacement hits half of line spacing
                if LANE_DIRECTION * self.lateral_position >= LANE_SPACING_M / 2:
                    self.state = TURN_BACK #go into turn back state
            #reverse turn dir. requested
            elif self.state == TURN_BACK:
                requests = yaw_wheel_requests(
                    self.imu,
                    -LANE_DIRECTION * TURN_RATE_RAD_S
                )
                if self.cleared_line and line_position is not None:
                    self.state = FOLLOW_TARGET #once the robot is turning back go back to line following
            elif self.state == FOLLOW_TARGET:
                requests = line_wheel_requests(line_position)
            else:
                self.run_flag.set(False)
                requests = (0.0, 0.0)
            self.wheel_requests.set(requests)
            yield
        
def line_wheel_requests(line_position):
    angular_v = FORWARD_SPEED_M_S / WHEEL_RADIUS_M #driveing wheel speed when going straight
    position = line_position if line_position is not None else 0.0 #go straight if line missing
    correction = angular_v * LINE_TURN_RATIO * position / 25 #scale using 25mm reference distance
    limit = MAX_WHEEL_RAD_S - abs(angular_v) #correction limmit
    correction = max(-limit, min(limit, correction)) #keep correction in negative and pos limits
    #decrease one wheel v and increase the other
    return angular_v - correction, angular_v + correction


def yaw_wheel_requests(imu, yaw_request):
    measured_yaw_rate = GYRO_SIGN * imu.read_gyro_z() * pi / 180

    error = yaw_request - measured_yaw_rate
    correction = YAW_KP * error

    angular_v = FORWARD_SPEED_M_S / WHEEL_RADIUS_M

    #correction is limmitted so both wheel requests are in bounds.
    max_steering_correction = max(0.0, MAX_WHEEL_RAD_S - abs(angular_v))

    #limit the correction between the positive and negative max steering correction
    correction = max(-max_steering_correction, min(max_steering_correction, correction))

    left_request = angular_v - correction
    right_request = angular_v + correction
    return left_request, right_request
