"""this is the nonblocking test for the motor. Here we set up the motor task 
then it can be instantiated by the left and right motors"""
from time import ticks_us, ticks_diff, ticks_add
from driver import Motor
from encoder import Encoder
from tshare import Share
from math import pi, isfinite, sin, radians
import line_sensor # Only for constant access. Maybe a better solution

class TaskManeuver:

    #Task state constants
    S0_IDLE = 0
    S1_PREPARE = 1
    S2_FOLLOW_LINE = 2
    S3_MERGE_LANE = 3
    S4_STOP_MOTOR = 4

    # Other task constants
    KP: float = 0.001
    KI: float = 0.00
    KD: float = 0
    KM: float = 0.7 # V / (rad/s)
    MAX_TURN_RATIO: float = 0.5 # Max multiplier applied to target speed to stay on line
    MERGE_TURN_RATIO: float = 0.2 # Multiplier applied to target speed when changing lanes
    LANE_DISTANCE: float = 20.0


    '''1) determining whethere motor driver side is left or right 
    2) Store side as a string on object 
    3) Inititializing timers for motor 
    4) Init motor's timer channels
    5)calling the Motor class for left or right motor '''
    def __init__(self, left_mode : Share, right_mode: Share, left_target: Share, right_target: Share,
                 left_velocity:Share, right_velocity:Share, left_position_cm:Share, right_position_cm:Share,
                 next_effort:Share , sensor_data: Share, line_follow_flag: Share, lane_change_flag:Share,
                 lane_change_dir:Share, following_target:Share, test_flags=(), period_ms=30):
        #Pull args
        self.left_mode = left_mode
        self.right_mode = right_mode
        self.left_target = left_target
        self.right_target = right_target
        self.left_velocity = left_velocity
        self.right_velocity = right_velocity
        self.left_position_cm = left_position_cm
        self.right_position_cm = right_position_cm
        self.next_effort = next_effort
        self.sensor_data = sensor_data
        self.line_follow_flag = line_follow_flag
        self.lane_change_flag = lane_change_flag
        self.lane_change_dir = lane_change_dir
        self.following_target = following_target
        self.test_flags = test_flags
        if period_ms <= 0:
            raise ValueError("Maneuver period must be positive")
        self.period_ms = period_ms
        # Set starting values
        self.active_setpoint = 0.0
        self.state = self.S0_IDLE

    def run(self):
        # Keep the generator alive so the scheduler can run future requests.
        while True:
            #if lest or right test flag dissable lane change flags
            if any(flag.value for flag in self.test_flags):
                self.line_follow_flag.set(False)
                self.lane_change_flag.set(False)
                self.set_targets(0.0, 0.0, "idle")
                self.state = self.S0_IDLE
                yield self.state
                continue

            if self.state == self.S0_IDLE:
                if self.line_follow_flag.value: #check for line follow
                    #the base wheel speed (rad/s)
                    self.active_setpoint = float(self.next_effort.value)
                    self.state = self.S2_FOLLOW_LINE
                else:
                    #set the left and right target value shares for the motor task
                    self.set_targets(0.0, 0.0, "idle")

            elif self.state == self.S2_FOLLOW_LINE:
                if not self.line_follow_flag.value:
                    self.state = self.S4_STOP_MOTOR
                    self.set_targets(0.0, 0.0, "stop")#set the left and right target value shares for the motor task
                elif self.following_target.value and self.sensor_data.value["all_black"]:#if on ending line
                    self.state = self.S4_STOP_MOTOR
                    self.set_targets(0.0, 0.0, "stop")#set the left and right target value shares for the motor task
                elif self.lane_change_flag.value:
                    imu = self.sensor_data.value.get("imu")
                    if imu is not None: #check if the imu has data
                        self.theta_ref = imu[0] #yaw angle
                        self.direction = 1 if self.lane_change_dir.value == "left" else -1
                        self.displacement = 0.0
                        self.passed_center = False #for start of lane change
                        self.last_position = (self.left_position_cm.value + self.right_position_cm.value) / 2
                        self.state = self.S3_MERGE_LANE
                else:
                    line_position = self.sensor_data.value.get("line")
                    if line_position is None:
                        line_position = 0.0
                    base = self.active_setpoint #set varriable for base wheel speed
                    #set left and right target value shares for the motor task based on line pos. and base wheel speed
                    self.set_targets(base * (1 + self.MAX_TURN_RATIO * line_position / 25),
                                     base * (1 + self.MAX_TURN_RATIO * line_position / -24))

            elif self.state == self.S3_MERGE_LANE:
                imu = self.sensor_data.value.get("imu")
                #see if data for imu
                if not self.line_follow_flag.value or imu is None:
                    self.set_targets(0.0, 0.0, "stop")
                    self.state = self.S4_STOP_MOTOR

                #go back to line following if no lane change flag
                elif not self.lane_change_flag.value:
                    self.state = self.S2_FOLLOW_LINE

                #lane change 
                else:
                    #yaw angle (heading) relative ot the starting yaw angle
                    theta = ((imu[0] - self.theta_ref) + 180) % 360 - 180
                    position = (self.left_position_cm.value + self.right_position_cm.value) / 2 #center pos.
                    #sideways displacement is sin(relative yaw angle)/change in poition
                    self.displacement += sin(radians(theta)) * (position - self.last_position)
                    self.last_position = position
                    #if passed half way then reverse curve
                    if abs(self.displacement) >= self.LANE_DISTANCE / 2 and not self.passed_center:
                        self.direction *= -1
                        self.passed_center = True
                    #go back to line following once yaw angle is near zero
                    if abs(theta) <= 10 and self.passed_center:
                        self.lane_change_flag.set(False)
                        self.following_target.set(True)
                        self.state = self.S2_FOLLOW_LINE
                    base = self.active_setpoint
                    # Vl= Vbase(1-(k*dir))
                    self.set_targets(base * (1 - self.MERGE_TURN_RATIO * self.direction),
                                     base * (1 + self.MERGE_TURN_RATIO * self.direction))

            #left and right targets should be 0 for stop
            elif self.state == self.S4_STOP_MOTOR:
                self.set_targets(0.0, 0.0, "stop")
                self.line_follow_flag.set(False)
                self.lane_change_flag.set(False)
                self.state = self.S0_IDLE

            yield self.state

    #function for setting left and right targets value shares for motor task
    def set_targets(self, left, right, mode="rad/s"):
        self.left_target.set(float(left))
        self.right_target.set(float(right))
        self.left_mode.set(mode)
        self.right_mode.set(mode)
