"""this is the nonblocking test for the motor. Here we set up the motor task 
then it can be instantiated by the left and right motors"""
from time import ticks_us, ticks_diff, ticks_add
from driver import Motor
from encoder import Encoder
from tshare import Share
from PIDcontroler import PID
from math import pi, isfinite, sin, radians
import line_sensor # Only for constant access. Maybe a better solution

class TaskMotor:

    #Task state constants
    S0_IDLE = 0
    S1_DUTY = 1
    S2_SET_RAD_PER_SEC_CL = 2
    S3_SET_CM_PER_SEC_CL = 3
    S4_STOP = 4
    S5_TEST = 5

    # Other task constants
    KP: float = 0.001
    KI: float = 0.0001
    KD: float = 0
    KM: float = 0.7 # V / (rad/s)
    MAX_TURN_RATIO: float = 0.2 # Max multiplier applied to target speed to stay on line
    MERGE_TURN_RATIO: float = 0.2 # Multiplier applied to target speed when changing lanes
    LANE_DISTANCE: float = 20.0


    '''1) determining whethere motor driver side is left or right 
    2) Store side as a string on object 
    3) Inititializing timers for motor 
    4) Init motor's timer channels
    5)calling the Motor class for left or right motor '''
    def __init__(self, motor,encoder, pid, mode: Share, target: Share, actual_velocity: Share, position_cm: Share, test_flag: Share,
                 next_kp: Share, data:Share, name: str, battery_voltage, counts_per_revolution, wheel_radius_cm, driver_sign, encoder_sign, period_ms=100, test_target=None, test_duration_ms=2000, test_flags=()):
        #Pull args
        self.test_target = test_target
        self.test_duration_us = test_duration_ms * 1000
        self.test_flags = test_flags
        self.motor = motor
        self.encoder=encoder          # Encoder object measuring the left wheel position
        self.pid=pid                   # PID controller for the left motor
        self.mode=mode                 # Shared motor mode from task maneuver
        self.target=target             # Shared target: duty%, rad/s, or cm/s based on mode
        self.actual_velocity=actual_velocity  # Shared measured velocity (rad/s)
        self.position_cm= position_cm   # Shared wheel position (cm)
        self.test_flag=test_flag       # Shared flag used to request/indicate a left-motor test
        self.next_kp=next_kp                #Gain fo next test
        self.data=data                 #shared data for motor
        self.name=name                  #left or right
        self.battery_voltage= battery_voltage
        self.counts_per_revolution= counts_per_revolution
        self.wheel_radius_cm= wheel_radius_cm
        self.driver_sign=driver_sign
        self.encoder_sign=encoder_sign
        self.period_ms=period_ms # set motor task to scheduling period.
    
        
        if period_ms <= 0:
            raise ValueError("Motor period must be positive")
        self.period_ms = period_ms

        # Set starting values
        self.test_flag.set(False)
        self.state = self.S0_IDLE
        self.motor.set_effort(0)
        self.motor.disable()


    def run(self):
        # Keep the generator alive so the scheduler can run future requests.
        while True:

            self.feedback()

            # Yield without driving the motor until start() changes the state.
            if self.state == self.S0_IDLE:
                if self.test_flag.value and self.mode.value != "stop":
                    self.start_test()
                elif any(flag.value for flag in self.test_flags):
                    self.motor.set_effort(0)
                    self.motor.disable()
                elif self.mode.value == "duty":
                    self.motor.enable()
                    # Mark the task active and let its next scheduled turn prepare the motor.
                    self.state = self.S1_DUTY
                elif self.mode.value == "rad/s":
                    self.prepare_pid()
                    self.motor.enable()
                    self.state = self.S2_SET_RAD_PER_SEC_CL
                elif self.mode.value == "cm/s":
                    self.prepare_pid()
                    self.motor.enable()
                    self.state = self.S3_SET_CM_PER_SEC_CL
                elif self.mode.value == "stop":
                    self.motor.set_effort(0)
                    self.motor.disable()
                    self.state = self.S4_STOP


            elif (self.state == self.S1_DUTY):
                if self.mode.value != "duty" or any(flag.value for flag in self.test_flags):
                    self.motor.set_effort(0)
                    self.motor.disable()
                    self.state = self.S0_IDLE

                elif self.mode.value == "duty":
                    self.duty_percent = self.target.value
                    if self.duty_percent > 100:
                        self.duty_percent = 100
                    elif self.duty_percent < -100:
                        self.duty_percent = -100
                    self.effort = self.duty_percent * self.driver_sign
                    self.motor.set_effort(self.effort)

            elif (self.state == self.S2_SET_RAD_PER_SEC_CL):
                if self.mode.value != "rad/s" or any(flag.value for flag in self.test_flags):
                    self.motor.set_effort(0)
                    self.motor.disable()
                    self.state = self.S0_IDLE

                else:
                    self.control_velocity(self.target.value)

            elif (self.state == self.S3_SET_CM_PER_SEC_CL):
                if self.mode.value != "cm/s" or any(flag.value for flag in self.test_flags):
                    self.motor.set_effort(0)
                    self.motor.disable()
                    self.state = self.S0_IDLE

                else:
                    #get the velocity in rad/s from cm/s
                    target_rad_s = self.target.value / self.wheel_radius_cm
                    self.control_velocity(target_rad_s)
                    

            elif (self.state == self.S4_STOP):
                self.motor.set_effort(0)
                self.motor.disable()
                if self.mode.value != "stop":
                    self.state = self.S0_IDLE

            elif self.state == self.S5_TEST:
                if self.mode.value == "stop" or not self.test_flag.value:
                    self.finish_test("cancelled")
                else:
                    elapsed = ticks_diff(ticks_us(), self.test_start)
                    buffer = self.data.value
                    i = buffer["count"]
                    if i >= len(buffer["time_us"]):
                        self.finish_test("buffer_full")
                    else:
                        buffer["time_us"][i] = elapsed
                        buffer["velocity_rad_s"][i] = self.actual_velocity.value
                        buffer["count"] = i + 1
                        if elapsed >= self.test_duration_us:
                            self.finish_test("completed")
                        else:
                            self.control_velocity(self.test_speed)

            yield self.state
                
    def control_velocity(self, target_rad_s):

        #get the measured velocity from the encoder
        measured_velocity = self.actual_velocity.value

        #clip voltage to battery voltage
        voltage = self.pid.next_output(target_rad_s, measured_velocity)
        voltage = max(-self.battery_voltage, min(self.battery_voltage, voltage))

        #set motor effort based on duty cycle from calculated votage that attains desired velocity
        duty_percent = 100 * voltage / self.battery_voltage
        self.motor.set_effort(duty_percent * self.driver_sign)

    def feedback(self):
        """update the encoder and get the measured and actual velocity and position"""
        #update encoder for each itteration of run
        self.encoder.update()
        #measure velocity in rad/s
        measured_velocity = (self.encoder_sign * self.encoder.get_velocity()* 2 * pi / self.counts_per_revolution)
        self.actual_velocity.set(measured_velocity)
        #get postion in cm
        measured_position = (self.encoder_sign * self.encoder.get_position() * 2 * pi / self.counts_per_revolution)
        self.position_cm.set(measured_position * self.wheel_radius_cm) #must use set to write into shares

           
    def prepare_pid(self):
        if self.next_kp is not None:
            self.pid.set_kp(float(self.next_kp.value))
        self.pid.reset()

    def start_test(self):
        buffer = self.data.value
        if buffer["count"]:
            self.test_flag.set(False)
            return
        self.test_speed = float(self.test_target.value if self.test_target is not None else self.target.value)
        buffer["status"] = "running"
        buffer["target_rad_s"] = self.test_speed
        self.prepare_pid()
        self.test_start = ticks_us()
        self.motor.enable()
        self.state = self.S5_TEST
        self.control_velocity(self.test_speed)

    def finish_test(self, status):
        self.motor.set_effort(0)
        self.motor.disable()
        self.data.value["status"] = status
        self.test_flag.set(False)
        self.state = self.S0_IDLE
