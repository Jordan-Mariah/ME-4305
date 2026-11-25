"""this is the nonblocking test for the motor. Here we set up the motor task 
then it can be instantiated by the left and right motors"""
from time import ticks_us, ticks_diff, ticks_add
from driver import Motor
from encoder import Encoder
from tshare import Share
from PIDcontroler import PID
from observer import Observer, OBSERVER_PERIOD_MS
from discrete_pi import DiscretePI
from math import pi, isfinite, sin, radians
# Only for constant access. Maybe a better solution
import line_sensor




class TaskMotor:

    REST_SPEED_RAD_S = 0.1
    REST_TIME_US = 300_000
    SETTLE_TIMEOUT_US = 5_000_000


    #Task state constants
    S0_IDLE = 0
    S1_DUTY = 1
    S2_SET_RAD_PER_SEC_CL = 2
    S3_SET_CM_PER_SEC_CL = 3
    S4_STOP = 4
    S5_TEST = 5
    S6_SETTLE = 6
    S7_START_TEST = 7
    S8_TAKE_SAMPLES = 8
    S9_STOP_TEST = 9



    # Other task constants
    KP: float = 0.001
    KI: float = 0.0001
    KD: float = 0
    # V / (rad/s)
    KM: float = 0.7
    # Max multiplier applied to target speed to stay on line
    MAX_TURN_RATIO: float = 0.2
    # Multiplier applied to target speed when changing lanes
    MERGE_TURN_RATIO: float = 0.2
    LANE_DISTANCE: float = 20.0


    '''1) determining whethere motor driver side is left or right 
    2) Store side as a string on object 
    3) Inititializing timers for motor 
    4) Init motor's timer channels
    5)calling the Motor class for left or right motor '''
    def __init__(self, motor: Motor,encoder: Encoder, pid: PID, mode: Share, target: Share, actual_velocity: Share, position_cm: Share, test_flag: Share,
                 next_kp: Share, data:Share, name: str, battery_voltage, counts_per_revolution, wheel_radius_cm, driver_sign, encoder_sign, period_ms=100, test_target=None, test_duration_ms=2000, test_flags=(), velocity_feedback="encoder"):

        #Pull args
        self.test_target = test_target
        self.test_duration_us = test_duration_ms * 1000
        self.test_flags = test_flags
        self.motor = motor
        # Encoder object measuring the left wheel position
        self.encoder=encoder
        # PID controller for the left motor
        self.pid=pid
        # Shared motor mode from task maneuver
        self.mode=mode
        # Shared target: duty%, rad/s, or cm/s based on mode
        self.target=target
        # Shared measured velocity (rad/s)
        self.actual_velocity=actual_velocity
        # Shared wheel position (cm)
        self.position_cm= position_cm
        # Shared flag used to request/indicate a left-motor test
        self.test_flag=test_flag
        #Gain fo next test
        self.next_kp=next_kp
        #shared data for motor
        self.data=data
        #left or right
        self.name=name
        self.battery_voltage= battery_voltage
        self.counts_per_revolution= counts_per_revolution
        self.wheel_radius_cm= wheel_radius_cm
        self.driver_sign=driver_sign
        self.encoder_sign=encoder_sign
        # set motor task to scheduling period.
        self.period_ms=period_ms
        if velocity_feedback not in ("encoder", "observer"):
            raise ValueError("Feedback must be encoder or observer")
        self.velocity_feedback = velocity_feedback
        if isinstance(self.pid, DiscretePI) and velocity_feedback != "observer":
            raise ValueError("Discrete PI requires observer feedback")
    
        
        if period_ms <= 0:
            raise ValueError("Motor period must be positive")
        self.period_ms = period_ms


        # Create observer
        # NOTE: I am doing this here and not main because
        # the observer has no construction parameters. If it
        # werent for the fact that it has it's own state, it
        # could be a function.
        if self.period_ms != OBSERVER_PERIOD_MS:
            raise ValueError("Motor and observer periods must match")
        if self.battery_voltage <= 0 or self.driver_sign not in (-1, 1):
            raise ValueError("Invalid voltage reference or driver sign")
        self.observer = Observer()
        self.position_rad = 0.0
        self.applied_voltage = 0.0
        # Set starting values
        self.test_flag.set(False)
        self.state = self.S0_IDLE
        # Copy of cm/s control loop
        # Abort test
        # if self.mode.value != "test": 
        #     self.motor.set_effort(0)
        #     self.motor.disable()
        #     self.state = self.S9_STOP_TEST
        #else:
        # Record data if sample period elapsed
        # Assumes self.next_test has been set beforehand
        # Power off motors and lower test_flag
        self.motor.set_effort(0)
        self.motor.disable()
        self.experiment = None


    def run(self):

        # Keep the generator alive so the scheduler can run future requests.
        while True:

            self.feedback()

            if self.experiment is not None and self.experiment["active"]:
                self.run_test()
                yield self.state
                continue


            # Yield without driving the motor until start() changes the state.
            if self.state == self.S0_IDLE:
                if self.test_flag.value and self.mode.value != "stop":
                    self.start_test()
                elif any(flag.value for flag in self.test_flags):
                    self.motor.set_effort(0)
                    self.motor.disable()
                elif self.mode.value == "duty":
                    self.observer.reset(self.position_rad)
                    self.motor.enable()

                    # Mark the task active and let its next scheduled turn prepare the motor.
                    self.state = self.S1_DUTY
                elif self.mode.value == "rad/s":
                    self.observer.reset(self.position_rad)
                    self.prepare_pid()
                    self.motor.enable()
                    self.state = self.S2_SET_RAD_PER_SEC_CL
                elif self.mode.value == "cm/s":
                    self.observer.reset(self.position_rad)
                    self.prepare_pid()
                    self.motor.enable()
                    self.state = self.S3_SET_CM_PER_SEC_CL
                elif self.mode.value == "stop":
                    self.motor.set_effort(0)
                    self.motor.disable()
                    #check that there is no stop value
                    #set test flag true
                    #move to settle state
                    self.state = self.S4_STOP
                elif self.mode.value == "test":
                    self.state = self.S5_TEST


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
                    applied_effort = self.motor.set_effort(self.effort)
                    self.applied_voltage = (
                        applied_effort / self.driver_sign
                        * self.battery_voltage / 100
                    )
                    self.observer.next_estimate(self.position_rad, self.applied_voltage)

            elif (self.state == self.S2_SET_RAD_PER_SEC_CL):
                if self.mode.value != "rad/s" or any(flag.value for flag in self.test_flags):
                    self.motor.set_effort(0)
                    self.motor.disable()
                    self.state = self.S0_IDLE

                else:
                    voltage = self.control_velocity(self.target.value)
                    self.observer.next_estimate(self.position_rad, voltage)

            elif (self.state == self.S3_SET_CM_PER_SEC_CL):
                if self.mode.value != "cm/s" or any(flag.value for flag in self.test_flags):
                    self.motor.set_effort(0)
                    self.motor.disable()
                    self.state = self.S0_IDLE

                else:

                    #convert to rad/s from cm/s so that we can use the shared control_velocity func.
                    target_rad_s = self.target.value / self.wheel_radius_cm
                    voltage = self.control_velocity(target_rad_s)
                    self.observer.next_estimate(self.position_rad, voltage)
                    

            elif (self.state == self.S4_STOP):
                self.motor.set_effort(0)
                self.motor.disable()
                if self.mode.value != "stop":
                    self.state = self.S0_IDLE


            yield self.state
                
    def control_velocity(self, target_rad_s):


        #get the measured velocity from the encoder
        measured_velocity = self.actual_velocity.value
        if self.velocity_feedback == "observer":
            measured_velocity = self.observer.velocity()


        #clip voltage to battery voltage
        voltage = self.pid.next_output(target_rad_s, measured_velocity)
        voltage = max(-self.battery_voltage, min(self.battery_voltage, voltage))


        #set motor effort based on duty cycle from calculated votage that attains desired velocity
        duty_percent = 100 * voltage / self.battery_voltage
        applied_effort = self.motor.set_effort(duty_percent * self.driver_sign)
        self.applied_voltage = (
            applied_effort / self.driver_sign * self.battery_voltage / 100
        )
        if isinstance(self.pid, DiscretePI):
            self.pid.update_integral(self.applied_voltage)
        return self.applied_voltage

    def feedback(self):
        """update the encoder and get the measured and actual velocity and position"""

        #update the encoder and start clock
        #update encoder for each itteration of run
        self.encoder.update()

        measured_velocity = (self.encoder_sign * self.encoder.get_velocity()* 2 * pi / self.counts_per_revolution)
        # Pass new values into observer to estimate current state
        #TODO running this causes out of memory error (maybe?)
        # measure velocity in rad/s and set actual velocity to be measured velocity
        self.actual_velocity.set(measured_velocity)

        #get postion in cm and set position to be measured position
        measured_position = (self.encoder_sign * self.encoder.get_position() * 2 * pi / self.counts_per_revolution)
        self.position_rad = measured_position
        #must use set to write into shares
        self.position_cm.set(measured_position * self.wheel_radius_cm)

           
    def prepare_pid(self):
        if self.next_kp is not None:
            self.pid.set_kp(float(self.next_kp.value))
        self.pid.reset()

    def start_test(self, selected=None, speed=None, kp=None):
        if selected is None:
            return False
        if self.experiment["active"] or any(m.data.value["count"] for m in self.experiment["motors"]):
            return False
        if selected not in ("left", "right") or not isfinite(speed) or speed <= 0:
            raise ValueError("Forward test speed must be positive and finite")
        if not isfinite(kp) or kp < 0:
            raise ValueError("Test Kp must be nonnegative and finite")
        self.experiment["selected"] = selected
        self.experiment["speed"] = float(speed)
        self.experiment["kp"] = float(kp)
        self.experiment["request_time"] = ticks_us()
        self.experiment["start"] = None
        self.experiment["ready"][0] = self.experiment["ready"][1] = None
        self.experiment["active"] = True
        for motor in self.experiment["motors"]:
            motor.motor.set_effort(0)
            motor.motor.disable()
            motor.applied_voltage = 0.0
            motor.mode.value = "test"
            motor.state = motor.S6_SETTLE
            motor.test_idx = 0
            motor.test_flag.set(motor.name == selected)
            motor.data.value["status"] = "running"
            motor.data.value["target_rad_s"] = self.experiment["speed"]
        return True


    def finish_test(self, status):

        for motor in self.experiment["motors"]:
            motor.motor.set_effort(0)
            motor.motor.disable()
            motor.applied_voltage = 0.0
            motor.test_flag.set(False)
            motor.data.value["status"] = status
            motor.mode.value = "idle"
            motor.state = motor.S0_IDLE
        self.experiment["active"] = False


    def run_test(self):
        motor = self
        now = ticks_us()
        selected = next(m for m in self.experiment["motors"] if m.name == self.experiment["selected"])
        if (not selected.test_flag.value
                or any(m.mode.value != "test" for m in self.experiment["motors"])):
            self.finish_test("aborted")
            return

        if self.experiment["start"] is None:
            if ticks_diff(now, self.experiment["request_time"]) >= self.SETTLE_TIMEOUT_US:
                self.finish_test("settle_timeout")
                return
            index = 0 if motor is self.experiment["motors"][0] else 1
            if abs(motor.actual_velocity.value) <= self.REST_SPEED_RAD_S:
                if self.experiment["ready"][index] is None:
                    self.experiment["ready"][index] = now
            else:
                self.experiment["ready"][index] = None
            if all(t is not None and ticks_diff(now, t) >= self.REST_TIME_US
                   for t in self.experiment["ready"]):
                for item in self.experiment["motors"]:
                    item.pid.set_kp(self.experiment["kp"])
                    item.pid.reset()
                    item.observer.reset(item.position_rad)
                    item.motor.enable()
                    item.state = item.S8_TAKE_SAMPLES

                self.experiment["start"] = ticks_add(now, motor.period_ms * 1000)
            return

        elapsed = ticks_diff(now, self.experiment["start"])
        if elapsed < 0:
            return
        if elapsed >= min(m.test_duration_us for m in self.experiment["motors"]):
            self.finish_test("completed")
            return
        data = selected.data.value
        if selected.test_idx >= len(data["time_us"]):
            self.finish_test("buffer_full")
            return
        estimate = motor.observer.velocity()
        voltage = motor.control_velocity(self.experiment["speed"])
        if motor is selected:
            i = motor.test_idx
            data["time_us"][i] = elapsed
            data["setpoint_rad_s"][i] = self.experiment["speed"]
            data["velocity_rad_s"][i] = motor.actual_velocity.value
            data["estimated_velocity_rad_s"][i] = estimate
            data["voltage_V"][i] = voltage
            motor.test_idx += 1
            data["count"] = motor.test_idx
        motor.observer.next_estimate(motor.position_rad, voltage)

