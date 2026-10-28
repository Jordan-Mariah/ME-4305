"""this is the nonblocking test for the motor. Here we set up the motor task 
then it can be instantiated by the left and right motors"""
from time import ticks_us, ticks_diff, ticks_add
from driver import Motor
from encoder import Encoder
from tshare import Share
from PIDcontroler import PID
from math import pi, isfinite, sin
import line_sensor # Only for constant access. Maybe a better solution

class TaskMotor:

    #Task state constants
    S0_IDLE = 0
    S1_PREPARE = 1
    S2_SETTLE = 2
    S3_START_TEST = 3
    S4_TAKE_SAMPLES = 4
    S5_STOP_MOTOR = 5
    S6_FOLLOW_LINE = 6
    S7_MERGE_LANE = 7

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
    def __init__(self, motor: Motor, encoder: Encoder, test_flag: Share, next_effort: Share, data: Share, sensor_data: Share, line_follow_flag: Share, lane_change_flag: Share, lane_change_dir: Share, name: str,
                 battery_voltage, counts_per_revolution,
                 driver_sign, encoder_sign, next_kp, period_ms=100):
        #Pull args
        self.motor = motor
        self.encoder = encoder
        self.test_flag = test_flag
        self.line_follow_flag = line_follow_flag
        self.next_effort = next_effort
        self.next_kp = next_kp #gain from main.py
        self.data_to_user = data.value #from the the dictionary allocated in main.py.
        self.sensor_data = sensor_data # Dictionary of different sensor outputs
        self.lane_change_flag = lane_change_flag
        self.lane_change_dir = lane_change_dir
        self.side = name
        #config based on voltage encoder scale, and wheel sign
        self.battery_voltage = battery_voltage
        self.counts_per_revolution: float = counts_per_revolution
        self.driver_sign = driver_sign
        self.encoder_sign = encoder_sign
        #same period for scheduler execution and sample log
        if period_ms <= 0:
            raise ValueError("Motor period must be positive")
        self.period_ms = period_ms

        #Configure PID controller
        self.pid = PID(
            kp = self.KP,
            # ki = self.KI,
            # kd = self.KD,
            ff_gain=self.KM
        )

        # Set starting values
        self.test_flag.set(False)
        self.state = self.S0_IDLE


    def run(self):
        # Keep the generator alive so the scheduler can run future requests.
        while True:

            # Yield without driving the motor until start() changes the state.
            if self.state == self.S0_IDLE:
                if self.test_flag.value == True:
                    # Mark the task active and let its next scheduled turn prepare the motor.
                    self.state = self.S1_PREPARE

                if self.lane_change_flag.value == True:
                    self.first_run = True
                    self.state = self.S7_MERGE_LANE

                if self.line_follow_flag.value == True and not self.lane_change_flag.value:
                    self.motor_startup()

                    self.sample_start = ticks_us()
                    record_data = False
                    self.state = self.S6_FOLLOW_LINE

            #inidcates the starting state of the FSM
            elif (self.state == self.S1_PREPARE): 
                

                # Dissable motor and reset timer for next state
                self.motor.disable()
                # mark response bufferes empty
                self.data_to_user["count"] = 0
                self.data_to_user["status"] = "completed" #compkleted is default outcome
               #both setting must be real numbers and positive
                if self.battery_voltage is None or self.counts_per_revolution is None:
                    raise ValueError(" battery voltage and counts per wheel revolution should be set in main.py")
                if (not isfinite(self.battery_voltage) or self.battery_voltage <= 0
                        or not isfinite(self.counts_per_revolution) or self.counts_per_revolution <= 0):
                    raise ValueError("keep the Battery voltage and encoder scale finite/positive")
                #each dirrection must be 1 or -1
                if self.driver_sign not in (-1, 1) or self.encoder_sign not in (-1, 1):
                    raise ValueError("Set verified driver and encoder signs in main.py")
                self.active_setpoint = float(self.next_effort.value) #setpoint = (1550 encoder count/s)/(2pi/1437)  = 6.8 rad/s
                active_kp = float(self.next_kp.value) if self.next_kp is not None else self.KP
                if not isfinite(self.active_setpoint) or not isfinite(active_kp) or active_kp < 0:
                    raise ValueError("Setpoint and gain must be finite and gain must be positive or 0")
                self.pid.set_kp(active_kp)
                self.pid.reset()
                self.settle_start = ticks_us()
                self.state = self.S2_SETTLE         

            #settle the motor
            elif (self.state == self.S2_SETTLE): 
                self.encoder.update() #refresh encoders measurments but dont repetedly zero
                #the difference between the current time and the start is the settling time
                self.elapsed_settling = ticks_diff(ticks_us(), self.settle_start)
                #give 1 second for the wheel to die before the next trial starts

                if self.elapsed_settling >= 1_000_000:
                    self.state = self.S3_START_TEST

            elif self.state == self.S3_START_TEST: 
                # Activate motor
                self.motor.enable()
                #Reset position and  time right before step
                self.encoder.zero()
                self.pid.reset()

                # here we set the cycle time values for the first time
                self.sample_start = ticks_us()
                #right now we only have proportional controll
                pid_out = self.pid.next_output(self.active_setpoint, 0.0)
                if not isfinite(pid_out):
                    self.motor.disable()
                    raise ValueError("Controller requested a nonfinite voltage")
                Vsat = max(-self.battery_voltage, min(self.battery_voltage, pid_out))
                self.motor.set_effort(self.driver_sign * 100 * Vsat / self.battery_voltage)
                # Match sample loggingwith teh MOTOR_PERIOD_MS passed in main.py
                # convert the period from ms to us for ticks_us().
                self.sample_interval = self.period_ms * 1000
                self.sample_duration = 5_000_000 #2 seconds
                self.sample_next = self.sample_start #first sample imidiately scheduled


                self.state = self.S4_TAKE_SAMPLES

            elif (self.state == self.S4_TAKE_SAMPLES):
                #Stop on the first scheduled turn at/after the deadline
                if ticks_diff(ticks_us(), self.sample_start) >= self.sample_duration:
                    self.motor.set_effort(0)
                    self.motor.disable()
                    self.test_flag.set(False)
                    self.state = self.S0_IDLE
                    yield self.state
                    continue

                #update encoder for each itteration of run
                self.encoder.update()

                # Update motor for each itteration of run
                #here we are tracking the encoder usinging encoder objects
                current_velocity = (self.encoder_sign * self.encoder.get_velocity()
                                    * 2 * pi / self.counts_per_revolution)
                #where next_effort is the velocity setpoint in rad/s
                target_velocity: float = self.active_setpoint
                pid_out = self.pid.next_output(target_velocity, current_velocity)

                # process the outputs of the controller so then we can apply Kpwm
                # do not take invaludi output clamp.

                #convert volts to duty
                if not isfinite(pid_out):
                    self.motor.disable()
                    raise ValueError("Controller requested a nonfinite voltage")
                Vsat = max(-self.battery_voltage, min(self.battery_voltage, pid_out))
                pwm_pct = self.driver_sign * 100 * Vsat / self.battery_voltage

                self.motor.set_effort(pwm_pct)

                current_time = ticks_us()
                time_elapse = ticks_diff(current_time, self.sample_start)

                #save test data into arrays - got help from copiolot because still wrapping my head around
                #using buffer arrays to store data and then have the user read from arrays and print to serial
                buffer = self.data_to_user
                i = buffer["count"]
                if i >= len(buffer["time_us"]):
                    #report failure here rather than writing beyond capacity.
                    buffer["status"] = "failed"
                    self.motor.set_effort(0)
                    self.motor.disable()
                    self.state = self.S5_STOP_MOTOR
                    yield self.state
                    continue
                buffer["time_us"][i] = time_elapse
                buffer["setpoint"][i] = self.active_setpoint
                buffer["velocity"][i] = current_velocity
                buffer["voltage"][i] = Vsat
                buffer["count"] = i + 1 
                

            elif (self.state == self.S5_STOP_MOTOR):
                # Power off motors and lower test_flag
                self.motor.set_effort(0)
                self.motor.disable()

                self.test_flag.value = False
                self.state = self.S0_IDLE

            elif self.state == self.S6_FOLLOW_LINE:
                self.encoder.update()

                # Lane changing behavior overrides line follow. Idle state should switch to lane change
                if self.lane_change_flag.value == True:
                    self.state = self.S0_IDLE

                # Get share data, set velocity and sensor data
                line_position: float = self.sensor_data.value["line"] if not self.sensor_data.value["line"] == None else 0
                target_velocity: float = self.active_setpoint
                current_velocity = self.encoder_sign * self.encoder.get_velocity() * 2 * pi /self.counts_per_revolution

                # Scale speed if sensor indicates line it curving left or right.
                if self.side == "right":
                    target_velocity += target_velocity * self.MAX_TURN_RATIO * (line_position / -24)
                elif self.side == "left":
                    target_velocity += target_velocity * self.MAX_TURN_RATIO * (line_position / 25)

                # Set next motor voltage
                pid_out = self.pid.next_output(target_velocity, current_velocity) 
                Vsat = max(-self.battery_voltage, min(self.battery_voltage, pid_out))
                self.motor.set_effort(self.driver_sign * 100 * Vsat / self.battery_voltage)

                # TODO Sacrilege
                if self.side == "left" and record_data:
                    current_time = ticks_us()
                    time_elapse = ticks_diff(current_time, self.sample_start)
    
                    #save test data into arrays - got help from copiolot because still wrapping my head around
                    #using buffer arrays to store data and then have the user read from arrays and print to serial
                    buffer = self.data_to_user
                    i = buffer["count"]
                    if i >= len(buffer["time_us"]) - 1:
                        record_data = False
                        buffer["time_us"][i] = -1
                        buffer["centroid_pos"][i] = -1
                        buffer["count"] = i + 1
                    else:
                        buffer["time_us"][i] = time_elapse
                        buffer["centroid_pos"][i] = line_position
                        buffer["count"] = i + 1 


                if self.line_follow_flag.value == False:
                    self.motor.disable()
                    self.state = self.S0_IDLE

            elif self.state == self.S7_MERGE_LANE:
                # Perform first run initialisation
                if self.first_run == True:
                    dir = 1 if self.lane_change_dir.value == "left" else -1
                    passed_center = False
                    last_disp: float = 0
                    last_time = ticks_us()
                    displacement: float = 0.0
                    theta_ref: float = self.sensor_data.value["imu"][0]
                    
                    # If we are starting with a lane change, init the motor
                    if self.line_follow_flag == False:
                        self.motor_startup()

                self.encoder.update()

                # Get and set speeds in rad/s and cm/s
                target_velocity: float = self.active_setpoint
                if self.side == "right":
                    target_velocity += target_velocity * self.MAX_TURN_RATIO * dir
                elif self.side == "left":
                    target_velocity += target_velocity * self.MAX_TURN_RATIO * -dir
                    
                current_velocity = self.encoder_sign * self.encoder.get_velocity() * 2 * pi /self.counts_per_revolution
                
                # Set next motor voltage
                pid_out = self.pid.next_output(target_velocity, current_velocity) 
                Vsat = max(-self.battery_voltage, min(self.battery_voltage, pid_out))
                self.motor.set_effort(self.driver_sign * 100 * Vsat / self.battery_voltage)

                # Lane changing logic

                # Get the (linearized) displacement since last run and add to running total TODO: make this a trapazoidal sum
                theta: float = (((-self.sensor_data.value["imu"][0] - theta_ref)+180)%360 - 180)
                velocity_cm = current_velocity * 3.5 # Convert rad/s to cm/s
                dt = ticks_diff(ticks_us(), last_time) / 1_000_000 # convert us to s
                last_time = ticks_us()
                this_disp = sin(theta) * velocity_cm * dt # this displacement in cm

                displacement += this_disp
                last_disp = this_disp

                '''Print output for debugging:
                side: left, vel: 26.813426, displacement: 0.0, dt: 0.000969
                side: right, vel: 26.372146, displacement: 0.0, dt: 0.000926
                side: left, vel: 24.149692, displacement: 0.0, dt: 0.04938
                side: right, vel: 28.64887, displacement: 0.0, dt: 0.049107
                side: left, vel: 20.899238, displacement: 0.38101852, dt: 0.049775
                side: right, vel: 31.912288, displacement: 0.5939092, dt: 0.050811
                side: left, vel: 20.205162, displacement: 1.1367164, dt: 0.050726
                side: right, vel: 32.324004, displacement: 1.80074544, dt: 0.050637
                side: left, vel: 19.637136, displacement: 0.36478352, dt: 0.049861
                side: right, vel: 31.875834, displacement: 0.54617832, dt: 0.049922
                side: left, vel: 19.811526, displacement: 1.203807, dt: 0.04942
                side: right, vel: 31.498268, displacement: 1.8699084, dt: 0.049041
                side: left, vel: 20.256484, displacement: 0.881238768, dt: 0.049838
                side: right, vel: 31.80206, displacement: 1.356678, dt: 0.050508
                side: left, vel: 19.52439, displacement: 0.35594864, dt: 0.050141
                side: right, vel: 31.725656, displacement: 0.51120732, dt: 0.049666
                side: left, vel: 20.054568, displacement: 1.3371749, dt: 0.050345
                side: right, vel: 31.851232, displacement: 2.0721264, dt: 0.050426
                side: left, vel: 19.760518, displacement: 0.478654632, dt: 0.050318
                side: right, vel: 31.638618, displacement: 0.6852268, dt: 0.050769
                side: left, vel: 19.710248, displacement: 1.0715427, dt: 0.049675
                side: right, vel: 31.061068, displacement: 1.6119697, dt: 0.049272
                side: left, vel: 19.761696, displacement: 1.7429434, dt: 0.050323
                side: right, vel: 31.592352, displacement: 2.6855712, dt: 0.050335
                side: left, vel: 19.771124, displacement: 0.7562446, dt: 0.050295
                side: right, vel: 30.544614, displacement: 1.1676656, dt: 0.050082
                side: left, vel: 19.70486, displacement: 1.2115972, dt: 0.048895
                side: right, vel: 31.02566, displacement: 1.88339468, dt: 0.048811
                side: left, vel: 19.847658, displacement: 1.7093742, dt: 0.050096
                side: right, vel: 30.962062, displacement: 2.6645854, dt: 0.050397
                side: left, vel: 20.019028, displacement: 0.71535256, dt: 0.049676
                side: right, vel: 31.047126, displacement: 1.1200281, dt: 0.049771
                side: left, vel: 19.96221, displacement: 1.2721438, dt: 0.050582
                side: right, vel: 31.052612, displacement: 1.98038532, dt: 0.050245
                side: left, vel: 20.11986, displacement: 1.68902492, dt: 0.049432
                side: right, vel: 30.773112, displacement: 2.6215864, dt: 0.04971
                side: left, vel: 19.896492, displacement: 0.70246492, dt: 0.049954
                side: right, vel: 30.930934, displacement: 1.0879798, dt: 0.049951
                side: left, vel: 20.27709, displacement: 1.343636, dt: 0.050545
                side: right, vel: 31.217814, displacement: 2.064087, dt: 0.049981
                side: left, vel: 20.080082, displacement: 1.6810106, dt: 0.050281
                side: right, vel: 30.710468, displacement: 2.5854856, dt: 0.050809
                side: left, vel: 20.233756, displacement: 0.6749549, dt: 0.049917
                side: right, vel: 30.736026, displacement: 1.0615567, dt: 0.049776
                side: left, vel: 20.163872, displacement: 1.2798917, dt: 0.050055
                side: right, vel: 31.077342, displacement: 1.9969822, dt: 0.05022
                side: left, vel: 20.347516, displacement: 1.71314, dt: 0.050376
                side: right, vel: 30.687334, displacement: 2.6433344, dt: 0.049832
                side: left, vel: 20.386508, displacement: 0.712286, dt: 0.049526
                side: right, vel: 30.805946, displacement: 1.1269792, dt: 0.049656
                side: left, vel: 20.370472, displacement: 1.007712, dt: 0.049553
                side: right, vel: 30.799744, displacement: 1.5746851, dt: 0.049667
                side: left, vel: 19.950068, displacement: 1.8198828, dt: 0.054448
                side: right, vel: 29.956132, displacement: 2.7987066, dt: 0.054649
                side: left, vel: 19.841654, displacement: 1.00492, dt: 0.046255
                side: right, vel: 30.010874, displacement: 1.5762135, dt: 0.045874
                side: left, vel: 20.005742, displacement: 0.8330561, dt: 0.049716
                side: right, vel: 30.251244, displacement: 1.3143886, dt: 0.050088
                side: left, vel: 20.208652, displacement: 1.83385032, dt: 0.049948
                side: right, vel: 30.1387, displacement: 2.8150172, dt: 0.050218
                side: left, vel: 19.979222, displacement: 1.3595747, dt: 0.049766
                side: right, vel: 30.223292, displacement: 2.0999422, dt: 0.049601
                side: left, vel: 20.208652, displacement: 0.7038671, dt: 0.049961
                side: right, vel: 30.13583, displacement: 1.1263366, dt: 0.049746
                side: left, vel: 20.640968, displacement: 1.6609366, dt: 0.050398
                side: right, vel: 30.2709, displacement: 2.5339044, dt: 0.050541

                '''

                # Check if past centerpoint, if so reverse offset (curve in other dir)
                if displacement >= self.LANE_DISTANCE / 2 and not passed_center:
                    dir *= -1
                    passed_center = True
                # Check if total line distance travelled, if so finish lane change
                elif displacement >= self.LANE_DISTANCE and passed_center:
                    # If line follow behavior is after this, go stright there and do not reset motor.
                    # Note: Both motors act independantly, if one motor finishes and the other doesnt, bad things will happen.
                    if self.line_follow_flag.value:
                        self.state = self.S6_FOLLOW_LINE
                    else:
                        self.motor.disable()
                        self.state = self.S0_IDLE
                    self.lane_change_flag.value = False

                if self.first_run:
                    self.first_run = False
                
                
            yield self.state
    
    
    def motor_startup(self):
        self.active_setpoint = float(self.next_effort.value)
        # Activate motor
        self.motor.enable()
        #Reset position and  time right before step
        self.encoder.zero()
        self.pid.reset()

        active_kp = float(self.next_kp.value) if self.next_kp is not None else self.KP
        self.pid.set_kp(active_kp)