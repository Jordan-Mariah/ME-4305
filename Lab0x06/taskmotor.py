"""this is the nonblocking test for the motor. Here we set up the motor task 
then it can be instantiated by the left and right motors"""
from time import ticks_us, ticks_diff, ticks_add
from driver import Motor
from encoder import Encoder
from tshare import Share
from PIDcontroler import PID
from math import pi, isfinite


class TaskMotor:

    #Task state constants
    S0_IDLE = 0
    S1_PREPARE = 1
    S2_SETTLE = 2
    S3_START_TEST = 3
    S4_TAKE_SAMPLES = 4
    S5_STOP_MOTOR = 5

    # Other task constants
    KP: float = 0.001
    KI: float = 0.0001
    KD: float = 0


    '''1) determining whethere motor driver side is left or right 
    2) Store side as a string on object 
    3) Inititializing timers for motor 
    4) Init motor's timer channels
    5)calling the Motor class for left or right motor '''
    def __init__(self, motor: Motor, encoder: Encoder, test_flag: Share, next_effort: Share, data: Share, name: str,
                 battery_voltage=None, counts_per_revolution=None,
                 driver_sign=None, encoder_sign=None, next_kp=None, period_ms=100):
        #Pull args
        self.motor = motor
        self.encoder = encoder
        self.test_flag = test_flag
        self.next_effort = next_effort
        self.next_kp = next_kp #gain from main.py
        self.data_to_user = data.value #from the the dictionary allocated in main.py.
        self.side = name
        #config based on voltage encoder scale, and wheel sign
        self.battery_voltage = battery_voltage
        self.counts_per_revolution = counts_per_revolution
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

            #inidcates the starting state of the FSM
            elif (self.state == self.S1_PREPARE): 
                

                # Dissable motor and reset timer for next state
                self.motor.disable()
                # mark response bufferes empty
                self.data_to_user["count"] = 0
                self.data_to_user["status"] = "completed" #compkleted is default outcome
               #both setting must be real numbers and positive
                if self.battery_voltage is None or self.counts_per_revolution is None:
                    raise ValueError("Set battery voltage and counts per wheel revolution in main.py")
                if (not isfinite(self.battery_voltage) or self.battery_voltage <= 0
                        or not isfinite(self.counts_per_revolution) or self.counts_per_revolution <= 0):
                    raise ValueError("Battery voltage and encoder scale must be finite and positive")
                #each dirrection must be 1 or -1
                if self.driver_sign not in (-1, 1) or self.encoder_sign not in (-1, 1):
                    raise ValueError("Set verified driver and encoder signs in main.py")
                self.active_setpoint = float(self.next_effort.value) #setpoint = (1550 encoder count/s)/(2pi/1437)  = 6.8 rad/s
                active_kp = float(self.next_kp.value) if self.next_kp is not None else self.KP
                if not isfinite(self.active_setpoint) or not isfinite(active_kp) or active_kp < 0:
                    raise ValueError("Setpoint and gain must be finite; gain must be nonnegative")
                self.pid.set_kp(active_kp)
                self.pid.reset()
                self.settle_start = ticks_us()
                self.state = self.S2_SETTLE         

            #settle the motor
            elif (self.state == self.S2_SETTLE): 
                self.encoder.update() #refresh encoders measurments but dont repetedly zero
                #elapsed settling time: the difference between the current time and the start of the settling time
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

                # Set first-cycle time values
                self.sample_start = ticks_us()
                #initial P command. then sample_start marks the step.
                pid_out = self.pid.next_output(self.active_setpoint, 0.0)
                if not isfinite(pid_out):
                    self.motor.disable()
                    raise ValueError("Controller requested a nonfinite voltage")
                Vsat = max(-self.battery_voltage, min(self.battery_voltage, pid_out))
                self.motor.set_effort(self.driver_sign * 100 * Vsat / self.battery_voltage)
                # Match sample loggingwith teh MOTOR_PERIOD_MS passed in main.py
                # convert the period from ms to us for ticks_us().
                self.sample_interval = self.period_ms * 1000
                self.sample_duration = 2_000_000 #2 seconds
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
                #Encoder reports counts/s
                current_velocity = (self.encoder_sign * self.encoder.get_velocity()
                                    * 2 * pi / self.counts_per_revolution)
                #next_effort -> velocity setpoint in rad/s.
                target_velocity: float = self.active_setpoint
                pid_out = self.pid.next_output(target_velocity, current_velocity)

                # Process controler output (saturate voltage and apply Kpwm)
                # Reject invalid output clamp the signs, and convert volts to duty
                if not isfinite(pid_out):
                    self.motor.disable()
                    raise ValueError("Controller requested a nonfinite voltage")
                pid_out += 1.5 * self.driver_sign # Voltage offset to compensate for... something...
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

            yield self.state
