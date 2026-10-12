"""Nonblocking, single-motor experiment task; all trial logic lives here."""
from time import ticks_us, ticks_diff, ticks_add
from driver import Motor
from encoder import Encoder
from tshare import Share
from PIDcontroler import PID


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
    KPWM: float = 14.29


    '''1) determining whethere the motor driver side is left or right 
    2) Storing that side as a string on an object 
    3) Inititializing the timers for the motor 
    4) Initializing the channel for that motor's timer 
    5) Initializing the PWM, DIR and nSLP by calling the Motor class for that specific left or right motor'''
    def __init__(self, motor: Motor, encoder: Encoder, test_flag: Share, next_effort: Share, data: Share, name: str):
        #Pull args
        self.motor = motor
        self.encoder = encoder
        self.test_flag = test_flag
        self.next_effort = next_effort
        self.data_to_user: list[tuple[int, int]] = data.value # type: ignore
        self.side = name

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
                self.settle_start = ticks_us()
                self.state = self.S2_SETTLE         

            #settle the motor
            elif (self.state == self.S2_SETTLE): 
                self.encoder.update() #refresh encoders measurments but dont repetedly zero
                #elapsed settling time: the difference between the current time and the start of the settling time
                self.elapsed_settling = ticks_diff(ticks_us(), self.settle_start)
                #give 1 second for the wheel to die before the next trial starts

                if self.elapsed_settling >= 1_000_000:
                    #access the encoder object passed into this task and call its zero method to reset the position
                    self.encoder.zero()
                    self.state = self.S3_START_TEST

            elif self.state == self.S3_START_TEST: 
                # Activate motor
                self.motor.enable()

                # Set first-cycle time values
                self.sample_start = ticks_us()
                self.sample_interval = 10_000 #10 ms sampling interval
                self.sample_duration = 2_000_000 #2 seconds
                self.sample_next = self.sample_start #first sample imidiately scheduled


                self.state = self.S4_TAKE_SAMPLES

            elif (self.state == self.S4_TAKE_SAMPLES): 

                #update encoder for each itteration of run
                self.encoder.update()

                # Update motor for each itteration of run
                current_velocity = self.encoder.get_velocity()
                target_velocity: float = self.next_effort.value # type: ignore
                pid_out = self.pid.next_output(target_velocity, current_velocity)

                # Process controler output (saturate voltage and apply Kpwm)
                Vsat = 7 if pid_out > 7 else pid_out
                Vsat = -7 if pid_out < -7 else pid_out
                pwm_pct = Vsat * self.KPWM

                self.motor.set_effort(pwm_pct)

                current_time = ticks_us()
                time_elapse = ticks_diff(current_time, self.sample_start)

                #if current time is greater than or equal to the next sample time
                if ticks_diff(current_time, self.sample_next) >= 0:
                    # Build data packet to send to task user
                    this_packet: tuple[int, int] = (time_elapse, self.encoder.get_position())
                    self.data_to_user.append(this_packet) #append the current sample to the data buffer for user task

                    # Reset sample clock
                    self.sample_next = ticks_add(current_time, self.sample_interval)

                if time_elapse >= self.sample_duration:
                    self.state = self.S5_STOP_MOTOR

            elif (self.state == self.S5_STOP_MOTOR):
                # Power off motors and lower test_flag
                self.motor.set_effort(0)
                self.motor.disable()

                self.test_flag.value = False
                self.state = self.S0_IDLE

            yield self.state
