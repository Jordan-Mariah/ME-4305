"""Nonblocking, single-motor experiment task; all trial logic lives here."""
from array import array
from time import ticks_us, ticks_diff, ticks_add
from pyb import Pin, Timer
from driver import Motor
from encoder import Encoder
from tshare import Share


class TaskMotor:

    # This integer state keeps the task asleep until start() is called.
    S_IDLE = -1

    #class attributes listed in uppercase to represent them as constants
    S0_PREPARE = 0
    S1_SETTLE = 1
    S2_STEP_SAMPLE = 2
    S3_STOP_MOTOR = 3
    S4_EXPORT_DATA = 4 

    '''1) determining whethere the motor driver side is left or right 
    2) Storing that side as a string on an object 
    3) Inititializing the timers for the motor 
    4) Initializing the channel for that motor's timer 
    5) Initializing the PWM, DIR and nSLP by calling the Motor class for that specific left or right motor'''
    def __init__(self, motor: Motor, encoder: Encoder, test_flag: Share, name: str):
        #Pull args
        self.motor = motor
        self.encoder = encoder
        self.test_flag = test_flag
        self.side = name

        #specify that the self object holds the varriable
        self.state = self.S_IDLE
        self.effort = [10, 20, 30, 40, 50, 60, 70, 80]
        self.trial_run = 0
        self.done = True
        self.test_flag.set(False)

    # Calling this method prepares another sweep using the existing drivers.
    # def start(self):
    #     # Reject another request while this task is already running.
    #     if not self.done:
    #         return False
    #     # Reset the integer trial index and discard the previous sample timer.
    #     self.trial_run = 0
    #     if hasattr(self, "sample_start"):
    #         del self.sample_start
    #     # Mark the task active and let its next scheduled turn prepare the motor.
    #     self.done = False
    #     self.state = self.S0_PREPARE
    #     return True
        
    #tasks should be non-blocking meaning there should not be while loops or log delays in run
    #run is now a generator funciton because it includes yield
    def run(self):
        # Keep the generator alive so the scheduler can run future requests.
        while True:

            # Yield without driving the motor until start() changes the state.
            if self.state == self.S_IDLE:
                if self.test_flag.value == True:
                    # Reset the integer trial index and discard the previous sample timer.
                    self.trial_run = 0
                    if hasattr(self, "sample_start"):
                        del self.sample_start

                    # Mark the task active and let its next scheduled turn prepare the motor.
                    self.done = False
                    self.state = self.S0_PREPARE
                yield self.state

            #inidcates the starting state of the FSM
            elif (self.state == self.S0_PREPARE):


                #access the encoder object stored in right_encoder and call its zero method to reset the position
                self.encoder.zero()

                #store current clock reading then S1 can later know how mcuh time has elapsed
                self.settle_start = ticks_us()

                #initialize current state to next step in FSM
                print("state0")
                self.state = self.S1_SETTLE         

            #settle the motor
            elif (self.state == self.S1_SETTLE): 
                self.motor.disable()
                self.encoder.update() #refresh encoders measurments but dont repetedly zero
                #elapsed settling time: the difference between the current time and the start of the settling time
                self.elapsed_settling = (ticks_diff(ticks_us(), self.settle_start))
                #give 1 second for the wheel to die before the next trial starts
                if self.elapsed_settling >= 1_000_000:
                    self.state = self.S2_STEP_SAMPLE

            elif (self.state == self.S2_STEP_SAMPLE):

                #if self does not have a sample start attribute then initialize the 
                #...sample start, interval, duration, next, time, and possition
                    if not hasattr(self, "sample_start"):
                        #enable motor and set duty cycle
                        self.motor.enable()
                        self.motor.set_effort(self.effort[self.trial_run])


                        self.sample_start = ticks_us()
                        self.sample_interval = 10_000 #10 ms sampling interval
                        self.sample_duration = 2_000_000 #2 seconds
                        self.sample_next = self.sample_start #first sample imidiately scheduled
                        self.sample_time = array("L") #specify with type code that sample_time stores a time value unsigned long int
                        self.sample_positions = array("i") #specify with type code that sample_position stores signed encoder 
                        #...position as a signed int

                    #update encoder for each itteration of run
                    self.encoder.update()

                    current_time = ticks_us()
                    time_elapse = ticks_diff(current_time, self.sample_start)

                    #if current time is greater than or equal to the next sample time
                    if ticks_diff(current_time, self.sample_next) >= 0:
                        self.sample_time.append(time_elapse) #when the sample occurs
                        self.sample_positions.append(self.encoder.get_position()) #encoder position when sampled
                        self.sample_next = ticks_add(current_time, self.sample_interval) # Handle clock wraparound.
                        #sample interval

                    if time_elapse >= self.sample_duration:
                        self.state = self.S3_STOP_MOTOR

            elif (self.state == self.S3_STOP_MOTOR):
                self.motor.set_effort(0)
                self.motor.disable()

                self.state = self.S4_EXPORT_DATA
                print("state3")

            #note that copilot was used to find the formatting needed for the hasattr structure along with the file.write line
            #This external support was helpful in better organizing the data into a csv instead of manually collecting it
            elif self.state == self.S4_EXPORT_DATA:
                effort = self.effort[self.trial_run]
                # Include the wheel name so left and right data stay separate.
                filename = "step_{}_{}pct.csv".format(self.side, effort)
                with open(filename, "w") as file:
                    file.write("time_us,position\n")

                    for time, position in zip(
                        self.sample_time,
                        self.sample_positions
                    ):
                        file.write("{},{}\n".format(time, position))

                print("Saved:", filename)
                self.trial_run += 1

                if self.trial_run < len(self.effort):
                    # S2 will recreate timing and sample buffers for the next trial.
                    del self.sample_start
                    self.state = self.S0_PREPARE
                else:
                    self.stop()
                    self.done = True
                    self.test_flag.set(False)
                    print("All trials complete")
            else:
                raise ValueError("Invalid state")

            #stop here and wait until main calls next()
            yield self.state

    def stop(self):
        """Leave the selected motor disabled, including after interruption."""
        self.motor.set_effort(0)
        self.motor.disable()
        # Return to idle after normal completion or an interrupted experiment.
        self.state = self.S_IDLE
        self.done = True
        self.test_flag.set(False)

