"""Nonblocking, single-motor experiment task; all trial logic lives here."""
from array import array
from time import ticks_us, ticks_diff, ticks_add
from pyb import Pin, Timer
from driver import Motor
from encoder import Encoder


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
    def __init__(self, side):
        # The side parameter receives the string argument "left" or "right".
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        self.side = side # Store the string wheel name as a new motor task object.

        #Configure the hardware, create drivers, and store the starting state
        # Configure the PWM timers once.
        #initialize the selected PWM timer as an object from the py built in Timer class 
        self.pwm_timer = Timer(4 if side == "left" else 1, freq=20_000)

        '''this calls the timer object's channel method to configure channel 1.
        The method returns a timer channel object that I store in pwm_right'''
        pwm = self.pwm_timer.channel(
            1, #channel 1
            mode=Timer.PWM, #gen PWM
            pin=Pin.cpu.B6 if side == "left" else Pin.cpu.A8,
            pulse_width_percent=0 #initially have 0% duty cycle
        )

        #call the motor class from motor.py and pass in it's initializations
        self.motor = Motor(
            pwm,
            Pin.cpu.B5 if side == "left" else Pin.cpu.A9,
            Pin.cpu.A10 if side == "left" else Pin.cpu.B4
        )

        #Initialize the selected encoder's timer  as an object in the TaskMotor class from the py Timer class
        self.encoder_timer = Timer(2 if side == "left" else 3, prescaler=0, period=65535)

        '''Initialize the selected encoder as an object 
        Store a reference to this object in the encoder attribute where an attribute is a named value'''
        self.encoder = Encoder(
            self.encoder_timer,
            Pin.cpu.A0 if side == "left" else Pin.cpu.A6,
            Pin.cpu.A1 if side == "left" else Pin.cpu.A7,
        )

        #specify that the self object holds the varriable
        self.state = self.S_IDLE
        self.effort = [0.0]
        self.trial_run = 0
        self.done = True

    # Calling this method prepares one selected-duty test using the existing drivers.
    def start(self, duty_cycle):
        # Reject another request while this task is already running.
        if not self.done:
            return False
        if not -100.0 <= duty_cycle <= 100.0:
            raise ValueError("Duty cycle must be between -100 and +100")
        self.effort = [float(duty_cycle)]
        # Reset the integer trial index and discard the previous sample timer.
        self.trial_run = 0
        if hasattr(self, "sample_start"):
            del self.sample_start
        # Mark the task active and let its next scheduled turn prepare the motor.
        self.done = False
        #** Enter state 0 when the UI requests a test with l or r.
        self.state = self.S0_PREPARE
        return True
        
    #tasks should be non-blocking meaning there should not be while loops or log delays in run
    #run is now a generator funciton because it includes yield
    def run(self):
        # Keep the generator alive so the scheduler can run future requests.
        while True:
            # Yield without driving the motor until start() changes the state.
            if self.state == self.S_IDLE:
                yield self.state
                continue
            #inidcates the starting state of the FSM
            if (self.state == self.S0_PREPARE):
                #access the encoder object stored in right_encoder and call its zero method to reset the position
                self.encoder.zero()

                #store current clock reading then S1 can later know how mcuh time has elapsed
                self.settle_start = ticks_us()

                #initialize current state to next step in FSM
                print("Preparing {} motor; settling for 1 second before the 2-second test.".format(self.side))
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
                        #** Advance from the scheduled deadline so small delays do not skip every other sample.
                        self.sample_next = ticks_add(self.sample_next, self.sample_interval) # Handle clock wraparound.
                        #sample interval

                    if time_elapse >= self.sample_duration:
                        #** Enter state 3 after the two-second sampling period.
                        self.state = self.S3_STOP_MOTOR

            elif (self.state == self.S3_STOP_MOTOR):
                self.motor.set_effort(0)
                self.motor.disable()

                self.state = self.S4_EXPORT_DATA
                print("Motor stopped; exporting elapsed time [us] and encoder position [counts].")

            #note that copilot was used to find the formatting needed for the hasattr structure along with the file.write line
            #This external support was helpful in better organizing the data into a csv instead of manually collecting it
            elif self.state == self.S4_EXPORT_DATA:
                # Export one short line per scheduled turn instead of a whole file.
                if not hasattr(self, "export_index"):
                    self.export_index = -2
                if self.export_index == -2:
                    print("Begin data: motor={},duty_cycle_pct={}".format(
                        self.side, self.effort[self.trial_run]))
                elif self.export_index == -1:
                    print("time_us,position_counts")
                elif self.export_index < len(self.sample_time):
                    print("{},{}".format(self.sample_time[self.export_index],
                                        self.sample_positions[self.export_index]))
                else:
                    print("End of data")
                    del self.export_index
                    self.stop()
                if hasattr(self, "export_index"):
                    self.export_index += 1
            else:
                raise ValueError("Invalid state")

            #stop here and wait until main calls next()
            yield self.state

    def stop(self):
        """Leave the selected motor disabled, including after interruption."""
        self.motor.set_effort(0)
        self.motor.disable()
        if hasattr(self, "export_index"):
            del self.export_index
        # Return to idle after normal completion or an interrupted experiment.
        self.state = self.S_IDLE
        self.done = True

