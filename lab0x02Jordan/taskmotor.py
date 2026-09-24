"""Nonblocking, single-motor experiment task; all trial logic lives here."""
from array import array
from time import ticks_us, ticks_diff
from pyb import Pin, Timer
from driver import Motor
from encoder import Encoder


class Task1:

    #class attributes listed in uppercase to represent them as constants
    S0_PREPARE = 0
    S1_SETTLE = 1
    S2_STEP_SAMPLE = 2
    S3_STOP_MOTOR = 3
    S4_EXPORT_DATA = 4 

    #define a method with a parameter called self
    #the body will not run until python calls this methods
    def __init__(self):
        #Configure the hardware, create drivers, and store the starting state
        # Configure the PWM timers once.

        #initialize the right PWM timer as an object from the py built in Timer class 
        self.pwm_timer_right = Timer(1, freq=20_000)

        '''this calls the timer object's channel method to configure channel 1.
        The method returns a timer channel object that I store in pwm_right'''
        pwm_right = self.pwm_timer_right.channel(
            1, #channel 1
            mode=Timer.PWM, #gen PWM
            pin=Pin.cpu.A8, #output to hardware on PA8
            pulse_width_percent=0 #initially have 0% duty cycle
        )

        #call the motor class from motor.py and pass in it's initializations
        self.right_motor = Motor(
            pwm_right, Pin.cpu.A9, Pin.cpu.B4
        )

        #Initialize the right encoder's timer  as an object in the Task1 class from the py Timer class
        self.encoder_timer_right = Timer(3, prescaler=0, period=65535)

        '''Initialize the right encoder as an object 
        Store a reference to this object in the right_encoder attribute where an attribute is a named value'''
        self.right_encoder = Encoder(
            self.encoder_timer_right,
            Pin.cpu.A6,
            Pin.cpu.A7,
        )

        #specify that the self object holds the varriable
        self.state = self.S0_PREPARE
        self.effort = [10, 20, 30, 40, 50, 60, 70, 80]
        self.trial_run = 0
        self.done = False
        
    #tasks should be non-blocking meaning there should not be while loops or log delays in run
    def run(self):
        if self.done:
            return
        #inidcates the starting state of the FSM
        elif (self.state == self.S0_PREPARE):
            #access the encoder object stored in right_encoder and call its zero method to reset the position
            self.right_encoder.zero()

            #store current clock reading then S1 can later know how mcuh time has elapsed
            self.settle_start = ticks_us()

            #initialize current state to next step in FSM
            print("state0")
            self.state = self.S1_SETTLE         

        #settle the motor
        elif (self.state == self.S1_SETTLE): 
            self.right_motor.disable()
            self.right_encoder.update() #refresh encoders measurments but dont repetedly zero
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
                    self.right_motor.enable()
                    self.right_motor.set_effort(self.effort[self.trial_run])


                    self.sample_start = ticks_us()
                    self.sample_interval = 10_000 #10 ms sampling interval
                    self.sample_duration = 2_000_000 #2 seconds
                    self.sample_next = self.sample_start #first sample imidiately scheduled
                    self.sample_time = array("L") #specify with type code that sample_time stores a time value unsigned long int
                    self.sample_positions = array("i") #specify with type code that sample_position stores signed encoder 
                    #...position as a signed int

                #update encoder for each itteration of run
                self.right_encoder.update()

                current_time = ticks_us()
                time_elapse = ticks_diff(current_time, self.sample_start)

                #if current time is greater than or equal to the next sample time
                if ticks_diff(current_time, self.sample_next) >= 0:
                    self.sample_time.append(time_elapse) #when the sample occurs
                    self.sample_positions.append(self.right_encoder.get_position()) #encoder position when sampled
                    self.sample_next += self.sample_interval #schedule the next sample #scedule next sample as current +
                    #sample interval

                if time_elapse >= self.sample_duration:
                    self.state = self.S3_STOP_MOTOR

        elif (self.state == self.S3_STOP_MOTOR):
            self.right_motor.set_effort(0)
            self.right_motor.disable()

            self.state = self.S4_EXPORT_DATA
            print("state3")

        #note that copilot was used to find the formatting needed for the hasattr structure along with the file.write line
        #This external support was helpful in better organizing the data into a csv instead of manually collecting it
        elif self.state == self.S4_EXPORT_DATA:
            effort = self.effort[self.trial_run]
            filename = "step_{}pct.csv".format(effort)
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
                print("All trials complete")

    def stop(self):
        """Leave the selected motor disabled, including after interruption."""
        self.right_motor.set_effort(0)
        self.right_motor.disable()


