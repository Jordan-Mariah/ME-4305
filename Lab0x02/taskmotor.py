from encoder import Encoder
from driver import Motor
from pyb import Pin, Timer
from time import ticks_ms, ticks_diff, sleep_ms

class TaskMotor():


    def __init__(self, voltages, motor, encoder):
            self.state: int = 0
            self.motor: Motor = motor
            self.encoder: Encoder = encoder
            self.test_idx: int = 0
            self.voltages: tuple  = voltages
            self.last_vel: float = 0
            self.last_time: float = 0
            self.time: float = 0


    def run(self): 
        state = self.state

        if state == 0: #Initialise motor and encoder

            # Zombie state, ignore.
            
            state = 1
            return
        
        elif state == 1: #Stop motor, then start next test
            self.motor.disable

            # Check if motor is stopped
            if abs(self.encoder.get_velocity()) <= 10: #If stopped wait a moment then change state
                sleep_ms(100)
                state = 2
            return
        
        elif state == 2: #Get next test and start motor
            self.motor.enable
            self.motor.set_effort(self.voltages[self.test_idx])

            self.last_vel = 0
            self.last_time = ticks_ms()
            self.time = 0

            print("---Begin "+str(self.voltages[self.test_idx])+"V test---")
            state = 3
            return

        elif state == 3: #Run motor until SS
            now = ticks_ms()
            dt = ticks_diff(now, self.last_time)
            self.time += dt
            print("Time: "+str(self.time)+" | Velocity: "+str(self.encoder.get_velocity())) 

            if abs(self.encoder.get_velocity() - self.last_vel) < 10 and self.time > 100:
                print("---End "+str(self.voltages[self.test_idx])+"V test---")

                self.test_idx += 1
                if len(self.voltages) - 1 < self.test_idx:
                    state = 4
                    return

                state = 1
            return

        elif state == 4: #Test is over, do nothing.
            pass

        self.state = state

