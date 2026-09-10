from time import ticks_us, ticks_diff   # Use to get dt value in update()
from pyb import Pin, Timer, timerchannel

class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class'''

    def __init__(self, tim: Timer, chA_pin: Pin, chB_pin: Pin):
        '''Initializes an Encoder object'''

        self.AR = 65535
    
        self.position   = 0     # Total accumulated position of the encoder
        self.prev_count = 0     # Counter value from the most recent update
        self.delta      = 0     # Change in count between last two updates
        self.dt         = 0     # Amount of time between last two updates
        self.last_t     = ticks_us() # Get current time for next compare
        self.timer      = tim   # Make timer accessable to methods

        '''Configure timer channels so that Timer.count() indicates direction'''
        tim.channel(1, pin=chA_pin, mode=Timer.ENC_A)
        tim.channel(2, pin=chB_pin, mode=Timer.ENC_B)

    
    def update(self):
        '''Runs one update step on the encoder's timer counter to keep
           track of the change in count and check for counter reload'''

        # Mesure time step
        this_t = ticks_us()
        self.dt = ticks_diff(this_t, self.last_t)
        self.last_t = this_t

        # Mesure counter step
        this_count = self.timer.counter()
        self.delta = this_count - self.prev_count
        self.prev_count = this_count

        # Check if autoreload (I still dont know what this means)
        if self.delta < -(self.AR + 1)/2:
            self.delta += self.AR + 1
        elif self.delta > (self.AR + 1)/2:
            self.delta -= self.AR + 1

        self.position += self.delta

        # Todo: Check if dt is too big for the AR checking algorithm to work
            
    def get_position(self):
        '''Returns the most recently updated value of position as determined
           within the update() method. Returns as [encoder ticks]'''
        return self.position
            
    def get_velocity(self):
        '''Returns a measure of velocity using the the most recently updated
           value of delta as determined within the update() method. Returns
           as [encoder ticks/microsecond]'''
        return self.delta/self.dt
    
    def zero(self):
        '''Sets the present encoder position to zero and causes future updates
           to measure with respect to the new zero position'''
        self.position = 0
