from time import ticks_us, ticks_diff   # Use to get dt value in update()

class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class'''

    def __init__(self, tim, chA_pin, chB_pin):
        '''Initializes an Encoder object'''

        self.position   = 0     # Total accumulated position of the encoder
        self.prev_count = 0     # Counter value from the most recent update
        self.delta      = 0     # Change in count between last two updates
        self.dt         = 0     # Amount of time between last two updates
        self.velocity   = 0.0   # Most recently calculated counts per second

        # Configure both channels for quadrature encoder input.
        self.tim = tim
        tim.channel(1, pin=chA_pin, mode=tim.ENC_AB)
        tim.channel(2, pin=chB_pin, mode=tim.ENC_AB)
        self.prev_count = tim.counter()
        self.prev_time = ticks_us()

    #defined a method called update. We will use this in the main loop for updating the encoder's position and velocity
    def update(self):
        '''Runs one update step on the encoder's timer counter to keep
           track of the change in count and check for counter reload'''
        #-----measure the time elapsed------

        #read current clock value in us
        current_time = ticks_us()
        #calculate diff. between time now and prev. time
        self.dt = ticks_diff(current_time, self.prev_time)
        #save this reading as the previous time to be used in the next update
        self.prev_time = current_time

        #measure the change in the encoder count

        # read the encoder's timers current count and stroes it in current_count
        current_count = self.tim.counter()
        #num of counts since last reading.
        self.delta = current_count - self.prev_count
        #save this reading as the previous count to be used in the next update
        self.prev_count = current_count
        # Update before the encoder moves half the timer's count range.
        count_range = self.tim.period() + 1
        if self.delta < -count_range / 2:
            self.delta += count_range
        elif self.delta > count_range / 2:
            self.delta -= count_range
        self.position += self.delta
        # Convert elapsed microseconds to seconds and cache the velocity.
        if self.dt > 0:
            self.velocity = self.delta * 1_000_000 / self.dt
        else:
            self.velocity = 0.0

    def get_position(self):
        '''Returns the most recently updated value of position as determined
           within the update() method'''

        # Position is accumulated in encoder counts by update().

        return self.position

    def get_velocity(self):
        '''Returns signed velocity in encoder counts per second.'''
        return self.velocity


    def zero(self):
        '''Sets the present encoder position to zero and causes future updates
           to measure with respect to the new zero position'''
        # Sample the current count so movement before zero() is discarded.
        # The hardware counter itself does not need to be reset.
        self.prev_count = self.tim.counter()
        self.prev_time = ticks_us()
        self.position = 0
        self.delta = 0
        self.dt = 0
        self.velocity = 0.0
