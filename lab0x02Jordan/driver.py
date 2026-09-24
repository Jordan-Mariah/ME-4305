import pyb
from pyb import Pin

# the motor class groups the related motor driver pins and funcs into a single driver class. Two distinct wheel objects can be created from this
class Motor:
    '''A motor driver interface encapsulated in a Python class. Works with
       motor drivers using separate PWM and direction inputs such as the DRV8838
       drivers present on the Romi chassis from Pololu.'''

    #methods are functions within the motor class. These can be called on the motor object
    #self is the motor object
    def __init__(self, PWM, DIR, nSLP):
        '''Initializes a Motor object'''
        #nsply identifies the hardware pin
        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)
        #dir identifies the given hardware pin. here we configure it as an ouput and store it. 0 sets init dir to forward
        self.DIR_pin = Pin(DIR, mode=Pin.OUT_PP, value = 0)
        self.PWM = PWM
        self.PWM.pulse_width_percent(0)  # initialize the PWM duty cycle at 0
    
    def set_effort(self, effort):
        '''Sets the present effort requested from the motor based on an input value
           between -100 and 100'''
        
        #create effort constraints
        if effort > 100:
            effort = 100
        elif effort <-100:
            effort = -100

        #set the dirrection pin as it relates to the sign of the effort
        if effort >= 0:
            self.DIR_pin.low()
        else:
            self.DIR_pin.high()

        # the duty cycle should be the magnitude of effort (Can't have a negative duty cycle)
        self.PWM.pulse_width_percent(abs(effort))
       
            
    def enable(self):
        '''Enables the motor driver by taking it out of sleep mode into brake mode'''
        self.PWM.pulse_width_percent(0) #set the PWM pulse width to 0. This makes it so we don't start driving right as we enable it
        self.nSLP_pin.high()


    def disable(self):
        '''Disables the motor driver by taking it into sleep mode'''
        self.nSLP_pin.low()
