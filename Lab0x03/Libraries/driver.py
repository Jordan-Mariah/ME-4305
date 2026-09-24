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

if __name__ == "__main__":
    #initialize a timer channel

    #------ENABLE MOTOR DRIVERS AND SET DIRRECTIONS-------

    # Enable and direction pins (left and right).
    # enable the left motor driver by setting A10 the left slp pin high, OUT_PP is the push-pull output, then value 0 starts with dissabled driver
    enable_left = pyb.Pin(pyb.Pin.cpu.A10, mode=pyb.Pin.OUT_PP, value=0)
    #same initialization as the left motor
    enable_right = pyb.Pin(pyb.Pin.cpu.B4, mode=pyb.Pin.OUT_PP, value=0)
    #set the initial dirrectional output to low (forward) for right and left motors 
    dir_left = pyb.Pin(pyb.Pin.cpu.B5, mode=pyb.Pin.OUT_PP, value=0)
    dir_right = pyb.Pin(pyb.Pin.cpu.A9, mode=pyb.Pin.OUT_PP, value=0)

    #------SET THRE FREQUENCY OF THE TIMERS AND INITIALIZE THE PWMs------

    # The PWM pins use separate timers, both on channel 1 where each timer's is set to 20kHz
    tim_4 = pyb.Timer(4, freq=20_000)
    tim_1 = pyb.Timer(1, freq=20_000)
    #for the left PWM select channel 1, then send the signal to PB6, select PWM, then initialyze D=0
    pwm_left = tim_4.channel(1, pin=pyb.Pin.cpu.B6, mode=pyb.Timer.PWM, pulse_width_percent=0)
    # do the same for the left but with timer 1
    pwm_right = tim_1.channel(1, pin=pyb.Pin.cpu.A8, mode=pyb.Timer.PWM, pulse_width_percent=0)

    my_motor_left = Motor(pwm_left, dir_left, enable_left)
    my_motor_right = Motor(pwm_right, dir_right, enable_right)

    print('Running left motor')
    my_motor_left.enable()
    my_motor_left.set_effort(25)
    pyb.delay(2000)

    #turn off the motor
    my_motor_left.set_effort(0)
    pyb.delay(1000)

    #reverse the motor
    print('Reversing left motor')
    my_motor_left.set_effort(-25)
    pyb.delay(2000)

    #turn off the motor
    my_motor_left.disable()
    print('Finished')