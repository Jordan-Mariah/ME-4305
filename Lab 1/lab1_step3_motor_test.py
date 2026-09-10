import pyb
print('Test both motors')

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

#-----5 TIMER OBJECTS TO COUNT FROM ENCODERS-----

# Encoder inputs for PA0/PA1 (left), and PA6/PA7 (right) only
#set the encoder count from 0-65535 with a prescaler of 0 so there is no clock division
tim_2 = pyb.Timer(2, period=0xFFFF, prescaler=0)
tim_2.channel(1, pin=pyb.Pin.cpu.A0, mode=pyb.Timer.ENC_AB)
tim_2.channel(2, pin=pyb.Pin.cpu.A1, mode=pyb.Timer.ENC_AB)

tim_3 = pyb.Timer(3, period=0xFFFF, prescaler=0)
tim_3.channel(1, pin=pyb.Pin.cpu.A6, mode=pyb.Timer.ENC_AB)
tim_3.channel(2, pin=pyb.Pin.cpu.A7, mode=pyb.Timer.ENC_AB)
#reset each starting count to 0
tim_2.counter(0)
tim_3.counter(0)

# Run both motors at 25% duty for 2 seconds, then reverse.
enable_left.high()
enable_right.high()
pwm_left.pulse_width_percent(25)
pwm_right.pulse_width_percent(25)
print('Both motors commanded to 25% for 2 seconds')

# Read encoders each 100 milliseconds and count for a total of 5 seconds.
for i in range(50):
    pyb.delay(100)
    print('Left:', tim_2.counter(), 'Right:', tim_3.counter())

#-----REVERSE THE MOTOR DRIVERS-----

#set the initial dirrectional output to high (backwards) for right and left motors 
dir_left = pyb.Pin(pyb.Pin.cpu.B5, mode=pyb.Pin.OUT_PP, value=1)
dir_right = pyb.Pin(pyb.Pin.cpu.A9, mode=pyb.Pin.OUT_PP, value=1)

# Run both motors at 25% duty for 2 seconds, then stop.
enable_left.high()
enable_right.high()
pwm_left.pulse_width_percent(25)
pwm_right.pulse_width_percent(25)
print('Both motors commanded to 25% for 2 seconds')

# Read both encoders again every 100 miliseconds now going backwards for 5 seconds.
for i in range(50):
    pyb.delay(100)
    print('Left:', tim_2.counter(), 'Right:', tim_3.counter())

#set the PWM duty cycle to 0 to stop the motors
pwm_left.pulse_width_percent(0)
pwm_right.pulse_width_percent(0)
enable_left.low()
enable_right.low()
print('Motor test finished; both motors disabled')
