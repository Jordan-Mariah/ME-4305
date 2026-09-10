import pyb

#create pin objects for the two enable inptus and the two dirrections
LEFT_ENABLE = pyb.Pin(pyb.Pin.cpu.PA10, mode=pyb.Pin.OUT_PP)
LEFT_DIRECTION = pyb.Pin(pyb.Pin.cpu.PB5, mode=pyb.Pin.OUT_PP)
#fill in for right also may need to change the ports for both

#Create a timer object and two timer channel objects for the two PWM inputs 
tim_N = pyb.Timer(N, freq=20_000)
PWM_1 = tim_N.channel(1, pin=pyb.Pin.CPU.XY, mode=pyb.Timer.PWM)
PWM_2 = tim_N.channel(2, pin=pyb.Pin.CPU.XY, mode=pyb.Timer.PWM)