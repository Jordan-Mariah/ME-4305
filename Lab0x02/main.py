import taskmotor
from pyb import Pin, Timer
from driver import Motor
from encoder import Encoder

efforts = (10, 20, 30, 40, 50)

# clear of the bench before running this automatic motion sequence.
pwm_timer_right = Timer(1, freq=20_000)
#connect the PWM of the right motor to chanell 1 of timer 1. Then mode to PWM, pin to A8, and initial pulse width to 0
pwm_right = pwm_timer_right.channel(1, mode=Timer.PWM, pin=Pin.cpu.A8, pulse_width_percent=0)
#create a motor object and assign it to the right motor with pins for PWM, DIR, and nSLP
motor = Motor(pwm_right, Pin.cpu.A9, Pin.cpu.B4)

right_timer = Timer(3, prescaler=0, period=65535)
encoder = Encoder(right_timer, Pin.cpu.A6, Pin.cpu.A7)

task_1 = taskmotor.TaskMotor(efforts, motor, encoder)

def main():
    try:
        while True:
            task_1.run()
    except KeyboardInterrupt:
        motor.disable()

if __name__ == "__main__":
    main()
