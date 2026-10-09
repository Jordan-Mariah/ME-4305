
"""Schedule user interaction and both motor experiments."""
from taskmotor import TaskMotor
from taskuser import TaskUser
from pyb import Pin, Timer
from driver import Motor
from encoder import Encoder
import cotask #allows the motor to share processro time amoung task
#creates the driver and task objects, then repeatedly schedules their generators


def main():

    '''Bingus schimgus welp I think this is the modularization you were talking about because it 
    seems my initializations were fricked up.
    While this is set up like a priority scheduler the priority
    of each is 1 so it behaves like a round robin scheduler'''
    
    #set the left and right motor timers and channels here in main
    left_pwm_timer = Timer(4 , freq=20_000)
    #this calls the timer object's channel method to configure channel 1.
    #The method returns a timer channel object that I store in pwm_right
    left_pwm = left_pwm_timer.channel(
        1, #channel 1
        mode=Timer.PWM, #gen PWM
        pin=Pin.cpu.B6,
        pulse_width_percent=0 #initially have 0% duty cycle
    )
    left_driver = Motor(left_pwm, Pin.cpu.B5, Pin.cpu.A10)
    left_encoder_timer = Timer(2 , prescaler=0, period=65535)
    left_encoder = Encoder(left_encoder_timer, Pin.cpu.A0, Pin.cpu.A1)
    

    right_pwm_timer = Timer(1 , freq=20_000)
    right_pwm = right_pwm_timer.channel(
            1, #channel 1
            mode=Timer.PWM, #gen PWM
            pin =Pin.cpu.A8,
            pulse_width_percent=0 #initially have 0% duty cycle
        )
    right_driver = Motor(right_pwm, Pin.cpu.A9, Pin.cpu.B4)
    right_encoder_timer = Timer(3, prescaler=0, period=65535)
    right_encoder = Encoder(right_encoder_timer, Pin.cpu.A6, Pin.cpu.A7)

    left_motor = TaskMotor("left", left_driver, left_encoder)
    right_motor = TaskMotor("right", right_driver, right_encoder)
    task_user = TaskUser(left_motor, right_motor)


    #create the generator and add it to the scheduler
    cotask.task_list.append(
        cotask.Task(
            left_motor.run(),
            name = "Left Task",
            priority = 1,
            profile = True,
            period = 10
        )
    )

    #create the generator and add it to the scheduler
    cotask.task_list.append(
        cotask.Task(
            right_motor.run(),
            name = "Right Task",
            priority = 1,
            profile = True,
            period = 10
        )
    )

    #create user input generator object and add it to the scheduler
    cotask.task_list.append(
        cotask.Task(
            task_user.user_interaction(),
            name = "User Input",
            priority = 1,
            profile = True,
            period = 100
        )
    )

    #Advance the generators until the user requests exit.
    try:
        while not task_user.exit_initiated:

            #pri_sched is a method from the cotask lib. It selects a task adn calls next on that tasks generator
            cotask.task_list.pri_sched()

    except KeyboardInterrupt:
        print("Program terminated")

    finally:
        left_motor.stop()
        right_motor.stop()
        print(cotask.task_list.profile())
    
    


if __name__ == '__main__':
    main()
