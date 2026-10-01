
"""Schedule user interaction and both motor experiments."""
from taskmotor import TaskMotor
from taskuser import TaskUser
from tshare import Share
from driver import Motor
from encoder import Encoder
from pyb import Timer, Pin, USB_VCP
import cotask # type: ignore
#creates the driver and task objects, then repeatedly schedules their generators


def main():
    #Create objects to be passed to tasks

    #Create left motor

    left_pwm = Timer(4, freq=20_000).channel(
        1,
        mode=Timer.PWM, #gen PWM
        pin=Pin.cpu.B6, #left pwm pin
        pulse_width_percent=0 #initially have 0% duty cycle
    )
    left_motor = Motor(
        left_pwm,
        Pin.cpu.B5,
        Pin.cpu.A10
    )

    #Create right motor

    right_pwm = Timer(1, freq=20_000).channel(
        1,
        mode=Timer.PWM, #gen PWM
        pin=Pin.cpu.A8, #right pwm pin
        pulse_width_percent=0 #initially have 0% duty cycle
    )
    right_motor = Motor(
        right_pwm,
        Pin.cpu.A9,
        Pin.cpu.B4
    )

    #Create left encoder

    left_encoder_timer = Timer(2, prescaler=0, period=65535)
    left_encoder = Encoder(
        left_encoder_timer,
        Pin.cpu.A0,
        Pin.cpu.A1,
    )   

    #Create right encoder

    right_encoder_timer = Timer(3, prescaler=0, period=65535)
    right_encoder = Encoder(
        right_encoder_timer,
        Pin.cpu.A6,
        Pin.cpu.A7,
    )   

    #Create serial object

    serial = USB_VCP()

    #Create Shares for inter-task communication

    left_test_flag = Share(False, bool)
    right_test_flag = Share(False, bool)
    next_effort = Share(10, int)

    #Create tasks
    left_motor = TaskMotor(left_motor, left_encoder, left_test_flag, "left")
    right_motor = TaskMotor(right_motor, right_encoder, right_test_flag, "right")
    task_user = TaskUser(left_test_flag, right_test_flag, next_effort, serial)

    '''while this is set up like a priority scheduler the priority of each is 1 so it behaves like a round robin scheduler'''
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
            task_user.run(),
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
