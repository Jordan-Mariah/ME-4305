"""Lab 0x02: construct one task and repeatedly give it a turn."""
from pyb import Pin, Timer, USB_VCP
from time import ticks_ms, ticks_diff, sleep_ms
from driver import Motor
from encoder import Encoder
from taskmotor import TaskMotor
import cotask # type: ignore
#creates the driver and task objects, then repeatedly calls the single task

if __name__ == '__main__':
    print("Bingus Horse: Starting main()")

    # Create inter-task signals
    right_test_flag: bool = False
    left_test_flag: bool = False

    # Create task objects
    task_motor_right = TaskMotor(right_test_flag)
    task_motor_left = TaskMotor(left_test_flag)

    # Add task generators to cotask.task_list with priority
    cotask.task_list.append(cotask.Task(
        task_motor_right.run(), 
        name="Motor task", 
        priority=1, 
        profile=True,
        period=50
    ))
    cotask.task_list.append(cotask.Task(
        task_motor_left.run(), 
        name="Motor task", 
        priority=1, 
        profile=True,
        period=50
    ))
    print("Bingus Horse: Added task_motor.run() to task_list")



    #immidiately calls init then goes into state diagrams
    try:
        print("Bingus Horse: Starting main loop")

        while True:
            cotask.task_list.pri_sched()
            #pause for 1ms between run cycles
            sleep_ms(1)
    except Exception as error:
        print(error)

    finally:
        task_motor_left.stop()
        task_motor_right.stop()