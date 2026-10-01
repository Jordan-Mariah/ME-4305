
"""Schedule user interaction and both motor experiments."""
from taskmotor import TaskMotor
from taskuser import TaskUser
import cotask
#creates the driver and task objects, then repeatedly schedules their generators


def main():
    #create a new instance of left and right motor objects
    left_motor = TaskMotor("left")
    right_motor = TaskMotor("right")

    task_user = TaskUser(left_motor, right_motor)

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
