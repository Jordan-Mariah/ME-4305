
"""Schedule user interaction and both motor experiments."""
from taskmotor import TaskMotor
from taskuser import TaskUser
from tshare import Share
from array import array
from driver import Motor
from encoder import Encoder
from line_sensor import LineSensor
from pyb import Timer, Pin, USB_VCP
import cotask # type: ignore
#creates the driver and task objects, then repeatedly schedules their generators


#* Fill these values from measurements and sign checks before requesting a test.
BATTERY_VOLTAGE = 8
COUNTS_PER_WHEEL_REVOLUTION = 12 * 3952 / 33
LEFT_DRIVER_SIGN = 1
LEFT_ENCODER_SIGN = 1
RIGHT_DRIVER_SIGN = 1
RIGHT_ENCODER_SIGN = 1
# Initial P gain in V/(rad/s) -tunned in testing
INITIAL_KP = 0.1
# Keep timing in firmware configuration
MOTOR_PERIOD_MS = 50
UI_PERIOD_MS = 0


def make_data():#buffer: memory that temporalily hold data
    #size buffer for the samples int he 2 second test. 
    capacity = 2000 // MOTOR_PERIOD_MS + 2
    return Share({
        "time_us": array("L", [0] * capacity), #microseconds elapsed
        "setpoint": array("f", [0.0] * capacity), #desired wheel rad/s
        "velocity": array("f", [0.0] * capacity), #Measured wheel rad/s
        "voltage": array("f", [0.0] * capacity), #Limited controller voltage.
        "count": 0, #sample count for indexing arrays
        "status": "completed", #failed to connect if over capacity.
    }, dict)


def main():
    #Create objects to be passed to tasks

    #Create left motor

    left_pwm = Timer(4, freq=20_000).channel(
        1,
        mode=Timer.PWM, #gen PWM
        pin=Pin.cpu.B6, #left pwm pin
        pulse_width_percent=0 #initially have 0% duty cycle
    )
    left_mot = Motor(
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
    right_mot = Motor(
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
    left_data = make_data() #\preallocate a buffers for each wheel seperately.
    right_data = make_data()
    next_effort = Share(0.0, float) # Velocity setpoint (rad/s)
    next_kp = Share(INITIAL_KP, float) # Selected gain from user interface

    #Create tasks
    left_motor = TaskMotor(
        motor=left_mot,                 # Motor-driver object controlling the left motor
        encoder=left_encoder,           # Encoder object measuring the left wheel position
        test_flag=left_test_flag,       # Shared flag used to request/indicate a left-motor test
        next_effort=next_effort,        #Shared velocity setpoint (rad/s)
        next_kp=next_kp,                #Gain selected for next test
        data=left_data,                 #left motor shared response motors
        name="left",                    # Human-readable identifier for this motor task
        battery_voltage=BATTERY_VOLTAGE,
        counts_per_revolution=COUNTS_PER_WHEEL_REVOLUTION,
        driver_sign=LEFT_DRIVER_SIGN,
        encoder_sign=LEFT_ENCODER_SIGN,
        period_ms=MOTOR_PERIOD_MS, #* Match the motor task configuration to scheduling.
    )

    right_motor = TaskMotor(
        motor=right_mot,                # Motor-driver object controlling the right motor
        encoder=right_encoder,          # Encoder object measuring the right wheel position
        test_flag=right_test_flag,      # Shared flag used to request/indicate a right-motor test
        next_effort=next_effort,        # Shared velocity setpoint (rad/s)
        next_kp=next_kp,                # Gain selected for next test-PID instance
        data=right_data,                #* Shared response buffers-right motor
        name="right",                   # Human-readable identifier for this motor task
        battery_voltage=BATTERY_VOLTAGE,
        counts_per_revolution=COUNTS_PER_WHEEL_REVOLUTION,
        driver_sign=RIGHT_DRIVER_SIGN,
        encoder_sign=RIGHT_ENCODER_SIGN,
        period_ms=MOTOR_PERIOD_MS, #* Match the motor task configuration to scheduling.
    )

    task_user = TaskUser(
        left_test_flag=left_test_flag,  # Starts/checks the left motor test
        right_test_flag=right_test_flag,# Starts/checks the right motor test
        left_data=left_data,            # Reads left test samples for serial output
        right_data=right_data,          # Reads right test samples for serial output
        next_effort=next_effort,        # Reads/updates velocity setpoint
        next_kp=next_kp,                # UI will update through gain entry
        serial=serial,                  # USB serial interface for commands and CSV output
        #* Pass firmware configuration for the descriptive test header.
        motor_period_ms=MOTOR_PERIOD_MS,
        ui_period_ms=UI_PERIOD_MS,
        battery_voltage=BATTERY_VOLTAGE,
    )

    '''while this is set up like a priority scheduler the priority of each is 1 so it behaves like a round robin scheduler'''
    #create the generator and add it to the scheduler
    cotask.task_list.append(
        cotask.Task(
            left_motor.run(),
            name = "Left Task",
            priority = 1,
            profile = True,
            period = MOTOR_PERIOD_MS
        )
    )

    #create the generator and add it to the scheduler
    cotask.task_list.append(
        cotask.Task(
            right_motor.run(),
            name = "Right Task",
            priority = 1,
            profile = True,
            period = MOTOR_PERIOD_MS
        )
    )

    #create user input generator object and add it to the scheduler
    cotask.task_list.append(
        cotask.Task(
            task_user.run(),
            name = "User Input",
            priority = 0,
            profile = True,
            period = UI_PERIOD_MS #Period of Zero keeps current idle-task behavior
        )
    )

    #Advance the generators until the user requests exit.
    try:
        while not task_user.exit_initiated:

            #pri_sched is a method from the cotask lib. It selects a task and calls next on that tasks generator
            cotask.task_list.pri_sched()

    except KeyboardInterrupt:
        print("Program terminated")

    finally:
        left_mot.disable()
        right_mot.disable()
        print(cotask.task_list.profile())
    
    


if __name__ == '__main__':
    main()
