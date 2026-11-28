
"""Schedule user interaction and both motor experiments."""
import os
import gc
from taskmotor import TaskMotor
from taskmaneuver import TaskManeuver
from taskuser import TaskUser
from taskcourse import TaskCourse
from tasksensor import TaskSensor
from taskimu import TaskImu
from tshare import Share
from time import ticks_ms, ticks_diff, sleep_ms
from array import array
from driver import Motor
from encoder import Encoder
from PIDcontroler import PID
from discrete_pi import DiscretePI
from line_calibration import load_calibration, save_calibration
from pyb import Timer, Pin, USB_VCP, I2C
from IMU import BNO055, IMU_I2C_BUS
from IMU_calibration import load_imu_calibration, save_imu_calibration
# type: ignore
import cotask


#creates the driver and task objects, then repeatedly schedules their generators
from line_sensor import(
    LineSensor,
    SENSOR_PINS,
    SENSOR_CHANNELS,
    SENSOR_POSITIONS_MM,
)


#all threeare defined in line_sensor.py
#config constants
BATTERY_VOLTAGE = 8
COUNTS_PER_WHEEL_REVOLUTION = 12 * 3952 / 33
LEFT_DRIVER_SIGN = -1
LEFT_ENCODER_SIGN = -1
RIGHT_DRIVER_SIGN = -1
RIGHT_ENCODER_SIGN = -1

# Initial P gain in V/(rad/s) -tunned in testing
INITIAL_KP = 0.6
INITIAL_KI = 4.0

# Keep timing in firmware configuration
MANEUV_PERIOD_MS = 30

#settings for observer and discrete PI implimentation
MOTOR_PERIOD_MS = 20
VELOCITY_FEEDBACK = "encoder" #how do we check speed, "encoder" or "observer"
CONTROLLER_MODE = "original" #how do we control voltage to motors, "original" or "discrete_pi"
PI_KP = 0.6
PI_KI_SAMPLE = 4.0 * MOTOR_PERIOD_MS / 1000
WHEEL_RADIUS_CM = 3.5
UI_PERIOD_MS = 0
SENSOR_PERIOD_MS = 50


#PID parameters
KP: float = 0.001
KI = None
KD: float = 0.0
# V / (rad/s)
KM: float = 0.7


#lane changing
#where 1 is lef tand -1 is right
LANE_DIRRECTION = 1


#dictionary for necessary IMU configuration
IMU_CONFIG = {
    "sensor": "BNO055",
    "sensor_id": "romi_imu_1",
    "mounting": "original_mount_v1",
    "mode": 0x08,
    "unit_sel": 0x00,
    "axis_map_config": 0x24,
    "axis_map_sign": 0x00,
}


def make_data():

    #may but 
    #buffer: memory that temporalily hold data
    #size buffer for the samples in the 2 second test. 
    capacity = 2000 // MOTOR_PERIOD_MS + 2
    return Share({
        "time_us": array("L", [0] * capacity),
        "setpoint_rad_s": array("f", [0.0] * capacity),
        "velocity_rad_s": array("f", [0.0] * capacity),
        "voltage_V": array("f", [0.0] * capacity),
        "count": 0,
        "status": "completed",
        "estimated_velocity_rad_s": array("f", [0.0] * capacity), #This way we also keep origonal velocity signal and can compare with estimate
    }, dict)

#microseconds elapsed
#Target wheel rad/s
#Current wheel rad/s
#sample count for indexing arrays
#failed to connect if over capacity.
def setup_line_calibration(sensor, force=False):
    filename = "line_calibration.json"

    if filename in os.listdir() and not force:
        calibration = load_calibration(
            filename, sensor.channels, sensor.min_span
        )

    else:
        input("Place all 13 sensors above the black line->press Enter.")
        black = list(sensor.read_averages())

        input("Place all 13 sensors above the white line->press Enter.")
        white = list(sensor.read_averages())

        calibration = {
            "channels": list(sensor.channels),
            "black": black,
            "white": white,
        }

    sensor.set_calibration(calibration)
    print("Ready for line calibration")
    save_calibration(filename, calibration, sensor.channels, sensor.min_span
        )

    

#fix calibartion or make new calibration json
def setup_imu_calibration(imu: BNO055, force=False):
    filename = "imu_calibration.json"


    # Try to restore saved validated profile
    if not force:
        try:
            calibration = load_imu_calibration(filename, IMU_CONFIG)
        except (OSError, ValueError, KeyError, TypeError) as error:
            print("Need a new IMU calibration", error)
        else:
            imu.write_calibration_profile(bytes(calibration["profile"]))
            print("Restored the saved IMU calibration ")
            return


    # Need to dissable motors before calling this func.
    print("For gyroscope calibration dont move the IMU")
    print("Now hold IMU in six different ways (keep still in each orientation).")

    start = ticks_ms()

    while True:
        system, gyro, accel, mag = imu.calib_status()
        print("Gyro:", gyro, "Accelerometer:", accel)

        if gyro == 3 and accel == 3:
            #can break bc 3 when fully calibrated
            break

        if ticks_diff(ticks_ms(), start) >= 180_000:
            raise RuntimeError("Calibration took too long")

        sleep_ms(500)


    # Read the learned settings & save as JSON
    calibration = {
        "version": 1,
        "config": dict(IMU_CONFIG),
        "profile": list(imu.read_calibration_profile()),
    }

    save_imu_calibration(filename, calibration, IMU_CONFIG)
    print("saved IMU calibration")

def main():
    # Call dibs on a large enough section of RAM to gurrentee room for profiler
    # Ensure there is enough memory for profiler output
    gc.collect()
    profiler_reserve = bytearray(2048)


    #Create objects to be passed to tasks
    #Create left motor
    left_pwm = Timer(4, freq=20_000).channel(
        1,
        mode=Timer.PWM,
        pin=Pin.cpu.B6,
        pulse_width_percent=0
    )
    #gen PWM
    #left pwm pin
    #initially have 0% duty cycle
    left_mot = Motor(
        left_pwm,
        Pin.cpu.B5,
        Pin.cpu.A10
    )

    #Create right motor
    right_pwm = Timer(1, freq=20_000).channel(
        1,
        mode=Timer.PWM,
        pin=Pin.cpu.A8,
        pulse_width_percent=0
    )
    #gen PWM
    #right pwm pin
    #initially have 0% duty cycle
    right_mot = Motor(
        right_pwm,
        Pin.cpu.A9,
        Pin.cpu.B4
    )


    #Create left encoder
    left_encoder_timer = Timer(2, prescaler=0, period=65535)
    left_encoder = Encoder(
        left_encoder_timer,
        Pin.cpu.A15,
        Pin.cpu.B3,
    )   


    # switched to PA15 (CN7)- frees PA0 for sensor 1
    # switched to PB3 (CN10)- frees PA1 for sensor 2
    #Create right encoder
    right_encoder_timer = Timer(3, prescaler=0, period=65535)
    right_encoder = Encoder(
        right_encoder_timer,
        Pin.cpu.C6,
        Pin.cpu.C7,
    )   


    # PA6(CN10), PA7(CN10)
    #Configure PID control left and right based on mode
    left_pid = PID(INITIAL_KP,INITIAL_KI,KD,KM)
    right_pid = PID(INITIAL_KP,INITIAL_KI,KD,KM)
    if CONTROLLER_MODE == "discrete_pi":
        if VELOCITY_FEEDBACK != "observer":
            raise ValueError("Discrete PI requires observer feedback")
        left_pid = DiscretePI(PI_KP, PI_KI_SAMPLE)
        right_pid = DiscretePI(PI_KP, PI_KI_SAMPLE)
    elif CONTROLLER_MODE != "original":
        raise ValueError("Unknown controller mode")


    #instantiate timer specifically for ADC channels
    sensor_timer = Timer(6)

    #create the IMU object
    #instantiate the line sensor object
    line_sensor = LineSensor(
        pins=SENSOR_PINS,
        channels=SENSOR_CHANNELS,
        positions_mm=SENSOR_POSITIONS_MM,
        timer=sensor_timer,
        sample_count=10,
        sample_rate_hz=5000,
        min_span=10,
    )

    

    #use adc channel timer
    #dissable motors before calling the calibration functions
    left_mot.disable()
    right_mot.disable()


    #call calibration funcitons
    #init IMU object and store it in varriable imu
    imu = BNO055(bus=IMU_I2C_BUS)
    setup_line_calibration(line_sensor)
    setup_imu_calibration(imu)


    #Create serial object
    serial = USB_VCP()

    #Create Shares for inter-task communication Share(value, type)
    left_test_flag = Share(False, bool)
    right_test_flag = Share(False, bool)
    line_follow_flag = Share(False, bool)
    imu_flag = Share(False, bool)
    #preallocate a buffers for each wheel seperately.
    left_data = make_data()
    right_data = make_data()
    # Velocity setpoint (rad/s)
    next_effort = Share(0.0, float)
    next_kp = Share(PI_KP if CONTROLLER_MODE == "discrete_pi" else INITIAL_KP, float)
    # Selected gain from user interface
    # Flag to start or stop collecting data
    sensor_flag = Share(False, bool)
    # empty dictionary {} of type dict-last captured sensor data i.e sensor_data["line"]
    sensor_data = Share({}, dict)
    maneuver_request = Share({}, dict)
    maneuver_status = Share({}, dict)
    #start or stop lane change
    lane_change_flag = Share (False, bool)
    lane_chane_dir = Share("left", str)
    following_target = Share(False, bool)



    #new sensor dictionary values for line stop
    sensor_data.value["line"] = None
    sensor_data.value["all_black"] = False
    sensor_data.value["imu"] = None
    sensor_data.value["white_background"] = False
    sensor_data.value["line_detected"] = False
    sensor_data.value["sensor_time_us"] = None

    #TaskCousre sensds a command and TaskManeuver send progress back
    maneuver_request.value["request_id"] = -1
    maneuver_request.value["mode"] = None
    maneuver_request.value["distance"] = None
    maneuver_request.value["angle"] = None
    maneuver_request.value["speed"] = None

    maneuver_status.value["ID"] = -1
    maneuver_status.value["status"] = None #running complete or failed


    left_mode = Share("idle", str)
    right_mode = Share("idle", str)
    left_target = Share(0.0, float)
    right_target = Share(0.0, float)
    left_velocity = Share(0.0, float)
    right_velocity = Share(0.0, float)
    left_position_cm = Share(0.0, float)
    right_position_cm = Share(0.0, float)
    test_flags = (left_test_flag, right_test_flag)

    #shares for taskcourse
    b1_press = Share(False, bool)
    bump_status = Share(False, bool)
    maneuver_request = Share("line follow", str)
    maneuver_status = Share(False, bool)
    course_status = Share("State", str) #for debugging just tells ui the state
    stop_latch = Share(False, bool)

    #Create tasks
    left_motor = TaskMotor(
        velocity_feedback=VELOCITY_FEEDBACK,
        motor=left_mot,
        encoder=left_encoder,
        pid=left_pid,
        mode=left_mode,
        target=left_target,
        actual_velocity=left_velocity,
        position_cm=left_position_cm,
        test_flag=left_test_flag,
        next_kp=next_kp,
        data=left_data,
        name="left",
        battery_voltage=BATTERY_VOLTAGE,
        counts_per_revolution=COUNTS_PER_WHEEL_REVOLUTION,
        wheel_radius_cm=WHEEL_RADIUS_CM,
        driver_sign=LEFT_DRIVER_SIGN,
        encoder_sign=LEFT_ENCODER_SIGN,
        period_ms=MOTOR_PERIOD_MS,
        test_target=next_effort,
        test_flags=test_flags,
    )

    right_motor = TaskMotor(
        velocity_feedback=VELOCITY_FEEDBACK,
        motor=right_mot,
        encoder=right_encoder,
        pid=right_pid,
        mode=right_mode,
        target=right_target,
        actual_velocity=right_velocity,
        position_cm=right_position_cm,
        test_flag=right_test_flag,
        next_kp=next_kp,
        data=right_data,
        name="right",
        battery_voltage=BATTERY_VOLTAGE,
        counts_per_revolution=COUNTS_PER_WHEEL_REVOLUTION,
        wheel_radius_cm=WHEEL_RADIUS_CM,
        driver_sign=RIGHT_DRIVER_SIGN,
        encoder_sign=RIGHT_ENCODER_SIGN,
        period_ms=MOTOR_PERIOD_MS,
        test_target=next_effort,
        test_flags=test_flags,
    )

    #to coordinate the experiments
    motor_test = {
        "motors": (left_motor, right_motor),
        "active": False,
        "selected": None,
        "start": None,
        "ready": [None, None], #time each wheel met near rest condition
    }
    left_motor.experiment = motor_test
    right_motor.experiment = motor_test

    maneuver = TaskManeuver(
        left_mode=left_mode, 
        right_mode=right_mode,
        left_target=left_target, 
        right_target=right_target,
        left_velocity=left_velocity, 
        right_velocity=right_velocity,
        left_position_cm=left_position_cm, 
        right_position_cm=right_position_cm,
        next_effort=next_effort, 
        sensor_data=sensor_data,
        line_follow_flag=line_follow_flag, 
        lane_change_flag=lane_change_flag,
        lane_change_dir=lane_chane_dir, 
        following_target=following_target,
        maneuver_request = maneuver_request,
        maneuver_status = maneuver_status,
        test_flags=test_flags, 
        period_ms=MANEUV_PERIOD_MS,
    )

    task_course = TaskCourse(
        start_event = b1_press,
        bump_status = bump_status,
        sensor_data = sensor_data,
        left_position_cm = left_position_cm,
        right_postion_cm = right_position_cm,
        maneuver_request = maneuver_request,
        maneuver_status = maneuver_status,
        course_status = course_status,
        stop_latch = stop_latch

    )

    task_user = TaskUser(
        motor_test=motor_test,
        left_test_flag=left_test_flag,
        right_test_flag=right_test_flag,
        left_data=left_data,
        line_follow_flag=line_follow_flag,
        sensor_flag=sensor_flag,
        imu_flag=imu_flag,
        lane_change_flag=lane_change_flag,
        lane_change_dir=lane_chane_dir,
        right_data=right_data,
        next_effort=next_effort,
        next_kp=next_kp,
        serial=serial,

        motor_period_ms=MOTOR_PERIOD_MS,
        ui_period_ms=UI_PERIOD_MS,
        battery_voltage=BATTERY_VOLTAGE,
        following_target=following_target,
    )

    # Starts/checks the left motor test
    # Starts/checks the right motor test
    # Reads left test samples for serial output
    # Flag for line following behavior
    # Enables and dissables the line sensor
    # Enables and dissables the IMU
    # Flag for lane change behavior
    #direction of lane change
    # Reads right test samples for serial output
    # Reads/updates velocity setpoint
    # UI will update through gain entry
    # USB serial interface for commands and CSV output
    #* Pass firmware configuration for the descriptive test header.
    task_sensor = TaskSensor(
        line_sensor=line_sensor,
        sensor_flag=sensor_flag,
        sensor_data=sensor_data
    )

    task_imu = TaskImu(
        imu=imu,
        imu_flag=imu_flag,
        sensor_data=sensor_data
    )

    '''while this is set up like a priority scheduler the priority of each is 1 so it behaves like a round robin scheduler'''

    #create the generator and add it to the scheduler
    cotask.task_list.append(
        cotask.Task(
            maneuver.run(),
            name = "Maneuver",
            priority = 1,
            profile = True,
            period = MANEUV_PERIOD_MS
        )
    )

    for motor_task, task_name in ((left_motor, "Left Motor"), (right_motor, "Right Motor")):
        cotask.task_list.append(
            cotask.Task(motor_task.run(), name=task_name, priority=1,
                        profile=True, period=MOTOR_PERIOD_MS)
        )


    #create user input generator object and add it to the scheduler
    cotask.task_list.append(
        cotask.Task(
            task_user.run(),
            name = "User Input",
            priority = 0,
            profile = True,
            period = UI_PERIOD_MS
        )
    )


    #Period of Zero keeps current idle-task behavior
    #create sensor task generator object and add it to the scheduler
    cotask.task_list.append(
        cotask.Task(
            task_sensor.run(),
            name = "Sensor Reader",
            priority = 0,
            profile = True,
            period = SENSOR_PERIOD_MS
        )
    )    


    #Period of Zero keeps current idle-task behavior
    #create IMU task generator object and add it to the scheduler
    cotask.task_list.append(
        cotask.Task(
            task_imu.run(),
            name = "IMU Reader",
            priority = 0,
            profile = True,
            period = SENSOR_PERIOD_MS
        )
    )


    #Period of Zero keeps current idle-task behavior
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
        profiler_reserve = None
        gc.collect()
        print(cotask.task_list.profile())
    
    


if __name__ == '__main__':
    main()
