"""Cooperative velocity/gain entry and response transmission."""
# type: ignore
from pyb import USB_VCP
from tshare import Share
from math import isfinite
from discrete_pi import DiscretePI




#steps
#prompt user for input
#go into FSM depending on user string response
class TaskUser:


    #State definitions
    S0_AWAIT_COMMAND = 0
    S1_PROCESS_COMMAND = 1
    S2_AWAIT_DIGIT_INPUT = 2
    S3_PROCESS_DIGIT_INPUT = 3



    #Other constants
    VALID_DIGITS: set = set(map(str, range(10)))
    TERMINATORS: set = {"\r", "\n"}

    LINE_HEADER = "Begin motor-test data"

    LINE_COLUMNS = "time_us,setpoint_rad_s,velocity_rad_s,estimated_velocity_rad_s,limited_voltage_V"
    LINE_FOOTER = "End of motor-test data"
    LINE_READY = "Ready for next test"


    def __init__(self, left_test_flag: Share, right_test_flag: Share, left_data: Share, right_data: Share, next_effort: Share, line_follow_flag: Share, lane_change_flag: Share, lane_change_dir: Share, imu_flag: Share, following_target: Share, serial: USB_VCP, next_kp=None, motor_period_ms=100, ui_period_ms=0, battery_voltage=None, sensor_flag=None, motor_test=None):

        #initialize at the begining of the task
        # initialize all of the data as finite objects in task user
        self.left_test_flag = left_test_flag
        self.right_test_flag = right_test_flag
        self.line_follow_flag = line_follow_flag
        self.lane_change_flag = lane_change_flag
        self.lane_change_dir = lane_change_dir
        #added this sensor flag 
        self.sensor_flag = sensor_flag
        self.imu_flag = imu_flag
        self.following_target = following_target
        self.left_data = left_data
        self.right_data = right_data
        self.next_effort = next_effort
        #The proportional gain is set through p/P- shared
        self.next_kp = next_kp
        self.serial = serial
        self.motor_period_ms = motor_period_ms
        self.ui_period_ms = ui_period_ms
        self.battery_voltage = battery_voltage
        self.motor_test = motor_test
        self.exit_initiated = False


        # Task variables
        self.new_char = ""
        self.out_buff: list[str] = []
        self.current_test: str = ""
        self.entry_target = self.next_effort
        self.entry_invalid = False
        self.skip_lf = False
        self.run_setpoint = 0.0
        self.run_kp = 0.0

        #allows us to keep track of exported rows and bytes not yet accepted by USB
        self.export_index = 0
        self.pending_output = b""
        self.export_phase = "header"

        #here we store the output gain
        #if no gainw as shared use .001 for the gain
        # Make sure there is no residual data in left data buffer before starting new test
        #store Kp and velocity had to get help from copiolot on structureing flag conditionals
        # check for residual data in right data buffer before starting new test
        #* Read the preallocated response dictionary.
        #check if motor is already running, then make flag so that TaskMotor starts the right motor test
        # Enter detected: go back to await
        self.state = self.S0_AWAIT_COMMAND


    #added a way to enter velocity and proportional gain as a user
    def print_help(self):
        print("Welcome to the user interface! Print one of the following commands to get started.")
        print("+------------------------------------------------------------------------------+")
        print("| ME 4305 Romi Tuning Interface Help Menu                                      |")
        print("+-----+------------------------------------------------------------------------+")
        print("| h/H | Print help menu                                                        |")
        print("| v/V | Enter velocity setpoint rad/s                                          |")
        print("| p/P | Enter proportional gain  in V/(rad/s)                                  |")
        print("| b/B | Both-wheel forward test; record left motor                             |")
        print("| l/L | Both-wheel forward test; record left motor                             |")
        print("| r/R | Both-wheel forward test; record right motor                            |")
        print("| w/W | Start line following                                                   |")
        print("| s/S | Stop line following                                                    |")
        print("| d/D | Merge right                                                            |")
        print("| a/A | Merge left                                                             |")
        print("| e/E | Exit interface                                                         |")
        print("+-----+------------------------------------------------------------------------+")
        print("Validation: signed decimal velocity +/-1000 rad/s; Kp 0.1000; max 24 characters.")

    def test_busy(self):

            return bool((self.motor_test is not None and self.motor_test["active"])
                    or self.left_test_flag.value or self.right_test_flag.value
                    or self.left_data.value["count"] or self.right_data.value["count"] or self.current_test)

    #returns true if test running, requested, or sending data and otherwise false
    def request_motor_test(self, selected):
        if self.test_busy():
            self.serial.write(b"Motor test busy\r\n")
            # Wait for the recorder to add another sample.
            return
        speed = float(self.next_effort.value)
        kp = float(self.next_kp.value if self.next_kp is not None else 0.001)
        if self.motor_test is None:
            self.serial.write(b"Test rejected: coordinator unavailable\r\n")
            return
        try:
            accepted = self.motor_test["motors"][0].start_test(selected, speed, kp)
        except ValueError:
            self.serial.write(b"Test rejected: positive finite speed required\r\n")
            return
        if not accepted:
            self.serial.write(b"Motor test busy\r\n")
            return
        #clear request for line sensing
        self.line_follow_flag.set(False)
        self.lane_change_flag.set(False)
        #start the line following here
        #all following flags true
        self.following_target.set(False)
        self.imu_flag.set(False)
        if self.sensor_flag is not None:
            self.sensor_flag.set(False)
        self.run_setpoint = speed
        self.run_kp = kp
        self.current_test = selected
        self.export_index = 0
        self.export_phase = "header"

    def run(self):
        self.print_help()

        while True:

            # display menu one time and keep looping through the state machine
            if self.state == self.S0_AWAIT_COMMAND: 
                #see if incoming bytes are available
                if self.serial.any():
                    #reads one byte and sets it to 
                    # type: ignore
                    buff: bytes = self.serial.read(1)
                    #convert byutes into character and set new_char
                    self.new_char = buff.decode()
                    self.state = self.S1_PROCESS_COMMAND

                
                #if string is empty
                if self.current_test == "":
                    if self.left_test_flag.value or self.left_data.value["count"]:
                        #select motor if test requested for that motor
                        self.current_test = "left"
                    elif self.right_test_flag.value or self.right_data.value["count"]:
                        self.current_test = "right"
                    #for data just selected se tthe sampleindex to 0
                    if self.current_test:
                        self.export_index = 0
                        self.export_phase = "header"
                #send part of selected data
                if self.current_test:
                    self.transmit_sample()

            elif self.state == self.S1_PROCESS_COMMAND:

                # Convert recieved char to a lowercase 
                low_char = self.new_char.lower()


                #check if exit requested
                if low_char == "e":
                    if self.motor_test is not None and self.motor_test["active"]:
                        self.motor_test["motors"][0].finish_test("aborted")

                    self.line_follow_flag.set(False)
                    if self.sensor_flag is not None:
                        self.sensor_flag.set(False)

                    #send request for program exit
                    #exit even if theres an unfinished numeric entry.
                    self.exit_initiated = True
                    return


                #here stop the line following, but keep going in the program
                elif low_char == "s": 
                    if self.motor_test is not None and self.motor_test["active"]:
                        self.motor_test["motors"][0].finish_test("aborted")

                    #motor tests and line following -stop
                    #from refactoring we now have a left and right test flag
                    self.left_test_flag.set(False)
                    self.right_test_flag.set(False)
                    self.line_follow_flag.set(False)
                    self.lane_change_flag.set(False)
                    self.imu_flag.set(False)
                    if self.sensor_flag is not None:
                        self.sensor_flag.set(False)
                    self.state = self.S0_AWAIT_COMMAND


                elif low_char == "w":
                    if self.test_busy():
                        self.serial.write(b"Motor test busy\r\n")
                        self.state = self.S0_AWAIT_COMMAND
                        yield self.state
                        continue
                    self.following_target.set(False)
                    self.sensor_flag.set(True)
                    self.imu_flag.set(True)
                    self.line_follow_flag.set(True)
                    self.state = self.S0_AWAIT_COMMAND


                #print help menu
                elif low_char == "h":
                    self.print_help()
                    self.state = self.S0_AWAIT_COMMAND

                elif low_char in ("b", "l", "r"):
                    self.request_motor_test("right" if low_char == "r" else "left")
                    self.state = self.S0_AWAIT_COMMAND

                elif low_char in ("v", "p"):
                    if self.test_busy():
                        self.serial.write(b"Motor test busy\r\n")
                        self.state = self.S0_AWAIT_COMMAND
                        yield self.state
                        continue

                    #get reference to left motor shared dict
                    #initialize the shared varriables that will recieve the number
                    #entry target share is based on if the user enters v or p or none
                    self.entry_target = self.next_effort if low_char == "v" else self.next_kp
                    #print error if share not supplied
                    if self.entry_target is None:
                        self.serial.write(b"Gain share is not yet configured\r\n")
                        self.state = self.S0_AWAIT_COMMAND
                    else:
                        prompt = "Enter velocity setpoint [rad/s]:" if low_char == "v" else "Enter Kp [V/(rad/s)]:"
                        self.serial.write((prompt + "\r\n").encode())
                        self.out_buff = []
                        self.entry_invalid = False
                        self.state = self.S2_AWAIT_DIGIT_INPUT


                #d for the implimentation of awsd
                elif low_char == "d":
                    if self.line_follow_flag.value:
                        self.lane_change_flag.set(True)
                        self.imu_flag.set(True)
                        self.lane_change_dir.value = "right"
                        self.state = self.S0_AWAIT_COMMAND
                    else:
                        self.state = self.S0_AWAIT_COMMAND


                #a for the implimentation of awsd
                elif low_char == "a":
                    if self.line_follow_flag.value:
                        self.lane_change_flag.set(True)
                        self.imu_flag.set(True)
                        self.lane_change_dir.value = "left"
                        self.state = self.S0_AWAIT_COMMAND
                    else:
                        self.state = self.S0_AWAIT_COMMAND

                self.new_char = ""


            #check for bytes then read byte then convert byte to text and process text
            elif self.state == self.S2_AWAIT_DIGIT_INPUT:
                #Keep checking for new inputs in buffer
                if self.serial.any():
                    buff: bytes = self.serial.read(1)
                    self.new_char = buff.decode()
                    self.state = self.S3_PROCESS_DIGIT_INPUT


            #if exit then request termination
            elif self.state == self.S3_PROCESS_DIGIT_INPUT:

                #Digit input logic tree:
                if self.new_char.lower() == "e":
                    self.line_follow_flag.set(False)
                    if self.sensor_flag is not None:
                        self.sensor_flag.set(False)

                    self.exit_initiated = True
                    return


                #skip newline if enter already happened(had toroubles with entering twice)
                if self.skip_lf and self.new_char == "\n":
                    self.skip_lf = False
                #if the ending char is a newline it is the end of the entry
                elif self.new_char in self.TERMINATORS:
                    self.skip_lf = self.new_char == "\r"
                    #validate the entry
                    try:
                        if self.entry_invalid:
                            raise ValueError("Malformed entry")
                        value = float("".join(self.out_buff))
                        #reject entries with mag greater 1000
                        if not isfinite(value) or abs(value) > 1000:
                            raise ValueError("Nonfinite or out of range")
                        #gain must be positive
                        if self.entry_target is self.next_kp and value < 0:
                            raise ValueError("Kp must be nonnegative")
                        self.entry_target.set(value)
                        self.serial.write(("\r\nValue set to {}\r\n".format(value)).encode())

                    except (ValueError, OverflowError): 
                        #if cant validate or convert then report error and keep old valid settings
                        self.serial.write(("\r\nInvalid input; value unchanged: {}\r\n".format(
                            self.entry_target.value)).encode())

                    self.state = self.S0_AWAIT_COMMAND
                    self.out_buff = []
                    self.new_char = ""
                    yield self.state
                    continue


                #handling delete and backspace
                elif self.new_char in ("\b", "\x7f"):
                    if self.out_buff:
                        self.out_buff.pop()
                        self.serial.write(b"\b \b")


                #accept numeric entries (valid ones)
                elif (self.new_char in self.VALID_DIGITS
                      or (self.new_char == "." and "." not in self.out_buff)
                      or (self.new_char in ("-", "+") and not self.out_buff)):
                    self.skip_lf = False
                    if len(self.out_buff) < 24:
                        self.serial.write(self.new_char.encode())
                        self.out_buff.append(self.new_char)
                    else:
                        self.entry_invalid = True
                else:

                    # Reject malformed entries -dont silently dropping characters
                    self.entry_invalid = True

                self.new_char = ""
                #clear invalid flag, numeric text, and wait for num now
                self.state = self.S2_AWAIT_DIGIT_INPUT

                
            yield self.state

    def transmit_sample(self):


        #set data and flag based on motor side
        data = self.left_data.value if self.current_test == "left" else self.right_data.value
        test_flag = self.left_test_flag if self.current_test == "left" else self.right_test_flag

        if not self.pending_output:
            if self.export_phase == "header":
                line = (
                    "Begin data: motor={},mode={},setpoint_rad_s={},"
                    "kp={},motor_period_ms={},battery_voltage_V={},"
                    "feedback={},ki={},kd={},feedforward=disabled,"
                    "observer_period_ms={},voltage_min_V={},voltage_max_V={},"
                    "velocity_filter=none,velocity_window=encoder_update_interval,"
                    "duration_ms={},integral_convention={}"
                ).format(
                    self.current_test,
                    "discrete_pi" if isinstance(self.motor_test["motors"][0].pid, DiscretePI) else "original",
                    self.run_setpoint,
                    self.run_kp,
                    self.motor_period_ms,
                    self.battery_voltage,
                    self.motor_test["motors"][0].velocity_feedback,
                    self.motor_test["motors"][0].pid.ki,
                    self.motor_test["motors"][0].pid.kd,
                    self.motor_test["motors"][0].period_ms,
                    -self.battery_voltage,
                    self.battery_voltage,
                    self.motor_test["motors"][0].test_duration_us // 1000,
                    "per_sample" if isinstance(self.motor_test["motors"][0].pid, DiscretePI) else "time_weighted_seconds",
                )
                self.export_phase = "columns"

            elif self.export_phase == "columns":
                line = self.LINE_COLUMNS
                self.export_phase = "rows"

            elif self.export_phase == "rows":
                if self.export_index < data["count"]:
                    i = self.export_index
                    time_us = data["time_us"][i]

                    if time_us == 4294967295:

                        # Consume the end marker, but don't send it as a data row.
                        self.export_index += 1
                        line = "End of data: " + data.get("status", "unknown")
                        # The recorder stopped and there are no more buffered samples.
                        self.export_phase = "finished"
                    else:
                        line = "{},{},{},{},{}".format(
                            time_us,
                            data["setpoint_rad_s"][i],
                            data["velocity_rad_s"][i],
                            data["estimated_velocity_rad_s"][i],
                            data["voltage_V"][i],
                        )
                        self.export_index += 1

                elif test_flag.value or data.get("status") == "running":

                    return

                else:

                    line = "End of data: " + data.get("status", "unknown")
                    self.export_phase = "finished"

            elif self.export_phase == "finished":
                data["count"] = 0
                self.export_index = 0
                line = self.LINE_READY
                self.export_phase = "released"

            else:
                data["count"] = 0
                self.export_index = 0
                return

            self.pending_output = (line + "\r\n").encode()

        sent = self.serial.write(self.pending_output[:128])
        if sent:
            self.pending_output = self.pending_output[sent:]
        if not self.pending_output and self.export_phase == "released":
            #clear current test once tramit done
            self.current_test = ""
            self.export_phase = "header"
