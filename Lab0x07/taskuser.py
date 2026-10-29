"""Cooperative velocity/gain entry and response transmission."""
from pyb import USB_VCP # type: ignore
from tshare import Share
from math import isfinite
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

    LINE_HEADER = "Begin line-follow data"
    LINE_COLUMNS = "time_us,centroid_pos"
    LINE_FOOTER = "End of line-follow data"
    LINE_READY = "Ready for next line-follow run"

    #initialize at the begining of the task
    #****new
    def __init__(self, left_test_flag: Share, right_test_flag: Share, left_data: Share, right_data: Share, next_effort: Share, line_follow_flag: Share, lane_change_flag: Share, lane_change_dir: Share, imu_flag: Share, serial: USB_VCP, next_kp=None, motor_period_ms=100, ui_period_ms=0, battery_voltage=None, sensor_flag=None):
        # initialize all of the data as finite objects in task user
        self.left_test_flag = left_test_flag
        self.right_test_flag = right_test_flag
        self.line_follow_flag = line_follow_flag
        self.lane_change_flag = lane_change_flag
        self.lane_change_dir = lane_change_dir
        self.sensor_flag = sensor_flag #added this sensor flag 
        self.imu_flag = imu_flag
        self.left_data = left_data
        self.right_data = right_data
        self.next_effort = next_effort
        self.next_kp = next_kp #The proportional gain is set through p/P- shared
        self.serial = serial
        self.motor_period_ms = motor_period_ms
        self.ui_period_ms = ui_period_ms
        self.battery_voltage = battery_voltage
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
        print("| b/B | Equal out velocity of both motors                                      |")
        print("| l/L | Left motor step response and printed results                           |")
        print("| r/R | Right motor step respoinse and print results                           |")
        print("| w/W | Start line following                                                   |")
        print("| s/S | Stop line following                                                    |")
        print("| d/D | Merge right                                                            |")
        print("| a/A | Merge left                                                             |")
        print("| e/E | Exit interface                                                         |")
        print("+-----+------------------------------------------------------------------------+")
        print("Validation: signed decimal velocity +/-1000 rad/s; Kp 0.1000; max 24 characters.")

    def test_busy(self):
            #returns true if test running, requested, or sending data and otherwise false
            return bool(self.left_test_flag.value or self.right_test_flag.value
                    or self.left_data.value["count"] or self.right_data.value["count"] or self.current_test)

    def run(self):
        self.print_help()

        while True:
            # display menu one time and keep looping through the state machine
            if self.state == self.S0_AWAIT_COMMAND: 
                if self.serial.any(): #see if incoming bytes are available
                    buff: bytes = self.serial.read(1) #reads one byte and sets it to 
                    self.new_char = buff.decode()#convert byutes into character and set new_char
                    self.state = self.S1_PROCESS_COMMAND

                
                if self.current_test == "":#if string is empty
                    if self.left_test_flag.value or self.left_data.value["count"]:
                        self.current_test = "left" #select motor if test requested for that motor
                    elif self.right_test_flag.value or self.right_data.value["count"]:
                        self.current_test = "right"
                    if self.current_test:#for data just selected se tthe sampleindex to 0
                        self.export_index = 0
                        self.export_phase = "header"
                if self.current_test:#send part of selected data
                    self.transmit_sample()

            elif self.state == self.S1_PROCESS_COMMAND:
                # Convert recieved char to a lowercase 
                low_char = self.new_char.lower()

                #check if exit requested
                if low_char == "e":
                    #clear request for line sensing
                    self.line_follow_flag.set(False)
                    if self.sensor_flag is not None:
                        self.sensor_flag.set(False)
                    #send request for program exit
                    self.exit_initiated = True
                    return

                #here stop the line following, but keep going in the program
                elif low_char == "s": 
                    self.line_follow_flag.set(False)
                    if self.sensor_flag is not None:
                        self.sensor_flag.set(False)
                    self.state = self.S0_AWAIT_COMMAND

                #start the line following here
                elif low_char == "w":
                    self.sensor_flag.set(True)
                    self.line_follow_flag.set(True)
                    self.state = self.S0_AWAIT_COMMAND

                #print help menu
                elif low_char == "h":
                    self.print_help()
                    self.state = self.S0_AWAIT_COMMAND

                elif low_char == "b":
                    self.run_setpoint = self.next_effort.value
                    #here we store the output gain
                    self.run_kp = (
                        self.next_kp.value
                        #if no gainw as shared use .001 for the gain
                        if self.next_kp is not None
                        else 0.001
                    )
                    self.left_test_flag.set(True)
                    self.right_test_flag.set(True)
                    self.state = self.S0_AWAIT_COMMAND

                #get reference to left motor shared dict
                elif low_char == "l":

                    # Make sure there is no residual data in left data buffer before starting new test
                    data = self.left_data.value 

                    #store Kp and velocity had to get help from copiolot on structureing flag conditionals
                    if self.left_test_flag.value == False and data["count"] == 0:
                        self.run_setpoint = self.next_effort.value
                        self.run_kp = self.next_kp.value if self.next_kp is not None else 0.001
                        self.left_test_flag.set(True)
                    else:
                        self.serial.write(b"Left motor is already running\r\n")
                    self.state = self.S0_AWAIT_COMMAND

                elif low_char == "r":
                    # check for residual data in right data buffer before starting new test
                    data = self.right_data.value #* Read the preallocated response dictionary.

                    #check if motor is already running, then make flag so that TaskMotor starts the right motor test
                    if self.right_test_flag.value == False and data["count"] == 0:
                        self.run_setpoint = self.next_effort.value
                        self.run_kp = self.next_kp.value if self.next_kp is not None else 0.001
                        self.right_test_flag.set(True)
                    else:
                        self.serial.write(b"Right motor is already running\r\n")
                    self.state = self.S0_AWAIT_COMMAND

                elif low_char in ("v", "p"):#initialize the shared varriables that will recieve the number
                    #entry target share is based on if the user enters v or p or none
                    self.entry_target = self.next_effort if low_char == "v" else self.next_kp
                    if self.entry_target is None: #print error if share not supplied
                        self.serial.write(b"Gain share is not yet configured\r\n")
                        self.state = self.S0_AWAIT_COMMAND
                    else:
                        prompt = "Enter velocity setpoint [rad/s]:" if low_char == "v" else "Enter Kp [V/(rad/s)]:"
                        self.serial.write((prompt + "\r\n").encode())
                        self.out_buff = []
                        self.entry_invalid = False
                        self.state = self.S2_AWAIT_DIGIT_INPUT#clear invalid flag, numeric text, and wait for num now

                elif low_char == "d":
                    if self.line_follow_flag.value:
                        self.lane_change_flag.set(True)
                        self.imu_flag.set(True)
                        self.lane_change_dir.value = "right"
                        self.state = self.S0_AWAIT_COMMAND
                    else:
                        self.state = self.S0_AWAIT_COMMAND

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
                if self.serial.any(): #Keep checking for new inputs in buffer
                    buff: bytes = self.serial.read(1) # type: ignore
                    self.new_char = buff.decode()
                    self.state = self.S3_PROCESS_DIGIT_INPUT

            #if exit then request termination
            elif self.state == self.S3_PROCESS_DIGIT_INPUT:
                #Digit input logic tree:
                if self.new_char.lower() == "e":
                    self.line_follow_flag.set(False)
                    if self.sensor_flag is not None:
                        self.sensor_flag.set(False)
                    #exit even if theres an unfinished numeric entry.
                    self.exit_initiated = True
                    return

                #skip newline if enter already happened(had toroubles with entering twice)
                if self.skip_lf and self.new_char == "\n":
                    self.skip_lf = False
                elif self.new_char in self.TERMINATORS:#if the ending char is a newline it is the end of the entry
                    self.skip_lf = self.new_char == "\r"
                    try: #validate the entry
                        if self.entry_invalid:
                            raise ValueError("Malformed entry")
                        value = float("".join(self.out_buff))
                        if not isfinite(value) or abs(value) > 1000: #reject entries with mag greater 1000
                            raise ValueError("Nonfinite or out of range")
                        if self.entry_target is self.next_kp and value < 0: #gain must be positive
                            raise ValueError("Kp must be nonnegative")
                        self.entry_target.set(value)
                        self.serial.write(("\r\nValue set to {}\r\n".format(value)).encode())
                        #if cant validate or convert then report error and keep old valid settings
                    except (ValueError, OverflowError): 
                        self.serial.write(("\r\nInvalid input; value unchanged: {}\r\n".format(
                            self.entry_target.value)).encode())
                    # Enter detected: go back to await
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
                self.state = self.S2_AWAIT_DIGIT_INPUT

                
            yield self.state

    def transmit_sample(self):
        data = self.left_data.value

        if not self.pending_output:
            if self.export_phase == "header":
                line = self.LINE_HEADER
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
                        line = self.LINE_FOOTER
                        self.export_phase = "finished"
                    else:
                        line = "{},{}".format(
                            time_us,
                            data["centroid_pos"][i],
                        )
                        self.export_index += 1

                elif self.line_follow_flag.value:
                    # Wait for the recorder to add another sample.
                    return

                else:
                    # The recorder stopped and there are no more buffered samples.
                    line = self.LINE_FOOTER
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
