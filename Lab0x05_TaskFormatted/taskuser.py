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

    #initialize at the begining of the task
    def __init__(self, left_test_flag: Share, right_test_flag: Share, left_data: Share, right_data: Share, next_effort: Share, serial: USB_VCP, next_kp=None, motor_period_ms=100, ui_period_ms=0, battery_voltage=None):
        # initialize all of the data as finite objects in task user
        self.left_test_flag = left_test_flag
        self.right_test_flag = right_test_flag
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
        print("| l/L | Left motor step response and printed results                           |")
        print("| r/R | Right motor step respoinse and print results                           |")
        print("| e/E | Exit interface                                                         |")
        print("+-----+------------------------------------------------------------------------+")
        print("Validation: signed decimal velocity +/-1000 rad/s; Kp 0..1000; max 24 characters.")

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

                if low_char == "e":
                    #request program to exit
                    self.exit_initiated = True
                    return

                #rejects commands while testing happening unless comand is e
                elif self.test_busy():
                    self.serial.write(b"Motor test busy; only e exits during a run.\r\n")
                    self.state = self.S0_AWAIT_COMMAND

                #print help menu
                elif low_char == "h":
                    self.print_help()
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

                else:
                    self.state = self.S0_AWAIT_COMMAND

                self.new_char = ""

            #check for bytes, read byte, convert byte to text and process text
            elif self.state == self.S2_AWAIT_DIGIT_INPUT:
                if self.serial.any(): #Keep checking for new inputs in buffer
                    buff: bytes = self.serial.read(1) # type: ignore
                    self.new_char = buff.decode()
                    self.state = self.S3_PROCESS_DIGIT_INPUT

            #if exit then request termination
            elif self.state == self.S3_PROCESS_DIGIT_INPUT:
                #Digit input logic tree:
                if self.new_char.lower() == "e":
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
        #set the test flag and data for the motor
        data = self.left_data.value if self.current_test == "left" else self.right_data.value
        flag = self.left_test_flag if self.current_test == "left" else self.right_test_flag
        if not self.pending_output:
            if self.export_phase == "header": #here we had to use copiolot to format the header string
                line = ("Begin data: motor={},mode=P,setpoint_rad_s={},kp={},motor_period_ms={},"
                        "ui_period_ms={},duration_s=2,time_units=us,velocity_units=rad/s,"
                        "voltage_units=V,battery_voltage={},voltage_min={},voltage_max={}").format(
                            self.current_test, self.run_setpoint, self.run_kp, self.motor_period_ms,
                            self.ui_period_ms, self.battery_voltage,
                            -self.battery_voltage if self.battery_voltage is not None else "unset",
                            self.battery_voltage)
                self.export_phase = "columns"
            elif self.export_phase == "columns":#make the csv column names
                line = "time_us,setpoint_rad_s,velocity_rad_s,limited_voltage_V"
                self.export_phase = "rows"
            elif self.export_index < data["count"]: #check if an unread sample exists
                i = self.export_index
                line = "{},{},{},{}".format(data["time_us"][i], data["setpoint"][i],
                                           data["velocity"][i], data["voltage"][i])
                self.export_index += 1
            elif flag.value:
                return #If the motor is still collecting thent ry the next scheduling turn
            elif self.export_phase == "rows":
                line = "End of data: " + data["status"]
                self.export_phase = "finished"
            elif self.export_phase == "finished":
                #Inidicate that the buffers for this run have been released 
                data["count"] = 0
                line = "Ready for next test"
                self.export_phase = "released"
            else:
                #Release dataset after every bytetransmitted.
                data["count"] = 0
                self.current_test = ""
                return
            self.pending_output = (line + "\r\n").encode()
        
        #try to send 128 bytes then report the number accepted
        sent = self.serial.write(self.pending_output[:128])
        if sent:
            self.pending_output = self.pending_output[sent:]
