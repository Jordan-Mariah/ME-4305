"""User interaction with a blocking input loop for standalone testing."""
from pyb import USB_VCP
from tshare import Share
from typing import cast
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


    #Task variables



    #initialize at the begining of the task
    def __init__(self, left_test_flag: Share, right_test_flag: Share, left_data: Share, right_data: Share, next_effort: Share, serial: USB_VCP):
        # Argument handling
        self.left_test_flag = left_test_flag
        self.right_test_flag = right_test_flag
        self.left_data = left_data
        self.right_data = right_data
        self.next_effort = next_effort
        self.serial = serial
        self.exit_initiated = False

        # Task variables
        self.new_char = ""
        self.out_buff: list[str] = []
        self.current_test: str = ""

        self.state = self.S0_AWAIT_COMMAND

    def print_help(self):
        print("Welcome to the user interface! Print one of the following commands to get started.")
        print("+------------------------------------------------------------------------------+")
        print("| ME 4305 Romi Tuning Interface Help Menu                                      |")
        print("+-----+------------------------------------------------------------------------+")
        print("| h/H | Print help menu                                                        |")
        print("| l/L | Trigger step response sequence on left motor and print results         |")
        print("| r/R | Trigger step response sequence on right motor and print results        |")
        print("| e/E | Exit program                                                           |")
        print("+-----+------------------------------------------------------------------------+")


    def run(self):
        self.print_help()

        while True:

            if self.state == self.S0_AWAIT_COMMAND: 
                if self.serial.any(): #Keep checking for new inputs in buffer
                    buff: bytes = self.serial.read(1) # type: ignore
                    self.new_char = buff.decode()
                    self.state = self.S1_PROCESS_COMMAND

                # if there is no current test, check if there is data waiting and set current_test
                if self.current_test == "":
                    if self.left_test_flag.value or cast(list[tuple[int, int]], self.left_data.value):
                        self.current_test = "left"
                        self.serial.write(
                            f"Begin data: motor=left,duty_cycle_pct={self.next_effort.value}\r\n".encode()
                        )
                        self.serial.write(b"time_us,position_counts\r\n")

                    elif self.right_test_flag.value or cast(list[tuple[int, int]], self.right_data.value):
                        self.current_test = "right"
                        self.serial.write(
                            f"Begin data: motor=right,duty_cycle_pct={self.next_effort.value}\r\n".encode()
                        )
                        self.serial.write(b"time_us,position_counts\r\n")

                # Process left and right test outputs 
                # TODO impliment arbatrary serial output from any task. Currently handles left and right tests explicitly

                if self.current_test == "left":
                    data = cast(list[tuple[int, int]], self.left_data.value) # cast() is just for typing in vscode
                    if data: # data is a list that taskmotor continuously adds time, position points to
                        time, position = data.pop(0) # Get oldest data point
                        self.serial.write(f"{time},{position}\r\n".encode()) # Send it through serial

                    # If left test is over push end string through serial
                    elif self.left_test_flag.value == False: 
                        self.serial.write("End of data\r\n")
                        self.current_test = ""

                if self.current_test == "right":
                    data = cast(list[tuple[int, int]], self.right_data.value) # Just for typing in vscode
                    if data:
                        time, position = data.pop(0)
                        self.serial.write(f"{time},{position}\r\n".encode())

                    # If right test is over push end string through serial
                    elif self.right_test_flag.value == False:
                        self.serial.write(b"End of data\r\n")
                        self.current_test = ""

            elif self.state == self.S1_PROCESS_COMMAND:
                # Convert the char recieved to a lowercase character.
                low_char = self.new_char.lower()

                if low_char == "h":
                    self.print_help()
                    self.state = self.S0_AWAIT_COMMAND

                elif low_char == "l":
                    # Make sure there is no residual data in left data buffer before starting new test
                    data = cast(list[tuple[int, int]], self.left_data.value)

                    if self.left_test_flag.value == False and not data:
                        self.left_test_flag.set(True)
                    else:
                        self.serial.write(b"Left motor is already running\r\n")
                    self.state = self.S0_AWAIT_COMMAND

                elif low_char == "r":
                    # Make sure there is no residual data in right data buffer before starting new test
                    data = cast(list[tuple[int, int]], self.left_data.value)

                    if self.right_test_flag.value == False and not data:
                        self.right_test_flag.set(True)
                    else:
                        self.serial.write(b"Right motor is already running\r\n")
                    self.state = self.S0_AWAIT_COMMAND

                elif low_char == "d":
                    self.serial.write("Enter duty cycle [%]:\r\n".encode())
                    self.out_buff = []
                    self.state = self.S2_AWAIT_DIGIT_INPUT

                elif low_char == "e":
                    raise KeyboardInterrupt("Stopped through 'e' key")
                
                else:
                    self.state = self.S0_AWAIT_COMMAND

                self.new_char = ""

            elif self.state == self.S2_AWAIT_DIGIT_INPUT:
                if self.serial.any(): #Keep checking for new inputs in buffer
                    buff: bytes = self.serial.read(1) # type: ignore
                    self.new_char = buff.decode()
                    self.state = self.S3_PROCESS_DIGIT_INPUT

            elif self.state == self.S3_PROCESS_DIGIT_INPUT:
                #Digit input logic tree:

                if self.new_char in self.VALID_DIGITS:
                    self.serial.write(self.new_char)
                    self.out_buff.append(self.new_char)

                elif self.new_char == "." and not self.new_char in self.out_buff:
                    self.serial.write(self.new_char)
                    self.out_buff.append(self.new_char)

                elif self.new_char == "-" and len(self.out_buff) == 0:
                    self.serial.write(self.new_char)
                    self.out_buff.append(self.new_char)

                elif self.new_char == "\x7f" and not self.out_buff == []:
                    self.serial.write(self.new_char.encode())
                    self.out_buff.pop()

                elif self.new_char in self.TERMINATORS:

                    if len(self.out_buff) == 0:
                        self.serial.write(f"\r\nInvalid input (empty). Effort unchanged: {self.next_effort.value}".encode())

                    elif self.out_buff in (["-"], ["."]):
                        self.serial.write(f"\r\nInvalid input (only '-'). Effort unchanged: {self.next_effort.value}".encode())

                    elif self.out_buff[-1] == ".":
                        self.serial.write(f"\r\nInvalid input (nothing after '.'). Effort unchanged: {self.next_effort.value}".encode())

                    else:
                        self.next_effort.set(float("".join(self.out_buff)))
                        self.serial.write(f"\r\nValue set to {self.next_effort.value}")

                    # Enter detected: go back to await
                    self.state = self.S0_AWAIT_COMMAND
                    self.new_char = ""
                    continue

                self.new_char = ""
                self.state = self.S2_AWAIT_DIGIT_INPUT

                
            yield self.state