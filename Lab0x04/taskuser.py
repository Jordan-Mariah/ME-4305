"""User interaction with a blocking input loop for standalone testing."""
from pyb import USB_VCP
from tshare import Share
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

    #Task variables

    new_char = ""
    out_buff: list[str] = []

    #initialize at the begining of the task
    def __init__(self, left_test_flag: Share, right_test_flag: Share, next_effort: Share, serial: USB_VCP):

        self.left_test_flag = left_test_flag
        self.right_test_flag = right_test_flag
        self.serial = serial
        self.exit_initiated = False

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

            elif self.state == self.S1_PROCESS_COMMAND:
                # Convert the char recieved to a lowercase character.
                low_char = self.new_char.lower()

                if low_char == "h":
                    self.print_help()
                    self.state = self.S0_AWAIT_COMMAND

                elif low_char == "l":
                    if self.left_test_flag.value == False:
                        self.left_test_flag.set(True)
                        print("Starting left motor sequence")
                    else:
                        print("Left motor is already running")
                    self.state = self.S0_AWAIT_COMMAND

                elif low_char == "r":
                    if self.right_test_flag.value == False:
                        self.right_test_flag.set(True)
                        print("Starting right motor sequence")
                    else:
                        print("Right motor is already running")
                    self.state = self.S0_AWAIT_COMMAND

                elif low_char == "d":
                    print("Enter new target effort%:\n")
                    self.out_buff.clear()
                    self.state = self.S2_AWAIT_DIGIT_INPUT

                elif low_char == "e":
                    raise KeyboardInterrupt("Stopped through 'e' key")
                    # self.left_motor.stop()
                    # self.right_motor.stop()
                    # self.exit_initiated = True
                    self.state = self.S0_AWAIT_COMMAND
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
                    self.out_buff.append(self.new_char)

                elif self.new_char == "." and not self.new_char in self.out_buff:
                    self.out_buff.append(self.new_char)

                elif self.new_char == "-" and len(self.out_buff) == 0:
                    self.out_buff.append(self.new_char)

                elif self.new_char ==

                self.new_char = ""
                self.state = self.S2_AWAIT_DIGIT_INPUT

                
            yield self.state
