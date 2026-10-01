"""User interaction with a blocking input loop for standalone testing."""
from pyb import USB_VCP
#steps
#prompt user for input
#go into FSM depending on user string response

class TaskUser:

    #initialize at the begining of the task
    def __init__(self, left_motor, right_motor):

        self.left_motor = left_motor
        self.right_motor = right_motor
        self.exit_initiated = False

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


    def user_interaction(self):
        serial = USB_VCP()
        self.print_help()

        while True:
            #are there current chars in serial input
            if serial.any():
                received = serial.recv(1, timeout=0) #reads up to a byte and doesnt wait if nothing there

                if received:
                    # Convert the byte recieved to a lowercase character.
                    self.state = chr(received[0]).lower()

                    if self.state == "h":
                        self.print_help()

                    elif self.state == "l":
                        if self.left_motor.start():
                            print("Starting left motor sequence")
                        else:
                            print("Left motor is already running")

                    elif self.state == "r":
                        if self.right_motor.start():
                            print("Starting right motor sequence")
                        else:
                            print("Right motor is already running")

                    elif self.state == "e":
                        self.left_motor.stop()
                        self.right_motor.stop()
                        self.exit_initiated = True

            # Outside both if blocks: yield even when no input arrives.
            yield 0
