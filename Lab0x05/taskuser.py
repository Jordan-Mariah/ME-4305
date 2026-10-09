"""
Lab 0x04.

"""

from pyb import USB_VCP

#class attributes
WAIT_FOR_COMMAND = 0
READ_DUTY_CYCLE = 1


class TaskUser:
    # Define integer class attributes for the three scheduled UI states.
    WAIT_FOR_COMMAND = 0
    READ_DUTY_CYCLE = 1
    WAIT_FOR_GAIN_COMMAND = 2

    def __init__(self, left_motor, right_motor):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.exit_initiated = False

        # FSM states
        # Store the initial state using the class's integer state constant.
        self.state = self.WAIT_FOR_COMMAND
        self.duty_cycle = 0.0
        self.entry_chars = []
        self.digits = set(map(str, range(10)))
        self.term = {"\r", "\n"}
        self.serial = USB_VCP()
        self.test_active = False

    def print_help(self):
        print("ME 4305 Romi Open-Loop Test Interface")
        print("+-----+-----------------------------------------------+")
        print("| h/H | Print help menu                               |")
        print("| d/D | Enter duty cycle for next test                |")
        # Display the command that opens the nonblocking gain-selection menu.
        print("| k/K | Open gain menu (entry not implemented yet)    |")
        print("| l/L | Run left motor open-loop test                 |")
        print("| r/R | Run right motor open-loop test                |")
        print("| e/E | Exit program                                  |")
        print("+-----+-----------------------------------------------+")

    def print_gain_menu(self):
        print("ME 4305 Romi PID Gain Settings")
        print("+-----+-----------------------------------------------+")
        print("| p  | Proportional gain(kp)                         |")
        print("| i  | Integral gain (ki)                            |")
        print("| d  | Derivative gain (kd)                          |")
        print("| e  | Exit gain settings                            |")
        print("+-----+-----------------------------------------------+")

    def print_prompt(self):
        # State the selected value and the next available actions.
        print("Selected duty: {}%.".format(self.duty_cycle))
        print("Choose motor: l = left, r = right; d = change duty; h = help; e = exit.")

    def process_numeric_char(self, ch):
        """Process one character; return True when numeric entry is complete."""
        if ch in self.digits:
            self.serial.write(ch)
            self.entry_chars.append(ch)

        elif ch == ".":
            # The flowchart permits one period, including as the first character.
            if "." not in self.entry_chars:
                self.serial.write(ch)
                self.entry_chars.append(ch)

        elif ch == "-" and len(self.entry_chars) == 0:
            self.serial.write(ch)
            self.entry_chars.append(ch)

        elif ch in {"\b", "\x7f"} and len(self.entry_chars) > 0:
            self.serial.write("\b \b")
            self.entry_chars.pop()

        elif ch in self.term:
            # Empty entries and entries without digits return to START.
            if not self.entry_chars:
                self.serial.write("\r\nNo value entered; duty unchanged. Enter a number or e to exit.\r\n")
                return False
            if not any(char in self.digits for char in self.entry_chars):
                self.serial.write("\r\nIncomplete entry; add digits or use rubout to correct.\r\n")
                return False

            self.serial.write("\r\n")
            value = float("".join(self.entry_chars))
            if not -100.0 <= value <= 100.0:
                self.serial.write("Duty cycle out of range. Use -100 to +100.\r\n")
                self.entry_chars = []
                self.serial.write("Enter a replacement duty cycle and press Enter, or e to exit.\r\n")
                return False
            self.duty_cycle = value
            self.serial.write(f"Value set to {self.duty_cycle}\r\n")
            self.entry_chars = []
            return True

        # All other characters are ignored, as in the starter code.
        return False

    def user_interaction(self):
        serial = self.serial
        self.print_help()
        self.print_prompt()

        while True:
            # Announce completion after export without inserting text into the CSV.
            if self.test_active and self.left_motor.done and self.right_motor.done:
                self.test_active = False
                print("Test complete; both motors disabled.")
                if self.state == self.WAIT_FOR_COMMAND:
                    self.print_prompt()
                else:
                    print("Continue duty entry and press Enter, or e to exit.")

            if self.state == self.WAIT_FOR_COMMAND:
                #not blocking because this is just if serial.any at the current point in time not using while
                if serial.any():
                    data = serial.read(1)
                    if not data:
                        yield 0
                        continue

                    ch = data.decode("ascii", "ignore")
                    ch = ch.lower()

                    if ch == "h":
                        self.print_help()
                        self.print_prompt()

                    elif ch == "d":
                        self.state = self.READ_DUTY_CYCLE
                        self.entry_chars = []
                        print("Type a duty from -100 to +100; decimals allowed. Rubout corrects; Enter accepts; e exits.")
                        print("Enter duty cycle [%]: ")

                    elif ch == "k":
                        # Call the menu method before waiting for a new letter.
                        self.print_gain_menu()
                        # Update the state attribute for the next scheduler turn.
                        self.state = self.WAIT_FOR_GAIN_COMMAND

                    elif ch == "l":
                        if self.left_motor.done and self.right_motor.done:
                            self.left_motor.start(self.duty_cycle)
                            self.test_active = True
                            print("Left test requested at {}%; wait for End of data, or e to stop.".format(self.duty_cycle))
                        else:
                            print("Motor test busy")

                    elif ch == "r":
                        if self.left_motor.done and self.right_motor.done:
                            self.right_motor.start(self.duty_cycle)
                            self.test_active = True
                            print("Right test requested at {}%; wait for End of data, or e to stop.".format(self.duty_cycle))
                        else:
                            print("Motor test busy")

                    elif ch == "e":
                        self.left_motor.stop()
                        self.right_motor.stop()
                        self.exit_initiated = True
                        print("Exiting; both motors disabled.")
                        break

                yield 0

            elif self.state == self.READ_DUTY_CYCLE:
                if serial.any():
                    #returns a byte object
                    data = serial.read(1)
                    if not data:
                        yield 0
                        continue

                    ch = data.decode("ascii", "ignore")

                    #** Permit safe exit even while a numeric entry is incomplete.
                    if ch.lower() == "e":
                        self.left_motor.stop()
                        self.right_motor.stop()
                        self.exit_initiated = True
                        print("Exiting; both motors disabled.")
                        break

                    # this is the flowchart's 'join digits convert to float' step
                    if self.process_numeric_char(ch):
                        self.state = self.WAIT_FOR_COMMAND
                        #** Prompt for the motor after a valid duty cycle is accepted.
                        self.print_prompt()

                yield 0

            # Read a gain selection separately from the original k command.
            elif self.state == self.WAIT_FOR_GAIN_COMMAND:
                # Poll the VCP object before attempting a nonblocking read.
                if serial.any():
                    # Read one new character as a bytes object.
                    data = serial.read(1)
                    # Yield if the read returns no bytes.
                    if not data:
                        yield 0
                        continue

                    # Decode the new bytes and normalize the letter's case.
                    ch = data.decode("ascii", "ignore").lower()

                    # Recognize selections without calling unfinished gain methods.
                    if ch in {"p", "i", "d"}:
                        # TODO: add a separate nonblocking numeric-entry state.
                        print("{} selected; gain entry is not implemented yet.".format(ch))

                    # Return to the command state without ending the generator.
                    elif ch == "e":
                        # Store the main-menu state in this object's attribute.
                        self.state = self.WAIT_FOR_COMMAND
                        # Call the prompt method to show the available commands.
                        self.print_prompt()

                # Yield regularly even when no input is available.
                yield 0
