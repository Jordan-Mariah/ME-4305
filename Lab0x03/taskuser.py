'''User interface task
'''
from pyb import USB_VCP

class TaskUser:

    # State id constants
    S0_INIT = 0
    S1_QUERY_USER = 1
    S2_TEST_RIGHT = 2
    S3_TEST_LEFT = 3
    S4_EXIT = 4

    # Other constants
    HELP_SCREEN ='''
+------------------------------------------------------------------------------+
| ME 4305 Romi Tuning Interface Help Menu                                      |
+-----+------------------------------------------------------------------------+
| h/H | Print help menu                                                        |
| l/L | Trigger step response sequence on left motor and print results         |
| r/R | Trigger step response sequence on right motor and print results        |
| e/E | Exit program                                                           |
+-----+------------------------------------------------------------------------+'''


    def __init__(self, right_flag: bool, left_flag: bool) -> None:
        '''Pass right and left motor flags in that order.'''
        self.right_flag = right_flag
        self.left_flag = left_flag

        # Create serial interface
        self.ser = USB_VCP()

        self.state = self.S1_QUERY_USER


    def run(self):
        while True:
            if self.state == self.S0_INIT: #Zombie
                pass

            elif self.state == self.S1_QUERY_USER:
                self.ser.write(self.HELP_SCREEN)

            elif self.state == self.S2_TEST_RIGHT:
                pass

            elif self.state == self.S3_TEST_LEFT:
                pass

            elif self.state == self.S4_EXIT:
                pass

            yield


if __name__ == "__main__":
    