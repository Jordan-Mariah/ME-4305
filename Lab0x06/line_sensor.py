from pyb import Pin, ADC, Timer
from time import ticks_us, ticks_diff, ticks_add
from array import array
from ulab import numpy as np # type: ignore
#import numpy as np # COMMENT THIS OUT BEFORE SENDING TO ROMI!!!!!!!

class LineSensor():
    '''Driver object for the Pololu QTR-HD-13A Reflectance Sensor Array: 13-Channel, 4mm Pitch, Analog Output'''

    # SET CONSTANTS
    OVERSAMPLES: int = 25 


    def __init__(self, pins: tuple[Pin, ...], calib_black: tuple[int, ...] = (), calib_white: tuple[int, ...] = (), reader_timer: Timer | None = None) -> None:
        '''13 ADC capable pins are nessisary. Calibration values are needed for full function'''
         # Generic variable declarations
        self.last_raw: list[int] = []
        self.last_normal: list[float] = []
        self.value_buffer = tuple([ array('H', ( 0 for _ in range(self.OVERSAMPLES) )) for _ in range(13) ])
        self.value_average = array('f', (0 for _ in range(13)))
        self.is_calib = False
        self.has_timer = False

        # Check pin count
        if len(pins) == 13:
            self.pins = tuple([ADC(pin) for pin in pins])
        else:
            raise ValueError("LineSensor must be passed exactly 13 pins")

        # Handle optional inputs

        # Set calibration values and raise internal calibration flag
        if len(calib_black) == 13 and len(calib_white) == 13:
            self.calib_black = calib_black
            self.calib_white = calib_white
            self.calibrate()
            self.is_calib = True

        # Set internal timer and raise internal timer flag
        if isinstance(reader_timer, Timer):
            self.timer = reader_timer
            self.has_timer = True


    def read_raw(self, do_output = False) -> tuple | None:
        '''Read raw unsigned 12 bit (0 - 2048) value from all pins. Set do_output to True to create a unique set of values'''
        self.last_raw = [pin.read() for pin in self.pins]

        # If set to output construct a new tuple. Otherwise save memory
        if do_output:
            return tuple(self.last_raw)
        else:
            return
        

    def read_flat(self, do_output = False) -> tuple | None:
        '''Read all ADC values as floats from 0 to 1. Set do_output to True to create a unique set of values'''
        self.read_raw()
        self.last_normal = [float(val) / 2048 for val in self.last_raw]

        # If set to output construct a new tuple. Otherwise save memory
        if do_output:
            return tuple(self.last_normal)
        else:
            return


    def calibrate(self, calib_black: list = [], calib_white: list = []) -> None:
        '''Re-do calibration or calibrate for first time if not done allready'''

        # Calibrate with internal values
        if (calib_black, calib_white) == ([],[]) and not (self.calib_black, self.calib_white) == ([],[]):
            #TODO impliment calibration
            pass
        elif not (calib_black, calib_white) == ([],[]):
            #TODO impliment calibration
            pass
        else:
            raise ValueError("Line Sensor calibration failed: No calibration values specified or none saved internally")


    def read_average(self) -> array[float]:
        '''Reads values stored in buffer and finds current rolling average pin value.'''
        if not self.has_timer:
            raise TypeError("Line sensor read failed: no timer has been configured")

        # Read values of ADC pins until value_buffer is full. Then take it's average.
        if not ADC.read_timed_multi(self.pins, self.value_buffer, self.timer):
            raise RuntimeError("ADC acquisition overrun")
        for i, buffer in enumerate(self.value_buffer):
            self.value_average[i] = np.mean(buffer)
        return self.value_average
