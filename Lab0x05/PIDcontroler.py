'''Generic object for configurable PID control'''

class PID():

    # Define parameter types
    input_signal: float
    feetback_signal: float
    output_signal: float

    error_now: float
    error_last: float
    error_sum: float

    kp: float | None
    ki: float | None
    kd: float | None

    def __init__(self, kp = None, ki = None, kd = None, **kwargs) -> None:
        # Check passed parameters and set object variables
        if not kp == None:
            if isinstance(kp, float):
                self.kp = kp
            else:
                raise TypeError(f"{self}: kp is not type 'float'")
        else:
            self.kp = None

        if not ki == None:
            if isinstance(ki, float):
                self.ki = ki
            else:
                raise TypeError(f"{self}: ki is not type 'float'")
        else:
            self.ki = None

        if not kd == None:
            if isinstance(kd, float):
                self.kd = kd
            else:
                raise TypeError(f"{self}: kd is not type 'float'")
        else:
            self.kd = None

        # Make sure at least one parameter is set
        if (self.kp, self.ki, self.kd) == (None, None, None):
            raise ValueError(f"{self}: No input for kp, ki, or kd")
            #TODO: Prevent further use of this PID if error is caught and ignored.

    


