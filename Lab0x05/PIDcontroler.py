# test

from time import ticks_us, ticks_diff, ticks_add

class PID():
    '''Generic object for configurable PID control'''

    # Define parameter types

    kp: float | None
    ki: float | None
    kd: float | None

    input_now: float
    input_last: float

    error_now: float
    error_last: float
    error_sum: float

    output_gain: float
    output_bias: float

    def __init__(self, kp = None, ki = None, kd = None, **kwargs) -> None:
        '''Pass in values for kp, ki, kd, or any other special parameters'''

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

        for arg, val in kwargs:
            setattr(self, arg, val)

        # Make sure at least one parameter is set
        if (self.kp, self.ki, self.kd) == (None, None, None):
            raise ValueError(f"{self}: No input for kp, ki, or kd")
            #TODO: Prevent further use of this PID if error is caught and ignored.

    
    def next_output(self, target: float, current: float) -> float:
        '''Evaluate PID cycle. Input target as well as sensor value'''

        out: list[float] = []
        dt: float
        # Get error
        self.error_now = target - current

        # Get dt (and check if first run)
        if self.time_last in locals():
            dt = ticks_diff(ticks_us() ,self.time_last)
        else:
            dt = 0
            self.error_last = self.error_now
            self.error_sum = 0
        self.time_last = ticks_us()

        # Evaluate PID gains
        if not self.kp == None:
            outkp = self.error_now * self.kp
            out.append(outkp)

        if not self.ki == None:
            self.error_sum += ((self.error_now + self.error_last) * dt ) / 2 # Using trapazoidal sum to approximate integration
            outki = self.error_sum * self.ki
            out.append(outki)

        if not self.kd == None and dt > 0:
            outkd = self.kd * (self.error_now - self.error_last) / dt #TODO confirm whether to use error or input delta
            out.append(outkd)

        self.error_last = self.error_now

        output = sum(out)

        # Use output post-processors if specified
        output *= self.output_gain if not self.output_gain == None else output
        output += self.output_bias if not self.output_bias == None else output
        
        return output


    def reset(self):
        self.error_last = 0
        del(self.time_last)

    def onTarget(self, threshold: float) -> bool:
        return abs(self.error_last) <= abs(threshold)
    

    def set_kp(self, kp: float):
        if not isinstance(kp, float):
            raise TypeError(f"{self}: kp is not type 'float'")
        self.kp = kp


    def set_ki(self, ki: float):
        if not isinstance(ki, float):
            raise TypeError(f"{self}: ki is not type 'float'")
        self.ki = ki


    def set_kd(self, kd: float):
        if not isinstance(kd, float):
            raise TypeError(f"{self}: kd is not type 'float'")
        self.kd = kd


    def set_output_gain(self, output_gain: float):
        if not isinstance(output_gain, float):
            raise TypeError(f"{self}: output_gain is not type 'float'")
        self.output_gain = output_gain


    def set_output_bias(self, output_bias: float):
        if not isinstance(output_bias, float):
            raise TypeError(f"{self}: output_bias is not type 'float'")
        self.output_bias = output_bias
