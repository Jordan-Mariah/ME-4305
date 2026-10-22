# test

from time import ticks_us, ticks_diff, ticks_add

class PID():
    '''Generic object for configurable PID control'''

    #parameter types defined here for clarity

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

        # Catch all other arguments
        for arg, val in kwargs.items():
            setattr(self, arg, val)

        # Make sure at least one parameter is set
        self._invalid = False
        if (self.kp, self.ki, self.kd) == (None, None, None):
            self._invalid = True
            raise ValueError(f"{self}: No input for kp, ki, or kd")

        # Set history values to -1 to indicate no run yet
        self.error_last = -1

    
    def next_output(self, target: float, current: float) -> float:
        '''Evaluate PID cycle. Input target as well as sensor value'''

        if self._invalid:
            raise RuntimeError(f"{self}: PID initialization failed")

        out: list[float] = []#initialization of out
        dt: float
        # Get error
        self.error_now = target - current

        # find the elapssed time, set dt to zero for the first call after initializing
        if not self.error_last == -1:
            dt = ticks_diff(ticks_us(), self.time_last)
        else:
            dt = 0
            self.error_last = self.error_now
            self.error_sum = 0
        self.time_last = ticks_us()

        # when we have Kp enabled then get P=Kp(e)where P is proportional control
        if not self.kp == None:
            outkp = self.error_now * self.kp
            out.append(outkp)

        #calc. integral contribution (Save this for later if you add in integral control)
        if not self.ki == None:
            self.error_sum += ((self.error_now + self.error_last) * dt ) / 2 # Using trapazoidal sum to approximate integration
            outki = self.error_sum * self.ki
            out.append(outki)

        # Feed forward gain
        if not self.ff_gain == None:
            outff = target * self.ff_gain
            out.append(outff)

        #add the accumulated error from the interval to the error sum
        self.error_last = self.error_now

        # print(self.onTarget(0.1))
        output = sum(out)

        return output


    def reset(self):
        self.error_last = -1

    def onTarget(self, threshold: float) -> bool:
        return abs(self.error_last) <= abs(threshold)
    

    def set_kp(self, kp: float):
        if not isinstance(kp, float):
            raise TypeError(f"{self}: kp is not type 'float'")
        self.kp = kp

    #may or may not use later if dont use then delete ttis func. 
    def set_ki(self, ki: float):
        if not isinstance(ki, float):
            raise TypeError(f"{self}: ki is not type 'float'")
        self.ki = ki


    def set_output_gain(self, output_gain: float):
        if not isinstance(output_gain, float):
            raise TypeError(f"{self}: output_gain is not type 'float'")
        self.output_gain = output_gain

    #sets constant output that we can add to the PID output so that we can use 
    # Vadjusted= gain_out(Vcontroller) + bias_out
    def set_output_bias(self, output_bias: float):
        if not isinstance(output_bias, float):
            raise TypeError(f"{self}: output_bias is not type 'float'")
        self.output_bias = output_bias


    def set_feed_foward_gain(self, ff_gain: float):
        if not isinstance(ff_gain, float):
            raise TypeError(f"{self}: ff_gain is not type 'float'")
        self.ff_gain = ff_gain