# test

from time import ticks_us, ticks_diff, ticks_add

class PID():
    '''Generic object for configurable PID control'''


    def __init__(self, 
        kp: float = 0, 
        ki: float = 0, 
        kd: float = 0, 
        ff_gain: float = 0, 
        out_gain: float = 0, 
        out_bias: float = 0,
        out_saturation: tuple[float, float] | None = None
        ) -> None:
        '''Pass in values for kp, ki, kd, or any other special parameters'''

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ff_gain = ff_gain
        self.out_gain = out_gain
        self.out_bias = out_bias
        self.out_saturation = out_saturation
        self.error_sum = 0
        self.saturated = False

        # Set history values to None to indicate first itteration
        self.error_last = None
        self.time_last = None
        self.input_last = None
        self.out_last = None

    
    def next_output(self, target: float, current: float) -> float:
        '''Evaluate PID cycle. Input target as well as sensor value'''

        out: list[float] = [] #initialization of out
        # Get current state
        self.error_now = target - current
        self.time_now = ticks_us()
        self.input_now = target


        outkp = self.error_now * self.kp
        out.append(outkp)

        # Feed forward gain
        outff = self.input_now * self.ff_gain
        out.append(outff)

        # Only run controls that reference previous values if previous values exist
        if not self.time_last == None and not self.error_last == None and not self.input_last == None and not self.out_last == None:

            dt = ticks_diff(self.time_now, self.time_last)

            # If the last output was saturated, do not accumulate error sum
            if not self.saturated:
                self.error_sum += ((self.error_now + self.error_last) * dt ) / 2 # Using trapazoidal sum to approximate integration
                
            outki = self.error_sum * self.ki
            out.append(outki)

        output = sum(out)

        # Saturate output if set
        if not self.out_saturation == None:
            output = max(output, self.out_saturation[0]) 
            output = min(output, self.out_saturation[1])

            # Set saturation flag for integration
            if output == self.out_saturation[0] or output == self.out_saturation[1]:
                self.saturated = True
            else:
                self.saturated = False

        #add the accumulated error from the interval to the error sum
        self.error_last = self.error_now
        self.input_last = self.input_now
        self.time_last = self.time_now
        self.out_last = output

        return output


    def reset(self):
        self.error_last = None
        self.time_last = None
        self.input_last = None


    def onTarget(self, threshold: float) -> bool:
        if not self.error_last == None:
            return abs(self.error_last) <= abs(threshold)
        else:
            return False
    

    def set_kp(self, kp: float):
        self.kp = kp


    #may or may not use later if dont use then delete ttis func. 
    def set_ki(self, ki: float):
        self.ki = ki


    def set_output_gain(self, output_gain: float):
        self.output_gain = output_gain

    #sets constant output that we can add to the PID output so that we can use 
    # Vadjusted= gain_out(Vcontroller) + bias_out
    def set_output_bias(self, output_bias: float):
        self.output_bias = output_bias


    def set_feed_foward_gain(self, ff_gain: float):
        self.ff_gain = ff_gain