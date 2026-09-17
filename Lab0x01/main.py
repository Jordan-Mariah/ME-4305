from pyb import Pin, Timer
from time import ticks_ms, ticks_diff, sleep_ms
from driver import Motor
from encoder import Encoder


def main():
    # Pins match the earlier Lab 1 motor/encoder test. Lift both wheels
    # clear of the bench before running this automatic motion sequence.
    pwm_timer_left = Timer(4, freq=20_000)
    pwm_timer_right = Timer(1, freq=20_000)
    #connect the PWM of the left motor to chanell 1 of timer 4. Then mode to PWM, pin to B6, and initial pulse width to 0
    pwm_left = pwm_timer_left.channel(
        1, mode=Timer.PWM, pin=Pin.cpu.B6, pulse_width_percent=0)
    #connect the PWM of the right motor to chanell 1 of timer 1. Then mode to PWM, pin to A8, and initial pulse width to 0
    pwm_right = pwm_timer_right.channel(
        1, mode=Timer.PWM, pin=Pin.cpu.A8, pulse_width_percent=0)

    #create a motor object and assign it to the left motor with pins for PWM, DIR, and nSLP
    left_motor = Motor(pwm_left, Pin.cpu.B5, Pin.cpu.A10)
    #create a motor object and assign it to the right motor with pins for PWM, DIR, and nSLP
    right_motor = Motor(pwm_right, Pin.cpu.A9, Pin.cpu.B4)

    # Set up both encoders and monitor them before testing.
    try:
        #hardware timers to count the econder moving - these are different from the PWM timers
        #the counters range from 0-65535 and then wrap around
        left_timer = Timer(2, prescaler=0, period=65535)
        right_timer = Timer(3, prescaler=0, period=65535)
        #create left and right encoder ojbects using the encoder class from encoder.py
        left_encoder = Encoder(left_timer, Pin.cpu.A0, Pin.cpu.A1)
        right_encoder = Encoder(right_timer, Pin.cpu.A6, Pin.cpu.A7)

        #create a samplising function to later use to monitor the encoders for the number of miliseconds
        def sample_for(duration_ms):
            # Update approximately every 10 ms and print every 500ms.
            # and continue sampling for stopping and disabled phases.
            #start is the count of the current clock in ms
            start = ticks_ms()
            last_print = start
            #durring the sample duration sample every 10ms and print every 500ms
            while ticks_diff(ticks_ms(), start) < duration_ms:
                left_encoder.update()
                right_encoder.update()
                now = ticks_ms()
                #if the time since the last print is more than 500 ms
                if ticks_diff(now, last_print) >= 500:
                    last_print = now
                    #every 500 ms print the velocity and position of the encoder
                    print('{},{},{},{},{},{}'.format(
                        left_encoder.get_position(),
                        left_encoder.get_velocity(),
                        right_encoder.get_position(),
                        right_encoder.get_velocity(),
                        left_timer.counter(), right_timer.counter()))
                sleep_ms(10)

        print('Lift wheels clear. Starting in 3 seconds.')
        print('L_counts,L_counts/s,R_counts,R_counts/s,L_raw,R_raw')
        sample_for(3000)

        def test_motor(motor):
            # Enabling should not start the wheel moving.
            print('Enabling: wheel should remain stationary')
            motor.enable()
            sample_for(1000)

            # Reset before EACH effort so reverse can show negative position.
            for effort in (25, 50, -25, -50):
                left_encoder.zero()
                right_encoder.zero()
                print('Effort:', effort)
                motor.set_effort(effort)
                sample_for(2000)

                motor.set_effort(0)
                sample_for(1000)

            # Disable while a nonzero effort is commanded.
            print('Run at 30%, then disable')
            motor.set_effort(30)
            sample_for(1000)
            motor.disable()
            sample_for(2000)

            # Re-enabling should clear the previous effort.
            print('Re-enable: wheel should remain stationary')
            motor.enable()
            sample_for(1000)
            motor.disable()

        def test_rollover(name, motor, encoder, timer):
            # The other motor must remain disabled during this test.
            motor.enable()
            sample_for(1000)
            count_range = timer.period() + 1

            for effort, initial_count in ((-40, count_range - 501), (40, 500)):
                # Seed the stopped counter near overflow, then underflow.
                timer.counter(initial_count)
                encoder.zero()
                previous = (encoder.prev_count, 0, 0.0)
                crossing_rows = []
                print(name, 'rollover test, effort:', effort)
                motor.set_effort(effort)
                start = ticks_ms()

                while ticks_diff(ticks_ms(), start) < 2000:
                    sleep_ms(10)
                    left_encoder.update()
                    right_encoder.update()
                    # Use the exact raw count sampled by update().
                    current = (
                        encoder.prev_count,
                        encoder.get_position(),
                        encoder.get_velocity(),
                    )
                    #print(current)
                    if not crossing_rows:
                        if abs(current[0] - previous[0]) > count_range / 2:
                            crossing_rows = [previous, current]
                    elif len(crossing_rows) == 2:
                        crossing_rows.append(current)
                    previous = current

                motor.set_effort(0)
                sample_for(1000)
                # Print the saved readings after stopping so printing does
                # not delay sampling at the boundary crossing.
                if crossing_rows:
                    print('Boundary crossing recorded: raw,position,counts/s')
                    for row in crossing_rows:
                        print('{},{},{}'.format(*row))
                    print('Check for continuous position and no rollover spike.')
                else:
                    print('NOT VERIFIED: no crossing recorded; check motion/sign.')

            motor.disable()

        # Test the left motor independently.
        print('\n--- LEFT MOTOR TEST ---')
        right_motor.disable()
        test_motor(left_motor)

        # Test the right motor independently.
        print('\n--- RIGHT MOTOR TEST ---')
        left_motor.disable()
        test_motor(right_motor)

        # Test overflow and underflow separately for each wheel.
        print('\n--- ROLLOVER TESTS ---')
        right_motor.disable()
        test_rollover('LEFT', left_motor, left_encoder, left_timer)
        left_motor.disable()
        test_rollover('RIGHT', right_motor, right_encoder, right_timer)

    finally:
        # Disable both motors on completion, Ctrl+C, or an exception.
        left_motor.disable()
        right_motor.disable()
        left_motor.set_effort(0)
        right_motor.set_effort(0)


if __name__ == '__main__':
    main()
