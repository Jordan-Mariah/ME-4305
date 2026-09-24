"""Lab 0x02: construct one task and repeatedly give it a turn."""
from pyb import Pin, Timer
from time import ticks_ms, ticks_diff, sleep_ms
from driver import Motor
from encoder import Encoder
from taskmotor import Task1
#creates the driver and task objects, then repeatedly calls the single task


def main():
    task1 = Task1()

    #immidiately calls init then goes into state diagrams
    try:
        while not task1.done:
            task1.run()
            #pause for 1ms between run cycles
            sleep_ms(1)
    finally:
        task1.stop()


if __name__ == '__main__':
    main()
