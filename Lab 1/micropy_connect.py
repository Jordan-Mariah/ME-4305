from pyb import ExtInt, Pin

PA5 = Pin(Pin.cpu.A5, mode=Pin.OUT_PP)

def button_LED_toggle(the_pin):
    if PA5.value():
        PA5.value(0) # or PA5.low()
    else:
        PA5.value(1) # or PA5.high()

button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING,
                    Pin.PULL_NONE, button_LED_toggle)