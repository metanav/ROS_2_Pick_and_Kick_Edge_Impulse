from ainex_sdk.gpio import *
import digitalio

led = digitalio.DigitalInOut(adapter_board_led)
led.direction = digitalio.Direction.OUTPUT

def on():
    led.value = 1

def off():
    led.value = 0

def set(new_state):
    led.value = new_state

if __name__ == '__main__':
    import time
    while True:
        on()
        time.sleep(0.3)
        off()
        time.sleep(0.3)
