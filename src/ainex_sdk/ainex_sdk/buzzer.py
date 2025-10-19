from ainex_sdk.gpio import *
import digitalio

buzzer = digitalio.DigitalInOut(adapter_board_buzzer)
buzzer.direction = digitalio.Direction.OUTPUT

def on():
    buzzer.value = 1

def off():
    buzzer.value = 0
    
def set(new_state):
    buzzer.value = new_state

if __name__ == '__main__':
    import time
    on()
    time.sleep(0.5)
    off()
    time.sleep(0.5)
