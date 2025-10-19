import time
import board
import neopixel
import digitalio 

pixels = neopixel.NeoPixel(adapter_board_rgb, 1) 
pixels.fill((0, 0, 0))
pixels.show()

def set_color(r, g, b):
    pixels.fill((r, g, b))
    pixels.show()

if __name__ == '__main__':
    while True:
        try:
            set_color(255, 0, 0)
            time.sleep(0.3)
            set_color(0, 255, 0)
            time.sleep(0.3)
            set_color(0, 0, 255)
            time.sleep(0.3)
        except KeyboardInterrupt:
            set_color(0, 0, 0)
            break
