import time
import threading
import subprocess
from ainex_sdk import button, buzzer, led, imu

def led_check():
    while True:
        led.on()
        time.sleep(0.1)
        led.off()
        time.sleep(0.1)

subprocess.Popen('sudo python3 rgb.py', shell=True)
led_thread = threading.Thread(target=led_check, daemon=True).start()
button_count = 0
button_status = 'off'
while True:
    try:
        (x, y, z), (ax, ay, az, gx, gy, gz) = imu.get_data()
        print("""
Accel: {:05.2f} {:05.2f} {:05.2f}
Gyro:  {:05.2f} {:05.2f} {:05.2f}
Mag:   {:05.2f} {:05.2f} {:05.2f}""".format(
            ax, ay, az, gx, gy, gz, x, y, z
            ))
        button_off = button.get_button_status()
        if not button_off and button_status == 'off':
            button_count = 0
            buzzer.on()
            time.sleep(0.1)
            buzzer.off()
            button_status = 'on'
        elif button_off:
            button_count += 1
            if button_count > 5:
                button_status = 'off'
                button_count = 0
    except KeyboardInterrupt:
        led.off()
        break
