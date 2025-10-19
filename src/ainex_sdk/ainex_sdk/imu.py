#!/usr/bin/env python
import time
from icm20948 import ICM20948

while True:
    try:
        imu = ICM20948()
        print('imu init finish!')
        break
    except:
        time.sleep(0.01)

def get_data():
    return imu.read_magnetometer_data(), imu.read_accelerometer_gyro_data()

if __name__ == '__main__':
    while True:
        try:
            (x, y, z), (ax, ay, az, gx, gy, gz) = get_data()
            print("""
    Accel: {:05.2f} {:05.2f} {:05.2f}
    Gyro:  {:05.2f} {:05.2f} {:05.2f}
    Mag:   {:05.2f} {:05.2f} {:05.2f}""".format(
                ax, ay, az, gx, gy, gz, x, y, z
                ))
            time.sleep(0.01)
        except KeyboardInterrupt:
            break
