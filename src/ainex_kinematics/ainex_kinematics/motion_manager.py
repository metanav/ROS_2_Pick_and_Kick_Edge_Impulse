import os
import time
import sqlite3 as sql
from ainex_sdk import hiwonder_servo_controller

class MotionManager:
    runningAction = False
    stopRunning = False
    
    #def __init__(self, action_path, serial_port='/dev/ttyAMA0', baudrate=115200):
    def __init__(self, action_path, serial_port='/dev/ttyHS2', baudrate=115200):
        self.servo_control = hiwonder_servo_controller.HiwonderServoController(serial_port, baudrate)
        self.action_path = action_path

    def set_servos_position(self, duration, *args):
        self.servo_control.set_servos_position(duration, args)

    def get_servos_position(self, *args):
        return self.servo_control.get_servos_position(args)

    def stop_action_group(self):
        self.stopRunning = True

    def run_action(self, action_name):
        if action_name is None and self.action_path is not None:
            return

        actNum = os.path.join(self.action_path, action_name + ".d6a")
        self.stopRunning = False

        if os.path.exists(actNum):
            if not self.runningAction:
                self.runningAction = True
                ag = sql.connect(actNum)
                cu = ag.cursor()
                cu.execute("select * from ActionGroup")

                while True:
                    act = cu.fetchone()
                    if self.stopRunning:
                        self.stopRunning = False                   
                        break
                    if act is not None:
                        data = []
                        for i in range(0, len(act) - 2, 1):
                            data.append([i + 1, act[2 + i]])
                        self.set_servos_position(act[1], data) 
                        time.sleep(float(act[1])/1000.0)
                    else:   # 运行完才退出
                        break
                self.runningAction = False
                
                cu.close()
                ag.close()
        else:
            self.runningAction = False
            raise ValueError(f'Invalid action name: {action_name}')

