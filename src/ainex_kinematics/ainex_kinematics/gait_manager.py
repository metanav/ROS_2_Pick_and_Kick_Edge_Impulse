import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ainex_interfaces.msg import WalkingParam
from rclpy.duration import Duration
from ainex_interfaces.srv import GetWalkingParam, SetWalkingCommand

class GaitManager:
    def __init__(self, node: Node):
        self.node = node
        self.state = 'enable'
        self.is_walking = False
        self.err = 1e-8
        self.dsp_ratio = [
            [300, 0.2, 0.02],
            [400, 0.2, 0.02],
            [500, 0.2, 0.02],
            [600, 0.1, 0.04]
        ]

        self.body_height_range = [0.015, 0.06] 
        self.x_amplitude_range = [0.0, 0.02]
        self.y_amplitude_range = [0.0, 0.02]
        self.step_height_range = [0.01, 0.04]
        self.rotation_angle_range = [0, 10]
        self.arm_swap_range = [0, 60]
        self.y_swap_range = [0, 0.05]
        self.dsp_ratio_range = [0, 1]

        self.walking_param = None

        self.get_walking_param_client = self.node.create_client(GetWalkingParam, '/walking/get_param')
        self.walking_command_client = self.node.create_client(SetWalkingCommand, '/walking/command')
        self.param_pub = self.node.create_publisher(WalkingParam, '/walking/set_param', 1)
        self.node.create_subscription(Bool, '/walking/is_walking',  self.walking_state_callback, 1)

        self.walking_param = self.get_walking_param()
        self.node.get_clock().sleep_for(Duration(seconds=0.1))
        self.node.get_logger().info("gait manager: initialized.")

    def get_walking_param(self):
        walking_param = None

        self.get_walking_param_client.wait_for_service()
        future = self.get_walking_param_client.call_async(GetWalkingParam.Request())
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            walking_param = future.result().parameters

        return walking_param

    def walking_command(self, command):
        #self.node.get_logger().info(f"walking_command({command})")
        self.walking_command_client.wait_for_service() 
        request = SetWalkingCommand.Request()
        request.command = command
        future = self.walking_command_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        #self.node.get_logger().info(f"walking_command({command}) OK")

    def walking_state_callback(self, msg):
        self.is_walking = msg.data

    def get_gait_param(self):
        walking_param = {}
        walking_param['init_x_offset']        = self.walking_param.init_x_offset
        walking_param['init_y_offset']        = self.walking_param.init_y_offset
        walking_param['body_height']          = self.walking_param.init_z_offset
        walking_param['init_roll_offset']     = self.walking_param.init_roll_offset
        walking_param['init_pitch_offset']    = self.walking_param.init_pitch_offset
        walking_param['init_yaw_offset']      = self.walking_param.init_yaw_offset
        walking_param['hip_pitch_offset']     = self.walking_param.hip_pitch_offset
        walking_param['step_fb_ratio']        = self.walking_param.step_fb_ratio
        walking_param['step_height']          = self.walking_param.z_move_amplitude
        walking_param['z_swap_amplitude']     = self.walking_param.z_swap_amplitude
        walking_param['pelvis_offset']        = float(self.walking_param.pelvis_offset)
        walking_param['move_aim_on']          = self.walking_param.move_aim_on
        walking_param['angle_move_amplitude'] = self.walking_param.angle_move_amplitude

        return walking_param

    def update_pose(self, walking_param):
        if self.state == 'disable':
            self.state = 'enable'
            self.walking_command('enable')

        if walking_param['body_height'] - self.body_height_range[1] > self.err or walking_param['body_height'] - self.body_height_range[0] < -self.err:
            raise Exception('body_height %d out of range(0.015~0.06)' % walking_param['body_height'])
        if walking_param['step_height'] - self.step_height_range[1] > self.err or walking_param['step_height'] - self.step_height_range[0] < -self.err:
            raise Exception('step_height %d out of range(0.01~0.04)' % walking_param['step_height'])

        self.walking_param.init_x_offset = walking_param['init_x_offset']
        self.walking_param.init_y_offset = walking_param['init_y_offset']
        self.walking_param.init_z_offset = walking_param['body_height']
        self.walking_param.init_roll_offset = walking_param['init_roll_offset']
        self.walking_param.init_pitch_offset = walking_param['init_pitch_offset']
        self.walking_param.init_yaw_offset = walking_param['init_yaw_offset']
        self.walking_param.hip_pitch_offset = walking_param['hip_pitch_offset']
        self.walking_param.step_fb_ratio = walking_param['step_fb_ratio']
        self.walking_param.z_move_amplitude = walking_param['step_height']
        self.walking_param.angle_move_amplitude = walking_param['angle_move_amplitude']
        self.walking_param.z_swap_amplitude = walking_param['z_swap_amplitude']
        self.walking_param.pelvis_offset = float(walking_param['pelvis_offset'])
        self.walking_param.move_aim_on = walking_param['move_aim_on']

        self.walking_param.x_move_amplitude = 0.0
        self.walking_param.y_move_amplitude = 0.0
        self.walking_param.angle_move_amplitude = 0.0

        self.param_pub.publish(self.walking_param)

    def update_param(self, step_velocity, x_amplitude, y_amplitude, rotation_angle, walking_param=None, arm_swap=30, step_num=0):
        if step_velocity[0] < 0:
            raise Exception('period_time cannot be negative' % step_velocity[0])

        if step_velocity[1] > self.dsp_ratio_range[1] or step_velocity[1] < self.dsp_ratio_range[0]:
            raise Exception('dsp_ratio_range %d out of range(0~1)' % step_velocity[1])

        if step_velocity[2] - self.y_swap_range[1] > self.err or step_velocity[2] < self.y_swap_range[0]:
            raise Exception('y_swap_range %d out of range(0~0.05)' % step_velocity[2])

        if abs(x_amplitude) - self.x_amplitude_range[1] > self.err or abs(x_amplitude) < self.x_amplitude_range[0]:
            raise Exception('x_amplitude %d out of range(-0.02~0.02)' % x_amplitude)

        if abs(y_amplitude) - self.y_amplitude_range[1] > self.err or abs(y_amplitude) < self.y_amplitude_range[0]:
            raise Exception('y_amplitude %d out of range(-0.02~0.02)' % y_amplitude)

        if abs(rotation_angle) - self.rotation_angle_range[1] > self.err or abs(rotation_angle) < self.rotation_angle_range[0]:
            raise Exception('rotation_angle %d out of range(-10~10)' % rotation_angle)

        if abs(arm_swap) - self.arm_swap_range[1] > self.err or arm_swap < self.arm_swap_range[0]:
            raise Exception('arm_swap %d out of range(0~60)' % arm_swap)

        if step_num < 0:
            raise Exception('step_num cannot be negative' % step_num)

        if walking_param is not None:
            if walking_param['body_height'] - self.body_height_range[1] > self.err or walking_param['body_height'] - self.body_height_range[0] < -self.err:
                raise Exception('body_height %d out of range(0.015~0.06)' % walking_param['body_height'])
            if walking_param['step_height'] - self.step_height_range[1] > self.err or walking_param['step_height'] - self.step_height_range[0] < -self.err:
                raise Exception('step_height %d out of range(0.01~0.04)' % walking_param['step_height'])

            self.walking_param.init_x_offset = walking_param['init_x_offset']
            self.walking_param.init_y_offset = walking_param['init_y_offset']
            self.walking_param.init_z_offset = walking_param['body_height']
            self.walking_param.init_roll_offset = walking_param['init_roll_offset']
            self.walking_param.init_pitch_offset = walking_param['init_pitch_offset']
            self.walking_param.init_yaw_offset = walking_param['init_yaw_offset']
            self.walking_param.hip_pitch_offset = walking_param['hip_pitch_offset']
            self.walking_param.step_fb_ratio = walking_param['step_fb_ratio']
            self.walking_param.z_move_amplitude = walking_param['step_height']
            self.walking_param.angle_move_amplitude = walking_param['angle_move_amplitude']
            self.walking_param.z_swap_amplitude = walking_param['z_swap_amplitude']
            self.walking_param.pelvis_offset = float(walking_param['pelvis_offset'])
            self.walking_param.move_aim_on = walking_param['move_aim_on']

        self.walking_param.period_time = float(step_velocity[0])
        self.walking_param.dsp_ratio = step_velocity[1]
        self.walking_param.y_swap_amplitude = float(step_velocity[2])
        self.walking_param.x_move_amplitude = float(x_amplitude)
        self.walking_param.y_move_amplitude = float(y_amplitude)
        self.walking_param.angle_move_amplitude = float(rotation_angle)
        self.walking_param.arm_swing_gain = math.radians(arm_swap)
        self.walking_param.period_times = step_num
        self.param_pub.publish(self.walking_param)

    def set_body_height(self, body_height, use_time):
        if self.state == 'disable':
            self.state = 'enable'
            self.walking_command('enable')
        times = int(abs(body_height - self.walking_param.init_z_offset)/0.005)
        for i in range(times):
            self.walking_param.init_z_offset += math.copysign(0.005, body_height - self.walking_param.init_z_offset)
            self.param_pub.publish(self.walking_param)
            self.node.get_clock().sleep_for(Duration(seconds=0.2))

    def set_step(self, step_velocity, x_amplitude, y_amplitude, rotation_angle, walking_param=None, arm_swap=30, step_num=0):
        try:
            #self.node.get_logger().info("update_param")
            self.update_param(step_velocity, x_amplitude, y_amplitude, rotation_angle, walking_param, arm_swap, step_num)
            #self.node.get_logger().info("update_param OK")

            if step_num != 0:
                #self.node.get_logger().info("calling get_walking_param")
                self.walking_param = self.get_walking_param()
                #self.node.get_logger().info("calling walking_command")
                self.start()

                #self.node.get_logger().info("OK 1")
                while not self.is_walking:
                    self.node.get_clock().sleep_for(Duration(seconds=0.01))

                #self.node.get_logger().info("OK 2")
                while self.is_walking:
                    self.node.get_clock().sleep_for(Duration(seconds=0.01))
                #self.node.get_logger().info("OK 3")
            else:
                #self.node.get_logger().info(f"state = {self.state} step_num = {step_num}")
                if self.state != 'walking':
                    if step_num == 0:
                        #self.node.get_logger().info("OK 5")
                        self.walking_param = self.get_walking_param()
                        self.start()
                        #self.node.get_logger().info("OK 6")
                        self.state = 'walking'
        except BaseException as e:
            self.node.get_logger().info(f"{e}")
            return

    def move(self, step_velocity, x_amplitude, y_amplitude, rotation_angle, arm_swap=30, step_num=0):
        if 0 < step_velocity < 5:
            self.set_step(self.dsp_ratio[step_velocity - 1], x_amplitude, y_amplitude, rotation_angle, self.get_gait_param(), arm_swap, step_num)

    def start(self):
        self.walking_command('start')
        self.state ='start'

    def stop(self):
        self.walking_command('stop')
        self.state ='stop'

    def disable(self):
        self.walking_command('disable')
        self.state = 'disable'

    def enable(self):
        self.walking_command('enable')
        self.state = 'enable'

