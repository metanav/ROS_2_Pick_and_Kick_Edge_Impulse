import time
import copy
import math
import signal
import rclpy
import yaml
from rclpy.node import Node
from ainex_sdk import common
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from std_msgs.msg import Float64, String, Bool, Int32MultiArray, MultiArrayDimension
from ainex_kinematics.motion_manager import MotionManager
from ainex_interfaces.msg import WalkingParam, AppWalkingParam, HeadState
from rclpy.duration import Duration
from ainex_kinematics.kinematics import LegIK
from ainex_kinematics.walking_module import WalkingModule
from ainex_interfaces.srv import SetWalkingParam, GetWalkingParam, SetWalkingCommand, GetWalkingState, SetServosPosition, GetServosPosition, RunAction

class JointPosition:
    def __init__(self):
        self.present_position = 0.0
        self.goal_position = 0.0

class ControllerNode(Node):
    RADIANS_PER_ENCODER_TICK = 240 * 3.1415926 / 180 / 1000
    ENCODER_TICKS_PER_RADIAN = 180 / 3.1415926 / 240 * 1000

    joint_index = {'r_hip_yaw':   0,
                   'r_hip_roll':  1,
                   'r_hip_pitch': 2,
                   'r_knee':      3,
                   'r_ank_pitch': 4,
                   'r_ank_roll':  5,
                   'l_hip_yaw':   6,
                   'l_hip_roll':  7,
                   'l_hip_pitch': 8,
                   'l_knee':      9,
                   'l_ank_pitch': 10,
                   'l_ank_roll':  11,
                   'r_sho_pitch': 12,
                   'l_sho_pitch': 13}
    
    joint_id = {'r_hip_yaw':   12,
                'r_hip_roll':  10,
                'r_hip_pitch': 8,
                'r_knee':      6,
                'r_ank_pitch': 4,
                'r_ank_roll':  2,
                'l_hip_yaw':   11,
                'l_hip_roll':  9,
                'l_hip_pitch': 7,
                'l_knee':      5,
                'l_ank_pitch': 3,
                'l_ank_roll':  1,
                'r_sho_pitch': 14,
                'l_sho_pitch': 13,
                'l_sho_roll':  15,
                'r_sho_roll':  16,
                'l_el_pitch':  17,
                'r_el_pitch':  18,
                'l_el_yaw':    19,
                'r_el_yaw':    20,
                'l_gripper':   21,
                'r_gripper':   22,
                'head_pan':    23,
                'head_tilt':   24}

    joint_name = {value: key for key, value in joint_id.items()}

    body_height_range = [0.015, 0.06] 
    x_amplitude_range = [-0.05, 0.05]
    y_amplitude_range = [-0.05, 0.05]
    step_height_range = [0.0, 0.05]
    angle_amplitude_range = [-10, 10]
    arm_swap_range = [0, math.radians(60)]

    y_swap_range = [0, 0.05]

    def __init__(self):
        super().__init__('controller_node')

        self.init_pose_finish = True
        self.joint_angles_convert_coef = {}
        
        self.declare_parameter('servo_params_file', '')
        servo_params_file = self.get_parameter('servo_params_file').get_parameter_value().string_value
        with open(servo_params_file, 'r', encoding='utf-8') as f:
          params  = yaml.safe_load(f)

        for ctl_name, ctl_params in params['controllers'].items():
            if ctl_params['type'] == 'JointPositionController':
                initial_position_raw = ctl_params['servo']['init']
                min_angle_raw = ctl_params['servo']['min']
                max_angle_raw = ctl_params['servo']['max']
                flipped = min_angle_raw > max_angle_raw

                if flipped:
                    self.joint_angles_convert_coef[ctl_params['servo']['id']] = [initial_position_raw, -self.ENCODER_TICKS_PER_RADIAN]
                else:
                    self.joint_angles_convert_coef[ctl_params['servo']['id']] = [initial_position_raw, self.ENCODER_TICKS_PER_RADIAN]

        
        self.declare_parameter('init_pose_params_file', '')
        init_pose_params_file = self.get_parameter('init_pose_params_file').get_parameter_value().string_value

        with open(init_pose_params_file, 'r', encoding='utf-8') as f:
          init_pose_params  = yaml.safe_load(f)

        self.init_pose_data = init_pose_params['init_pose']
        self.init_servo_data = []

        for joint_name in self.init_pose_data:
            id_ = self.joint_id[joint_name]
            angle = self.init_pose_data[joint_name]
            pulse = self.angle2pulse(id_, angle)
            self.init_servo_data.extend([[id_, pulse]])

        self.declare_parameter('action_group_path', '')  
        self.action_group_path = self.get_parameter('action_group_path').get_parameter_value().string_value

        self.motion_manager = MotionManager(self.action_group_path)
        self.motion_manager.set_servos_position(1000, self.init_servo_data)
        self.sleep(1)


        self.walking_enable = True
        self.walk_finish = False
        self.count_step = 0
        self.stop = False

        self.joint_position_pub = {}
        self.present_joint_state = {}
        self.all_joint_position_pub = {}

        for joint_name in self.joint_index:
            self.present_joint_state[joint_name] = JointPosition()
            self.present_joint_state[joint_name].present_position = self.init_pose_data[joint_name]

        self.init_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              math.radians(0.0), math.radians(-0.0)]

        self.declare_parameter('walking_params_file', '')
        self.walking_params_file = self.get_parameter('walking_params_file').get_parameter_value().string_value

        self.ik = LegIK()
        with open(self.walking_params_file, 'r', encoding='utf-8') as f:
            self.walking_param  = yaml.safe_load(f)

        self.walking_module = WalkingModule(self.ik, self.walking_param, self.joint_index, self.init_position, self.walking_param['trajectory_step_s'])
 
        self.walking_param = self.walking_module.get_walking_param()

        self.create_subscription(HeadState, '/head_pan_controller/command',  self.head_pan_controller_callback, 5)
        self.create_subscription(HeadState, '/head_tilt_controller/command',  self.head_tilt_controller_callback, 5)
        self.create_subscription(WalkingParam, '/walking/set_param', self.set_walking_param_callback, 5)
        self.create_subscription(AppWalkingParam, '/app/set_walking_param', self.set_app_walking_param_callback, 1)
        self.create_subscription(String, '/app/set_action', self.set_action_callback, 1)

        self.create_service(Empty, '/walking/init_pose', self.init_pose_callback)
        self.create_service(SetWalkingCommand, '/walking/command', self.walking_command_callback)
        self.create_service(GetWalkingParam, '/walking/get_param', self.get_walking_param_callback)
        #self.create_service(GetWalkingState, '/walking/is_walking', self.is_walking_callback)

        # Create more services
        self.create_service(SetServosPosition, 'set_servos_position', self.set_servos_position_callback)
        self.create_service(GetServosPosition, 'get_servos_position', self.get_servos_position_callback)
        self.create_service(RunAction, 'run_action', self.run_action_callback)

        self.walk_state_pub = self.create_publisher(Bool, '/walking/is_walking', 1)
        
        self.last_position = None
        self.err = 1e-8
        self.stop_stamp = self.walking_param['trajectory_step_s'] 
        self.servo_control_cycle = self.walking_param['servo_control_cycle']
        self.servo_max_move = 0
        self.max_stop_move = 0.24
        self.speed = 0.2/math.radians(60)  # 舵机最大速度0.2s/60deg
        self.sleep(0.2)
        
        self.get_logger().info('Kinematics controller initialized.')
        self.timer = self.create_timer(self.walking_param['servo_control_cycle'], self.run)

    def sleep(self, seconds):
        self.get_clock().sleep_for(Duration(seconds=seconds))

    def set_servos_position_callback(self, request, response):
        try:
            duration = request.duration
            dims = request.positions.layout.dim
            if len(dims) != 2:
                response.success = False
                response.message = 'Invalid dimensions. Expected 2D array [[id, position], ...].'
                return response

            num_servos = dims[0].size  # Number of servo entries
            pair_size = dims[1].size   # Size of each [id, position] pair (should be 2)
            stride = dims[1].stride    # Stride for each pair

            if pair_size != 2:
                response.success = False
                response.message = 'Invalid pair size. Expected [id, position] pairs.'
                return response

            # Extract servo positions from flattened data
            servo_positions = []
            data = request.positions.data
            for i in range(num_servos):
                start_idx = i * stride
                servo_id = data[start_idx]
                position = data[start_idx + 1]
                servo_positions.append([servo_id, position])

             
            self.motion_manager.set_servos_position(duration, servo_positions)
            #self.get_logger().info(f'Setting servo positions {servo_positions} with {duration} ms.')

            response.success = True
            response.message = f'Successfully set {num_servos} servo positions.'
        except Exception as e:
            response.success = False
            response.message = f'Error setting servo positions: {str(e)}'
        
        return response

    def get_servos_position_callback(self, request, response):
        try:
            servo_ids = request.servo_ids
            servo_positions = self.motion_manager.get_servos_position(*servo_ids)

            response.positions = Int32MultiArray()
            response.positions.layout.dim = [
                MultiArrayDimension(label='servos', size=len(servo_positions), stride=2*len(servo_positions)),
                MultiArrayDimension(label='id_pulse', size=2, stride=2)
            ]
            # Flatten the servo_positions list into [id1, pulse1, id2, pulse2, ...]
            response.positions.data = [val for pair in servo_positions for val in pair]

            response.success = True
            response.message = 'Servo positions retrieved successfully.'
        except Exception as e:
            response.success = False
            response.message = f'Error retrieving servo positions: {str(e)}'
        
        return response

    def run_action_callback(self, request, response):
        try:
            action_name = request.action_name
            self.motion_manager.run_action(action_name)
            response.success = True
            response.message = f'Action {action_name} executed successfully.'
        except Exception as e:
            response.success = False
            response.message = f'Error executing action: {str(e)}'
        
        return response

    def init_pose_callback(self, request, response):
        #self.get_logger().info("init_pose_callback")
        self.walking_module.stop()

        while not self.stop:
            #self.get_logger().info("init_pose_callback: while loop")
            self.sleep(0.01)

        self.walking_enable = False
        self.move_to_init_pose()
        self.walking_enable = True

        return response

    def save_servo_data(self, data, file_name):
        with open(file_name, "w") as file:
            file.write(data + "\n")

    def move_to_init_pose(self):
        self.init_pose_finish = False
        data = self.motion_manager.get_servos_position(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14)
        
        read_error = False

        for i in range(len(data)):
            if data[i][0] == 11:
                if data[i][1] < 385:
                    read_error = True
                    self.save_servo_data('11:' + str(i[1]), 'data3')
                if data[i][1] > 570:
                    read_error = True
                    self.save_servo_data('11:' + str(i[1]), 'data3')
            if data[i][0] == 12:
                if data[i][1] < 430:
                    read_error = True
                    self.save_servo_data('12:' + str(i[1]), 'data4')
                if data[i][1] > 615:
                    read_error = True
                    self.save_servo_data('12:' + str(i[1]), 'data4')

        if read_error:
            data = self.motion_manager.get_servos_position(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14)

            for i in range(len(data)):
                if data[i][0] == 11:
                    if data[i][1] < 385:
                        data[i][1] = 500
                    if data[i][1] > 570:
                        data[i][1] = 500
                if data[i][0] == 12:
                    if data[i][1] < 430:
                        data[i][1] = 500
                    if data[i][1] > 615:
                        data[i][1] = 500

        d = 0
        for i in data:
            if self.joint_name[i[0]] in self.joint_index:
                d += abs(self.init_pose_data[self.joint_name[i[0]]] - self.pulse2angle(i[0], i[1]))
        if d > self.max_stop_move:
            for i in data:
                if self.joint_name[i[0]] in self.joint_index:
                    self.present_joint_state[self.joint_name[i[0]]] = JointPosition()
                    self.present_joint_state[self.joint_name[i[0]]].present_position = self.pulse2angle(i[0], i[1])

        with open(self.walking_params_file, 'r', encoding='utf-8') as f:
            self.walking_param  = yaml.safe_load(f)

        self.walking_module = WalkingModule(self.ik, self.walking_param, self.joint_index, self.init_position, self.walking_param['trajectory_step_s'])

        self.init_servo_data = []

        for joint_name in self.init_pose_data:
            if joint_name not in self.joint_index and joint_name != 'head_pan' and joint_name != 'head_tilt':
                id_ = self.joint_id[joint_name]
                angle = self.init_pose_data[joint_name]
                pulse = self.angle2pulse(id_, angle)
                self.init_servo_data.extend([[id_, pulse]])

        self.motion_manager.set_servos_position(500, self.init_servo_data)
        self.sleep(0.5)
        self.init_pose_finish = True


    def run(self):
        if not self.init_pose_finish or not self.walking_enable:
            #self.get_logger().info(f".init_pose_finish={self.init_pose_finish}  walking_enable={self.walking_enable}")
            return
        next_time = time.monotonic()
        try:
            start = time.monotonic()
            joint_max_move, times_stamp, joint_state = self.walking_module.run(self.present_joint_state)
            #self.get_logger().debug(f"[{(time.monotonic() - start) * 1000:.1f} ms]: {joint_max_move} {times_stamp} {joint_state['r_hip_yaw'].present_position} {joint_state['r_hip_yaw'].goal_position}")

            if joint_max_move >= self.max_stop_move:
                #self.get_logger().debug(f"{joint_max_move} >= {self.max_stop_move}")
                if self.stop:
                    if rclpy.ok():
                        self.walk_state_pub.publish(Bool(data=True))
                self.stop = False
                curr_time = time.monotonic()
                delta_sec = curr_time - next_time
                delay_time = self.servo_max_move * self.speed - delta_sec

                if delay_time > 0:
                    if delta_sec + delay_time < self.walking_param['servo_control_cycle']:
                        self.sleep(self.walking_param['servo_control_cycle'] - delta_sec)
                    else:
                        self.sleep(delay_time)
                else:
                    if delta_sec < self.walking_param['servo_control_cycle']:
                        self.sleep(self.walking_param['servo_control_cycle'] - delta_sec)
                self.servo_max_move = 0
                data = []

                for joint_name in self.present_joint_state:
                    if self.walking_param['arm_swing_gain'] != 0 or (self.walking_param['arm_swing_gain'] == 0 and joint_name not in ['r_sho_pitch', 'l_sho_pitch']):
                        goal_position = joint_state[joint_name].goal_position
                        id_ = self.joint_id[joint_name]
                        pulse = self.angle2pulse(id_, goal_position)
                        data.append([id_, pulse])
                        self.present_joint_state[joint_name].present_position = goal_position

                        if self.last_position is not None:
                            d = abs(self.last_position[joint_name].goal_position - goal_position)
                            if self.servo_max_move < d:
                                self.servo_max_move = d
                self.motion_manager.set_servos_position(int(self.walking_param['servo_control_cycle'] * 1000), data)
                next_time = time.monotonic()
                self.last_position = copy.deepcopy(joint_state)
            elif self.walking_module.walk_finish():
                #self.get_logger().info("walk_finish==True")
                if not self.stop:
                    if rclpy.ok():
                        self.walk_state_pub.publish(Bool(data=False))
                self.servo_max_move = 0
                self.stop = True
            else:
                self.servo_max_move = 0

            if times_stamp >= self.walking_param['period_time']/1000.0 - self.walking_param['trajectory_step_s']:
                #self.get_logger().debug("times_stamp > period_time")
                self.servo_max_move = 0
                if self.walking_param['period_times'] != 0:
                    self.count_step += 1
                    if self.walking_param['period_times'] == self.count_step:
                        self.count_step = 0
                        self.walking_param['period_times'] = 0
                        self.walking_module.stop()
                else:
                    self.count_step = 0

        except Exception as e:
            if rclpy.ok():
                self.get_logger().error(f"Error in run: {e}")


    def angle2pulse(self, id_, angle):
        return self.joint_angles_convert_coef[id_][0] + int(round(angle * self.joint_angles_convert_coef[id_][1]))

    def pulse2angle(self, id_, angle):
        return (angle - self.joint_angles_convert_coef[id_][0]) / self.joint_angles_convert_coef[id_][1]

    def joint_states_callback(self, msg):
        if not self.init_pose_finish:
            self.init_pose_finish = True

        for i in range(len(msg.name)):
            if msg.name[i] in self.present_joint_state:
                self.present_joint_state[msg.name[i]].present_position = msg.position[i]

    def head_pan_controller_callback(self, msg):
        self.motion_manager.set_servos_position(int(msg.duration * 1000), [[self.joint_id['head_pan'], self.angle2pulse(self.joint_id['head_pan'], msg.position)]])

    def head_tilt_controller_callback(self, msg):
        self.motion_manager.set_servos_position(int(msg.duration * 1000), [[self.joint_id['head_tilt'], self.angle2pulse(self.joint_id['head_tilt'], msg.position)]])

    #def is_walking_callback(self, msg):
    #    # 当前是否在移动
    #    return [not self.stop, "is_walking"]

    def walking_command_callback(self, request, response):
        #self.get_logger().info(f"walking_command_callback: {request.command}")

        if self.init_pose_finish:
            if request.command == "start":
                self.walking_enable = True
                self.walking_module.start()
                self.walking_finish = False
            elif request.command == "stop":
                self.walking_module.stop()
                self.sleep(0.1)
                #while not self.stop:
                #    self.get_logger().info("walking_command_callback: while loop")
                #    self.sleep(0.1)
            elif request.command == 'enable':
                self.walking_enable = True
            elif request.command == 'disable':
                self.walking_module.stop()
                self.sleep(0.1)
                #while not self.stop:
                #    self.get_logger().info("walking_command_callback: while loop")
                #    self.sleep(0.1)
                self.walking_enable = False

        if request.command == 'enable_control':
            self.init_pose_finish = True
        elif request.command == 'disable_control':
            self.init_pose_finish = False

        response.result = True
        return response

    def set_action_callback(self, msg):
        self.get_logger().info("set_action_callback")

        self.walking_module.stop()

        while not self.stop:
            self.get_logger().info("set_action_callback:  while loop")
            self.sleep(0.05)

        self.walking_enable = False
        self.init_pose_finish = False
        self.motion_manager.run_action(msg.data)
        self.move_to_init_pose()
        self.walking_enable = True

    def set_app_walking_param_callback(self, msg):
        self.get_logger().info("set_app_walking_param_callback")

        self.walking_param['period_times'] = 0
        self.walking_param['init_x_offset'] = 0
        self.walking_param['init_y_offset'] = 0
        self.walking_param['init_z_offset'] = msg.height

        if self.body_height_range[0] > self.walking_param['init_z_offset']:
            self.walking_param['init_z_offset'] = self.body_height_range[0]
        elif self.body_height_range[1] < self.walking_param['init_z_offset']:
            self.walking_param['init_z_offset'] = self.body_height_range[1]

        self.walking_param['x_move_amplitude'] = msg.x
        if self.x_amplitude_range[0] > self.walking_param['x_move_amplitude']:
            self.walking_param['x_move_amplitude'] = self.x_amplitude_range[0]
        elif self.x_amplitude_range[1] < self.walking_param['x_move_amplitude']:
            self.walking_param['x_move_amplitude'] = self.x_amplitude_range[1]
        self.walking_param['y_move_amplitude'] = msg.y
        if self.y_amplitude_range[0] > self.walking_param['y_move_amplitude']:
            self.walking_param['y_move_amplitude'] = self.y_amplitude_range[0]
        elif self.y_amplitude_range[1] < self.walking_param['y_move_amplitude']:
            self.walking_param['y_move_amplitude'] = self.y_amplitude_range[1]

        self.walking_param['angle_move_amplitude'] = msg.angle

        if self.angle_amplitude_range[0] > self.walking_param['angle_move_amplitude']:
            self.walking_param['angle_move_amplitude'] = self.angle_amplitude_range[0]
        elif self.angle_amplitude_range[1] < self.walking_param['angle_move_amplitude']:
            self.walking_param['angle_move_amplitude'] = self.angle_amplitude_range[1]

        self.walking_param['init_roll_offset'] = 0.0
        self.walking_param['init_pitch_offset'] = 0.0
        self.walking_param['init_yaw_offset'] = 0.0
        self.walking_param['hip_pitch_offset'] = 15.0
        self.walking_param['z_move_amplitude'] = 0.02
        self.walking_param['pelvis_offset'] = 5.0
        self.walking_param['move_aim_on'] = False
        self.walking_param['arm_swing_gain'] = 0.5

        if msg.speed == 4:
            self.walking_param['period_time'] = 300
            self.walking_param['dsp_ratio'] = 0.2
            self.walking_param['init_y_offset'] = -0.008
            self.walking_param['step_fb_ratio'] = 0.028
            self.walking_param['y_swap_amplitude'] = 0.02
            self.walking_param['z_swap_amplitude'] = 0.006
            self.walking_param['pelvis_offset'] = 5.0
            self.walking_param['z_move_amplitude'] = 0.015

            if self.walking_param['x_move_amplitude'] != 0:
                self.walking_param['x_move_amplitude'] = math.copysign(0.01, self.walking_param['x_move_amplitude'])
            if self.walking_param['y_move_amplitude'] != 0:
                self.walking_param['y_move_amplitude'] = math.copysign(0.01, self.walking_param['y_move_amplitude'])
            if self.walking_param['angle_move_amplitude'] != 0:
                self.walking_param['angle_move_amplitude'] = math.copysign(8, self.walking_param['angle_move_amplitude'])
            
            if self.walking_param['x_move_amplitude'] != 0:
                self.walking_param['init_roll_offset'] = -3.0
                if self.walking_param['y_move_amplitude'] == 0 and self.walking_param['angle_move_amplitude'] == 0:
                    self.walking_param['x_move_amplitude'] = math.copysign(0.012, self.walking_param['x_move_amplitude'])
                if self.walking_param['x_move_amplitude'] > 0:
                    self.walking_param['angle_move_amplitude'] += 0.2
                else:
                    self.walking_param['angle_move_amplitude'] += 1.1

            if self.walking_param['y_move_amplitude'] != 0 and self.walking_param['x_move_amplitude'] == 0 and self.walking_param['angle_move_amplitude'] == 0:
                self.walking_param['y_move_amplitude'] = math.copysign(0.015, self.walking_param['y_move_amplitude'])
                if self.walking_param['y_move_amplitude'] > 0:
                    self.walking_param['angle_move_amplitude'] += 1.5
                else:
                    self.walking_param['angle_move_amplitude'] += -0.4

            if self.walking_param['angle_move_amplitude'] != 0 and self.walking_param['y_move_amplitude'] == 0 and self.walking_param['x_move_amplitude'] == 0:
                self.walking_param['init_roll_offset'] = -3.0
                self.walking_param['angle_move_amplitude'] = math.copysign(10, self.walking_param['angle_move_amplitude'])

        elif msg.speed == 3:
            self.walking_param['period_time'] = 400
            self.walking_param['dsp_ratio'] = 0.2
            self.walking_param['init_y_offset'] = -0.005
            self.walking_param['step_fb_ratio'] = 0.028
            self.walking_param['y_swap_amplitude'] = 0.02
            self.walking_param['z_swap_amplitude'] = 0.006

            if self.walking_param['x_move_amplitude'] != 0:
                self.walking_param['x_move_amplitude'] = math.copysign(0.01, self.walking_param['x_move_amplitude'])
            if self.walking_param['y_move_amplitude'] != 0:
                self.walking_param['y_move_amplitude'] = math.copysign(0.01, self.walking_param['y_move_amplitude'])
            if self.walking_param['angle_move_amplitude'] != 0:
                self.walking_param['angle_move_amplitude'] = math.copysign(7, self.walking_param['angle_move_amplitude'])

            if self.walking_param['angle_move_amplitude'] != 0 and self.walking_param['y_move_amplitude'] == 0 and self.walking_param['x_move_amplitude'] == 0:
                self.walking_param['y_swap_amplitude'] = 0.022
                self.walking_param['angle_move_amplitude'] = math.copysign(7, self.walking_param['angle_move_amplitude'])
            elif self.walking_param['angle_move_amplitude'] != 0:
                if self.walking_param['y_move_amplitude'] != 0:
                    self.walking_param['init_roll_offset'] = 3.0
                elif self.walking_param['x_move_amplitude'] != 0:
                    self.walking_param['angle_move_amplitude'] = math.copysign(7, self.walking_param['angle_move_amplitude'])                   

            if self.walking_param['y_move_amplitude'] != 0 and self.walking_param['x_move_amplitude'] == 0 and self.walking_param['angle_move_amplitude'] == 0:
                self.walking_param['init_y_offset'] = 0
                self.walking_param['init_roll_offset'] = 3.0
                self.walking_param['y_swap_amplitude'] = 0.025
                self.walking_param['y_move_amplitude'] = math.copysign(0.015, self.walking_param['y_move_amplitude'])
                if self.walking_param['y_move_amplitude'] > 0:
                    self.walking_param['angle_move_amplitude'] += 0.0
                else:
                    self.walking_param['angle_move_amplitude'] += 0.0

            if self.walking_param['x_move_amplitude'] != 0 :
                self.walking_param['x_move_amplitude'] = math.copysign(0.011, self.walking_param['x_move_amplitude'])
                if self.walking_param['x_move_amplitude'] > 0:
                    self.walking_param['angle_move_amplitude'] += 0.0
                else:
                    self.walking_param['y_swap_amplitude'] = 0.023
                    self.walking_param['angle_move_amplitude'] += 0.0

        elif msg.speed == 2:
            self.walking_param['period_time'] = 500
            self.walking_param['dsp_ratio'] = 0.2
            self.walking_param['step_fb_ratio'] = 0.028
            self.walking_param['y_swap_amplitude'] = 0.02
            self.walking_param['z_swap_amplitude'] = 0.006
            self.walking_param['init_y_offset'] = -0.008

            if self.walking_param['x_move_amplitude'] != 0:
                self.walking_param['x_move_amplitude'] = math.copysign(0.01, self.walking_param['x_move_amplitude'])
            if self.walking_param['y_move_amplitude'] != 0:
                self.walking_param['y_move_amplitude'] = math.copysign(0.01, self.walking_param['y_move_amplitude'])
            if self.walking_param['angle_move_amplitude'] != 0:
                self.walking_param['angle_move_amplitude'] = math.copysign(10, self.walking_param['angle_move_amplitude'])
        
            if self.walking_param['angle_move_amplitude'] != 0 and self.walking_param['y_move_amplitude'] == 0 and self.walking_param['x_move_amplitude'] == 0:
                self.walking_param['init_roll_offset'] = 0.0
                self.walking_param['init_y_offset'] = -0.005
                self.walking_param['y_swap_amplitude'] = 0.025
            elif self.walking_param['angle_move_amplitude'] != 0 and self.walking_param['x_move_amplitude'] != 0:
                self.walking_param['y_swap_amplitude'] = 0.025

            if self.walking_param['y_move_amplitude'] != 0 and self.walking_param['x_move_amplitude'] == 0 and self.walking_param['angle_move_amplitude'] == 0:
                self.walking_param['init_y_offset'] = -0.005
                self.walking_param['y_swap_amplitude'] = 0.028
                self.walking_param['init_roll_offset'] = 3.0
                if self.walking_param['y_move_amplitude'] > 0:
                    self.walking_param['angle_move_amplitude'] += 0.0
                else:
                    self.walking_param['angle_move_amplitude'] += 0.0
                self.walking_param['y_move_amplitude'] = math.copysign(0.015, self.walking_param['y_move_amplitude'])
            elif self.walking_param['y_move_amplitude'] != 0 and self.walking_param['angle_move_amplitude'] != 0:
                self.walking_param['init_y_offset'] = -0.005
                self.walking_param['y_swap_amplitude'] = 0.025
                self.walking_param['init_roll_offset'] = 3.0

            if self.walking_param['x_move_amplitude'] != 0 and self.walking_param['y_move_amplitude'] == 0 and self.walking_param['angle_move_amplitude'] == 0:
                # self.walking_param['init_y_offset'] = 0.0
                # self.walking_param['init_roll_offset'] = 3.0
                self.walking_param['x_move_amplitude'] = math.copysign(0.015, self.walking_param['x_move_amplitude'])
                if self.walking_param['x_move_amplitude'] > 0:
                    self.walking_param['angle_move_amplitude'] += 0.0
                else:
                    self.walking_param['angle_move_amplitude'] += 0.0

        elif msg.speed == 1:
            self.walking_param['period_time'] = 600
            self.walking_param['dsp_ratio'] = 0.2
            self.walking_param['step_fb_ratio'] = 0.028
            self.walking_param['y_swap_amplitude'] = 0.02
            self.walking_param['z_swap_amplitude'] = 0.006
            self.walking_param['init_y_offset'] = -0.008
            
            if self.walking_param['x_move_amplitude'] != 0:
                self.walking_param['x_move_amplitude'] = math.copysign(0.01, self.walking_param['x_move_amplitude'])
            if self.walking_param['y_move_amplitude'] != 0:
                self.walking_param['y_move_amplitude'] = math.copysign(0.01, self.walking_param['y_move_amplitude'])
            if self.walking_param['angle_move_amplitude'] != 0:
                self.walking_param['angle_move_amplitude'] = math.copysign(10, self.walking_param['angle_move_amplitude'])

            if self.walking_param['angle_move_amplitude'] != 0 and self.walking_param['y_move_amplitude'] == 0 and self.walking_param['x_move_amplitude'] == 0:
                self.walking_param['y_swap_amplitude'] = 0.025
                self.walking_param['init_roll_offset'] = 3.0
                self.walking_param['init_y_offset'] = 0.0
            elif self.walking_param['angle_move_amplitude'] != 0 and self.walking_param['x_move_amplitude'] != 0:
                self.walking_param['y_swap_amplitude'] = 0.025
                self.walking_param['init_roll_offset'] = 3.0
                self.walking_param['init_y_offset'] = 0.0

            if self.walking_param['y_move_amplitude'] != 0 and self.walking_param['x_move_amplitude'] == 0 and self.walking_param['angle_move_amplitude'] == 0:
                self.walking_param['init_y_offset'] = 0.0
                self.walking_param['y_swap_amplitude'] = 0.03
                self.walking_param['init_roll_offset'] = 5.0
                self.walking_param['y_move_amplitude'] = math.copysign(0.012, self.walking_param['y_move_amplitude'])
                if self.walking_param['y_move_amplitude'] > 0:
                    self.walking_param['angle_move_amplitude'] += 0.0
                else:
                    self.walking_param['angle_move_amplitude'] += 0.0
            elif self.walking_param['y_move_amplitude'] != 0 and self.walking_param['angle_move_amplitude'] != 0:
                self.walking_param['init_y_offset'] = -0.005
                self.walking_param['y_swap_amplitude'] = 0.03
                self.walking_param['init_roll_offset'] = 3.0

            if self.walking_param['x_move_amplitude'] != 0 and self.walking_param['y_move_amplitude'] == 0 and self.walking_param['angle_move_amplitude'] == 0:
                self.walking_param['init_y_offset'] = 0.0
                self.walking_param['init_roll_offset'] = 3.0
                self.walking_param['x_move_amplitude'] = math.copysign(0.015, self.walking_param['x_move_amplitude'])
                if self.walking_param['x_move_amplitude'] > 0:
                    self.walking_param['angle_move_amplitude'] += 0.0
                else:
                    self.walking_param['angle_move_amplitude'] += 0.0


        self.walking_module.set_walking_param(self.walking_param)

    def set_walking_param_callback(self, msg):
        self.walking_param['period_times'] = msg.period_times
        self.walking_param['init_x_offset'] = msg.init_x_offset
        self.walking_param['init_y_offset'] = msg.init_y_offset
        self.walking_param['init_z_offset'] = msg.init_z_offset

        if self.body_height_range[0] > self.walking_param['init_z_offset']:
            self.walking_param['init_z_offset'] = self.body_height_range[0]
        elif self.body_height_range[1] < self.walking_param['init_z_offset']:
            self.walking_param['init_z_offset'] = self.body_height_range[1]

        self.walking_param['init_roll_offset'] = msg.init_roll_offset
        self.walking_param['init_pitch_offset'] = msg.init_pitch_offset
        self.walking_param['init_yaw_offset'] = msg.init_yaw_offset
        self.walking_param['hip_pitch_offset'] = msg.hip_pitch_offset
        self.walking_param['period_time'] = msg.period_time
        self.walking_param['dsp_ratio'] = msg.dsp_ratio
        self.walking_param['step_fb_ratio'] = msg.step_fb_ratio
        self.walking_param['x_move_amplitude'] = msg.x_move_amplitude

        if self.x_amplitude_range[0] > self.walking_param['x_move_amplitude']:
            self.walking_param['x_move_amplitude'] = self.x_amplitude_range[0]
        elif self.x_amplitude_range[1] < self.walking_param['x_move_amplitude']:
            self.walking_param['x_move_amplitude'] = self.x_amplitude_range[1]

        self.walking_param['y_move_amplitude'] = msg.y_move_amplitude

        if self.y_amplitude_range[0] > self.walking_param['y_move_amplitude']:
            self.walking_param['y_move_amplitude'] = self.y_amplitude_range[0]
        elif self.y_amplitude_range[1] < self.walking_param['y_move_amplitude']:
            self.walking_param['y_move_amplitude'] = self.y_amplitude_range[1]

        self.walking_param['z_move_amplitude'] = msg.z_move_amplitude

        if self.step_height_range[0] > self.walking_param['z_move_amplitude']:
            self.walking_param['z_move_amplitude'] = self.step_height_range[0]
        elif self.step_height_range[1] < self.walking_param['z_move_amplitude']:
            self.walking_param['z_move_amplitude'] = self.step_height_range[1]

        self.walking_param['angle_move_amplitude'] = msg.angle_move_amplitude

        if self.angle_amplitude_range[0] > self.walking_param['angle_move_amplitude']:
            self.walking_param['angle_move_amplitude'] = self.angle_amplitude_range[0]
        elif self.angle_amplitude_range[1] < self.walking_param['angle_move_amplitude']:
            self.walking_param['angle_move_amplitude'] = self.angle_amplitude_range[1]

        self.walking_param['y_swap_amplitude'] = msg.y_swap_amplitude

        if self.y_swap_range[0] > self.walking_param['y_swap_amplitude']:
            self.walking_param['y_swap_amplitude'] = self.y_swap_range[0]
        elif self.y_swap_range[1] < self.walking_param['y_swap_amplitude']:
            self.walking_param['y_swap_amplitude'] = self.y_swap_range[1]

        self.walking_param['z_swap_amplitude'] = msg.z_swap_amplitude
        self.walking_param['pelvis_offset'] = float(msg.pelvis_offset)
        self.walking_param['move_aim_on'] = msg.move_aim_on
        self.walking_param['arm_swing_gain'] = msg.arm_swing_gain

        if self.arm_swap_range[0] > self.walking_param['arm_swing_gain']:
            self.walking_param['arm_swing_gain'] = self.arm_swap_range[0]
        elif self.arm_swap_range[1] < self.walking_param['arm_swing_gain']:
            self.walking_param['arm_swing_gain'] = self.arm_swap_range[1] 
        
        self.walking_module.set_walking_param(self.walking_param)

    def get_walking_param_callback(self, request, response):
        self.walking_param = self.walking_module.get_walking_param()
        
        param = WalkingParam()
        param.init_x_offset = self.walking_param['init_x_offset']
        param.init_y_offset = self.walking_param['init_y_offset']
        param.init_z_offset = self.walking_param['init_z_offset']
        param.init_roll_offset = float(self.walking_param['init_roll_offset'])
        param.init_pitch_offset = float(self.walking_param['init_pitch_offset'])
        param.init_yaw_offset = float(self.walking_param['init_yaw_offset'])
        param.hip_pitch_offset = float(self.walking_param['hip_pitch_offset'])
        param.period_time = float(self.walking_param['period_time'])
        param.dsp_ratio = self.walking_param['dsp_ratio']
        param.step_fb_ratio = self.walking_param['step_fb_ratio']
        param.x_move_amplitude = self.walking_param['x_move_amplitude']
        param.y_move_amplitude = self.walking_param['y_move_amplitude']
        param.z_move_amplitude = self.walking_param['z_move_amplitude']
        param.angle_move_amplitude = self.walking_param['angle_move_amplitude']
        param.y_swap_amplitude = self.walking_param['y_swap_amplitude']
        param.z_swap_amplitude = self.walking_param['z_swap_amplitude']
        param.pelvis_offset = float(self.walking_param['pelvis_offset'])
        param.move_aim_on = self.walking_param['move_aim_on']
        param.arm_swing_gain = self.walking_param['arm_swing_gain']
        param.period_times = 0

        response.parameters = param
        return response

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()

    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        controller_node.walking_enable = False
    finally:
        controller_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
