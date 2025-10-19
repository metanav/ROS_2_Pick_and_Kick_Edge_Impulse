import rclpy
import math
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Int32MultiArray, MultiArrayDimension
from vision_msgs.msg import Detection2DArray
from ainex_sdk import pid, common
from find_and_kick_ball.pid_track import PIDTrack
from find_and_kick_ball.approach_object import ApproachObject
from ainex_interfaces.srv import SetServosPosition, GetServosPosition, RunAction
from ainex_kinematics.gait_manager import GaitManager

class KickBallNode(Node):
    def __init__(self):
        super().__init__('kick_ball_node')
    
        self.img_width = 640
        self.img_height = 480
        self.start = True
        self.left_shot_action_name = 'left_shot'
        self.right_shot_action_name = 'right_shot'
        self.image_process_size = [160, 120]
        self.detections = []
        self.count_miss = 0
        self.start_index = 0
        self.start_find_ball = False
        self.head_pan_init = 500  
        self.head_tilt_init = 300 
        self.head_time_stamp = self.get_clock().now()

        self.create_subscription(Detection2DArray, '/edge_impulse/detections',  self.object_detection_callback, 1)
        self.run_action_client     = self.create_client(RunAction, 'run_action')
        self.set_servos_pos_client = self.create_client(SetServosPosition, 'set_servos_position')
        self.get_servos_pos_client = self.create_client(GetServosPosition, 'get_servos_position')
        self.wait_for_services()

        self.declare_parameter('calib_config_file', '')
        self.calib_config_file = self.get_parameter('calib_config_file').get_parameter_value().string_value
        self.calib_config = common.get_yaml_data(self.calib_config_file)

        self.rl_dis = None
        self.ud_dis = None
        self.last_rl_dis = None
        self.last_ud_dis = None


        self.gait_manager = GaitManager(self)
        self.approach_object = ApproachObject(self.gait_manager, self.calib_config, step_mode=0)
        self.approach_object.update_gait(dsp=[400, 0.2, 0.02])
        self.approach_object.update_stop_count(1)
        self.approach_object.update_gait_range(x_range=[-0.013, 0.013])
        self.approach_object.update_approach_stop_value(30, 0, 3)
        #self.approach_object.update_approach_stop_value(20, 0, 3)
        
        self.head_pan_range = [125, 875] 
        self.head_tilt_range = [260, 500] 
        #self.pid_rl = pid.PID(0.1, 0.0, 0.001)
        #self.pid_ud = pid.PID(0.1, 0.0, 0.001)
        self.pid_rl = pid.PID(0.05, 0.0, 0.005)
        self.pid_ud = pid.PID(0.05, 0.0, 0.005)
        self.head_pan_init = 500  
        self.head_tilt_init = 300 
        self.rl_track = PIDTrack(self.pid_rl, self.head_pan_range, self.head_pan_init)
        self.ud_track = PIDTrack(self.pid_ud, self.head_tilt_range, self.head_tilt_init)
        
        self.yaw_stop = 40
        
        # left_down, left_up, center_up, right_up, right_down
        #self.find_ball_position = [
        #    [650, 300, 1000], 
        #    [650, 500, 1000], 
        #    [500, 500, 1000], 
        #    [350, 500, 1000],
        #    [350, 300, 1000]
        #]
        self.find_ball_position = [
            [650, 300, 2000], 
            [650, 500, 2000], 
            [500, 500, 2000], 
            [350, 500, 2000],
            [350, 300, 2000]
        ]

        self.call_run_action('walk_ready')
        self.init_action(self.head_pan_init, self.head_tilt_init)

    def init_action(self, head_pan_init, head_tilt_init, delay=200):
        self.call_set_servos_position(delay, [[23, head_pan_init], [24, head_tilt_init]])
        self.gait_manager.stop()
        self.get_clock().sleep_for(Duration(seconds=delay/1000))

    def object_detection_callback(self, msg):
        self.detections = msg.detections
        for detection in self.detections:
            detection.bbox.center.position.x = detection.bbox.center.position.x * 480/320 + 80
            detection.bbox.center.position.y = detection.bbox.center.position.y * 480/320
            detection.bbox.center.position.x -= self.calib_config['center_x_offset']

    def head_track_process(self, detection):
        if abs(detection.bbox.center.position.x - self.img_width/2) < 10:
            detection.bbox.center.position.x = float(self.img_width/2)
    
        if abs(detection.bbox.center.position.y - self.img_height/2) < 10:
            detection.bbox.center.position.y = float(self.img_height/2)
    
        rl_dis = self.rl_track.track(detection.bbox.center.position.x, self.img_width/2)
        ud_dis = self.ud_track.track(detection.bbox.center.position.y, self.img_height/2)
        
        if rl_dis is None or ud_dis is None:
            self.get_logger().error(f"Invalid PID output: rl_dis={rl_dis}, ud_dis={ud_dis}")
            return None, None
        
        try:
            servo_positions = [[23, int(rl_dis)], [24, int(ud_dis)]]
            #self.call_set_servos_position(20, servo_positions)
            self.call_set_servos_position(50, servo_positions)
            #self.get_logger().info(f'head track: {rl_dis}, {ud_dis}')
            return rl_dis, ud_dis
        except ValueError as e:
            self.get_logger().error(f"Error converting PID outputs to int: {e}")
            return None, None

    def body_track_process(self, rl_dis, ud_dis, detection):                                                
        if detection is not None:
            yaw_stop = 500 + math.copysign(self.yaw_stop, rl_dis - 500)
            yaw_stop_ = (yaw_stop - rl_dis)/9

            if 15 < abs(yaw_stop - rl_dis) < 27:
                yaw_stop_ = math.copysign(4, yaw_stop - rl_dis)

            if self.approach_object.process(500 - ud_dis, 0 + self.calib_config['center_x_offset'], yaw_stop_, self.head_tilt_range[0], 0, 0, self.img_width, self.img_height):           
                #self.get_logger().info('diable gait_manager')
                self.gait_manager.disable()

                if rl_dis > 500:
                    self.call_run_action(self.left_shot_action_name)
                else:
                    self.call_run_action(self.right_shot_action_name)

    def find_ball_process(self):
        if self.get_clock().now() > self.head_time_stamp:
            if self.start_index > len(self.find_ball_position) - 1:
                self.start_index = 0

            rl_dis = self.find_ball_position[self.start_index][0]
            ud_dis = self.find_ball_position[self.start_index][1]
            self.rl_track.update_position(rl_dis) 
            self.ud_track.update_position(ud_dis)
            duration = self.find_ball_position[self.start_index][2]
            self.call_set_servos_position(duration, [[23, rl_dis], [24, ud_dis]])

            self.gait_manager.move(2, 0, 0, 5) 
            self.head_time_stamp = self.get_clock().now() + Duration(seconds=int(self.find_ball_position[self.start_index][2] / 1000))

            self.start_index += 1

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.start:
                detection = None
                for detection in self.detections:
                    self.rl_dis, self.ud_dis = self.head_track_process(detection)
                    self.get_logger().info(f'[{detection.bbox.center.position.x}, {detection.bbox.center.position.y}] rl:{self.rl_dis} ud:{self.ud_dis}')

                if self.rl_dis is not None:
                    self.body_track_process(self.rl_dis, self.ud_dis, detection)
                    self.rl_dis = None
                    self.count_miss = 0
                    self.start_find_ball = False
                else:
                    if not self.start_find_ball:
                        self.count_miss += 1  
                        if self.count_miss > 20:
                            self.count_miss = 0
                            self.start_find_ball = True
                            self.start_index = 0
                    else:
                        self.find_ball_process() 

            self.get_clock().sleep_for(Duration(seconds=0.01))

    def wait_for_services(self):
        services = [
            ('run_action', self.run_action_client),
            ('set_servos_pos', self.set_servos_pos_client),
            ('get_servos_pos', self.get_servos_pos_client),
        ]
        for service_name, client in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for service {service_name}...')
            self.get_logger().info(f'Service {service_name} is available.')

    def call_set_servos_position(self, duration, servo_positions):
        request = SetServosPosition.Request()
        request.duration = duration
        request.positions = Int32MultiArray()
        request.positions.layout.dim = [
            MultiArrayDimension(label='servos', size=len(servo_positions), stride=2*len(servo_positions)),
            MultiArrayDimension(label='id_pos', size=2, stride=2)
        ]
        request.positions.data = [val for pair in servo_positions for val in pair]
    
        future = self.set_servos_pos_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            #self.get_logger().info(f'SetServosPosition response: success={response.success}, message="{response.message}"')
        else:
            self.get_logger().error(f'SetServosPosition service call failed: {future.exception()}')

    def call_get_servos_position(self, servo_ids):
        request = GetServosPosition.Request()
        request.servo_ids = servo_ids

        future = self.get_servos_pos_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            # Parse the Int32MultiArray response into [[servo_id, pulse], ...]
            positions = []
            if response.success:
                dims = response.positions.layout.dim
                if len(dims) == 2 and dims[1].size == 2:
                    num_servos = dims[0].size
                    stride = dims[1].stride
                    data = response.positions.data
                    for i in range(num_servos):
                        start_idx = i * stride
                        servo_id = data[start_idx]
                        pulse = data[start_idx + 1]
                        positions.append([servo_id, pulse])
            self.get_logger().info(
                f'GetServosPosition response: success={response.success}, '
                f'message="{response.message}", positions={positions}'
            )
            return positions
        else:
            self.get_logger().error(f'GetServosPosition service call failed: {future.exception()}')
            return []

    def call_run_action(self, action_name):
        request = RunAction.Request()
        request.action_name = action_name
        future = self.run_action_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            #self.get_logger().info(f'RunAction response: success={response.success}, message="{response.message}"')
            return response
        else:
            self.get_logger().error(f'RunAction service call failed: {future.exception()}')
            return None

    def destroy_node(self):
        try:
            self.start = False
            #self.init_action(self.head_pan_init, self.head_tilt_init)
            #self.call_run_action('walk_ready')
            if rclpy.ok():
                self.get_logger().info("Shutting down.")
        except Exception as e:
            if rclpy.ok():
                self.get_logger().warning(f"Failed to destroy node: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    kick_ball_node = KickBallNode()

    try:
        kick_ball_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        kick_ball_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
