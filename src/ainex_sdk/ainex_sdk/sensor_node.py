import math
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from icm20948 import ICM20948
from std_msgs.msg import Int32
from std_srvs.srv import SetBool
from ainex_interfaces.srv import SetFloat, SetRGB
from sensor_msgs.msg import MagneticField, Imu
#from ainex_sdk import button, buzzer, led, rgb, imu
from ainex_sdk import button, buzzer, led, imu
from scipy.constants import g

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.enable_button = True
        self.enable_imu = True
        
        self.create_service(SetBool, '/sensor/button/enable', self.set_button_enable)
        self.create_service(SetBool, '/sensor/imu/enable',  self.set_imu_enable)
        self.create_service(SetRGB, '/sensor/rgb/set_rgb_state',  self.set_rgb_state_srv)
        self.create_service(SetBool, '/sensor/led/set_led_state', self.set_led_state_srv)
        self.create_service(SetBool, '/sensor/buzzer/set_buzzer_state', self.set_buzzer_state_srv)
        self.create_service(SetFloat, '/sensor/buzzer/set_buzzer_frequency', self.set_buzzer_frequency)

        self.imu = ICM20948()
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('imu_raw_topic', '/sensor/imu/imu_raw')
        self.declare_parameter('imu_mag_topic', '/sensor/imu/imu_mag')
        self.declare_parameter('btn_state_topic', '/sensor/button/get_button_state')
        self.declare_parameter('freq', 50)

        self.imu_frame = self.get_parameter('imu_frame').get_parameter_value().string_value
        imu_raw_topic = self.get_parameter('imu_raw_topic').get_parameter_value().string_value
        imu_mag_topic = self.get_parameter('imu_mag_topic').get_parameter_value().string_value
        btn_state_topic = self.get_parameter('btn_state_topic').get_parameter_value().string_value
        freq =  self.get_parameter('freq').get_parameter_value().integer_value
    
        self.raw_pub = self.create_publisher(Imu, imu_raw_topic, 1)
        self.mag_pub = self.create_publisher(MagneticField, imu_mag_topic, 1)
        self.btn_pub = self.create_publisher(Int32, btn_state_topic, 1)

        timer_period = 1/freq  # seconds
        self.timer = self.create_timer(timer_period, self.imu_timer_callback)
        self.get_logger().info(f'timer_period = {timer_period * 1000} ms')

        try:
            led.off()
            #rgb.set_color(0, 0, 0)
            buzzer.on()
            self.get_clock().sleep_for(Duration(seconds=0.2))
        finally:
            buzzer.off()
        
    def set_button_enable(self, request: SetBool.Request, response: SetBool.Response):
        self.enable_button = request.data
        response.success = True
        response.message = 'set_button_enable'
        return response

    def set_imu_enable(self, request: SetBool.Request, response: SetBool.Response):
        self.enable_imu = request.data
        response.success = True
        response.message = 'set_imu_enable'
        return response

    def set_buzzer_frequency(self, request: SetFloat.Request, response: SetFloat.Response):
        #buzzer.on()
        time.sleep(1.0/request.data)
        #buzzer.off()
        response.success = True
        response.message = 'set_buzzer_frequency'
        return response

    def set_buzzer_state_srv(self, request: SetBool.Request, response: SetBool.Response):
        if request.data: 
            buzzer.on()
        else:
            buzzer.off()

        response.success = True
        response.message = 'set_buzzer_state'
        return response

    def set_rgb_state_srv(self, request: SetRGB.Request, response: SetRGB.Response):
        rgb.set_color(request.data.r, request.data.g, request.data.b)

        response.success = True
        response.message = 'set_rgb_state'
        return response

    def set_led_state_srv(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            led.on()
        else:
            led.off()

        response.success = True
        response.message = 'set_led_state'
        return response

    def imu_timer_callback(self):
        try:
            ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()    
            mx, my, mz = self.imu.read_magnetometer_data()
            raw_msg = Imu()
            raw_msg.header.frame_id = self.imu_frame
            raw_msg.header.stamp = self.get_clock().now().to_msg()
                
            raw_msg.orientation.w = 0.0
            raw_msg.orientation.x = 0.0
            raw_msg.orientation.y = 0.0
            raw_msg.orientation.z = 0.0
                
            raw_msg.linear_acceleration.x = ax * g 
            raw_msg.linear_acceleration.y = ay * g 
            raw_msg.linear_acceleration.z = az * g 

            raw_msg.angular_velocity.x = math.radians(gx)
            raw_msg.angular_velocity.y = math.radians(gy)
            raw_msg.angular_velocity.z = math.radians(gz)

            raw_msg.orientation_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 1e-6]
            raw_msg.angular_velocity_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 1e-6]
            raw_msg.linear_acceleration_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                
            mag_msg = MagneticField()
            mag_msg.header.stamp = raw_msg.header.stamp
            mag_msg.magnetic_field.x = mx
            mag_msg.magnetic_field.y = my
            mag_msg.magnetic_field.z = mz
            mag_msg.magnetic_field_covariance[0] = -1.0
            
            self.raw_pub.publish(raw_msg)
            self.mag_pub.publish(mag_msg)
        except:
            if rclpy.ok():
                self.get_logger().info('imu message publish exeption')

def main(args=None):
    rclpy.init(args=args)
    sensor_node = SensorNode()

    try:
        rclpy.spin(sensor_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
