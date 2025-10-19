#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class YUV2RGBConverter(Node):
    def __init__(self):
        super().__init__('yuv2rgb_converter')
        self.bridge = CvBridge()
        # Subscribe to raw image topic (YUV422/YUY2)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        # Publish RGB image
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_rgb',
            10
        )
        self.get_logger().info('YUV2RGB Converter Node Started')

    def image_callback(self, msg):
        try:
            # Convert ROS 2 Image message to OpenCV image (yuv422)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='yuv422_yuy2')
            # Convert YUV422 (YUY2) to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_YUV2RGB_YUY2)
            # Convert back to ROS 2 Image message
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
            rgb_msg.header = msg.header
            if rclpy.ok():
                self.image_pub.publish(rgb_msg)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def destroy_node(self):
        try:
            with self.lock:
                self.running = False
        except Exception as e:
            if rclpy.ok():
                self.get_logger().warning(f"Failed to destroy node: {e}")
        super().destroy_node()
            

def main(args=None):
    rclpy.init(args=args)
    node = YUV2RGBConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down YUV2RGB Converter Node')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

