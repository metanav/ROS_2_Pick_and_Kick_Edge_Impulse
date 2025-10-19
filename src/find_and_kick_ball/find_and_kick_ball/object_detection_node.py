import rclpy
import cv2
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from edge_impulse_linux.image import ImageImpulseRunner

class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('object_detection')

        self.declare_parameter('resource_path', '')
        resource_path = self.get_parameter('resource_path').get_parameter_value().string_value
        model_file = os.path.join(resource_path, 'object_detection-linux-aarch64-qnn-v42.eim')
        self.cvbridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_rect_color', self.image_rgb_callback, 10)
        self.pub_dets = self.create_publisher(Detection2DArray, '/edge_impulse/detections', 10)
        self.runner = ImageImpulseRunner(model_file)
        model_info = self.runner.init()
        resizeMode = model_info['model_parameters'].get('image_resize_mode', 'not-reported')
        self.get_logger().info(f'resizeMode = {resizeMode}')


    def image_rgb_callback(self, msg):
        img = self.cvbridge.imgmsg_to_cv2(msg)
        features, cropped = self.runner.get_features_from_image_auto_studio_settings(img)
        res = self.runner.classify(features)

        vision_msg_dets  = Detection2DArray()
        #ts =self.get_clock().now().to_msg()
        #vision_msg_dets.header.stamp = ts 
        vision_msg_dets.header = msg.header

        for bb in res["result"]["bounding_boxes"]:
            det2D = Detection2D()
            #det2D.header.stamp = ts
            objHyp = ObjectHypothesisWithPose()
            det2D.bbox.center.position.x = float(bb['x'] + bb['width']/2)
            det2D.bbox.center.position.y = float(bb['y'] + bb['height']/2)
            det2D.bbox.size_x = float(bb['width'])
            det2D.bbox.size_y = float(bb['height'])
            objHyp.hypothesis.class_id = str(bb['label'])
            objHyp.hypothesis.score = float(bb['value'])
            det2D.results.append(objHyp)
            vision_msg_dets.detections.append(det2D)

        self.pub_dets.publish(vision_msg_dets)
    
    def destroy_node(self):
        if self.runner:
            self.runner.stop()
        if rclpy.ok():
            self.get_logger().info("Object detection node stopped")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()

    try:
        rclpy.spin(object_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        object_detection_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()




