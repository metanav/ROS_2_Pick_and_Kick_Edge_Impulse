import rclpy
import cv2
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from vision_msgs.msg import Detection2DArray

class ImageBboxNode(Node):

    def __init__(self):
        super().__init__('image_bbox_node')

        qos = QoSProfile(depth=10)
        self.sub_img = Subscriber(self, Image, '/camera/image_rect_color')
        self.sub_dets = Subscriber(self, Detection2DArray, '/edge_impulse/detections')
        self.pub_img = self.create_publisher(Image, '/edge_impulse/img_bbox', 10)

        self.cvbridge = CvBridge()
        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.sub_img, self.sub_dets], queue_size, max_delay)
        self.time_sync.registerCallback(self.SyncCallback)
        ####################################
       # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
       # self.out = cv2.VideoWriter('/home/particle/output.mp4', fourcc, 10, (640, 480))
        ####################################
  
    def SyncCallback(self, img_msg, detections_msg):
        img = self.cvbridge.imgmsg_to_cv2(img_msg)

        #self.out.write(img)

        img_bbox = img.copy()
        #new_width = 480
        #offset = (640 - 480) // 2
        #cropped_img = img[:, offset:offset + new_width]
        #img_bbox = cv2.resize(cropped_img, (320, 320), interpolation=cv2.INTER_AREA)

        scale = 480/320
        x_off = (640-480)/2
        y_off = (480-480)/2

        for det2D in detections_msg.detections:
            w = det2D.bbox.size_x
            h = det2D.bbox.size_y 
            x1 = int((det2D.bbox.center.position.x - w/2) * scale + x_off)
            y1 = int((det2D.bbox.center.position.y - h/2) * scale + y_off)
            x2 = int((det2D.bbox.center.position.x + w/2) * scale + x_off)
            y2 = int((det2D.bbox.center.position.y + h/2) * scale + y_off)
            #x1 = int(det2D.bbox.center.position.x)
            #y1 = int(det2D.bbox.center.position.y)
            #x2 = int(det2D.bbox.center.position.x + w)
            #y2 = int(det2D.bbox.center.position.y + h)
            img_bbox = cv2.rectangle(img_bbox, (x1, y1), (x2, y2), (255, 0, 0), 2)


        img_bbox_msg = self.cvbridge.cv2_to_imgmsg(img_bbox, 'rgb8')
        self.pub_img.publish(img_bbox_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        img_bbox_node = ImageBboxNode()
        rclpy.spin(img_bbox_node)
    except KeyboardInterrupt:
        pass
    finally:
        img_bbox_node.destroy_node()
        #self.out.release()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
