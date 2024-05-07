import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from ultralytics import YOLO

class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')
        self.publisher_ = self.create_publisher(Image, '/rgb/image', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10)
        self.subscription 
        self.bridge = CvBridge()
        self.model = YOLO('yolov8m.pt')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        results = self.model(cv_image)

        person_detected = False
        for result in results.pred:
            if result['name'] == 'person':
                person_detected = True
                break

        if person_detected:
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    rclpy.spin(object_detector)
    object_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
