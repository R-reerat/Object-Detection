import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from ultralytics import YOLO

class ObjectTracker(Node):

    def __init__(self):
        super().__init__('object_tracker')
        self.subscription = self.create_subscription(
            Image,
            '/rgb/image',
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

        for result in results.pred:
            if result['name'] == 'person':
                self.say_hello()
                break

        cv2.imshow("Object Detection", cv_image)
        cv2.waitKey(1)

    def say_hello(self):
        print("Hello there!")

def main(args=None):
    rclpy.init(args=args)
    object_tracker = ObjectTracker()
    rclpy.spin(object_tracker)
    object_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
