#!/usr/bin/python3

# Python
import copy

# Object detection
import cv2
import numpy as np
from ultralytics import YOLO

# ROS
import rospy
from sensor_msgs.msg import Image
from turtlebot3_object_tracker.srv import Detection, DetectionResponse

class ImageProcessor:
    def __init__(self) -> None:
        self.image_msg = Image() 
        self.image_res = 240, 320, 3 
        self.image_np = np.zeros(self.image_res) 

        self.camera_subscriber = rospy.Subscriber("/follower/camera/image", Image, callback=self.camera_listener)

        # Load YOLO model
        self.model = YOLO('yolov8n.pt')

        self.human_detection_server = rospy.Service('detection', Detection, self.human_detection)
        self.bounding_boxes = []

        self.update_view()

    def human_detection(self, req):
        self.bounding_boxes = []
        res = DetectionResponse()

        # Perform object detection
        results = self.model.predict(self.image_np, conf=0.25)

        # ดึงข้อมูลของวัตถุที่ตรวจจับได้จากผลลัพธ์
        objs = results.xyxy[0].numpy()
        obj_list = self.model.names

        if objs.shape[0] != 0: 
            for obj in objs:
                detected_obj = obj_list[int(obj[5])]
                if detected_obj == 'person':
                    x1, y1, x2, y2 = obj[0], obj[1], obj[2], obj[3]
                    self.bounding_boxes.append([x1, y1, x2, y2])

                    # กำหนดค่าข้อมูลใน response
                    res.box_x = x1
                    res.box_y = y1
                    res.box_width = x2 - x1
                    res.box_height = y2 - y1
                    res.image_width = self.image_res[1]
                    res.image_height = self.image_res[0]
                    res.in_sight_of_robot = True
        else:
            res.in_sight_of_robot = False

        return res

    def camera_listener(self, msg: Image):
        self.image_msg.data = copy.deepcopy(msg.data)

    def update_view(self):
        try:
            while not rospy.is_shutdown():
                if len(self.image_msg.data) == 0: 
                    continue

                self.image_np = np.frombuffer(self.image_msg.data, dtype=np.uint8)
                self.image_np = self.image_np.reshape(self.image_res)

                frame = copy.deepcopy(self.image_np)

                for bbox in self.bounding_boxes:
                    x1, y1, x2, y2 = bbox
                    frame = cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 255, 255), 2)

                cv2.imshow("robot_view", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)

        except rospy.exceptions.ROSInterruptException:
            pass

if __name__ == "__main__":
    rospy.init_node("image_processor", anonymous=True)
    rospy.on_shutdown(cv2.destroyAllWindows)

    image_processor = ImageProcessor()

    rospy.spin()
