## Requirements
- [opencv-python]
- [ROS Noetic](http://wiki.ros.org/noetic)
- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3), [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs) and [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
- [ultralytics](https://docs.ultralytics.com/quickstart/#install-ultralytics)

## Components
1. `image_processor.py`
- รับภาพจากกล้องและนำมาตรวจจับวัตถุโดยใช้ YOLOv8
- หากตรวจจับเจอวัตถุ(ในที่นี้คือคน) โปรแกรมจะส่งภาพที่ตรวจจับได้ไปยัง detection.py ผ่านอี topic '/rgb/image'
2. `detection.py`
- รับภาพที่ส่งมาจาก image_processor.py ผ่านอี topic '/rgb/image'
- ตรวจสอบภาพว่ามีคนหรือไม่ ถ้าเจอคนปริ้นข้อความ "Hello there!" ใน terminal เพื่อทักทาย

## Build this?
1. Install the requirements
2. Clone this package beside your `turtlebot3` packages.
3. Navigate to the root directory of your `catkin` workspace.
4. Do `catkin_make`.

## this is turtlebot3 **burger** not **waffle** you can change
