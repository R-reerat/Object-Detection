cmake_minimum_required(VERSION 3.5)
project(turtlebot3_object_tracker)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find packages in the ROS 2 ecosystem
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Add executable
add_executable(${PROJECT_NAME}_node src/turtlebot3_object_tracker_node.cpp)

# Include directories
include_directories(include ${ament_INCLUDE_DIRS})

# Link dependencies
target_link_libraries(${PROJECT_NAME}_node
  ${ament_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

# Install
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package generation
ament_package()
