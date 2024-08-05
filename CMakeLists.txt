cmake_minimum_required(VERSION 3.10.2)
project(lightweight_human_pose_estimation)

## Compile as C++14, supported in ROS Noetic and newer
add_compile_options(-std=c++14)

find_package(OpenCV 4.8.0 REQUIRED PATHS "/opt/opencv/opencv_4.8.0/")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PoseFlame.msg"
  "msg/KeyPoint2D.msg"
  "msg/KeyPoint3D.msg"
  "msg/KeyPoint2DArray.msg"
  "msg/KeyPoint3DArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

## Python
# ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  script/human_pose_2d.py
  DESTINATION lib/${PROJECT_NAME}
)
## Python end

## Declare a C++ executable
# add_executable(human_pose_3d src/human_pose_3d.cpp)
# ament_target_dependencies(human_pose_3d rclcpp sensor_msgs geometry_msgs std_msgs tf2_ros cv_bridge)

## Specify libraries to link a library or executable target against
# target_link_libraries(human_pose_3d ${OpenCV_LIBRARIES})

# install(TARGETS human_pose_3d
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()