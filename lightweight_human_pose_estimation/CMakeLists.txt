cmake_minimum_required(VERSION 3.8)
project(lightweight_human_pose_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(OpenCV 4.6.0 REQUIRED PATHS "/opt/opencv/opencv_4.6.0/")
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(lightweight_human_pose_estimation_msgs REQUIRED)
find_package(sobits_msgs REQUIRED)


#--- for C++
# include_directories(include)
# add_executable(human_pose_3d src/human_pose_3d.cpp)
# ament_target_dependencies(human_pose_3d rclcpp std_msgs geometry_msgs sensor_msgs pcl_conversions pcl_ros tf2_ros cv_bridge lightweight_human_pose_estimation_msgs sobits_msgs rosidl_default_runtime)

# install(TARGETS
#   human_pose_3d
#   DESTINATION lib/${PROJECT_NAME}
# )
#--- end C++

#-- for Python
install(PROGRAMS
  script/human_pose_2d.py
  DESTINATION lib/${PROJECT_NAME}
)
#--- end Python

#-- for Launch
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)
#--- end Launch


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()