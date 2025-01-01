#!/bin/bash

echo "╔══╣ Install: Lightweight Human Pose Estimation (STARTING) ╠══╗"


# Keep track of the current directory
CRT_DIR=$(pwd)


# Clone required packages
cd ..
git clone -b feature/humble-devel https://github.com/TeamSOBITS/sobits_msgs

sudo apt-get update
sudo apt-get install -y \
    v4l-utils

sudo apt-get install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-pluginlib \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs

sudo apt-get install -y \
    ros-${ROS_DISTRO}-v4l2-camera \
    ros-${ROS_DISTRO}-camera-calibration \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-common \
    ros-${ROS_DISTRO}-image-proc

python3 -m pip install -U pip
python3 -m pip install \
    pycocotools \
    numpy

cd lightweight_human_pose_estimation/weights/
wget https://download.01.org/opencv/openvino_training_extensions/models/human_pose_estimation/checkpoint_iter_370000.pth
cd ..

# # Setting `lubuvc_camera` permission
# echo "SUBSYSTEMS==\"usb\", ENV{DEVTYPE}==\"usb_device\", ATTRS{idVendor}==\"0458\", ATTRS{idProduct}==\"708c\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/99-uvc.rules
# sudo udevadm control --reload-rules
# sudo udevadm trigger

# # USB Reload
# sudo /etc/init.d/udev reload

# v4l2-ctl --list-devices
# v4l2-ctl --list-formats-ext

echo "╚══╣ Install: lightweight_human_pose_estimation (FINISHED) ╠══╝"