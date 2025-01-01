# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    input_image_topic = LaunchConfiguration("input_image_topic")
    input_image_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        description="ROS Topic Name of sensor_msgs/msg/Image message",
        default_value="/camera/camera/color/image_raw",   ## realsense
        # default_value="/rgb/image_raw",                   ## azure_kinect
    )

    weight_file = LaunchConfiguration("weight_file")
    weight_file_cmd = DeclareLaunchArgument(
        "weight_file", description="weight file path",
        default_value=os.path.join(get_package_share_directory("lightweight_human_pose_estimation"), "weights", "checkpoint_iter_370000.pth"),
    )

    init_detection = LaunchConfiguration("init_detection")
    init_detection_cmd = DeclareLaunchArgument(
        "init_detection", default_value="True", description="Whether to start Human Pose Estimation enabled"
    )

    height_size = LaunchConfiguration("height_size")
    height_size_cmd = DeclareLaunchArgument(
        "height_size",
        default_value="256",
        description="Image height for inference",
    )

    only_cpu = LaunchConfiguration("only_cpu")
    only_cpu_cmd = DeclareLaunchArgument(
        "only_cpu",
        default_value="False",
        description="only CPU : True, use GPU : False",
    )

    track = LaunchConfiguration("track")
    track_cmd = DeclareLaunchArgument(
        "track",
        default_value="True",
        description="tracker flag",
    )

    smooth = LaunchConfiguration("smooth")
    smooth_cmd = DeclareLaunchArgument(
        "image_show",
        default_value="True",
        description="smoother flag",
    )

    image_show = LaunchConfiguration("image_show")
    image_show_cmd = DeclareLaunchArgument(
        "image_show",
        default_value="False",
        description="image show flag",
    )

    keypoint_dictionary = os.path.join(
        get_package_share_directory("lightweight_human_pose_estimation"),
        "keypoints",
        "key_point_dictionary.yaml"
        )

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="human_pose", description="Namespace for the nodes"
    )

    human_pose_2d_cmd = Node(
        package="lightweight_human_pose_estimation",
        executable="human_pose_2d",
        name="human_pose_2d",
        namespace=namespace,
        parameters=[
            {
                "input_image_topic": input_image_topic,
                "weight_file": weight_file,
                "init_detection": init_detection,
                "height_size": height_size,
                "only_cpu": only_cpu,
                "track": track,
                "smooth": smooth,
                "image_show": image_show,
            },
            keypoint_dictionary
        ],
        output="screen"
    )

    use_3d = LaunchConfiguration("use_3d")
    use_3d_cmd = DeclareLaunchArgument(
        "use_3d", default_value="False", description="Whether to activate 3D detections"
    )

    return LaunchDescription(
        [
            use_3d_cmd,
            input_image_topic_cmd,
            weight_file_cmd,
            init_detection_cmd,
            height_size_cmd,
            only_cpu_cmd,
            track_cmd,
            smooth_cmd,
            image_show_cmd,
            namespace_cmd,
            human_pose_2d_cmd,
        ]
    )
