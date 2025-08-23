import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    input_image_topic = LaunchConfiguration("input_image_topic")
    input_image_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        description="ROS Topic Name of sensor_msgs/msg/Image message. (sensor_msgs/msg/Image)",
        default_value="/camera/camera/color/image_raw",   ## realsense
        # default_value="/rgb/image_raw",                   ## azure_kinect ##
        # default_value="/camera/color/image_raw",          ## orbbec_series ##
        # default_value="/camera/rgb/image_raw",            ## xtion
    )

    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    point_cloud_topic_cmd = DeclareLaunchArgument(
        "point_cloud_topic",
        description="Detection 3D Pose from 2D Pose (sensor_msgs/msg/PointCloud2). if you select the 'point_cloud' in 'positioning_detection_mode'.",
        default_value="/camera/camera/depth/color/points",   ## realsense
        # default_value="/points2",                            ## azure_kinect ##
        # default_value="/camera/depth_registered/points",     ## orbbec_series ##
        # default_value="/camera/depth_registered/points",     ## xtion
    )

    depth_image_topic_name = LaunchConfiguration("depth_image_topic_name")
    depth_image_topic_name_cmd = DeclareLaunchArgument(
        "depth_image_topic_name",
        description="Detection 3D Pose from 2D Pose (sensor_msgs/msg/Image). if you select the 'depth_image' in 'positioning_detection_mode'.",
        default_value="/camera/camera/depth/image_rect_raw", ## realsense
        # default_value="", ## azure_kinect ##
        # default_value="", ## orbbec_series ##
        # default_value="/camera/depth/image_raw", ## xtion
    )

    info_topic_name = LaunchConfiguration("info_topic_name")
    info_topic_name_cmd = DeclareLaunchArgument(
        "info_topic_name",
        description="Setup the camera info topic name. (sensor_msgs/msg/CameraInfo)",
        default_value="/camera/camera/color/camera_info", ## realsense
        # default_value="", ## azure_kinect ##
        # default_value="", ## orbbec_series ##
        # default_value="/camera/rgb/camera_info", ## xtion
    )

    positioning_detection_mode = LaunchConfiguration("positioning_detection_mode")
    positioning_detection_mode_cmd = DeclareLaunchArgument(
        "positioning_detection_mode",
        description="Choose of ['point_cloud', 'depth_image']",
        default_value="depth_image",
    )

    weight_file = LaunchConfiguration("weight_file")
    weight_file_cmd = DeclareLaunchArgument(
        "weight_file", description="weight file path",
        default_value=os.path.join(get_package_share_directory("lightweight_human_pose_estimation"), "weights", "checkpoint_iter_370000.pth"),
    )

    base_frame_name = LaunchConfiguration("base_frame_name")
    base_frame_name_cmd = DeclareLaunchArgument(
        "base_frame_name", description="Base frame name for the node. (String)",
        default_value="base_footprint",
    )

    enable_id = LaunchConfiguration("enable_id")
    enable_id_cmd = DeclareLaunchArgument(
        "enable_id", description="Enable assigning IDs to detected objects (Bool)",
        default_value="False",
    )

    execute_default = LaunchConfiguration("execute_default")
    execute_default_cmd = DeclareLaunchArgument(
        "execute_default", default_value="True", description="Whether to start Human Pose Estimation enabled"
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
        "smooth",
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
                "execute_default": execute_default,
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
        "use_3d", default_value="True", description="Whether to activate 3D detections"
    )

    human_pose_3d_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("image_to_position"),
                "launch",
                "keypoint_to_3d.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "base_frame_name": base_frame_name,
            "keypoints_topic_name": "/human_pose/pose_array",
            "cloud_topic_name": point_cloud_topic,
            "depth_image_topic_name": depth_image_topic_name,
            "info_topic_name": info_topic_name,
            "execute_default": execute_default,
            "enable_id": enable_id,
            "positioning_detection_mode": positioning_detection_mode
        }.items(),
        condition=IfCondition(use_3d),  # use_3dがTrueのときのみ実行
    )

    return LaunchDescription(
        [
            use_3d_cmd,
            input_image_topic_cmd,
            point_cloud_topic_cmd,
            depth_image_topic_name_cmd,
            info_topic_name_cmd,
            positioning_detection_mode_cmd,
            weight_file_cmd,
            base_frame_name_cmd,
            enable_id_cmd,
            execute_default_cmd,
            height_size_cmd,
            only_cpu_cmd,
            track_cmd,
            smooth_cmd,
            image_show_cmd,
            namespace_cmd,
            human_pose_2d_cmd,
            human_pose_3d_cmd,
        ]
    )
