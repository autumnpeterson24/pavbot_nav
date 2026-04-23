from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vision_pkg = FindPackageShare("pavbot_vision")
    nav_pkg = FindPackageShare("pavbot_nav")

    dual_cam_params = PathJoinSubstitution([
        vision_pkg, "config", "dual_cam_pub.yaml"
    ])

    lane_params = PathJoinSubstitution([
        vision_pkg, "config", "lane_detector_dual_real.yaml"
    ])

    follower_params = PathJoinSubstitution([
        nav_pkg, "config", "guidance_cmd_vel_follower.yaml"
    ])

    return LaunchDescription([
        # Dual camera publisher
        Node(
            package="pavbot_vision",
            executable="dual_cam_pub",
            name="dual_cam_pub",
            output="screen",
            parameters=[dual_cam_params]
        ),

        # Static TF: base_link -> left_camera_link
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="left_camera_tf",
            output="screen",
            arguments=[
                "0.55", "0.30", "1.00",
                "-0.1", "0.5", "0.0",
                "base_link",
                "left_camera_link",
            ]
        ),

        # Static TF: base_link -> right_camera_link
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="right_camera_tf",
            output="screen",
            arguments=[
                "0.55", "-0.30", "1.00",
                "0.1", "-0.5", "0.0",
                "base_link",
                "right_camera_link",
            ]
        ),

        # Lane detector
        Node(
            package="pavbot_vision",
            executable="lane_detector_dual",
            name="lane_detector_dual",
            output="screen",
            parameters=[lane_params]
        ),

        # Direct centerline follower -> cmd_vel
        Node(
            package="pavbot_nav",
            executable="guidance_cmd_vel_follower",
            name="guidance_cmd_vel_follower",
            output="screen",
            parameters=[follower_params]
        ),
    ])