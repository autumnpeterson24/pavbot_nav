"""
LAUNCH: 
ros2 service call /autonomy/set_enabled std_srvs/srv/SetBool "{data: true}"


"""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params = LaunchConfiguration("nav2_params")

    sim_pkg = get_package_share_directory("pavbot_sim_gz")
    sim_launch = os.path.join(sim_pkg, "launch", "sim_bringup.launch.py")

    nav2_pkg = get_package_share_directory("nav2_bringup")
    nav2_launch = os.path.join(nav2_pkg, "launch", "navigation_launch.py")

    default_params = os.path.join(
        get_package_share_directory("pavbot_nav"),
        "config",
        "nav2_lane_follow.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        ),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=default_params,
            description="Full path to the Nav2 params YAML",
        ),

        # 1) Your existing sim bringup (Gazebo + bridges + lane_detector_dual)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
        ),

        # 2) Publish TF from /odom (C++ node in this package)
        Node(
            package="pavbot_nav",
            executable="odom_tf_broadcaster",
            name="odom_tf_broadcaster",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "odom_topic": "/odom",
                "odom_frame": "pavbot_test/odom",
                "base_frame": "pavbot_test/base_link",
            }],
        ),

        Node(
            package="pavbot_nav",
            executable="centerline_nav2_autonomy",
            name="centerline_nav2_autonomy",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "centerline_topic": "/lanes/centerline",
                "global_frame": "pavbot_test/odom",
                "robot_frame": "pavbot_test/base_link",
                "lookahead_m": 6.0,
                "goal_update_hz": 10.0,
                "min_goal_separation_m": 1.0,
                "path_stale_sec": 0.5,
            }],
        ),

        # 3) Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": nav2_params,
            }.items(),
        ),
    ])