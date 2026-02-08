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
    # Launch args
    use_sim_time   = LaunchConfiguration("use_sim_time")
    nav2_params    = LaunchConfiguration("nav2_params")
    namespace      = LaunchConfiguration("namespace")
    autostart      = LaunchConfiguration("autostart")
    log_level      = LaunchConfiguration("log_level")
    use_composition = LaunchConfiguration("use_composition")
    container_name  = LaunchConfiguration("container_name")
    use_respawn     = LaunchConfiguration("use_respawn")

    # Paths
    sim_pkg = get_package_share_directory("pavbot_sim_gz")
    sim_launch = os.path.join(sim_pkg, "launch", "sim_bringup.launch.py")

    nav2_pkg = get_package_share_directory("nav2_bringup")
    nav2_launch = os.path.join(nav2_pkg, "launch", "navigation_launch.py")

    default_nav2_params = os.path.join(
        get_package_share_directory("pavbot_nav"),
        "config",
        "nav2_lane_follow.yaml",
    )

    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        ),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=default_nav2_params,
            description="Full path to the Nav2 params YAML",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Top-level namespace",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup the nav2 stack",
        ),
        DeclareLaunchArgument(
            "use_composition",
            default_value="False",
            description="Use composed bringup if True",
        ),
        DeclareLaunchArgument(
            "container_name",
            default_value="nav2_container",
            description="Container name if using composition",
        ),
        DeclareLaunchArgument(
            "use_respawn",
            default_value="False",
            description="Respawn nodes if they crash (composition disabled)",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="log level",
        ),

        # --- 1) Sim bringup ---
        # IMPORTANT: pass use_sim_time + namespace through so sim + nodes line up.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "namespace": namespace,
            }.items(),
        ),

        # --- 2) TF from /odom ---
        Node(
            package="pavbot_nav",
            executable="odom_tf_broadcaster",
            name="odom_tf_broadcaster",
            namespace=namespace,
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "odom_topic": "/odom",
                "odom_frame": "odom",
                "base_frame": "base_link",
            }],
        ),

        # --- 2.5) Pothole points bridge (publish /potholes/points for costmap) ---
        Node(
            package="pavbot_vision",
            executable="pothole_points_bridge",
            name="pothole_points_bridge",
            namespace=namespace,
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,


                # "input_topic": "/potholes/detections",
                # "output_topic": "/potholes/points",
                # "base_frame": "base_link",
            }],
            arguments=["--ros-args", "--log-level", log_level],
        ),

        # --- 3) Centerline -> Nav2 goals ---
        Node(
            package="pavbot_nav",
            executable="centerline_nav2_autonomy",
            name="centerline_nav2_autonomy",
            namespace=namespace,
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "centerline_topic": "/lanes/centerline",
                "global_frame": "odom",
                "robot_frame": "base_link",
                "lookahead_m": 4.0,
                "goal_update_hz": 10.0,
                "min_goal_separation_m": 1.0,
                "path_stale_sec": 0.5,
            }],
            arguments=["--ros-args", "--log-level", log_level],
        ),

        # --- 4) Nav2 bringup ---
        # CRITICAL: only pass ONE params file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "namespace": namespace,

                # This is the key: navigation_launch.py consumes params_file.
                "params_file": nav2_params,

                "autostart": autostart,
                "use_composition": use_composition,
                "container_name": container_name,
                "use_respawn": use_respawn,
                "log_level": log_level,
            }.items(),
        ),
    ])
