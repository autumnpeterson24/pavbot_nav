cmake_minimum_required(VERSION 3.8)
project(pavbot_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)
ament_target_dependencies(odom_tf_broadcaster
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

add_executable(centerline_nav2_autonomy src/centerline_nav2_autonomy.cpp)
ament_target_dependencies(centerline_nav2_autonomy
  rclcpp
  std_srvs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  rclcpp_action
  nav2_msgs
)

add_executable(corridor_path_builder src/corridor_path_builder.cpp)
ament_target_dependencies(corridor_path_builder
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  sensor_msgs
)

add_executable(corridor_path_follower src/corridor_path_follower.cpp)
ament_target_dependencies(corridor_path_follower
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

install(TARGETS
  odom_tf_broadcaster
  centerline_nav2_autonomy
  corridor_path_builder
  corridor_path_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()