cmake_minimum_required(VERSION 3.10)
project(robocon25_sim)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

add_library(robocon_cmd_vel_model_plugin SHARED
  src/cmd_vel_model_plugin.cpp
)
ament_target_dependencies(robocon_cmd_vel_model_plugin
  gazebo_dev
  gazebo_ros
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
)

install(TARGETS robocon_cmd_vel_model_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY launch materials meshes models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
