#ifndef ROBOCON_LOCALIZATION_ROS2_COMPAT_H
#define ROBOCON_LOCALIZATION_ROS2_COMPAT_H

#include <rclcpp/rclcpp.hpp>

#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("robocon_localization"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("robocon_localization"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("robocon_localization"), __VA_ARGS__)

#endif  // ROBOCON_LOCALIZATION_ROS2_COMPAT_H
