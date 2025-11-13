// Wrapper to suppress warnings from ROS headers
#ifndef __FPSDK_ROS2_EXT_MSGS_HPP__
#define __FPSDK_ROS2_EXT_MSGS_HPP__
#pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wpedantic"
// #pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#pragma GCC diagnostic pop
#endif  // __FPSDK_ROS2_EXT_MSGS_HPP__
