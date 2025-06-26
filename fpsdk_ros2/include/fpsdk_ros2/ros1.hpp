/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: ROS2 types conversion from ROS1
 *
 * @page FPSDK_ROS2_ROS1 ROS2 types conversion from ROS1
 *
 * **API**: fpsdk_ros2/ros1.hpp and fpsdk::ros2::ros1
 *
 */
#ifndef __FPSDK_ROS2_ROS1_HPP__
#define __FPSDK_ROS2_ROS1_HPP__

/* LIBC/STL */
#include <algorithm>

/* EXTERNAL */
#include <fpsdk_ros2/ext/msgs.hpp>

/* Fixposition SDK */
#include <fpsdk_common/ros1.hpp>

/* PACKAGE */

namespace fpsdk {
namespace ros2 {
/**
 * @brief ROS2 types conversion from ROS1
 */
namespace ros1 {
/* ****************************************************************************************************************** */
#ifdef _DOXYGEN_

// Dummy documentation
/**
 * @brief Convert ROS1 message to ROS2 message
 *
 * Several conversions in this form are implemented. See the source code for details.
 *
 * @tparam  Ros1MsgT  ROS1 message type
 * @tparam  Ros2MsgT  ROS2 message type
 * @param[in]   ros1  ROS1 message
 * @param[out]  ros2  ROS2 message
 */
template <typename Ros1MsgT, typename Ros2MsgT>
void Ros1ToRos2(Ros1MsgT& ros1, Ros2MsgT& ros2);

#else

inline void Ros1ToRos2(const ros::Time& ros1, builtin_interfaces::msg::Time& ros2)
{
    ros2.sec = ros1.sec;
    ros2.nanosec = ros1.nsec;
}

// ---------------------------------------------------------------------------------------------------------------------

inline void Ros1ToRos2(const boost::array<double, 9>& ros1, std::array<double, 9>& ros2)
{
    for (std::size_t ix = 0; ix < 9; ix++) {
        ros2[ix] = ros1[ix];
    }
}

inline void Ros1ToRos2(const boost::array<double, 36>& ros1, std::array<double, 36>& ros2)
{
    for (std::size_t ix = 0; ix < 36; ix++) {
        ros2[ix] = ros1[ix];
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void Ros1ToRos2(const std_msgs::Header& ros1, std_msgs::msg::Header& ros2)
{
    ros2.frame_id = ros1.frame_id;
    Ros1ToRos2(ros1.stamp, ros2.stamp);
}

// ---------------------------------------------------------------------------------------------------------------------

inline void Ros1ToRos2(const geometry_msgs::Quaternion& ros1, geometry_msgs::msg::Quaternion& ros2)
{
    ros2.x = ros1.x;
    ros2.y = ros1.y;
    ros2.z = ros1.z;
    ros2.w = ros1.w;
}

inline void Ros1ToRos2(const geometry_msgs::Vector3& ros1, geometry_msgs::msg::Vector3& ros2)
{
    ros2.x = ros1.x;
    ros2.y = ros1.y;
    ros2.z = ros1.z;
}

inline void Ros1ToRos2(const geometry_msgs::Point& ros1, geometry_msgs::msg::Point& ros2)
{
    ros2.x = ros1.x;
    ros2.y = ros1.y;
    ros2.z = ros1.z;
}

inline void Ros1ToRos2(const geometry_msgs::Pose& ros1, geometry_msgs::msg::Pose& ros2)
{
    Ros1ToRos2(ros1.position, ros2.position);
    Ros1ToRos2(ros1.orientation, ros2.orientation);
}

inline void Ros1ToRos2(const geometry_msgs::Twist& ros1, geometry_msgs::msg::Twist& ros2)
{
    Ros1ToRos2(ros1.linear, ros2.linear);
    Ros1ToRos2(ros1.angular, ros2.angular);
}

inline void Ros1ToRos2(const geometry_msgs::PoseWithCovariance& ros1, geometry_msgs::msg::PoseWithCovariance& ros2)
{
    Ros1ToRos2(ros1.pose, ros2.pose);
    Ros1ToRos2(ros1.covariance, ros2.covariance);
}

inline void Ros1ToRos2(const geometry_msgs::TwistWithCovariance& ros1, geometry_msgs::msg::TwistWithCovariance& ros2)
{
    Ros1ToRos2(ros1.twist, ros2.twist);
    Ros1ToRos2(ros1.covariance, ros2.covariance);
}

inline void Ros1ToRos2(const geometry_msgs::Transform& ros1, geometry_msgs::msg::Transform& ros2)
{
    Ros1ToRos2(ros1.translation, ros2.translation);
    Ros1ToRos2(ros1.rotation, ros2.rotation);
}

inline void Ros1ToRos2(const geometry_msgs::TransformStamped& ros1, geometry_msgs::msg::TransformStamped& ros2)
{
    Ros1ToRos2(ros1.header, ros2.header);
    ros2.child_frame_id = ros1.child_frame_id;
    Ros1ToRos2(ros1.transform, ros2.transform);
}

// ---------------------------------------------------------------------------------------------------------------------

inline void Ros1ToRos2(const sensor_msgs::Imu& ros1, sensor_msgs::msg::Imu& ros2)
{
    Ros1ToRos2(ros1.header, ros2.header);
    Ros1ToRos2(ros1.orientation, ros2.orientation);
    Ros1ToRos2(ros1.orientation_covariance, ros2.orientation_covariance);
    Ros1ToRos2(ros1.angular_velocity, ros2.angular_velocity);
    Ros1ToRos2(ros1.angular_velocity_covariance, ros2.angular_velocity_covariance);
    Ros1ToRos2(ros1.linear_acceleration, ros2.linear_acceleration);
    Ros1ToRos2(ros1.linear_acceleration_covariance, ros2.linear_acceleration_covariance);
}

inline void Ros1ToRos2(const sensor_msgs::Temperature& ros1, sensor_msgs::msg::Temperature& ros2)
{
    Ros1ToRos2(ros1.header, ros2.header);
    ros2.temperature = ros1.temperature;
    ros2.variance = ros1.variance;
}

inline void Ros1ToRos2(const sensor_msgs::Image& ros1, sensor_msgs::msg::Image& ros2)
{
    Ros1ToRos2(ros1.header, ros2.header);
    ros2.height = ros1.height;
    ros2.width = ros1.width;
    ros2.encoding = ros1.encoding;
    ros2.step = ros1.step;
    ros2.data = ros1.data;
}

// ---------------------------------------------------------------------------------------------------------------------

inline void Ros1ToRos2(const nav_msgs::Odometry& ros1, nav_msgs::msg::Odometry& ros2)
{
    Ros1ToRos2(ros1.header, ros2.header);
    ros2.child_frame_id = ros1.child_frame_id;
    Ros1ToRos2(ros1.pose, ros2.pose);
    Ros1ToRos2(ros1.twist, ros2.twist);
}

// ---------------------------------------------------------------------------------------------------------------------

inline void Ros1ToRos2(const tf2_msgs::TFMessage& ros1, tf2_msgs::msg::TFMessage& ros2)
{
    for (auto& tf_ros1 : ros1.transforms) {
        geometry_msgs::msg::TransformStamped tf_ros2;
        Ros1ToRos2(tf_ros1, tf_ros2);
        ros2.transforms.push_back(std::move(tf_ros2));
    }
}

#endif  // !_DOXYGEN_
/* ****************************************************************************************************************** */
}  // namespace ros1
}  // namespace ros2
}  // namespace fpsdk
#endif  // __FPSDK_ROS2_ROS1_HPP__
