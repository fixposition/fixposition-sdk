/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: to_json() helpers for some ROS1 types
 */
#ifndef __FPSDK_COMMON_TO_JSON_ROS_HPP__
#define __FPSDK_COMMON_TO_JSON_ROS_HPP__

/* LIBC/STL */

/* EXTERNAL */
#include <nlohmann/json.hpp>

/* PACKAGE */
#include "../ros1.hpp"
#include "../string.hpp"
#include "../time.hpp"

#ifndef _DOXYGEN_  // not documenting these
/* ****************************************************************************************************************** */
namespace ros {

inline void to_json(nlohmann::json& j, const Time& m)
{
    j = nlohmann::json::object({ { "sec", m.sec }, { "nsec", m.nsec } });
}

}  // namespace ros
/* ****************************************************************************************************************** */
namespace std_msgs {

inline void to_json(nlohmann::json& j, const Header& m)
{
    j = nlohmann::json::object({ { "seq", m.seq }, { "stamp", m.stamp }, { "frame_id", m.frame_id } });
}

}  // namespace std_msgs
/* ****************************************************************************************************************** */
namespace geometry_msgs {

inline void to_json(nlohmann::json& j, const Vector3& m)
{
    j = nlohmann::json::object({ { "x", m.x }, { "y", m.y }, { "z", m.z } });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const Point& m)
{
    j = nlohmann::json::object({ { "x", m.x }, { "y", m.y }, { "z", m.z } });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const Quaternion& m)
{
    j = nlohmann::json::object({ { "x", m.x }, { "y", m.y }, { "z", m.z }, { "w", m.w } });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const Transform& m)
{
    j = nlohmann::json::object({
        { "translation", m.translation },
        { "rotation", m.rotation },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const TransformStamped& m)
{
    j = nlohmann::json::object({
        { "header", m.header },
        { "child_frame_id", m.child_frame_id },
        { "transform", m.transform },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const Pose& m)
{
    j = nlohmann::json::object({
        { "position", m.position },
        { "orientation", m.orientation },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const PoseWithCovariance& m)
{
    j = nlohmann::json::object({
        { "pose", m.pose },
        { "covariance", m.covariance },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const Twist& m)
{
    j = nlohmann::json::object({
        { "linear", m.linear },
        { "angular", m.angular },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const TwistWithCovariance& m)
{
    j = nlohmann::json::object({
        { "twist", m.twist },
        { "covariance", m.covariance },
    });
}

}  // namespace geometry_msgs
/* ****************************************************************************************************************** */
namespace sensor_msgs {

inline void to_json(nlohmann::json& j, const Imu& m)
{
    j = nlohmann::json::object({
        { "header", m.header },
        { "orientation", m.orientation },
        { "orientation_covariance", m.orientation_covariance },
        { "angular_velocity", m.angular_velocity },
        { "linear_acceleration", m.linear_acceleration },
        { "linear_acceleration_covariance", m.linear_acceleration_covariance },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const Temperature& m)
{
    j = nlohmann::json::object({
        { "header", m.header },
        { "temperature", m.temperature },
        { "variance", m.variance },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const Image& m)
{
    j = nlohmann::json::object({
        { "header", m.header },
        { "height", m.height },
        { "width", m.width },
        { "encoding", m.encoding },
        { "is_bigendian", m.is_bigendian },
        { "step", m.step },
        { "data_b64", fpsdk::common::string::Base64Enc(m.data) },
    });
}

}  // namespace sensor_msgs
/* ****************************************************************************************************************** */
namespace tf2_msgs {

inline void to_json(nlohmann::json& j, const TFMessage& m)
{
    j = nlohmann::json::object({
        { "transforms", m.transforms },
    });
}

}  // namespace tf2_msgs
/* ****************************************************************************************************************** */
namespace nav_msgs {

inline void to_json(nlohmann::json& j, const Odometry& m)
{
    j = nlohmann::json::object({
        { "header", m.header },
        { "child_frame_id", m.child_frame_id },
        { "pose", m.pose },
        { "twist", m.twist },
    });
}

}  // namespace nav_msgs
/* ****************************************************************************************************************** */
#endif  // !_DOXYGEN_
#endif  // __FPSDK_COMMON_TO_JSON_ROS_HPP__
