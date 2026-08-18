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
 * @brief Fixposition SDK: ROS2 types and utils
 *
 * @page FPSDK_COMMON_ROS2 ROS2 types and utils
 *
 * **API**: fpsdk_common/ros2.hpp and fpsdk::common::ros2
 *
 * This is only available when built in a ROS2 environment.
 */
#ifndef __FPSDK_COMMON_ROS2_HPP__
#define __FPSDK_COMMON_ROS2_HPP__
#if FPSDK_USE_ROS2 || defined(_DOXYGEN_)

/* LIBC/STL */
#  include <cstdint>
#  include <string>

/* EXTERNAL */

/* ROS2 */
#  pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wpedantic"
// #pragma GCC diagnostic ignored "-Wunused-parameter"
#  pragma GCC diagnostic ignored "-Wshadow"
#  include <rclcpp/rclcpp.hpp>
//
#  include <rosbag2_cpp/writer.hpp>
#  include <rosbag2_storage/storage_options.hpp>
//
#  include <nav_msgs/msg/odometry.hpp>
#  include <sensor_msgs/msg/image.hpp>
#  include <sensor_msgs/msg/imu.hpp>
#  include <sensor_msgs/msg/temperature.hpp>
#  include <std_msgs/msg/byte_multi_array.hpp>
#  include <tf2_msgs/msg/tf_message.hpp>
#  pragma GCC diagnostic pop

/* PACKAGE */
#  include "fpsdk_common/ros1.hpp"
#  include "fpsdk_common/time.hpp"

namespace fpsdk {
namespace common {
/**
 * @brief ROS2 types and utils
 */
namespace ros2 {
/* ****************************************************************************************************************** */

/**
 * @brief Redirect fp:common::logging to ROS console
 *
 * This configures the fpsdk::common::logging facility to output via the ROS console. This does *not* configure the ROS
 * console (logger level, logger name, etc.).
 *
 * The mapping of fpsdk::common::logging::LoggingLevel to rclcpp levels is as follows:
 *
 * - TRACE and DEBUG --> DEBUG
 * - INFO and NOTICE --> INFO
 * - WARNING         --> WARN
 * - ERROR           --> ERROR
 * - FATAL           --> FATAL
 *
 * @param[in]  logger_name  The name of the logger. The recommended value is node->get_logger().get_name()
 */
void RedirectLoggingToRosConsole(const char* logger_name = "fpsdk_common");

/**
 * @brief Convert to ROS time (atomic -> POSIX)
 *
 * @param[in]  time        The Time object (atomic)
 * @param[in]  clock_type  The clock to use (to assume)
 *
 * @returns the ROS time object (POSIX)
 */
rclcpp::Time ConvTime(const fpsdk::common::time::Time& time, rcl_clock_type_t clock_type = RCL_ROS_TIME);

/**
 * @brief Convert from ROS time (POSIX -> atomic)
 *
 * @param[in]  time  The ROS time object (POSIX)
 *
 * @returns the Time object (atomic)
 */
fpsdk::common::time::Time ConvTime(const rclcpp::Time& time);

/**
 * @brief ROS2 bag writer helper
 */
class BagWriter
{
   public:
    BagWriter();
    ~BagWriter();

    /**
     * @brief Open bag for writing
     *
     * @param[in]  path      Path of the bag directory
     * @param[in]  mcap      Use mcap instead of sqlite3 format
     * @param[in]  compress  Compress bag more (only with mcap = true), 0 = zstd_small, >=1 = zstd_fast,
     *                       ignored with mcap = false
     *
     * @returns true if bag was sucessfully opened
     */
    bool Open(const std::string& path, const bool mcap = false, const int compress = 0);

    /**
     * @brief Close bag
     */
    void Close();

    /**
     * @brief Write a message to the bag
     *
     * @tparam     T      ROS message type
     * @param[in]  msg    The message
     * @param[in]  topic  Topic name
     * @param[in]  time   Bag record time
     *
     * @returns true if message was added, false otherwise (message definition missing)
     */
    template <typename T>
    bool WriteMessage(const T& msg, const std::string& topic, const rclcpp::Time& time)
    {
        bool ok = false;
        try {
            if (bag_) {
                bag_->write(msg, topic, time);
                ok = true;
            }
        } catch (const std::exception& ex) {
            WARNING("BagWriter: write fail: %s", ex.what());
        }
        return ok;
    }

    /**
     * @brief Write a message to the bag
     *
     * @tparam     T      ROS message type
     * @param[in]  msg    The message
     * @param[in]  topic  Topic name
     * @param[in]  time   Bag record time
     */
    template <typename T>
    bool WriteMessage(const T& msg, const std::string& topic, const common::time::RosTime& time)
    {
        return WriteMessage(msg, topic, rclcpp::Time(time.sec_, time.nsec_, RCL_ROS_TIME));
    }

    /**
     * @brief Add ROS message definition from .fpl
     *
     * @note No checks on the provided data are done!
     *
     * @param[in]  rosmsgdef  The message definition
     */
    void AddMsgDef(const common::fpl::RosMsgDef& rosmsgdef);

    /**
     * @brief Write message from .fpl
     *
     * @note No checks on the provided data are done!
     *
     * @param[in]  rosmsgbin  The recorded message
     *
     * @returns true if message was added, false otherwise (e.g. ROS1->ROS2 conversion not implemented)
     */
    bool WriteMessage(const common::fpl::RosMsgBin& rosmsgbin);

   private:
    std::unique_ptr<rosbag2_cpp::Writer> bag_;            //!< Bag file handle
    std::map<std::string, common::fpl::RosMsgDef> defs_;  //!< Message definitions (connection headers)
};
#  ifdef _DOXYGEN_

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

#  else

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

#  endif  // !_DOXYGEN_

/* ****************************************************************************************************************** */
}  // namespace ros2
}  // namespace common
}  // namespace fpsdk
#endif  // FPSDK_USE_ROS2 || _DOXYGEN_
#endif  // __FPSDK_COMMON_ROS2_HPP__
