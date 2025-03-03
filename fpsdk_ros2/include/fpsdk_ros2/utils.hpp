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
 * @brief Fixposition SDK: ROS2 utilities
 *
 * @page FPSDK_ROS2_UTILS ROS2 utilities
 *
 * **API**: fpsdk_ros2/utils.hpp and fpsdk::ros2::utils
 *
 */
#ifndef __FPSDK_ROS2_UTILS_HPP__
#define __FPSDK_ROS2_UTILS_HPP__

/* LIBC/STL */

/* EXTERNAL */
#include <fpsdk_ros2/ext/rclcpp.hpp>

/* Fixposition SDK */
#include <fpsdk_common/time.hpp>

/* PACKAGE */

namespace fpsdk {
namespace ros2 {
/**
 * @brief ROS2 utilities
 */
namespace utils {
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
void RedirectLoggingToRosConsole(const char* logger_name = "fpsdk_ros2");

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

/* ****************************************************************************************************************** */
}  // namespace utils
}  // namespace ros2
}  // namespace fpsdk
#endif  // __FPSDK_ROS2_UTILS_HPP__
