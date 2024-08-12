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
 * API: fpsdk::ros2::utils
 *
 */
#ifndef __FPSDK_ROS2_UTILS_HPP__
#define __FPSDK_ROS2_UTILS_HPP__

/* LIBC/STL */

/* EXTERNAL */

/* Fixposition SDK */

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
 * The mapping of fpsdk::common::logging::LoggingLevel to ros::console::levels is as follows:
 *
 * - TRACE and DEBUG --> DEBUG
 * - INFO and NOTICE --> INFO
 * - WARNING         --> WARN
 * - ERROR           --> ERROR
 * - FATAL           --> FATAL
 */
void RedirectLoggingToRosConsole();

/* ****************************************************************************************************************** */
}  // namespace utils
}  // namespace ros2
}  // namespace fpsdk
#endif  // __FPSDK_ROS2_UTILS_HPP__
