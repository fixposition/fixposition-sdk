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
 * @brief Fixposition SDK: ROS1 utilities
 *
 * @page FPSDK_ROS1_UTILS ROS1 utilities
 *
 * **API**: fpsdk_ros1/utils.hpp and fpsdk::ros1::utils
 *
 */
#ifndef __FPSDK_ROS1_UTILS_HPP__
#define __FPSDK_ROS1_UTILS_HPP__

/* LIBC/STL */
#include <cstring>
#include <string>
#include <vector>

/* EXTERNAL */
#include <fpsdk_ros1/ext/ros_console.hpp>
#include <fpsdk_ros1/ext/ros_time.hpp>

/* Fixposition SDK */
#include <fpsdk_common/time.hpp>

/* PACKAGE */

namespace fpsdk {
namespace ros1 {
/**
 * @brief ROS1 utilities
 */
namespace utils {
/* ****************************************************************************************************************** */

/**
 * @brief Redirect fpsdk:common::logging to ROS console
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
 *
 * @param[in]  logger_name  The name of the logger. The default value should give the caller package's
 *                          ROSCONSOLE_DEFAULT_NAME, for example, "ros1_fpsdk_demo". That is, typically this argument
 *                          should be left empty (the default value).
 */
void RedirectLoggingToRosConsole(const char* logger_name = ROSCONSOLE_DEFAULT_NAME /* = caller's package name */);

/**
 * @brief Loads a parameter from the ROS parameter server (int)
 *
 * @param[in]  name   The parameter name
 * @param[out] value  The value
 *
 * @returns true if parameter found and loaded, false otherwise
 */
bool LoadRosParam(const std::string& name, int& value);

/**
 * @brief Loads a parameter from the ROS parameter server (string)
 *
 * @param[in]  name   The parameter name
 * @param[out] value  The value
 *
 * @returns true if parameter found and loaded, false otherwise
 */
bool LoadRosParam(const std::string& name, std::string& value);

/**
 * @brief Loads a parameter from the ROS parameter server (bool)
 *
 * @param[in]  name   The parameter name
 * @param[out] value  The value
 *
 * @returns true if parameter found and loaded, false otherwise
 */
bool LoadRosParam(const std::string& name, bool& value);

/**
 * @brief Loads a parameter from the ROS parameter server (float)
 *
 * @param[in]  name   The parameter name
 * @param[out] value  The value
 *
 * @returns true if parameter found and loaded, false otherwise
 */
bool LoadRosParam(const std::string& name, float& value);

/**
 * @brief Loads a parameter from the ROS parameter server (double)
 *
 * @param[in]  name   The parameter name
 * @param[out] value  The value
 *
 * @returns true if parameter found and loaded, false otherwise
 */
bool LoadRosParam(const std::string& name, double& value);

/**
 * @brief Loads a parameter from the ROS parameter server (list of strings)
 *
 * @param[in]  name   The parameter name
 * @param[out] value  The value
 *
 * @returns true if parameter found and loaded, false otherwise
 */
bool LoadRosParam(const std::string& name, std::vector<std::string>& value);

/**
 * @brief Convert to ROS time (atomic -> POSIX)
 *
 * @param[in]  time  The Time object (atomic)
 *
 * @returns the ROS time object (POSIX)
 */
ros::Time ConvTime(const fpsdk::common::time::Time& time);

/**
 * @brief Convert from ROS time (POSIX -> atomic)
 *
 * @param[in]  time  The ROS time object (POSIX)
 *
 * @returns the Time object (atomic)
 */
fpsdk::common::time::Time ConvTime(const ros::Time& time);

/* ****************************************************************************************************************** */
}  // namespace utils
}  // namespace ros1
}  // namespace fpsdk
#endif  // __FPSDK_ROS1_UTILS_HPP__
