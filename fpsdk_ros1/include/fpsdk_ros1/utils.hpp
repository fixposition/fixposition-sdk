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

/* EXTERNAL */

/* Fixposition SDK */

/* PACKAGE */

namespace fpsdk {
namespace ros1 {
/**
 * @brief ROS1 utilities
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

/* ****************************************************************************************************************** */
}  // namespace utils
}  // namespace ros1
}  // namespace fpsdk
#endif  // __FPSDK_ROS1_UTILS_HPP__
