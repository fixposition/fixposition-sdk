/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG
 *  /  /\  \   License: MIT (see the LICENSE file)
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: ROS1 utilities
 *
 * @page FPROS1_UTILS ROS1 utilities
 *
 * @todo add documentation
 *
 */
#ifndef __FPROS1_UTILS_HPP__
#define __FPROS1_UTILS_HPP__

/* LIBC/STL */

/* EXTERNAL */

/* Fixposition SDK */

/* PACKAGE */

namespace fp {
namespace ros1 {
/**
 * @brief ROS1 utilities
 */
namespace utils {
/* ****************************************************************************************************************** */

/**
 * @brief Redirect fp:common::logging to ROS console
 *
 * This configures the fp::common::logging facility to output via the ROS console. This does *not* configure the ROS
 * console (logger level, logger name, etc.).
 *
 * The mapping of fp::common::logging::LoggingLevel to ros::console::levels is as follows:
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
}  // namespace ros1
}  // namespace fp
#endif  // __FPROS1_UTILS_HPP__
