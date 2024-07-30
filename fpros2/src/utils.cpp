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
 * @brief Fixposition SDK: ROS2 utilities
 */

/* LIBC/STL */
#include <memory>

/* EXTERNAL */
#include "fpros2/ext/rclcpp.hpp"

/* Fixposition SDK */
#include <fpcommon/logging.hpp>

/* PACKAGE */
#include "fpros2/utils.hpp"

namespace fp {
namespace ros2 {
namespace utils {
/* ****************************************************************************************************************** */

using namespace fp::common::logging;

// ---------------------------------------------------------------------------------------------------------------------

static std::unique_ptr<rclcpp::Logger> g_logger;

static void sLoggingFn(const LoggingParams& /*params*/, const LoggingLevel level, const char* str)
{
    // These will appear under the "fpros2" logger.
    switch (level) {  // clang-format off
        case LoggingLevel::TRACE:   RCLCPP_DEBUG((*g_logger), "%s", str); break;
        case LoggingLevel::DEBUG:   RCLCPP_DEBUG((*g_logger), "%s", str); break;
        case LoggingLevel::INFO:    RCLCPP_INFO( (*g_logger), "%s", str); break;
        case LoggingLevel::NOTICE:  RCLCPP_INFO( (*g_logger), "%s", str); break;
        case LoggingLevel::WARNING: RCLCPP_WARN( (*g_logger), "%s", str); break;
        case LoggingLevel::ERROR:   RCLCPP_ERROR((*g_logger), "%s", str); break;
        case LoggingLevel::FATAL:   RCLCPP_FATAL((*g_logger), "%s", str); break;
    }  // clang-format on
}

void RedirectLoggingToRosConsole()
{
    g_logger = std::make_unique<rclcpp::Logger>(rclcpp::get_logger("fpros2"));
    LoggingParams params = LoggingGetParams();
    params.fn_ = sLoggingFn;
    params.level_ = LoggingLevel::TRACE;  // We leave it up to ROS to decide what to print
    LoggingSetParams(params);
}

/* ****************************************************************************************************************** */
}  // namespace utils
}  // namespace ros2
}  // namespace fp
