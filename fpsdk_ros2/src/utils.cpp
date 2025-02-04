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

/* Fixposition SDK */
#include <fpsdk_common/logging.hpp>

/* PACKAGE */
#include "fpsdk_ros2/utils.hpp"

namespace fpsdk {
namespace ros2 {
namespace utils {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::logging;

// ---------------------------------------------------------------------------------------------------------------------

static std::unique_ptr<rclcpp::Logger> g_logger;

static void sLoggingFn(const LoggingParams& /*params*/, const LoggingLevel level, const char* str)
{
    // These will appear under the "fpsdk_ros2" logger.
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

void RedirectLoggingToRosConsole(const char* logger_name)
{
    g_logger = std::make_unique<rclcpp::Logger>(rclcpp::get_logger(logger_name));
    LoggingParams params = LoggingGetParams();
    params.fn_ = sLoggingFn;
    params.level_ = LoggingLevel::TRACE;  // We leave it up to ROS to decide what to print
    LoggingSetParams(params);
}

// ---------------------------------------------------------------------------------------------------------------------

rclcpp::Time ConvTime(const fpsdk::common::time::Time& time, rcl_clock_type_t clock_type)
{
    const auto rt = time.GetRosTime();
    return rclcpp::Time(rt.sec_, rt.nsec_, clock_type);
}

fpsdk::common::time::Time ConvTime(const rclcpp::Time& time)
{
    const uint64_t nsec = time.nanoseconds();
    return fpsdk::common::time::Time::FromRosTime(
        { static_cast<uint32_t>(nsec / 1000000000), static_cast<uint32_t>(nsec % 1000000000) });
}

/* ****************************************************************************************************************** */
}  // namespace utils
}  // namespace ros2
}  // namespace fpsdk
