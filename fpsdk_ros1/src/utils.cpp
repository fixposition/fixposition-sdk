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
 * @brief Fixposition SDK: ROS1 utilities
 */

/* LIBC/STL */

/* EXTERNAL */
#include "fpsdk_ros1/ext/ros.hpp"

/* Fixposition SDK */
#include <fpsdk_common/logging.hpp>

/* PACKAGE */
#include "fpsdk_ros1/utils.hpp"

namespace fpsdk {
namespace ros1 {
namespace utils {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::logging;

// ---------------------------------------------------------------------------------------------------------------------

static void sLoggingFn(const LoggingParams& /*params*/, const LoggingLevel level, const char* str)
{
    // These will appear under the "ros.fpsdk_ros1" logger.
    // @todo Can we make this log to "ros1.fpsdk_common"?
    switch (level) {  // clang-format off
        case LoggingLevel::TRACE:   ROS_DEBUG("%s", str); break;
        case LoggingLevel::DEBUG:   ROS_DEBUG("%s", str); break;
        case LoggingLevel::INFO:    ROS_INFO( "%s", str); break;
        case LoggingLevel::NOTICE:  ROS_INFO( "%s", str); break;
        case LoggingLevel::WARNING: ROS_WARN( "%s", str); break;
        case LoggingLevel::ERROR:   ROS_ERROR("%s", str); break;
        case LoggingLevel::FATAL:   ROS_FATAL("%s", str); break;
    }  // clang-format on
}

void RedirectLoggingToRosConsole()
{
    LoggingParams params = LoggingGetParams();
    params.fn_ = sLoggingFn;
    params.level_ = LoggingLevel::TRACE;  // We leave it up to ROS to decide what to print
    LoggingSetParams(params);
}

// ---------------------------------------------------------------------------------------------------------------------

template <typename T>
static bool LoadRosParamEx(const std::string& name, T& value)
{
    try {
        if (ros::param::has(name)) {
            ros::param::get(name, value);
        } else {
            return false;
        }
    } catch (ros::InvalidNameException& e) {
        return false;
    }
    return true;
}

bool LoadRosParam(const std::string& name, int& value)
{
    return LoadRosParamEx(name, value);
}

bool LoadRosParam(const std::string& name, std::string& value)
{
    return LoadRosParamEx(name, value);
}

bool LoadRosParam(const std::string& name, bool& value)
{
    return LoadRosParamEx(name, value);
}

bool LoadRosParam(const std::string& name, float& value)
{
    return LoadRosParamEx(name, value);
}

bool LoadRosParam(const std::string& name, double& value)
{
    return LoadRosParamEx(name, value);
}

/* ****************************************************************************************************************** */
}  // namespace utils
}  // namespace ros1
}  // namespace fpsdk
