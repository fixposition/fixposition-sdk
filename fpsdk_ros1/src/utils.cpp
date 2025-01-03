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
#include <cstring>

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

static char g_logger_name[100];

static void sLoggingFn(const LoggingParams& /*params*/, const LoggingLevel level, const char* str)
{
    switch (level) {  // clang-format off
        case LoggingLevel::TRACE:   ROS_LOG(ros::console::levels::Debug, g_logger_name, "%s", str); break;
        case LoggingLevel::DEBUG:   ROS_LOG(ros::console::levels::Debug, g_logger_name, "%s", str); break;
        case LoggingLevel::INFO:    ROS_LOG(ros::console::levels::Info,  g_logger_name, "%s", str); break;
        case LoggingLevel::NOTICE:  ROS_LOG(ros::console::levels::Info,  g_logger_name, "%s", str); break;
        case LoggingLevel::WARNING: ROS_LOG(ros::console::levels::Warn,  g_logger_name, "%s", str); break;
        case LoggingLevel::ERROR:   ROS_LOG(ros::console::levels::Error, g_logger_name, "%s", str); break;
        case LoggingLevel::FATAL:   ROS_LOG(ros::console::levels::Fatal, g_logger_name, "%s", str); break;
    }  // clang-format on
}

void RedirectLoggingToRosConsole(const char* logger_name)
{
    LoggingParams params = LoggingGetParams();
    params.fn_ = sLoggingFn;
    params.level_ = LoggingLevel::TRACE;  // We leave it up to ROS to decide what to print
    // Set logger name. Note that ROSCONSOLE_DEFAULT_NAME here is the "ros.fpsdk_ros1" package name. However, when
    // called from the app (node) then the default argument to logger_name is that package's ROSCONSOLE_DEFAULT_NAME,
    // e.g. "ros.ros1_fpsdk_demo"
    std::snprintf(
        g_logger_name, sizeof(g_logger_name), "%s", logger_name != NULL ? logger_name : ROSCONSOLE_DEFAULT_NAME);
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

bool LoadRosParam(const std::string& name, std::vector<std::string>& value)
{
    return LoadRosParamEx(name, value);
}

// ---------------------------------------------------------------------------------------------------------------------

ros::Time ConvTime(const fpsdk::common::time::Time& time)
{
    const auto rt = time.GetRosTime();
    return ros::Time(rt.sec_, rt.nsec_);
}

fpsdk::common::time::Time ConvTime(const ros::Time& time)
{
    return fpsdk::common::time::Time::FromRosTime({ time.sec, time.nsec });
}

/* ****************************************************************************************************************** */
}  // namespace utils
}  // namespace ros1
}  // namespace fpsdk
