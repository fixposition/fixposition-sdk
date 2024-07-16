/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: MIT (see the LICENSE file)
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: ROS1 utilities
 */

/* LIBC/STL */

/* EXTERNAL */
#include "fpros1/ext/ros_console.hpp"

/* Fixposition SDK */
#include <fpcommon/logging.hpp>

/* PACKAGE */
#include "fpros1/utils.hpp"

namespace fp {
namespace ros1 {
namespace utils {
/* ****************************************************************************************************************** */

using namespace fp::common::logging;

// ---------------------------------------------------------------------------------------------------------------------

static void sLoggingFn(const LoggingParams& params, const char* str)
{
    switch (params.level_) {  // clang-format off
        case LoggingLevel::TRACE:   ROS_DEBUG("fpcommon(T) %s", str); break;
        case LoggingLevel::DEBUG:   ROS_DEBUG("fpcommon(D) %s", str); break;
        case LoggingLevel::INFO:    ROS_INFO( "fpcommon(I) %s", str); break;
        case LoggingLevel::NOTICE:  ROS_INFO( "fpcommon(N) %s", str); break;
        case LoggingLevel::WARNING: ROS_WARN( "fpcommon(W) %s", str); break;
        case LoggingLevel::ERROR:   ROS_ERROR("fpcommon(E) %s", str); break;
        case LoggingLevel::FATAL:   ROS_FATAL("fpcommon(F) %s", str); break;
    }  // clang-format on
}

void RedirectLoggingToRosConsole()
{
    LoggingParams params = LoggingGetParams();
    params.fn_ = sLoggingFn;
    params.level_ = LoggingLevel::TRACE;
    LoggingSetParams(params);
}

/* ****************************************************************************************************************** */
}  // namespace utils
}  // namespace ros1
}  // namespace fp
