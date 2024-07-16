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
 * @brief Fixposition SDK: tests for fp::ros1::utils
 */

/* LIBC/STL */

/* EXTERNAL */
#include <gtest/gtest.h>
#include "fpros1/ext/ros_console.hpp"

/* PACKAGE */
#include <fpcommon/logging.hpp>
#include <fpros1/utils.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fp::ros1::utils;

TEST(UtilsTest, RedirectLoggingToRosConsole)
{
    // Silence the ROS console
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    // We're not actually testing much more than checking that this doesn't crash.
    // Run the test with -v -v -v to make it a bit more interesting in the output.

    ROS_DEBUG("Hello, this is a ros debug before redirect...");
    ROS_INFO("Hello, this is a ros info before redirect...");
    // ROS_WARN("Hello, this is a ros warn before redirect...");
    DEBUG("This is fpcommon debug before redirect...");
    INFO("This is fpcommon info before redirect...");
    // WARNING("This is fpcommon warning before redirect...");

    RedirectLoggingToRosConsole();

    ROS_DEBUG("Hello, this is a ros debug after redirect...");
    ROS_INFO("Hello, this is a ros info after redirect...");
    // ROS_WARN("Hello, this is a ros warn after redirect...");
    DEBUG("This is fpcommon debug after redirect...");
    INFO("This is fpcommon info after redirect...");
    // WARNING("This is fpcommon warning after redirect...");
}

/* ****************************************************************************************************************** */
}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto level = fp::common::logging::LoggingLevel::WARNING;
    for (int ix = 0; ix < argc; ix++) {
        if ((argv[ix][0] == '-') && argv[ix][1] == 'v') {
            level++;
        }
    }
    fp::common::logging::LoggingSetParams(level);
    return RUN_ALL_TESTS();
}
