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
 * @brief Fixposition SDK: tests for fpsdk::ros1::utils
 */

/* LIBC/STL */

/* EXTERNAL */
#include <fpsdk_ros1/ext/ros_console.hpp>
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_ros1/utils.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::ros1::utils;

TEST(UtilsTest, ConvTime)
{
    const ros::Time ros_time(1723650283, 125000000);
    DEBUG("ros::Time %" PRIu32 " %" PRIu32, ros_time.sec, ros_time.nsec);

    const fpsdk::common::time::Time sdk_time = ConvTime(ros_time);
    DEBUG("SDK Time  %" PRIu32 " %" PRIu32, sdk_time.sec_, sdk_time.nsec_);
    EXPECT_EQ(sdk_time.sec_, ros_time.sec + 27);  // 1723650310
    EXPECT_EQ(sdk_time.nsec_, ros_time.nsec);

    const ros::Time ros_time_again = ConvTime(sdk_time);
    DEBUG("ros::Time %" PRIu32 " %" PRIu32, ros_time_again.sec, ros_time_again.nsec);

    EXPECT_EQ(ros_time_again.sec, ros_time.sec);
    EXPECT_EQ(ros_time_again.nsec, ros_time.nsec);
}

// ---------------------------------------------------------------------------------------------------------------------

// This should be the last test as it messes with the console and we may want to use DEBUG() etc. in the tests
TEST(UtilsTest, RedirectLoggingToRosConsole)
{
    // Silence the ROS console
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    // We're not actually testing much more than checking that this doesn't crash.
    // Run the test with -v -v -v and set ROS console level to Debug (above) to make it a bit more interesting in the
    // output.
    // clang-format off
    //   clear; make INSTALL_PREFIX=fpsdk BUILD_TYPE=Debug build && ROSCONSOLE_FORMAT='${severity} ${time:%Y-%m-%d %H:%M:%S.%f} ${logger} - ${message}' build/Debug/fpsdk_ros1/fpsdk_ros1_utils_test -v -v -v
    // clang-format on

    ROS_DEBUG("Hello, this is a ros debug before redirect...");
    ROS_INFO("Hello, this is a ros info before redirect...");
    // ROS_WARN("Hello, this is a ros warn before redirect...");
    DEBUG("This is fpsdk_common debug before redirect...");
    INFO("This is fpsdk_common info before redirect...");
    // WARNING("This is fpsdk_common warning before redirect...");

    RedirectLoggingToRosConsole();

    ROS_DEBUG("Hello, this is a ros debug after redirect...");
    ROS_INFO("Hello, this is a ros info after redirect...");
    // ROS_WARN("Hello, this is a ros warn after redirect...");
    DEBUG("This is fpsdk_common debug after redirect...");
    INFO("This is fpsdk_common info after redirect...");
    // WARNING("This is fpsdk_common warning after redirect...");
}
// This should be the last test as it messes with the console and we may want to use DEBUG() etc. in the tests

/* ****************************************************************************************************************** */
}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto level = fpsdk::common::logging::LoggingLevel::WARNING;
    for (int ix = 0; ix < argc; ix++) {
        if ((argv[ix][0] == '-') && argv[ix][1] == 'v') {
            level++;
        }
    }
    fpsdk::common::logging::LoggingSetParams(level);
    return RUN_ALL_TESTS();
}
