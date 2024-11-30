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
 * @brief Fixposition SDK: tests for fpsdk::ros2::utils
 */

/* LIBC/STL */

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_ros2/utils.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::ros2::utils;

TEST(UtilsTest, ConvTime)
{
    const rclcpp::Time ros_time(1723650283, 125000000);
    DEBUG("rclcpp::Time %" PRIu64, ros_time.nanoseconds());

    const fpsdk::common::time::Time sdk_time = ConvTime(ros_time);
    DEBUG("SDK Time     %" PRIu64, sdk_time.GetNSec());
    EXPECT_EQ(sdk_time.GetNSec(), ros_time.nanoseconds() + 27000000000);  // 1723650310e9

    const rclcpp::Time ros_time_again = ConvTime(sdk_time);
    DEBUG("rclcpp::Time %" PRIu64, ros_time_again.nanoseconds());

    EXPECT_EQ(ros_time.nanoseconds(), ros_time_again.nanoseconds());
}

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
