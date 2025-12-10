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
 * @brief Fixposition SDK: tests for fpsdk_ros1 messages
 */

/* LIBC/STL */

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_ros1/msgs.hpp>

namespace {
/* ****************************************************************************************************************** */

// Checks that some of the messages didn't change. If we change the message definition, we change the MD5 sum of the
// message and make it binary incompatible. I.e. we cannot use the message from old bags (using the old .msg) with the
// changed .msg. In Python this is not a problem (as long no existing field is changed), but in c++ instantiating the
// message is compiled-in and not dynamic (as in Python). Note that changing the comments in a .msg file does not change
// the MD5 sum. Neither does changing the type names ("data type" in ROS speak) if done consistently (e.g. rename all
// fpsdk_ros1/Foo to fpsdk_ros1/Bar in all .msg). However, changing name of the field does change the MD5 sum, and so
// does changing the order, adding or removing fields.
TEST(MsgsTest, MustNeverChange)
{
    // clang-format off
    EXPECT_EQ(std::string(ros::message_traits::md5sum<fpsdk::ros1::msgs::ParserMsg >()), "74030d9f5f1291a0532d94562740ed9d");
    // clang-format on
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
