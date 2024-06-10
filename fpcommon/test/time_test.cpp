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
 * @brief Fixposition SDK: tests for fp::common::time
 */

/* LIBC/STL */
#include <algorithm>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpcommon/logging.hpp>
#include <fpcommon/time.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fp::common::time;

TEST(TimeTest, RosTime)
{
    const RosTime t0;
    EXPECT_TRUE(t0.IsZero());
    EXPECT_EQ(t0.sec_, 0);
    EXPECT_EQ(t0.nsec_, 0);

    const RosTime t1(123, 456);
    EXPECT_FALSE(t1.IsZero());
    EXPECT_EQ(t1.sec_, 123);
    EXPECT_EQ(t1.nsec_, 456);

    const RosTime t2(123, 999999999);
    EXPECT_FALSE(t2.IsZero());
    EXPECT_EQ(t2.sec_, 123);
    EXPECT_EQ(t2.nsec_, 999999999);

    const RosTime t3(123, 999999999 + 1);
    EXPECT_FALSE(t3.IsZero());
    EXPECT_EQ(t3.sec_, 124);
    EXPECT_EQ(t3.nsec_, 0);

    const RosTime t4(123, 999999999 + 1 + 1);
    EXPECT_FALSE(t4.IsZero());
    EXPECT_EQ(t4.sec_, 124);
    EXPECT_EQ(t4.nsec_, 1);
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
    fp::common::logging::LoggingSetup(level);
    return RUN_ALL_TESTS();
}
