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
 * @brief Fixposition SDK: tests for fpsdk::common::math
 */

/* LIBC/STL */

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/math.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::math;

TEST(MathTest, Dummy)
{
    EXPECT_TRUE(true);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(MathTest, DegToRad)
{
    EXPECT_EQ(DegToRad(0.0), 0.0);
    EXPECT_EQ(DegToRad(90.0), M_PI / 2.0);
    EXPECT_EQ(DegToRad(-90.0), -M_PI / 2.0);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(MathTest, RadToDeg)
{
    EXPECT_EQ(RadToDeg(0.0), 0.0);
    EXPECT_EQ(RadToDeg(M_PI / 2.0), 90.0);
    EXPECT_EQ(RadToDeg(-M_PI / 2.0), -90.0);
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
