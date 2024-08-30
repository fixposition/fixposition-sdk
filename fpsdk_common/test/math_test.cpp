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
#include <fpsdk_common/types.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::math;
using namespace fpsdk::common::types;

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

// ---------------------------------------------------------------------------------------------------------------------

TEST(MathTest, RoundToFracDigits)
{
    // clang-format off
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  0),  1.000000000000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  1),  1.100000000000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  2),  1.120000000000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  3),  1.123000000000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  4),  1.123500000000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  5),  1.123460000000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  6),  1.123457000000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  7),  1.123456800000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  8),  1.123456790000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  9),  1.123456789000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239, 10),  1.123456789100, 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239, 11),  1.123456789120, 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239, 12),  1.123456789124, 1e-15);

    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  0), -1.000000000000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  1), -1.100000000000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  2), -1.120000000000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  3), -1.123000000000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  4), -1.123500000000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  5), -1.123460000000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  6), -1.123457000000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  7), -1.123456800000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  8), -1.123456790000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  9), -1.123456789000, 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239, 10), -1.123456789100, 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239, 11), -1.123456789120, 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239, 12), -1.123456789124, 1e-15);
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
