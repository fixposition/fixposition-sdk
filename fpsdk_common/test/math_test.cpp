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
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  0),  1.0           , 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  1),  1.1           , 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  2),  1.12          , 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  3),  1.123         , 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  4),  1.1235        , 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  5),  1.12346       , 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  6),  1.123457      , 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  7),  1.1234568     , 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  8),  1.12345679    , 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239,  9),  1.123456789   , 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239, 10),  1.1234567891  , 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239, 11),  1.12345678912 , 1e-15);
    EXPECT_NEAR(RoundToFracDigits( 1.1234567891239, 12),  1.123456789124, 1e-15);

    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  0), -1.0           , 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  1), -1.1           , 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  2), -1.12          , 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  3), -1.123         , 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  4), -1.1235        , 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  5), -1.12346       , 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  6), -1.123457      , 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  7), -1.1234568     , 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  8), -1.12345679    , 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239,  9), -1.123456789   , 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239, 10), -1.1234567891  , 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239, 11), -1.12345678912 , 1e-15);
    EXPECT_NEAR(RoundToFracDigits(-1.1234567891239, 12), -1.123456789124, 1e-15);
    // clang-format on
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(MathTest, ClipToFracDigits)
{
    // clang-format off
    EXPECT_NEAR(ClipToFracDigits( 1.1234567891239,  0),  1.0           , 1e-15);
    EXPECT_NEAR(ClipToFracDigits( 1.1234567891239,  1),  1.1           , 1e-15);
    EXPECT_NEAR(ClipToFracDigits( 1.1234567891239,  2),  1.12          , 1e-15);
    EXPECT_NEAR(ClipToFracDigits( 1.1234567891239,  3),  1.123         , 1e-15);
    EXPECT_NEAR(ClipToFracDigits( 1.1234567891239,  4),  1.1234        , 1e-15);
    EXPECT_NEAR(ClipToFracDigits( 1.1234567891239,  5),  1.12345       , 1e-15);
    EXPECT_NEAR(ClipToFracDigits( 1.1234567891239,  6),  1.123456      , 1e-15);
    EXPECT_NEAR(ClipToFracDigits( 1.1234567891239,  7),  1.1234567     , 1e-15);
    EXPECT_NEAR(ClipToFracDigits( 1.1234567891239,  8),  1.12345678    , 1e-15);
    EXPECT_NEAR(ClipToFracDigits( 1.1234567891239,  9),  1.123456789   , 1e-15);
    EXPECT_NEAR(ClipToFracDigits( 1.1234567891239, 10),  1.1234567891  , 1e-15);
    EXPECT_NEAR(ClipToFracDigits( 1.1234567891239, 11),  1.12345678912 , 1e-15);
    EXPECT_NEAR(ClipToFracDigits( 1.1234567891239, 12),  1.123456789123, 1e-15);

    EXPECT_NEAR(ClipToFracDigits(-1.1234567891239,  0), -1.0           , 1e-15);
    EXPECT_NEAR(ClipToFracDigits(-1.1234567891239,  1), -1.1           , 1e-15);
    EXPECT_NEAR(ClipToFracDigits(-1.1234567891239,  2), -1.12          , 1e-15);
    EXPECT_NEAR(ClipToFracDigits(-1.1234567891239,  3), -1.123         , 1e-15);
    EXPECT_NEAR(ClipToFracDigits(-1.1234567891239,  4), -1.1234        , 1e-15);
    EXPECT_NEAR(ClipToFracDigits(-1.1234567891239,  5), -1.12345       , 1e-15);
    EXPECT_NEAR(ClipToFracDigits(-1.1234567891239,  6), -1.123456      , 1e-15);
    EXPECT_NEAR(ClipToFracDigits(-1.1234567891239,  7), -1.1234567     , 1e-15);
    EXPECT_NEAR(ClipToFracDigits(-1.1234567891239,  8), -1.12345678    , 1e-15);
    EXPECT_NEAR(ClipToFracDigits(-1.1234567891239,  9), -1.123456789   , 1e-15);
    EXPECT_NEAR(ClipToFracDigits(-1.1234567891239, 10), -1.1234567891  , 1e-15);
    EXPECT_NEAR(ClipToFracDigits(-1.1234567891239, 11), -1.12345678912 , 1e-15);
    EXPECT_NEAR(ClipToFracDigits(-1.1234567891239, 12), -1.123456789123, 1e-15);
    // clang-format on
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(MathTest, Bit)
{
    EXPECT_EQ(Bit<uint32_t>(0), (uint32_t)0x00000001);
    EXPECT_EQ(Bit<uint32_t>(31), (uint32_t)0x80000000);
    // too large bit number should clip to target size
    EXPECT_EQ(Bit<uint8_t>(8), (uint8_t)0x0000);
    EXPECT_EQ(Bit<uint8_t>(10), (uint8_t)0x0000);
    EXPECT_EQ(Bit<uint16_t>(16), (uint16_t)0x0000);
    EXPECT_EQ(Bit<uint16_t>(20), (uint16_t)0x0000);
    EXPECT_EQ(Bit<uint32_t>(32), (uint32_t)0x00000000);
    EXPECT_EQ(Bit<uint32_t>(35), (uint32_t)0x00000000);
    EXPECT_EQ(Bit<uint32_t>(40), (uint32_t)0x00000000);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(MathTest, CheckBitsAll)
{
    EXPECT_TRUE(CheckBitsAll(0x0f00, 0x0300));
    EXPECT_FALSE(CheckBitsAll(0x0300, 0x0f00));
    // Combinations of variable, constexpr and enums
    uint16_t var_mask = 0x0f00;
    uint16_t var_bits = 0x0300;
    constexpr uint16_t const_mask = 0x0f00;
    constexpr uint16_t const_bits = 0x0300;
    enum class foo : uint16_t
    {
        enum_mask = 0x0f00,
        enum_bits = 0x0300
    };
    EXPECT_TRUE(CheckBitsAll(var_mask, var_bits));
    EXPECT_TRUE(CheckBitsAll(var_mask, const_bits));
    EXPECT_TRUE(CheckBitsAll(var_mask, EnumToVal(foo::enum_bits)));
    EXPECT_TRUE(CheckBitsAll(const_mask, var_bits));
    EXPECT_TRUE(CheckBitsAll(const_mask, const_bits));
    EXPECT_TRUE(CheckBitsAll(const_mask, EnumToVal(foo::enum_bits)));
    EXPECT_TRUE(CheckBitsAll(EnumToVal(foo::enum_mask), var_bits));
    EXPECT_TRUE(CheckBitsAll(EnumToVal(foo::enum_mask), const_bits));
    EXPECT_TRUE(CheckBitsAll(EnumToVal(foo::enum_mask), EnumToVal(foo::enum_bits)));
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(MathTest, CheckBitsAny)
{
    EXPECT_TRUE(CheckBitsAny(0x0f00, 0x0300));
    EXPECT_TRUE(CheckBitsAny(0x0300, 0x0f00));
    EXPECT_FALSE(CheckBitsAny(0x0300, 0x00f0));
    // Combinations of variable, constexpr and enums
    uint16_t var_mask = 0x0f00;
    uint16_t var_bits = 0x0300;
    constexpr uint16_t const_mask = 0x0f00;
    constexpr uint16_t const_bits = 0x0300;
    enum class foo : uint16_t
    {
        enum_mask = 0x0f00,
        enum_bits = 0x0300
    };
    EXPECT_TRUE(CheckBitsAny(var_mask, var_bits));
    EXPECT_TRUE(CheckBitsAny(var_mask, const_bits));
    EXPECT_TRUE(CheckBitsAny(var_mask, EnumToVal(foo::enum_bits)));
    EXPECT_TRUE(CheckBitsAny(const_mask, var_bits));
    EXPECT_TRUE(CheckBitsAny(const_mask, const_bits));
    EXPECT_TRUE(CheckBitsAny(const_mask, EnumToVal(foo::enum_bits)));
    EXPECT_TRUE(CheckBitsAny(EnumToVal(foo::enum_mask), var_bits));
    EXPECT_TRUE(CheckBitsAny(EnumToVal(foo::enum_mask), const_bits));
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(MathTest, GetBits)
{
    EXPECT_EQ(GetBits<uint16_t>(0xffff, 0x55aa), 0x55aa);
    EXPECT_EQ(GetBits<uint8_t>(0xaa, 0xf0), 0xa0);
}

TEST(MathTest, SetBits)
{
    {
        uint16_t val = 0;
        SetBits(val, (uint16_t)0xaa55);
        EXPECT_EQ(val, 0xaa55);
        SetBits(val, (uint16_t)0x55aa);
        EXPECT_EQ(val, 0xffff);
    }
    {
        uint16_t val = 0;
        constexpr uint16_t bits1 = 0xaa55;
        SetBits(val, bits1);
        EXPECT_EQ(val, 0xaa55);
        constexpr uint16_t bits2 = 0x55aa;
        SetBits(val, bits2);
        EXPECT_EQ(val, 0xffff);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(MathTest, ClearBits)
{
    {
        uint16_t val = 0xffff;
        ClearBits(val, (uint16_t)0xaa55);
        EXPECT_EQ(val, 0x55aa);
        ClearBits(val, (uint16_t)0x55aa);
        EXPECT_EQ(val, 0x0000);
    }
    {
        uint16_t val = 0xffff;
        constexpr uint16_t bits1 = 0xaa55;
        ClearBits(val, bits1);
        EXPECT_EQ(val, 0x55aa);
        constexpr uint16_t bits2 = 0x55aa;
        ClearBits(val, bits2);
        EXPECT_EQ(val, 0x0000);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(MathTest, ToggleBits)
{
    {
        uint16_t val = 0xff00;
        ToggleBits(val, (uint16_t)0xaa55);
        EXPECT_EQ(val, 0x5555);
        ToggleBits(val, (uint16_t)0x55aa);
        EXPECT_EQ(val, 0x00ff);
    }
    {
        uint16_t val = 0xff00;
        constexpr uint16_t bits1 = 0xaa55;
        ToggleBits(val, bits1);
        EXPECT_EQ(val, 0x5555);
        constexpr uint16_t bits2 = 0x55aa;
        ToggleBits(val, bits2);
        EXPECT_EQ(val, 0x00ff);
    }
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
