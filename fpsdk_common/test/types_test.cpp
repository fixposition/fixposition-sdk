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
 * @brief Fixposition SDK: tests for fpsdk::common::types
 */

/* LIBC/STL */
#include <array>
#include <cstdint>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/types.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::types;

TEST(TypesTest, NumOf)
{
    std::array<int, 5> a5 = { { 1, 2, 3, 4, 5 } };
    EXPECT_EQ(a5.size(), 5);
    EXPECT_EQ(std::tuple_size<decltype(a5)>{}, 5);
    EXPECT_EQ(NumOf(a5), 5);

    using ArrayFour = std::array<int, 4>;
    EXPECT_EQ(std::tuple_size<ArrayFour>{}, 4);
    EXPECT_EQ(NumOf<ArrayFour>(), 4);

    int i6[6] = { 1, 2, 3, 4, 5, 6 };
    EXPECT_EQ(sizeof(i6) / sizeof(i6[0]), 6);
    EXPECT_EQ(NumOf(i6), 6);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TypesTest, NoCopyNoMove)
{
    class DoesItCompile : private NoCopyNoMove
    {
       public:
        DoesItCompile()
        {
            a = 3;
        }
        int a = 5;
    };
    DoesItCompile doesitcompile;
    EXPECT_EQ(doesitcompile.a, 3);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TypesTest, Macros)
{
    int u1 = 5;
    UNUSED(u1);

#define FIFTEEN 15
    const std::string fifteen = STRINGIFY(FIFTEEN);
    EXPECT_EQ(fifteen, "15");
#define ZEROZERO 00
    const int fifteenhundred = CONCAT(FIFTEEN, ZEROZERO);
    EXPECT_EQ(fifteenhundred, 1500);

    struct SomeStruct
    {
        int a;
        uint8_t b[3];
    };
    EXPECT_EQ(SIZEOF_FIELD(SomeStruct, b), 3 * sizeof(uint8_t));
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
