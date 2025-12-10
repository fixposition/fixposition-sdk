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
 * @brief Fixposition SDK: tests for fpsdk::common::parser::sbf
 */

/* LIBC/STL */
#include <array>
#include <string>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/sbf.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::parser::sbf;

TEST(ParserSbfTest, Macros)
{
    {
        const uint8_t msg[] = { 0x55, 0x55, 0x55, 0x55, 0x34, 0x12, 0xaa, 0xaa, 0xaa };
        ASSERT_EQ(SbfBlockType(msg), 0x1234);
        ASSERT_EQ(SbfBlockRev(msg), 0x0);
    }
    {
        const uint8_t msg[] = { 0x55, 0x55, 0x55, 0x55, 0x34, 0x12 + 0x20, 0xaa, 0xaa, 0xaa };
        ASSERT_EQ(SbfBlockType(msg), 0x1234);
        ASSERT_EQ(SbfBlockRev(msg), 0x1);
    }
    {
        const uint8_t msg[] = { 0x55, 0x55, 0x55, 0x55, 0x34, 0x12 + 0xe0, 0xaa, 0xaa, 0xaa };
        ASSERT_EQ(SbfBlockType(msg), 0x1234);
        ASSERT_EQ(SbfBlockRev(msg), 0x7);
    }
    {
        const uint8_t msg[] = { 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x34, 0x12, 0xaa, 0xaa, 0xaa };
        ASSERT_EQ(SbfMsgSize(msg), 0x1234);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserSbfTest, SbfGetMessageName)
{
    // Known message
    {
        const uint8_t msg[] = { // clang-format off
            SBF_SYNC_1, SBF_SYNC_2, 0x55, 0x55,
            (uint8_t)(SBF_NAVCART_MSGID & 0xff), (uint8_t)((SBF_NAVCART_MSGID >> 8) & 0xff), 0xaa, 0xaa };  // clang-format on
        char str[100];
        EXPECT_TRUE(SbfGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("SBF-NAVCART"));
    }
    // Unknown message
    {
        const uint8_t msg[] = { // clang-format off
            SBF_SYNC_1, SBF_SYNC_2, 0x55, 0x55, 0x34, 0x12, 0xaa, 0xaa };  // clang-format on
        char str[100];
        EXPECT_TRUE(SbfGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("SBF-BLOCK04660"));
    }

    // Bad arguments
    {
        const uint8_t msg[] = { // clang-format off
            SBF_SYNC_1, SBF_SYNC_2, 0x55, 0x55, 0x34, 0x12, 0xaa, 0xaa  };  // clang-format on
        char str[100];
        EXPECT_FALSE(SbfGetMessageName(str, 0, msg, sizeof(msg)));
        EXPECT_FALSE(SbfGetMessageName(NULL, 10, msg, sizeof(msg)));
        EXPECT_FALSE(SbfGetMessageName(NULL, 0, msg, sizeof(msg)));
        EXPECT_FALSE(SbfGetMessageName(str, sizeof(str), msg, 0));
        EXPECT_FALSE(SbfGetMessageName(str, sizeof(str), msg, 5));
        EXPECT_FALSE(SbfGetMessageName(str, sizeof(str), NULL, sizeof(msg)));
    }

    // Too small string is cut
    {
        const uint8_t msg[] = { // clang-format off
            SBF_SYNC_1, SBF_SYNC_2, 0x55, 0x55,
            (uint8_t)(SBF_NAVCART_MSGID & 0xff), (uint8_t)((SBF_NAVCART_MSGID >> 8) & 0xff), 0xaa, 0xaa };  // clang-format on
        char str[10];
        EXPECT_FALSE(SbfGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("SBF-NAVCA"));
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
