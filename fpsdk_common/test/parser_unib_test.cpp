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
 * @brief Fixposition SDK: tests for fpsdk::common::parser::unib
 */

/* LIBC/STL */
#include <array>
#include <string>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/unib.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::parser::unib;

TEST(ParserUnibTest, Macros)
{
    {
        const uint8_t msg[] = { 0x55, 0x55, 0x55, 0x55, 0x34, 0x12, 0xaa, 0xaa, 0xaa };
        ASSERT_EQ(UnibMsgId(msg), 0x1234);
    }

    {
        const uint8_t msg[] = { 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x34, 0x12, 0xaa, 0xaa, 0xaa };
        ASSERT_EQ(UnibMsgSize(msg), 0x1234);
    }
    {
        const uint8_t msg[] = { 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
            0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x12, 0xaa, 0xaa, 0xaa };
        ASSERT_EQ(UNI_B_MSGVER(msg), 0x12);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserUnibTest, UnibGetMessageName)
{
    // Known message
    {
        const uint8_t msg[] = { // clang-format off
            UNI_B_SYNC_1, UNI_B_SYNC_2, UNI_B_SYNC_3, 0x00,
            (uint8_t)(UNI_B_VERSION_MSGID & 0xff), (uint8_t)((UNI_B_VERSION_MSGID >> 8) & 0xff),
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // clang-format on
        char str[100];
        EXPECT_TRUE(UnibGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("UNI_B-VERSION"));
    }

    // Unknown message
    {
        const uint8_t msg[] = { // clang-format off
            UNI_B_SYNC_1, UNI_B_SYNC_2, UNI_B_SYNC_3, 0x00,
            0x34, 0x12,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // clang-format on
        char str[100];
        EXPECT_TRUE(UnibGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("UNI_B-MSG04660"));
    }

    // Bad arguments
    {
        const uint8_t msg[] = { // clang-format off
            UNI_B_SYNC_1, UNI_B_SYNC_2, UNI_B_SYNC_3, 0x00,
            0x34, 0x12,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // clang-format on
        char str[100];
        EXPECT_FALSE(UnibGetMessageName(str, 0, msg, sizeof(msg)));
        EXPECT_FALSE(UnibGetMessageName(NULL, 10, msg, sizeof(msg)));
        EXPECT_FALSE(UnibGetMessageName(NULL, 0, msg, sizeof(msg)));
        EXPECT_FALSE(UnibGetMessageName(str, sizeof(str), msg, 0));
        EXPECT_FALSE(UnibGetMessageName(str, sizeof(str), msg, 5));
        EXPECT_FALSE(UnibGetMessageName(str, sizeof(str), NULL, sizeof(msg)));
    }
    // Too small string is cut
    {
        const uint8_t msg[] = { // clang-format off
            UNI_B_SYNC_1, UNI_B_SYNC_2, UNI_B_SYNC_3, 0x00,
            (uint8_t)(UNI_B_VERSION_MSGID & 0xff), (uint8_t)((UNI_B_VERSION_MSGID >> 8) & 0xff),
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // clang-format on
        char str[10];
        EXPECT_FALSE(UnibGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("UNI_B-VER"));
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
