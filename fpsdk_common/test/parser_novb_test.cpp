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
 * @brief Fixposition SDK: tests for fpsdk::common::parser::novb
 */

/* LIBC/STL */
#include <array>
#include <cstring>
#include <string>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/novb.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::parser::novb;

TEST(ParserNovbTest, Macros)
{
    {
        const uint8_t msg[] = { 0x55, 0x55, 0x55, 0x55, 0x34, 0x12, 0xaa, 0xaa, 0xaa };
        ASSERT_EQ(NovbMsgId(msg), 0x1234);
    }

    {
        const uint8_t msg_long[] = { NOV_B_SYNC_1, NOV_B_SYNC_2, NOV_B_SYNC_3_LONG };
        ASSERT_TRUE(NovbIsLongHeader(msg_long));
        ASSERT_FALSE(NovbIsShortHeader(msg_long));
    }

    {
        const uint8_t msg_short[] = { NOV_B_SYNC_1, NOV_B_SYNC_2, NOV_B_SYNC_3_SHORT };
        ASSERT_FALSE(NovbIsLongHeader(msg_short));
        ASSERT_TRUE(NovbIsShortHeader(msg_short));
    }

    {
        const uint8_t msg_neither[] = { NOV_B_SYNC_1, NOV_B_SYNC_2, 0xaa };
        ASSERT_FALSE(NovbIsLongHeader(msg_neither));
        ASSERT_FALSE(NovbIsShortHeader(msg_neither));
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNovbTest, NovbGetMessageName)
{
    // Known message
    {
        const uint8_t msg[] = { // clang-format off
            NOV_B_SYNC_1, NOV_B_SYNC_2, NOV_B_SYNC_3_SHORT, 0x00,
            (uint8_t)(NOV_B_BESTGNSSPOS_MSGID & 0xff), (uint8_t)((NOV_B_BESTGNSSPOS_MSGID >> 8) & 0xff),
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // clang-format on
        char str[100];
        EXPECT_TRUE(NovbGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("NOV_B-BESTGNSSPOS"));
    }

    // Unknown message
    {
        const uint8_t msg[] = { // clang-format off
            NOV_B_SYNC_1, NOV_B_SYNC_2, NOV_B_SYNC_3_SHORT, 0x00,
            0x34, 0x12,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // clang-format on
        char str[100];
        EXPECT_TRUE(NovbGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("NOV_B-MSG04660"));
    }

    // Bad arguments
    {
        const uint8_t msg[] = { // clang-format off
            NOV_B_SYNC_1, NOV_B_SYNC_2, NOV_B_SYNC_3_SHORT, 0x00,
            0x34, 0x12,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // clang-format on
        char str[100];
        EXPECT_FALSE(NovbGetMessageName(str, 0, msg, sizeof(msg)));
        EXPECT_FALSE(NovbGetMessageName(NULL, 10, msg, sizeof(msg)));
        EXPECT_FALSE(NovbGetMessageName(NULL, 0, msg, sizeof(msg)));
        EXPECT_FALSE(NovbGetMessageName(str, sizeof(str), msg, 0));
        EXPECT_FALSE(NovbGetMessageName(str, sizeof(str), msg, 5));
        EXPECT_FALSE(NovbGetMessageName(str, sizeof(str), NULL, sizeof(msg)));
    }

    // Too small string is cut
    {
        const uint8_t msg[] = { // clang-format off
            NOV_B_SYNC_1, NOV_B_SYNC_2, NOV_B_SYNC_3_SHORT, 0x00,
            (uint8_t)(NOV_B_BESTGNSSPOS_MSGID & 0xff), (uint8_t)((NOV_B_BESTGNSSPOS_MSGID >> 8) & 0xff),
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // clang-format on
        char str[10];
        EXPECT_FALSE(NovbGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("NOV_B-BES"));
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNovbTest, NovbRawdmi)
{
    NovbRawdmi rawdmi;
    std::memset(&rawdmi, 0, sizeof(rawdmi));
    rawdmi.dmi1 = 0x12345678;
    rawdmi.dmi1_valid = 1;
    uint8_t bytes[sizeof(rawdmi)];
    std::memcpy(bytes, &rawdmi, sizeof(bytes));
    EXPECT_EQ(bytes[0], 0x78);
    EXPECT_EQ(bytes[1], 0x56);
    EXPECT_EQ(bytes[2], 0x34);
    EXPECT_EQ(bytes[3], 0x12);
    EXPECT_EQ(bytes[16], 0x01);
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
