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
 * @brief Fixposition SDK: tests for fpsdk::common::parser::fpb
 */

/* LIBC/STL */
#include <array>
#include <string>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser.hpp>
#include <fpsdk_common/parser/fpb.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::parser;
using namespace fpsdk::common::parser::fpb;

TEST(ParserFpbTest, Macros)
{
    const uint8_t msg[] = { 0x55, 0x55, 0x34, 0x12, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa };
    ASSERT_EQ(FpbMsgId(msg), 0x1234);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserFpbTest, FpbGetMessageName)
{
    // Known message
    {
        const uint8_t msg[] = { // clang-format off
            FP_B_SYNC_1, FP_B_SYNC_2,
            (uint8_t)(FP_B_UNITTEST1_MSGID & 0xff), (uint8_t)((FP_B_UNITTEST1_MSGID >> 8) & 0xff),
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // clang-format on
        char str[100];
        EXPECT_TRUE(FpbGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("FP_B-UNITTEST1"));
    }
    // Unknown message
    {
        const uint8_t msg[] = { // clang-format off
            FP_B_SYNC_1, FP_B_SYNC_2,
            0xdc, 0xfe,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // clang-format on
        char str[100];
        EXPECT_TRUE(FpbGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("FP_B-MSG65244"));
    }

    // Bad arguments
    {
        const uint8_t msg[] = { // clang-format off
            FP_B_SYNC_1, FP_B_SYNC_2, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // clang-format on
        char str[100];
        EXPECT_FALSE(FpbGetMessageName(str, 0, msg, sizeof(msg)));
        EXPECT_FALSE(FpbGetMessageName(str, -1, msg, sizeof(msg)));
        EXPECT_FALSE(FpbGetMessageName(NULL, 10, msg, sizeof(msg)));
        EXPECT_FALSE(FpbGetMessageName(NULL, 0, msg, sizeof(msg)));
        EXPECT_FALSE(FpbGetMessageName(NULL, -1, msg, sizeof(msg)));
        EXPECT_FALSE(FpbGetMessageName(str, sizeof(str), msg, 0));
        EXPECT_FALSE(FpbGetMessageName(str, sizeof(str), msg, 5));
        EXPECT_FALSE(FpbGetMessageName(str, sizeof(str), NULL, sizeof(msg)));
    }

    // Too small string is cut
    {
        const uint8_t msg[] = { // clang-format off
            FP_B_SYNC_1, FP_B_SYNC_2,
            (uint8_t)(FP_B_UNITTEST1_MSGID & 0xff), (uint8_t)((FP_B_UNITTEST1_MSGID >> 8) & 0xff),
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // clang-format on
        char str[10];
        EXPECT_FALSE(FpbGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("FP_B-UNIT"));
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserFpbTest, FpbMakeMessage)
{
    for (const int payload_size :
        { 0, 1, 2, 3, 4, 5, 6, 7, 8, 100, 200, 500, 1000, 2000, (int)(MAX_FP_B_SIZE - FP_B_FRAME_SIZE) }) {
        // Create payload with this many bytes of data
        std::vector<uint8_t> payload;
        for (int ix = 0; ix < payload_size; ix++) {
            payload.push_back((uint8_t)(ix + 1));
        }

        // We expect a message of this size
        const int msg_size = payload_size + FP_B_FRAME_SIZE;

        // Create message
        std::vector<uint8_t> msg;
        EXPECT_TRUE(FpbMakeMessage(msg, FP_B_UNITTEST2_MSGID, 0, payload));
        EXPECT_EQ((int)msg.size(), msg_size);

        // Create message using "raw" API
        std::vector<uint8_t> msg2;
        EXPECT_TRUE(FpbMakeMessage(msg2, FP_B_UNITTEST2_MSGID, 0, payload.data(), (int)payload.size()));
        EXPECT_EQ(msg.size(), msg2.size());
        EXPECT_EQ(std::memcmp(msg.data(), msg2.data(), msg.size()), 0);

        // Run it through the parser and check that it is still the same
        Parser parser;
        parser.Add(msg);
        ParserMsg parser_msg;
        EXPECT_TRUE(parser.Process(parser_msg));
        EXPECT_EQ(parser_msg.Size(), FP_B_FRAME_SIZE + payload.size());
        EXPECT_EQ(std::string(parser_msg.name_), std::string("FP_B-UNITTEST2"));
        // Payload should be the same
        EXPECT_TRUE(std::memcmp(payload.data(), &parser_msg.data_[FP_B_HEAD_SIZE], payload.size()) == 0);

        // There should be nothing left in the parser
        EXPECT_FALSE(parser.Flush(parser_msg));
    }

    // Too large (by one byte)
    {
        std::vector<uint8_t> payload(MAX_FP_B_SIZE - FP_B_FRAME_SIZE + 1);
        std::vector<uint8_t> msg;
        EXPECT_FALSE(FpbMakeMessage(msg, FP_B_UNITTEST2_MSGID, 0, payload));
        EXPECT_EQ(msg.size(), (std::size_t)0);
        EXPECT_FALSE(FpbMakeMessage(msg, FP_B_UNITTEST2_MSGID, 0, payload.data(), payload.size()));
        EXPECT_EQ(msg.size(), (std::size_t)0);
    }

    // Combinations of arguments
    {
        const uint8_t payload[] = { 0x55, 0x55 };
        std::vector<uint8_t> msg;
        // These are okay:
        EXPECT_TRUE(FpbMakeMessage(msg, FP_B_UNITTEST2_MSGID, 0, payload, 0));
        EXPECT_EQ(msg.size(), (std::size_t)FP_B_FRAME_SIZE);
        msg.clear();
        EXPECT_TRUE(FpbMakeMessage(msg, FP_B_UNITTEST2_MSGID, 0, NULL, 0));
        EXPECT_EQ(msg.size(), (std::size_t)FP_B_FRAME_SIZE);
        // But these are bad:
        msg.clear();
        EXPECT_FALSE(FpbMakeMessage(msg, FP_B_UNITTEST2_MSGID, 0, payload, -1));
        EXPECT_EQ(msg.size(), (std::size_t)0);
        EXPECT_FALSE(FpbMakeMessage(msg, FP_B_UNITTEST2_MSGID, 0, NULL, 1));
        EXPECT_EQ(msg.size(), (std::size_t)0);
        EXPECT_FALSE(FpbMakeMessage(msg, FP_B_UNITTEST2_MSGID, 0, NULL, -1));
        EXPECT_EQ(msg.size(), (std::size_t)0);
    }

    // Message time
    {
        std::vector<uint8_t> msg;
        EXPECT_TRUE(FpbMakeMessage(msg, FP_B_UNITTEST1_MSGID, 0x1234, NULL, 0));
        EXPECT_EQ(msg.size(), (std::size_t)FP_B_FRAME_SIZE);
        EXPECT_EQ(msg[6], 0x34);
        EXPECT_EQ(msg[7], 0x12);
        EXPECT_EQ(FpbMsgTime(msg.data()), 0x1234);
    }

    // Example in docu
    {
        std::vector<uint8_t> msg;
        const uint8_t payload[] = { 0x01, 0x02, 0x03, 0x04 };
        EXPECT_TRUE(FpbMakeMessage(msg, 0x1234, 0x4321, payload, sizeof(payload)));
        EXPECT_EQ(msg.size(), (std::size_t)FP_B_FRAME_SIZE + sizeof(payload));
        EXPECT_EQ(msg, std::vector<uint8_t>({ 0x66, 0x21, 0x34, 0x12, 0x04, 0x00, 0x21, 0x43, 0x01, 0x02, 0x03, 0x04,
                           0x61, 0xc4, 0xc5, 0x9c }));
    }

    // FP_B-VERSION
    {
        std::vector<uint8_t> msg;
        EXPECT_TRUE(FpbMakeMessage(msg, FP_B_VERSION_MSGID, 0x0000, NULL, 0));
        EXPECT_EQ(msg.size(), (std::size_t)FP_B_FRAME_SIZE);
        EXPECT_EQ(FpbMsgTime(msg.data()), 0x0000);
        DEBUG_HEXDUMP(msg.data(), msg.size(), NULL, "FP_B-VERSION poll");
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
