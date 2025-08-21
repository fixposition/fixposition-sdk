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
 * @brief Fixposition SDK: tests for fpsdk::common::parser::ubx
 */

/* LIBC/STL */
#include <cstring>
#include <string>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser.hpp>
#include <fpsdk_common/parser/ubx.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::parser;
using namespace fpsdk::common::parser::ubx;

TEST(ParserUbxTest, Macros)
{
    const uint8_t msg[] = { UBX_SYNC_1, UBX_SYNC_2, 0x12, 0x34, 0x00, 0x00, 0x55, 0x55 };
    ASSERT_EQ(UbxClsId(msg), 0x12);
    ASSERT_EQ(UbxMsgId(msg), 0x34);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserUbxTest, UbxGetMessageName)
{
    // Known message
    {
        const uint8_t msg[] = { UBX_SYNC_1, UBX_SYNC_2, UBX_NAV_CLSID, UBX_NAV_PVT_MSGID, 0x00, 0x00, 0x00, 0x00 };
        char str[100];
        EXPECT_TRUE(UbxGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("UBX-NAV-PVT"));
    }

    // Known class, but unknown message
    {
        const uint8_t msg[] = { UBX_SYNC_1, UBX_SYNC_2, UBX_NAV_CLSID, 0x99, 0x00, 0x00, 0x00, 0x00 };
        char str[100];
        EXPECT_TRUE(UbxGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("UBX-NAV-99"));
    }

    // Completely unknown message
    {
        const uint8_t msg[] = { UBX_SYNC_1, UBX_SYNC_2, 0x99, 0x99, 0x00, 0x00, 0x00, 0x00 };
        char str[100];
        EXPECT_TRUE(UbxGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("UBX-99-99"));
    }

    // Bad arguments
    {
        const uint8_t msg[] = { UBX_SYNC_1, UBX_SYNC_2, UBX_NAV_CLSID, UBX_NAV_PVT_MSGID, 0x00, 0x00, 0x00, 0x00 };
        char str[100];
        EXPECT_FALSE(UbxGetMessageName(str, 0, msg, sizeof(msg)));
        EXPECT_FALSE(UbxGetMessageName(NULL, 10, msg, sizeof(msg)));
        EXPECT_FALSE(UbxGetMessageName(NULL, 0, msg, sizeof(msg)));
        EXPECT_FALSE(UbxGetMessageName(str, sizeof(str), msg, 0));
        EXPECT_FALSE(UbxGetMessageName(str, sizeof(str), msg, UBX_FRAME_SIZE - 1));
        EXPECT_FALSE(UbxGetMessageName(str, sizeof(str), NULL, sizeof(msg)));
    }

    // Too small string is cut
    {
        const uint8_t msg[] = { UBX_SYNC_1, UBX_SYNC_2, UBX_NAV_CLSID, UBX_NAV_PVT_MSGID, 0x00, 0x00, 0x00, 0x00 };
        char str[10];
        EXPECT_FALSE(UbxGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("UBX-NAV-P"));
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserUbxTest, UbxMakeMessage)
{
    for (const int payload_size :
        { 0, 1, 2, 3, 4, 5, 6, 7, 8, 100, 200, 500, 1000, 2000, (int)(MAX_UBX_SIZE - UBX_FRAME_SIZE) }) {
        // Create payload with this many bytes of data
        std::vector<uint8_t> payload;
        for (int ix = 0; ix < payload_size; ix++) {
            payload.push_back((uint8_t)(ix + 1));
        }

        // We expect a message of this size
        const int msg_size = payload_size + UBX_FRAME_SIZE;

        // Create message
        std::vector<uint8_t> msg;
        EXPECT_TRUE(UbxMakeMessage(msg, 0xaa, 0x55, payload));
        EXPECT_EQ((int)msg.size(), msg_size);

        // Create message using "raw" API
        std::vector<uint8_t> msg2;
        EXPECT_TRUE(UbxMakeMessage(msg2, 0xaa, 0x55, payload.data(), (int)payload.size()));
        EXPECT_EQ(msg.size(), msg2.size());
        EXPECT_EQ(std::memcmp(msg.data(), msg2.data(), msg.size()), 0);

        // Run it through the parser and check that it is still the same
        Parser parser;
        parser.Add(msg);
        ParserMsg parser_message;
        EXPECT_TRUE(parser.Process(parser_message));
        EXPECT_EQ(parser_message.Size(), UBX_FRAME_SIZE + payload.size());
        EXPECT_EQ(std::string(parser_message.name_), std::string("UBX-AA-55"));
        // Payload should be the same
        EXPECT_TRUE(std::memcmp(payload.data(), &parser_message.data_[UBX_HEAD_SIZE], payload.size()) == 0);

        // There should be nothing left in the parser
        EXPECT_FALSE(parser.Flush(parser_message));
    }

    // Too large (by one byte)
    {
        std::vector<uint8_t> payload(MAX_UBX_SIZE - UBX_FRAME_SIZE + 1);
        std::vector<uint8_t> msg;
        EXPECT_FALSE(UbxMakeMessage(msg, 0xaa, 0x55, payload));
        EXPECT_EQ(msg.size(), (std::size_t)0);
        EXPECT_FALSE(UbxMakeMessage(msg, 0xaa, 0x55, payload.data(), payload.size()));
        EXPECT_EQ(msg.size(), (std::size_t)0);
    }

    // Combinations of arguments
    {
        const uint8_t payload[] = { 0x55, 0x55 };
        std::vector<uint8_t> msg;
        // These are okay:
        EXPECT_TRUE(UbxMakeMessage(msg, 0xaa, 0x55, payload, 0));
        EXPECT_EQ(msg.size(), (std::size_t)UBX_FRAME_SIZE);
        msg.clear();
        EXPECT_TRUE(UbxMakeMessage(msg, 0xaa, 0x55, NULL, 0));
        EXPECT_EQ(msg.size(), (std::size_t)UBX_FRAME_SIZE);
        // But these are bad:
        msg.clear();
        EXPECT_EQ(msg.size(), (std::size_t)0);
        EXPECT_FALSE(UbxMakeMessage(msg, 0xaa, 0x55, NULL, 1));
        EXPECT_EQ(msg.size(), (std::size_t)0);
        EXPECT_EQ(msg.size(), (std::size_t)0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserUbxTest, UbxMessageInfo)
{
    UbxMsgInfo info;
    EXPECT_FALSE(UbxMessageInfo("NOT_A_UBX_MESSAGE_NAME", info));
    const std::string name = "UBX-NAV-PVT";
    EXPECT_TRUE(UbxMessageInfo(name, info));
    EXPECT_EQ(info.cls_id_, UBX_NAV_CLSID);
    EXPECT_EQ(info.msg_id_, UBX_NAV_PVT_MSGID);
    EXPECT_EQ(std::string(info.name_), name);
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
