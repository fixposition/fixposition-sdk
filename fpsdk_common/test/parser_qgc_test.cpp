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
 * @brief Fixposition SDK: tests for fpsdk::common::parser::qgc
 */

/* LIBC/STL */
#include <array>
#include <string>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/qgc.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::parser::qgc;

TEST(ParserQgcTest, Macros)
{
    {
        const uint8_t msg[] = { 0x55, 0x55, 0x12, 0x34, 0xaa, 0xaa, 0xaa, 0xaa };
        ASSERT_EQ(QgcGrpId(msg), 0x12);
        ASSERT_EQ(QgcMsgId(msg), 0x34);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserSbfTest, QgcGetMessageName)
{
    // Known message
    {
        const uint8_t msg[] = { QGC_SYNC_1, QGC_SYNC_2, QGC_RAW_GRPID, QGC_RAW_HASE6_MSGID, 0x55, 0x55, 0xaa, 0xaa };
        char str[100];
        EXPECT_TRUE(QgcGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("QGC-RAW-HASE6"));
    }
    // Unknown message
    {
        const uint8_t msg[] = { QGC_SYNC_1, QGC_SYNC_2, QGC_RAW_GRPID, 0x9f, 0x55, 0x55, 0xaa, 0xaa };
        char str[100];
        EXPECT_TRUE(QgcGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("QGC-RAW-9F"));
    }
    // Unknown group and message
    {
        const uint8_t msg[] = { QGC_SYNC_1, QGC_SYNC_2, 0x8a, 0x7b, 0x55, 0x55, 0xaa, 0xaa };
        char str[100];
        EXPECT_TRUE(QgcGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("QGC-8A-7B"));
    }

    // Bad arguments
    {
        const uint8_t msg[] = { QGC_SYNC_1, QGC_SYNC_2, QGC_RAW_GRPID, QGC_RAW_HASE6_MSGID, 0x55, 0x55, 0xaa, 0xaa };
        char str[100];
        EXPECT_FALSE(QgcGetMessageName(str, 0, msg, sizeof(msg)));
        EXPECT_FALSE(QgcGetMessageName(NULL, 10, msg, sizeof(msg)));
        EXPECT_FALSE(QgcGetMessageName(NULL, 0, msg, sizeof(msg)));
        EXPECT_FALSE(QgcGetMessageName(str, sizeof(str), msg, 0));
        EXPECT_FALSE(QgcGetMessageName(str, sizeof(str), msg, 5));
        EXPECT_FALSE(QgcGetMessageName(str, sizeof(str), NULL, sizeof(msg)));
    }

    // Too small string is cut
    {
        const uint8_t msg[] = { QGC_SYNC_1, QGC_SYNC_2, QGC_RAW_GRPID, QGC_RAW_HASE6_MSGID, 0x55, 0x55, 0xaa, 0xaa };
        char str[10];
        EXPECT_FALSE(QgcGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        EXPECT_EQ(std::string(str), std::string("QGC-RAW-H"));
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
