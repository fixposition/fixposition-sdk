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
 * @brief Fixposition SDK: tests for fpsdk::common::parser::spartn
 */

/* LIBC/STL */
#include <string>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/spartn.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::parser::spartn;

TEST(ParserSpartnTest, Macros)
{
    const uint8_t msg[] = { 0x00, 0xfe, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00 };
    ASSERT_EQ(SpartnType(msg), 0x7f);
    ASSERT_EQ(SpartnSubType(msg), 0x0f);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserSpartnTest, SpartnGetMessageName)
{
    // Known message (type 1, subtype 2)
    {
        const uint8_t msg[] = { // clang-format off
            0x73, 0x01 | (SPARTN_HPAC_MSGID << 1), 0x50, 0x67, 0x08 | (SPARTN_HPAC_GAL_SUBID << 4),
            0xb9, 0xa0, 0x3e, 0xd0, 0x5b};  // clang-format on
        ASSERT_EQ(SpartnType(msg), SPARTN_HPAC_MSGID);
        ASSERT_EQ(SpartnSubType(msg), SPARTN_HPAC_GAL_SUBID);
        char str[100];
        ASSERT_TRUE(SpartnGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        ASSERT_EQ(std::string(str), std::string("SPARTN-HPAC-GAL"));
    }
    // Unknown message (type 42, subtype 3)
    {
        const uint8_t msg[] = { 0x73, 0x54, 0x50, 0x67, 0x38, 0xb9, 0xa0, 0x3e, 0xd0, 0x5b };
        ASSERT_EQ(SpartnType(msg), 42);
        ASSERT_EQ(SpartnSubType(msg), 3);
        char str[100];
        ASSERT_TRUE(SpartnGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        ASSERT_EQ(std::string(str), std::string("SPARTN-42-3"));
    }
    // Unknown subtype (type 42, subtype 3)
    {
        const uint8_t msg[] = { // clang-format off
            0x73, 0x01 | (SPARTN_PROP_MSGID << 1), 0x50, 0x67, 0x08 | (7 << 4),
            0xb9, 0xa0, 0x3e, 0xd0, 0x5b};  // clang-format on
        ASSERT_EQ(SpartnType(msg), SPARTN_PROP_MSGID);
        ASSERT_EQ(SpartnSubType(msg), 7);
        char str[100];
        ASSERT_TRUE(SpartnGetMessageName(str, sizeof(str), msg, sizeof(msg)));
        ASSERT_EQ(std::string(str), std::string("SPARTN-PROP-7"));
    }

    // Bad input
    {
        const uint8_t msg[] = { SPARTN_PREAMBLE, 0x12, 0x34, 0x56, 0x78, 0x9a };
        char name[100];
        EXPECT_FALSE(SpartnGetMessageName(name, sizeof(name), NULL, 0));
        EXPECT_FALSE(SpartnGetMessageName(name, sizeof(name), msg, 0));
        EXPECT_FALSE(SpartnGetMessageName(NULL, 0, msg, sizeof(msg)));
        EXPECT_FALSE(SpartnGetMessageName(name, 0, msg, sizeof(msg)));
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
