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
 * @brief Fixposition SDK: tests for fpsdk::common::parser (types)
 */

/* LIBC/STL */
#include <cstdint>
#include <string>
#include <vector>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser.hpp>
#include <fpsdk_common/parser/types.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::parser;

TEST(ParserTypesTest, ProtocolStr)
{
    // clang-format off
    EXPECT_EQ(std::string(ProtocolStr(Protocol::FP_A)),   std::string(PROTOCOL_NAME_FP_A));
    EXPECT_EQ(std::string(ProtocolStr(Protocol::FP_B)),   std::string(PROTOCOL_NAME_FP_B));
    EXPECT_EQ(std::string(ProtocolStr(Protocol::NMEA)),   std::string(PROTOCOL_NAME_NMEA));
    EXPECT_EQ(std::string(ProtocolStr(Protocol::UBX)),    std::string(PROTOCOL_NAME_UBX));
    EXPECT_EQ(std::string(ProtocolStr(Protocol::RTCM3)),  std::string(PROTOCOL_NAME_RTCM3));
    EXPECT_EQ(std::string(ProtocolStr(Protocol::UNI_B)),  std::string(PROTOCOL_NAME_UNI_B));
    EXPECT_EQ(std::string(ProtocolStr(Protocol::NOV_B)),  std::string(PROTOCOL_NAME_NOV_B));
    EXPECT_EQ(std::string(ProtocolStr(Protocol::SPARTN)), std::string(PROTOCOL_NAME_SPARTN));
    EXPECT_EQ(std::string(ProtocolStr(Protocol::OTHER)),  std::string(PROTOCOL_NAME_OTHER));
    EXPECT_EQ(std::string(ProtocolStr((Protocol)99)),     std::string("?"));
    // clang-format on
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserTypesTest, StrProtocol)
{
    // clang-format off
    EXPECT_EQ(StrProtocol(PROTOCOL_NAME_FP_A),   Protocol::FP_A);
    EXPECT_EQ(StrProtocol(PROTOCOL_NAME_FP_B),   Protocol::FP_B);
    EXPECT_EQ(StrProtocol(PROTOCOL_NAME_NMEA),   Protocol::NMEA);
    EXPECT_EQ(StrProtocol(PROTOCOL_NAME_UBX),    Protocol::UBX);
    EXPECT_EQ(StrProtocol(PROTOCOL_NAME_RTCM3),  Protocol::RTCM3);
    EXPECT_EQ(StrProtocol(PROTOCOL_NAME_UNI_B),  Protocol::UNI_B);
    EXPECT_EQ(StrProtocol(PROTOCOL_NAME_NOV_B),  Protocol::NOV_B);
    EXPECT_EQ(StrProtocol(PROTOCOL_NAME_SPARTN), Protocol::SPARTN);
    EXPECT_EQ(StrProtocol(PROTOCOL_NAME_OTHER),  Protocol::OTHER);
    EXPECT_EQ(StrProtocol("?"),                  Protocol::OTHER);
    EXPECT_EQ(StrProtocol("UBx"),                Protocol::OTHER);
    EXPECT_EQ(StrProtocol("UBXX"),               Protocol::OTHER);
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
