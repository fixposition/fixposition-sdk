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
 * @brief Fixposition SDK: tests for fpsdk::common::parser::rtcm3
 */

/* LIBC/STL */
#include <cstdint>
#include <string>
#include <vector>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/rtcm3.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::parser::rtcm3;

static const std::vector<uint8_t> gps_msm4 = { 0xd3, 0x00, 0x98, 0x43, 0x20, 0x00, 0x78, 0xae, 0x0b, 0x04, 0x00, 0x00,
    0x00, 0x2d, 0x43, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x80, 0x00, 0x5d, 0xb7, 0xc9, 0xe9, 0x09, 0xc9, 0x09,
    0x69, 0xc9, 0xc8, 0x6a, 0x33, 0xd5, 0x44, 0x10, 0x64, 0xb1, 0x31, 0x20, 0xa5, 0x8f, 0x0a, 0x50, 0x05, 0x71, 0x31,
    0x82, 0x21, 0x41, 0xca, 0xea, 0x72, 0xd4, 0x12, 0x23, 0x0f, 0xf3, 0xf0, 0x98, 0x11, 0x3a, 0xe5, 0x2e, 0xc9, 0x86,
    0x9b, 0xc0, 0xff, 0x92, 0x14, 0x15, 0xae, 0x60, 0x88, 0x2f, 0x00, 0xae, 0xbf, 0xf6, 0x29, 0x6b, 0xc9, 0x3b, 0x60,
    0x27, 0x90, 0xbf, 0xcf, 0x57, 0x04, 0x12, 0xb0, 0x14, 0xbd, 0xa0, 0xb6, 0x0b, 0x82, 0x60, 0xb9, 0x0d, 0xb7, 0x73,
    0x3f, 0xeb, 0xfe, 0x43, 0xfe, 0xc2, 0xc0, 0x44, 0xfd, 0x51, 0xb9, 0x74, 0xcf, 0xb7, 0x45, 0x0d, 0x32, 0xc0, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xed, 0xbc, 0xc4 };

static const std::vector<uint8_t> type1005 = { 0xd3, 0x00, 0x13, 0x3e, 0xd0, 0x00, 0x03, 0x89, 0xf6, 0x1e, 0x1e, 0x8c,
    0x81, 0x7a, 0xdc, 0x05, 0xfc, 0x0a, 0xe0, 0xee, 0x82, 0xd0, 0xa1, 0xf9, 0xbf };

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserRtcm3Test, Macros)
{
    const uint8_t msg[] = { 0x55, 0x55, 0x55, 0x12, 0x34, 0x56, 0xaa, 0xaa, 0xaa };
    ASSERT_EQ(Rtcm3Type(msg), 0x0123);
    ASSERT_EQ(Rtcm3SubType(msg), 0x0456);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserRtcm3Test, Rtcm3GetMessageName)
{
    // RTCM3 type 1234
    {
        const uint8_t msg[] = { 0x55, 0x55, 0x55, 0x4d, 0x20, 0xaa, 0xaa, 0xaa, 0xaa };
        ASSERT_EQ(Rtcm3Type(msg), 1234);
        char str[100];
        ASSERT_TRUE(Rtcm3GetMessageName(str, sizeof(str), msg, sizeof(msg)));
        ASSERT_EQ(std::string(str), std::string("RTCM3-TYPE1234"));
    }

    // RTCM3 type 4072, subtype 1
    {
        const uint8_t msg[] = { 0x55, 0x55, 0x55, 0xfe, 0x80, 0x01, 0xaa, 0xaa, 0xaa };
        ASSERT_EQ(Rtcm3Type(msg), 4072);
        ASSERT_EQ(Rtcm3SubType(msg), 1);
        char str[100];
        ASSERT_TRUE(Rtcm3GetMessageName(str, sizeof(str), msg, sizeof(msg)));
        ASSERT_EQ(std::string(str), std::string("RTCM3-TYPE4072_1"));
    }

    // Bad input
    {
        const uint8_t msg[] = { RTCM3_PREAMBLE, 0x12, 0x34, 0x56, 0x78, 0x9a };
        char name[100];
        EXPECT_FALSE(Rtcm3GetMessageName(name, sizeof(name), NULL, 0));
        EXPECT_FALSE(Rtcm3GetMessageName(name, sizeof(name), msg, 0));
        EXPECT_FALSE(Rtcm3GetMessageName(NULL, 0, msg, sizeof(msg)));
        EXPECT_FALSE(Rtcm3GetMessageName(name, 0, msg, sizeof(msg)));
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserRtcm3Test, Rtcm3GetTypeDesc)
{
    EXPECT_TRUE(Rtcm3GetTypeDesc(999) == NULL);
    EXPECT_TRUE(Rtcm3GetTypeDesc(RTCM3_TYPE1001_MSGID) != NULL);
    EXPECT_TRUE(Rtcm3GetTypeDesc(RTCM3_TYPE4072_MSGID, RTCM3_TYPE4072_1_SUBID) != NULL);
    const std::string str = Rtcm3GetTypeDesc(RTCM3_TYPE1074_MSGID);
    DEBUG("str=%s", str.c_str());
    EXPECT_GT(str.size(), 10);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserRtcm3Test, Rtcm3GetArp)
{
    Rtcm3Arp arp;
    EXPECT_FALSE(Rtcm3GetArp(NULL, arp));
    EXPECT_TRUE(Rtcm3GetArp(type1005.data(), arp));
    EXPECT_EQ(arp.ref_sta_id_, 0);
    EXPECT_NEAR(arp.ecef_x_, 4278387.47, 1e-4);
    EXPECT_NEAR(arp.ecef_y_, 635620.71, 1e-4);
    EXPECT_NEAR(arp.ecef_z_, 4672340.04, 1e-4);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserRtcm3Test, Rtcm3TypeToMsm)
{
    Rtcm3MsmGnss msm_gnss;
    Rtcm3MsmType msm_type;
    EXPECT_TRUE(Rtcm3TypeToMsm(RTCM3_TYPE1074_MSGID, msm_gnss, msm_type));
    EXPECT_EQ(msm_gnss, Rtcm3MsmGnss::GPS);
    EXPECT_EQ(msm_type, Rtcm3MsmType::MSM4);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserRtcm3Test, Rtcm3GetMsmHeader)
{
    Rtcm3MsmHeader msm;
    EXPECT_FALSE(Rtcm3GetMsmHeader(NULL, msm));
    EXPECT_TRUE(Rtcm3GetMsmHeader(gps_msm4.data(), msm));
    EXPECT_EQ(msm.gnss_, Rtcm3MsmGnss::GPS);
    EXPECT_EQ(msm.msm_, Rtcm3MsmType::MSM4);
    EXPECT_EQ(msm.msg_type_, RTCM3_TYPE1074_MSGID);
    EXPECT_EQ(msm.ref_sta_id_, 0);
    EXPECT_NEAR(msm.any_tow_, 506168.001, 1e-4);
    EXPECT_FALSE(msm.multi_msg_bit_);
    EXPECT_EQ(msm.num_sat_, 9);
    EXPECT_EQ(msm.num_sig_, 2);
    EXPECT_EQ(msm.num_cell_, 18);
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
