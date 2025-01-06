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
 * @brief Fixposition SDK: tests for fpsdk::common::parser::fpa
 */

/* LIBC/STL */
#include <string>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/fpa.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::parser::fpa;

TEST(ParserFpaTest, FpaGetMessageMeta)
{
    {
        //                 012345678901234567890123456789
        //                     TTTTTTTT V PPPPPPPPPPP
        const char* msg = "$FP,ODOMETRY,1,...,...,...*xx\r\n";
        FpaMessageMeta meta;
        EXPECT_TRUE(FpaGetMessageMeta(meta, (const uint8_t*)msg, strlen(msg))) << msg;
        EXPECT_EQ(std::string(meta.msg_type_), std::string("ODOMETRY")) << msg;
        EXPECT_EQ(meta.msg_version_, 1) << msg;
        EXPECT_EQ(meta.payload_ix0_, 15) << msg;
        EXPECT_EQ(meta.payload_ix1_, 25) << msg;
    }
    {
        //                 012345678901234567890123456789
        //                     TTTTTTTT V
        const char* msg = "$FP,ODOMETRY,1*xx\r\n";
        FpaMessageMeta meta;
        EXPECT_TRUE(FpaGetMessageMeta(meta, (const uint8_t*)msg, strlen(msg))) << msg;
        EXPECT_EQ(std::string(meta.msg_type_), std::string("ODOMETRY")) << msg;
        EXPECT_EQ(meta.msg_version_, 1) << msg;
        EXPECT_EQ(meta.payload_ix0_, 0) << msg;
        EXPECT_EQ(meta.payload_ix1_, 0) << msg;
    }
    {
        //                 012345678901234567890123456789
        //                     TTTTTTTT
        const char* msg = "$FP,ODOMETRY*xx\r\n";
        FpaMessageMeta meta;
        EXPECT_TRUE(FpaGetMessageMeta(meta, (const uint8_t*)msg, strlen(msg))) << msg;
        EXPECT_EQ(std::string(meta.msg_type_), std::string("ODOMETRY")) << msg;
        EXPECT_LT(meta.msg_version_, 0) << msg;
        EXPECT_EQ(meta.payload_ix0_, 0) << msg;
        EXPECT_EQ(meta.payload_ix1_, 0) << msg;
    }
    {
        //                 012345678901234567890123456789
        //                     TTTTTTTT
        const char* msg = "$FP,ODOMETRY,*xx\r\n";
        FpaMessageMeta meta;
        EXPECT_TRUE(FpaGetMessageMeta(meta, (const uint8_t*)msg, strlen(msg))) << msg;
        EXPECT_EQ(std::string(meta.msg_type_), std::string("ODOMETRY")) << msg;
        EXPECT_LT(meta.msg_version_, 0) << msg;
        EXPECT_EQ(meta.payload_ix0_, 0) << msg;
        EXPECT_EQ(meta.payload_ix1_, 0) << msg;
    }
    {
        //                 012345678901234567890123456789
        //                     TTTTTTTT
        const char* msg = "$FP,ODOMETRY,notanumber*xx\r\n";
        FpaMessageMeta meta;
        EXPECT_TRUE(FpaGetMessageMeta(meta, (const uint8_t*)msg, strlen(msg))) << msg;
        EXPECT_EQ(std::string(meta.msg_type_), std::string("ODOMETRY")) << msg;
        EXPECT_LT(meta.msg_version_, 0) << msg;
        EXPECT_EQ(meta.payload_ix0_, 13) << msg;
        EXPECT_EQ(meta.payload_ix1_, 22) << msg;
    }
    // Bad input
    {
        const char* msg = "whateverwhatever";
        FpaMessageMeta meta;
        EXPECT_FALSE(FpaGetMessageMeta(meta, NULL, 0)) << msg;
        EXPECT_FALSE(FpaGetMessageMeta(meta, NULL, -1)) << msg;
        EXPECT_FALSE(FpaGetMessageMeta(meta, (const uint8_t*)msg, 0)) << msg;
        EXPECT_FALSE(FpaGetMessageMeta(meta, (const uint8_t*)msg, -1)) << msg;
    }
    {
        const char* msg = "$ABCDEFGHIJKLMNOP";
        FpaMessageMeta meta;
        EXPECT_FALSE(FpaGetMessageMeta(meta, (const uint8_t*)msg, strlen(msg)));
    }
    {
        const char* msg = "$GNABCDEFGHIJKLMNOPQRSTUVW,123,456,789*xx\r\n";
        FpaMessageMeta meta;
        EXPECT_FALSE(FpaGetMessageMeta(meta, (const uint8_t*)msg, strlen(msg))) << msg;
        EXPECT_EQ(std::string(meta.msg_type_), std::string("")) << msg;
        EXPECT_EQ(meta.msg_version_, 0) << msg;
        EXPECT_EQ(meta.payload_ix0_, 0) << msg;
        EXPECT_EQ(meta.payload_ix1_, 0) << msg;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserFpaTest, FpaGetMessageName)
{
    {
        const char* msg = "$FP,ODOMETRY,1,...,...,...*xx\r\n";
        char name[100];
        EXPECT_TRUE(FpaGetMessageName(name, sizeof(name), (const uint8_t*)msg, strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("FP_A-ODOMETRY")) << msg;
    }
    {
        const char* msg = "$FP,ODOMETRY,1*xx\r\n";
        char name[100];
        EXPECT_TRUE(FpaGetMessageName(name, sizeof(name), (const uint8_t*)msg, strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("FP_A-ODOMETRY")) << msg;
    }
    {
        const char* msg = "$FP,ODOMETRY,X*xx\r\n";  // Bad msg_version
        char name[100];
        EXPECT_TRUE(FpaGetMessageName(name, sizeof(name), (const uint8_t*)msg, strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("FP_A-ODOMETRY")) << msg;
    }
    // Bad input
    {
        const char* msg = "whateverwhatever";
        char name[100];
        EXPECT_FALSE(FpaGetMessageName(name, sizeof(name), NULL, 0)) << msg;
        EXPECT_FALSE(FpaGetMessageName(name, sizeof(name), NULL, -1)) << msg;
        EXPECT_FALSE(FpaGetMessageName(name, sizeof(name), (const uint8_t*)msg, 0)) << msg;
        EXPECT_FALSE(FpaGetMessageName(name, sizeof(name), (const uint8_t*)msg, -1)) << msg;
        EXPECT_FALSE(FpaGetMessageName(NULL, 0, (const uint8_t*)msg, sizeof(msg))) << msg;
        EXPECT_FALSE(FpaGetMessageName(NULL, -1, (const uint8_t*)msg, sizeof(msg))) << msg;
        EXPECT_FALSE(FpaGetMessageName(name, 0, (const uint8_t*)msg, sizeof(msg))) << msg;
        EXPECT_FALSE(FpaGetMessageName(name, -1, (const uint8_t*)msg, sizeof(msg))) << msg;
        EXPECT_FALSE(FpaGetMessageName(NULL, 0, NULL, 0)) << msg;
        EXPECT_FALSE(FpaGetMessageName(NULL, 0, NULL, -1)) << msg;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(FpaTest, FpaGpsTime)
{
    const FpaGpsTime t1;
    const FpaGpsTime t2 = { { true, 1234 }, { true, 456789.123456 } };
    const FpaGpsTime t3 = { { true, 1234 }, { true, 456789.123457 } };
    const FpaGpsTime t4 = { { true, 1235 }, { true, 456789.123456 } };
    const FpaGpsTime t5 = { { true, 1235 }, { true, 456789.1234564 } };

    EXPECT_EQ(t1, t1);
    EXPECT_EQ(t2, t2);
    EXPECT_NE(t1, t2);

    EXPECT_GT(t3, t2);
    EXPECT_GT(t4, t3);
    EXPECT_GT(t4, t2);

    EXPECT_LT(t2, t3);
    EXPECT_LT(t3, t4);
    EXPECT_LT(t2, t4);

    EXPECT_GE(t3, t2);
    EXPECT_GE(t4, t3);
    EXPECT_GE(t4, t2);

    EXPECT_LE(t2, t3);
    EXPECT_LE(t3, t4);
    EXPECT_LE(t2, t4);

    EXPECT_EQ(t5, t4);
    EXPECT_GE(t5, t4);
    EXPECT_LE(t4, t5);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(FpaTest, FpaEoePayload)
{
    {
        FpaEoePayload payload;
        const char* msg = "$FP,EOE,1,2322,309663.800000,FUSION*62\r\n";
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_TRUE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 2322);
        EXPECT_TRUE(payload.gps_time.tow.valid);
        EXPECT_NEAR(payload.gps_time.tow.value, 309663.8, 1e-9);
        EXPECT_EQ(payload.epoch, FpaEpoch::FUSION);
    }
    {
        FpaEoePayload payload;
        const char* msg = "$FP,EOE,1,,,GNSS1*7C\r\n";
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_FALSE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 0);
        EXPECT_FALSE(payload.gps_time.tow.valid);
        EXPECT_EQ(payload.gps_time.tow.value, 0.0);
        EXPECT_EQ(payload.epoch, FpaEpoch::GNSS1);
    }
    {
        FpaEoePayload payload;
        const char* msg = "$FP,EOE,1,,,CHABIS*XX\r\n";
        EXPECT_FALSE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_FALSE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 0);
        EXPECT_FALSE(payload.gps_time.tow.valid);
        EXPECT_EQ(payload.gps_time.tow.value, 0.0);
        EXPECT_EQ(payload.epoch, FpaEpoch::UNSPECIFIED);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(FpaTest, FpaGnssantPayload)
{
    {
        FpaGnssantPayload payload;
        const char* msg = "$FP,GNSSANT,1,2234,305129.200151,short,off,0,open,on,28*65\r\n";
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_TRUE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 2234);
        EXPECT_TRUE(payload.gps_time.tow.valid);
        EXPECT_NEAR(payload.gps_time.tow.value, 305129.200151, 1e-9);
        EXPECT_EQ(payload.gnss1_state, FpaAntState::SHORT);
        EXPECT_EQ(payload.gnss1_power, FpaAntPower::OFF);
        EXPECT_TRUE(payload.gnss1_age.valid);
        EXPECT_EQ(payload.gnss1_age.value, 0);
        EXPECT_EQ(payload.gnss2_state, FpaAntState::OPEN);
        EXPECT_EQ(payload.gnss2_power, FpaAntPower::ON);
        EXPECT_TRUE(payload.gnss2_age.valid);
        EXPECT_EQ(payload.gnss2_age.value, 28);
    }
    {
        FpaGnssantPayload payload;
        const char* msg = "$FP,GNSSANT,1,,,,,,,,*XX\r\n";
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_FALSE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 0);
        EXPECT_FALSE(payload.gps_time.tow.valid);
        EXPECT_EQ(payload.gps_time.tow.value, 0.0);
        EXPECT_EQ(payload.gnss1_state, FpaAntState::UNSPECIFIED);
        EXPECT_EQ(payload.gnss1_power, FpaAntPower::UNSPECIFIED);
        EXPECT_FALSE(payload.gnss1_age.valid);
        EXPECT_EQ(payload.gnss1_age.value, 0);
        EXPECT_EQ(payload.gnss2_state, FpaAntState::UNSPECIFIED);
        EXPECT_EQ(payload.gnss2_power, FpaAntPower::UNSPECIFIED);
        EXPECT_FALSE(payload.gnss2_age.valid);
        EXPECT_EQ(payload.gnss2_age.value, 0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(FpaTest, FpaGnsscorrPayload)
{
    {
        FpaGnsscorrPayload payload;
        const char* msg =  // clang-format off
            "$FP,GNSSCORR,1,2237,250035.999865,8,25,18,8,25,18,0.2,1.0,0.8,5.3,2,47.366986804,8.532965023,481.1094,7254*3F\r\n";  // clang-format on
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_TRUE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 2237);
        EXPECT_TRUE(payload.gps_time.tow.valid);
        EXPECT_NEAR(payload.gps_time.tow.value, 250035.999865, 1e-9);
        EXPECT_EQ(payload.gnss1_fix, FpaGnssFix::RTK_FIXED);
        EXPECT_TRUE(payload.gnss1_nsig_l1.valid);
        EXPECT_EQ(payload.gnss1_nsig_l1.value, 25);
        EXPECT_TRUE(payload.gnss1_nsig_l2.valid);
        EXPECT_EQ(payload.gnss1_nsig_l2.value, 18);
        EXPECT_EQ(payload.gnss2_fix, FpaGnssFix::RTK_FIXED);
        EXPECT_TRUE(payload.gnss2_nsig_l1.valid);
        EXPECT_EQ(payload.gnss2_nsig_l1.value, 25);
        EXPECT_TRUE(payload.gnss2_nsig_l2.valid);
        EXPECT_EQ(payload.gnss2_nsig_l2.value, 18);
        EXPECT_TRUE(payload.corr_latency.valid);
        EXPECT_NEAR(payload.corr_latency.value, 0.2, 1e-3);
        EXPECT_TRUE(payload.corr_update_rate.valid);
        EXPECT_NEAR(payload.corr_update_rate.value, 1.0, 1e-3);
        EXPECT_TRUE(payload.corr_data_rate.valid);
        EXPECT_NEAR(payload.corr_data_rate.value, 0.8, 1e-3);
        EXPECT_TRUE(payload.corr_msg_rate.valid);
        EXPECT_NEAR(payload.corr_msg_rate.value, 5.3, 1e-3);
        EXPECT_TRUE(payload.sta_id.valid);
        EXPECT_EQ(payload.sta_id.value, 2);
        EXPECT_TRUE(payload.sta_llh.valid);
        EXPECT_NEAR(payload.sta_llh.values[0], 47.366986804, 1e-9);
        EXPECT_NEAR(payload.sta_llh.values[1], 8.532965023, 1e-9);
        EXPECT_NEAR(payload.sta_llh.values[2], 481.1094, 1e-4);
        EXPECT_TRUE(payload.sta_dist.valid);
        EXPECT_EQ(payload.sta_dist.value, 7254);
    }
    {
        FpaGnsscorrPayload payload;
        const char* msg = "$FP,GNSSCORR,1,2237,250548.999744,8,25,18,8,25,18,,,,,,,,,*24\r\n";
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_TRUE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 2237);
        EXPECT_TRUE(payload.gps_time.tow.valid);
        EXPECT_NEAR(payload.gps_time.tow.value, 250548.999744, 1e-9);

        EXPECT_EQ(payload.gnss1_fix, FpaGnssFix::RTK_FIXED);
        EXPECT_TRUE(payload.gnss1_nsig_l1.valid);
        EXPECT_EQ(payload.gnss1_nsig_l1.value, 25);
        EXPECT_TRUE(payload.gnss1_nsig_l2.valid);
        EXPECT_EQ(payload.gnss1_nsig_l2.value, 18);
        EXPECT_EQ(payload.gnss2_fix, FpaGnssFix::RTK_FIXED);
        EXPECT_TRUE(payload.gnss2_nsig_l1.valid);
        EXPECT_EQ(payload.gnss2_nsig_l1.value, 25);
        EXPECT_TRUE(payload.gnss2_nsig_l2.valid);
        EXPECT_EQ(payload.gnss2_nsig_l2.value, 18);
        EXPECT_FALSE(payload.corr_latency.valid);
        EXPECT_EQ(payload.corr_latency.value, 0.0);
        EXPECT_FALSE(payload.corr_update_rate.valid);
        EXPECT_EQ(payload.corr_update_rate.value, 0.0);
        EXPECT_FALSE(payload.corr_data_rate.valid);
        EXPECT_EQ(payload.corr_data_rate.value, 0.0);
        EXPECT_FALSE(payload.corr_msg_rate.valid);
        EXPECT_EQ(payload.corr_msg_rate.value, 0.0);
        EXPECT_FALSE(payload.sta_id.valid);
        EXPECT_EQ(payload.sta_id.value, 0);
        EXPECT_FALSE(payload.sta_llh.valid);
        EXPECT_EQ(payload.sta_llh.values[0], 0.0);
        EXPECT_EQ(payload.sta_llh.values[1], 0.0);
        EXPECT_EQ(payload.sta_llh.values[2], 0.0);
        EXPECT_FALSE(payload.sta_dist.valid);
        EXPECT_EQ(payload.sta_dist.value, 0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(FpaTest, FpaRawimuPayload)
{
    {
        FpaRawimuPayload payload;
        const char* msg =  // clang-format off
            "$FP,RAWIMU,1,2197,126191.777855,-0.199914,0.472851,9.917973,0.023436,0.007723,0.002131*34\r\n";  // clang-format on
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_EQ(payload.which, FpaImuPayload::Which::RAWIMU);
        EXPECT_TRUE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 2197);
        EXPECT_TRUE(payload.gps_time.tow.valid);
        EXPECT_NEAR(payload.gps_time.tow.value, 126191.777855, 1e-9);
        EXPECT_TRUE(payload.acc.valid);
        EXPECT_NEAR(payload.acc.values[0], -0.199914, 1e-9);
        EXPECT_NEAR(payload.acc.values[1], 0.472851, 1e-9);
        EXPECT_NEAR(payload.acc.values[2], 9.917973, 1e-9);
        EXPECT_TRUE(payload.rot.valid);
        EXPECT_NEAR(payload.rot.values[0], 0.023436, 1e-9);
        EXPECT_NEAR(payload.rot.values[1], 0.007723, 1e-9);
        EXPECT_NEAR(payload.rot.values[2], 0.002131, 1e-9);
    }
    {
        FpaRawimuPayload payload;
        const char* msg =  // clang-format off
            "$FP,RAWIMU,1,,,,,,,,*XX\r\n";  // clang-format on
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_EQ(payload.which, FpaImuPayload::Which::RAWIMU);
        EXPECT_FALSE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 0);
        EXPECT_FALSE(payload.gps_time.tow.valid);
        EXPECT_EQ(payload.gps_time.tow.value, 0.0);
        EXPECT_FALSE(payload.acc.valid);
        EXPECT_EQ(payload.acc.values[0], 0.0);
        EXPECT_EQ(payload.acc.values[1], 0.0);
        EXPECT_EQ(payload.acc.values[2], 0.0);
        EXPECT_FALSE(payload.rot.valid);
        EXPECT_EQ(payload.rot.values[0], 0.0);
        EXPECT_EQ(payload.rot.values[1], 0.0);
        EXPECT_EQ(payload.rot.values[2], 0.0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(FpaTest, FpaCorrimuPayload)
{
    {
        FpaCorrimuPayload payload;
        const char* msg =  // clang-format off
            "$FP,CORRIMU,1,2197,126191.777855,-0.195224,0.393969,9.869998,0.013342,-0.004620,-0.000728*7D\r\n";  // clang-format on
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_EQ(payload.which, FpaImuPayload::Which::CORRIMU);
        EXPECT_TRUE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 2197);
        EXPECT_TRUE(payload.gps_time.tow.valid);
        EXPECT_NEAR(payload.gps_time.tow.value, 126191.777855, 1e-9);
        EXPECT_TRUE(payload.acc.valid);
        EXPECT_NEAR(payload.acc.values[0], -0.195224, 1e-9);
        EXPECT_NEAR(payload.acc.values[1], 0.393969, 1e-9);
        EXPECT_NEAR(payload.acc.values[2], 9.869998, 1e-9);
        EXPECT_TRUE(payload.rot.valid);
        EXPECT_NEAR(payload.rot.values[0], 0.013342, 1e-9);
        EXPECT_NEAR(payload.rot.values[1], -0.004620, 1e-9);
        EXPECT_NEAR(payload.rot.values[2], -0.000728, 1e-9);
    }
    {
        FpaCorrimuPayload payload;
        const char* msg =  // clang-format off
            "$FP,CORRIMU,1,,,,,,,,*XX\r\n";  // clang-format on
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_EQ(payload.which, FpaImuPayload::Which::CORRIMU);
        EXPECT_FALSE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 0);
        EXPECT_FALSE(payload.gps_time.tow.valid);
        EXPECT_EQ(payload.gps_time.tow.value, 0.0);
        EXPECT_FALSE(payload.acc.valid);
        EXPECT_EQ(payload.acc.values[0], 0.0);
        EXPECT_EQ(payload.acc.values[1], 0.0);
        EXPECT_EQ(payload.acc.values[2], 0.0);
        EXPECT_FALSE(payload.rot.valid);
        EXPECT_EQ(payload.rot.values[0], 0.0);
        EXPECT_EQ(payload.rot.values[1], 0.0);
        EXPECT_EQ(payload.rot.values[2], 0.0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(FpaTest, FpaImubiasPayload)
{
    {
        FpaImubiasPayload payload;
        const char* msg =  // clang-format off
            "$FP,IMUBIAS,1,2342,321247.000000,2,1,1,3,0.008914,0.019806,0.150631,0.027202,0.010599,0.011393,0.00001,0.00001,0.00001,0.00001,0.00001,0.00001*4C\r\n";  // clang-format on
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_TRUE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 2342);
        EXPECT_TRUE(payload.gps_time.tow.valid);
        EXPECT_NEAR(payload.gps_time.tow.value, 321247.0, 1e-9);
        EXPECT_EQ(payload.fusion_imu, FpaMeasStatus::DEGRADED);
        EXPECT_EQ(payload.imu_status, FpaImuStatus::WARMSTARTED);
        EXPECT_EQ(payload.imu_noise, FpaImuNoise::LOW_NOISE);
        EXPECT_EQ(payload.imu_conv, FpaImuConv::WAIT_MOTION);
        EXPECT_TRUE(payload.bias_acc.valid);
        EXPECT_NEAR(payload.bias_acc.values[0], 0.008914, 1e-9);
        EXPECT_NEAR(payload.bias_acc.values[1], 0.019806, 1e-9);
        EXPECT_NEAR(payload.bias_acc.values[2], 0.150631, 1e-9);
        EXPECT_TRUE(payload.bias_gyr.valid);
        EXPECT_NEAR(payload.bias_gyr.values[0], 0.027202, 1e-9);
        EXPECT_NEAR(payload.bias_gyr.values[1], 0.010599, 1e-9);
        EXPECT_NEAR(payload.bias_gyr.values[2], 0.011393, 1e-9);
        EXPECT_TRUE(payload.bias_cov_acc.valid);
        EXPECT_NEAR(payload.bias_cov_acc.values[0], 0.00001, 1e-9);
        EXPECT_NEAR(payload.bias_cov_acc.values[1], 0.00001, 1e-9);
        EXPECT_NEAR(payload.bias_cov_acc.values[2], 0.00001, 1e-9);
        EXPECT_TRUE(payload.bias_cov_gyr.valid);
        EXPECT_NEAR(payload.bias_cov_gyr.values[0], 0.00001, 1e-9);
        EXPECT_NEAR(payload.bias_cov_gyr.values[1], 0.00001, 1e-9);
        EXPECT_NEAR(payload.bias_cov_gyr.values[2], 0.00001, 1e-9);
    }
    {
        FpaImubiasPayload payload;
        const char* msg = "$FP,IMUBIAS,1,2342,323844.000000,,,,,,,,,,,,,,,,*4C\r\n";
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_TRUE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 2342);
        EXPECT_TRUE(payload.gps_time.tow.valid);
        EXPECT_NEAR(payload.gps_time.tow.value, 323844.0, 1e-9);
        EXPECT_EQ(payload.fusion_imu, FpaMeasStatus::UNSPECIFIED);
        EXPECT_EQ(payload.imu_status, FpaImuStatus::UNSPECIFIED);
        EXPECT_EQ(payload.imu_noise, FpaImuNoise::UNSPECIFIED);
        EXPECT_EQ(payload.imu_conv, FpaImuConv::UNSPECIFIED);
        EXPECT_FALSE(payload.bias_acc.valid);
        EXPECT_EQ(payload.bias_acc.values[0], 0.0);
        EXPECT_EQ(payload.bias_acc.values[1], 0.0);
        EXPECT_EQ(payload.bias_acc.values[2], 0.0);
        EXPECT_FALSE(payload.bias_gyr.valid);
        EXPECT_EQ(payload.bias_gyr.values[0], 0.0);
        EXPECT_EQ(payload.bias_gyr.values[1], 0.0);
        EXPECT_EQ(payload.bias_gyr.values[2], 0.0);
        EXPECT_FALSE(payload.bias_cov_acc.valid);
        EXPECT_EQ(payload.bias_cov_acc.values[0], 0.0);
        EXPECT_EQ(payload.bias_cov_acc.values[1], 0.0);
        EXPECT_EQ(payload.bias_cov_acc.values[2], 0.0);
        EXPECT_FALSE(payload.bias_cov_gyr.valid);
        EXPECT_EQ(payload.bias_cov_gyr.values[0], 0.0);
        EXPECT_EQ(payload.bias_cov_gyr.values[1], 0.0);
        EXPECT_EQ(payload.bias_cov_gyr.values[2], 0.0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(FpaTest, FpaLlhPayload)
{
    {
        FpaLlhPayload payload;
        const char* msg =  // clang-format off
            "$FP,LLH,1,2231,227563.250000,47.392357470,8.448121451,473.5857,0.04533,0.03363,0.02884,0.00417,0.00086,-0.00136*62\r\n";  // clang-format on
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_TRUE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 2231);
        EXPECT_TRUE(payload.gps_time.tow.valid);
        EXPECT_NEAR(payload.gps_time.tow.value, 227563.250000, 1e-9);
        EXPECT_TRUE(payload.llh.valid);
        EXPECT_NEAR(payload.llh.values[0], 47.392357470, 1e-9);
        EXPECT_NEAR(payload.llh.values[1], 8.448121451, 1e-9);
        EXPECT_NEAR(payload.llh.values[2], 473.5857, 1e-9);
        EXPECT_TRUE(payload.cov_enu.valid);
        EXPECT_NEAR(payload.cov_enu.values[0], 0.04533, 1e-9);
        EXPECT_NEAR(payload.cov_enu.values[1], 0.03363, 1e-9);
        EXPECT_NEAR(payload.cov_enu.values[2], 0.02884, 1e-9);
        EXPECT_NEAR(payload.cov_enu.values[3], 0.00417, 1e-9);
        EXPECT_NEAR(payload.cov_enu.values[4], 0.00086, 1e-9);
        EXPECT_NEAR(payload.cov_enu.values[5], -0.00136, 1e-9);
    }
    {
        FpaLlhPayload payload;
        const char* msg = "$FP,LLH,1,2231,227563.250000,,,,,,,,,*XX\r\n";
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_TRUE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 2231);
        EXPECT_TRUE(payload.gps_time.tow.valid);
        EXPECT_NEAR(payload.gps_time.tow.value, 227563.250000, 1e-9);
        EXPECT_FALSE(payload.llh.valid);
        EXPECT_EQ(payload.llh.values[0], 0.0);
        EXPECT_EQ(payload.llh.values[1], 0.0);
        EXPECT_EQ(payload.llh.values[2], 0.0);
        EXPECT_FALSE(payload.cov_enu.valid);
        EXPECT_EQ(payload.cov_enu.values[0], 0.0);
        EXPECT_EQ(payload.cov_enu.values[1], 0.0);
        EXPECT_EQ(payload.cov_enu.values[2], 0.0);
        EXPECT_EQ(payload.cov_enu.values[3], 0.0);
        EXPECT_EQ(payload.cov_enu.values[4], 0.0);
        EXPECT_EQ(payload.cov_enu.values[5], 0.0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(FpaTest, FpaOdometryPayload)
{
    {
        FpaOdometryPayload payload;
        const char* msg =  // clang-format off
            "$FP,ODOMETRY,2,2231,227610.750000,4279243.1641,635824.2171,4671589.8683,-0.412792,0.290804,-0.123898,0.854216,"
            "-17.1078,-0.0526,-0.3252,0.02245,0.00275,0.10369,-1.0385,-1.3707,9.8249,4,1,8,8,1,"
            "0.01761,0.02274,0.01713,-0.00818,0.00235,0.00129,0.00013,0.00015,0.00014,-0.00001,0.00001,0.00002,"
            "0.03482,0.06244,0.05480,0.00096,0.00509,0.00054,fp_release_vr2_2.54.0_160*4F\r\n";  // clang-format on
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_EQ(payload.which, FpaOdomPayload::Which::ODOMETRY);
        EXPECT_TRUE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 2231);
        EXPECT_TRUE(payload.gps_time.tow.valid);
        EXPECT_NEAR(payload.gps_time.tow.value, 227610.750000, 1e-9);
        EXPECT_TRUE(payload.pos.valid);
        EXPECT_NEAR(payload.pos.values[0], 4279243.1641, 1e-9);
        EXPECT_NEAR(payload.pos.values[1], 635824.2171, 1e-9);
        EXPECT_NEAR(payload.pos.values[2], 4671589.8683, 1e-9);
        EXPECT_TRUE(payload.orientation.valid);
        EXPECT_NEAR(payload.orientation.values[0], -0.412792, 1e-9);
        EXPECT_NEAR(payload.orientation.values[1], 0.290804, 1e-9);
        EXPECT_NEAR(payload.orientation.values[2], -0.123898, 1e-9);
        EXPECT_NEAR(payload.orientation.values[3], 0.854216, 1e-9);
        EXPECT_TRUE(payload.vel.valid);
        EXPECT_NEAR(payload.vel.values[0], -17.1078, 1e-9);
        EXPECT_NEAR(payload.vel.values[1], -0.0526, 1e-9);
        EXPECT_NEAR(payload.vel.values[2], -0.3252, 1e-9);
        EXPECT_TRUE(payload.rot.valid);
        EXPECT_NEAR(payload.rot.values[0], 0.02245, 1e-9);
        EXPECT_NEAR(payload.rot.values[1], 0.00275, 1e-9);
        EXPECT_NEAR(payload.rot.values[2], 0.10369, 1e-9);
        EXPECT_TRUE(payload.acc.valid);
        EXPECT_NEAR(payload.acc.values[0], -1.0385, 1e-9);
        EXPECT_NEAR(payload.acc.values[1], -1.3707, 1e-9);
        EXPECT_NEAR(payload.acc.values[2], 9.8249, 1e-9);
        EXPECT_EQ(payload.fusion_status, FpaFusionStatusLegacy::VIO_GNSS);
        EXPECT_EQ(payload.imu_bias_status, FpaImuStatusLegacy::CONVERGED);
        EXPECT_EQ(payload.gnss1_fix, FpaGnssFix::RTK_FIXED);
        EXPECT_EQ(payload.gnss2_fix, FpaGnssFix::RTK_FIXED);
        EXPECT_EQ(payload.wheelspeed_status, FpaWsStatusLegacy::ONE_OR_MORE_CONVERGED);
        EXPECT_TRUE(payload.pos_cov.valid);
        EXPECT_NEAR(payload.pos_cov.values[0], 0.01761, 1e-9);
        EXPECT_NEAR(payload.pos_cov.values[1], 0.02274, 1e-9);
        EXPECT_NEAR(payload.pos_cov.values[2], 0.01713, 1e-9);
        EXPECT_NEAR(payload.pos_cov.values[3], -0.00818, 1e-9);
        EXPECT_NEAR(payload.pos_cov.values[4], 0.00235, 1e-9);
        EXPECT_NEAR(payload.pos_cov.values[5], 0.00129, 1e-9);
        EXPECT_TRUE(payload.orientation_cov.valid);
        EXPECT_NEAR(payload.orientation_cov.values[0], 0.00013, 1e-9);
        EXPECT_NEAR(payload.orientation_cov.values[1], 0.00015, 1e-9);
        EXPECT_NEAR(payload.orientation_cov.values[2], 0.00014, 1e-9);
        EXPECT_NEAR(payload.orientation_cov.values[3], -0.00001, 1e-9);
        EXPECT_NEAR(payload.orientation_cov.values[4], 0.00001, 1e-9);
        EXPECT_NEAR(payload.orientation_cov.values[5], 0.00002, 1e-9);
        EXPECT_TRUE(payload.vel_cov.valid);
        EXPECT_NEAR(payload.vel_cov.values[0], 0.03482, 1e-9);
        EXPECT_NEAR(payload.vel_cov.values[1], 0.06244, 1e-9);
        EXPECT_NEAR(payload.vel_cov.values[2], 0.05480, 1e-9);
        EXPECT_NEAR(payload.vel_cov.values[3], 0.00096, 1e-9);
        EXPECT_NEAR(payload.vel_cov.values[4], 0.00509, 1e-9);
        EXPECT_NEAR(payload.vel_cov.values[5], 0.00054, 1e-9);
        EXPECT_EQ(std::string(payload.version), "fp_release_vr2_2.54.0_160");
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(FpaTest, FpaOdomenuPayload)
{
    {
        FpaOdomenuPayload payload;
        const char* msg =  // clang-format off
            "$FP,ODOMENU,1,2180,298591.500000,-1.8339,2.6517,-0.0584,0.556794,-0.042551,-0.007850,0.829523,2.2993,"
            "-1.6994,-0.0222,0.20063,0.08621,-1.21972,-3.6947,-3.3827,9.7482,4,1,8,8,1,0.00415,0.00946,0.00746,"
            "-0.00149,-0.00084,0.00025,0.00003,0.00003,0.00012,0.00000,0.00000,0.00000,0.01742,0.01146,0.01612,"
            "-0.00550,-0.00007,-0.00050*74\r\n";  // clang-format on
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_EQ(payload.which, FpaOdomPayload::Which::ODOMENU);
        EXPECT_TRUE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 2180);
        EXPECT_TRUE(payload.gps_time.tow.valid);
        EXPECT_NEAR(payload.gps_time.tow.value, 298591.500000, 1e-9);
        EXPECT_TRUE(std::string(payload.version).empty());
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(FpaTest, FpaOdomshPayload)
{
    {
        FpaOdomshPayload payload;
        const char* msg =  // clang-format off
            "$FP,ODOMSH,1,2180,298591.500000,-1.8339,2.6517,-0.0584,0.556794,-0.042551,-0.007850,0.829523,2.2993,"
            "-1.6994,-0.0222,0.20063,0.08621,-1.21972,-3.6947,-3.3827,9.7482,4,1,8,8,1,0.00415,0.00946,0.00746,"
            "-0.00149,-0.00084,0.00025,0.00003,0.00003,0.00012,0.00000,0.00000,0.00000,0.01742,0.01146,0.01612,"
            "-0.00550,-0.00007,-0.00050*74\r\n";  // clang-format on
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_EQ(payload.which, FpaOdomPayload::Which::ODOMSH);
        EXPECT_TRUE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 2180);
        EXPECT_TRUE(payload.gps_time.tow.valid);
        EXPECT_NEAR(payload.gps_time.tow.value, 298591.500000, 1e-9);
        EXPECT_TRUE(std::string(payload.version).empty());
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(FpaTest, FpaOdomstatusPayload)
{
    {
        FpaOdomstatusPayload payload;
        const char* msg =  // clang-format off
            "$FP,ODOMSTATUS,1,2342,321241.350000,2,2,1,1,1,1,,0,,,,,,1,1,3,8,8,3,5,5,,0,6,,,,,,,,,,,,*24\r\n";  // clang-format on
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_TRUE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 2342);
        EXPECT_TRUE(payload.gps_time.tow.valid);
        EXPECT_NEAR(payload.gps_time.tow.value, 321241.350000, 1e-9);
        EXPECT_EQ(payload.init_status, FpaInitStatus::GLOBAL_INIT);
        EXPECT_EQ(payload.fusion_imu, FpaMeasStatus::DEGRADED);
        EXPECT_EQ(payload.fusion_gnss1, FpaMeasStatus::USED);
        EXPECT_EQ(payload.fusion_gnss2, FpaMeasStatus::USED);
        EXPECT_EQ(payload.fusion_corr, FpaMeasStatus::USED);
        EXPECT_EQ(payload.fusion_cam1, FpaMeasStatus::USED);
        EXPECT_EQ(payload.fusion_ws, FpaMeasStatus::NOT_USED);
        EXPECT_EQ(payload.fusion_markers, FpaMeasStatus::UNSPECIFIED);
        EXPECT_EQ(payload.imu_status, FpaImuStatus::WARMSTARTED);
        EXPECT_EQ(payload.imu_noise, FpaImuNoise::LOW_NOISE);
        EXPECT_EQ(payload.imu_conv, FpaImuConv::WAIT_MOTION);
        EXPECT_EQ(payload.gnss1_status, FpaGnssStatus::RTK_FIXED);
        EXPECT_EQ(payload.gnss2_status, FpaGnssStatus::RTK_FIXED);
        EXPECT_EQ(payload.baseline_status, FpaBaselineStatus::PASSING);
        EXPECT_EQ(payload.corr_status, FpaCorrStatus::GOOD_CORR);
        EXPECT_EQ(payload.cam1_status, FpaCamStatus::GOOD);
        EXPECT_EQ(payload.ws_status, FpaWsStatus::NOT_ENABLED);
        EXPECT_EQ(payload.ws_conv, FpaWsConv::IDLE);
        EXPECT_EQ(payload.markers_status, FpaMarkersStatus::UNSPECIFIED);
        EXPECT_EQ(payload.markers_conv, FpaMarkersConv::UNSPECIFIED);
    }
    {
        FpaOdomstatusPayload payload;
        const char* msg = "$FP,ODOMSTATUS,1,2342,324917.700000,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,*1E\r\n";
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_TRUE(payload.gps_time.week.valid);
        EXPECT_EQ(payload.gps_time.week.value, 2342);
        EXPECT_TRUE(payload.gps_time.tow.valid);
        EXPECT_NEAR(payload.gps_time.tow.value, 324917.700000, 1e-9);
        EXPECT_EQ(payload.init_status, FpaInitStatus::UNSPECIFIED);
        EXPECT_EQ(payload.fusion_imu, FpaMeasStatus::UNSPECIFIED);
        EXPECT_EQ(payload.fusion_gnss1, FpaMeasStatus::UNSPECIFIED);
        EXPECT_EQ(payload.fusion_gnss2, FpaMeasStatus::UNSPECIFIED);
        EXPECT_EQ(payload.fusion_corr, FpaMeasStatus::UNSPECIFIED);
        EXPECT_EQ(payload.fusion_cam1, FpaMeasStatus::UNSPECIFIED);
        EXPECT_EQ(payload.fusion_ws, FpaMeasStatus::UNSPECIFIED);
        EXPECT_EQ(payload.fusion_markers, FpaMeasStatus::UNSPECIFIED);
        EXPECT_EQ(payload.imu_status, FpaImuStatus::UNSPECIFIED);
        EXPECT_EQ(payload.imu_noise, FpaImuNoise::UNSPECIFIED);
        EXPECT_EQ(payload.imu_conv, FpaImuConv::UNSPECIFIED);
        EXPECT_EQ(payload.gnss1_status, FpaGnssStatus::UNSPECIFIED);
        EXPECT_EQ(payload.gnss2_status, FpaGnssStatus::UNSPECIFIED);
        EXPECT_EQ(payload.baseline_status, FpaBaselineStatus::UNSPECIFIED);
        EXPECT_EQ(payload.corr_status, FpaCorrStatus::UNSPECIFIED);
        EXPECT_EQ(payload.cam1_status, FpaCamStatus::UNSPECIFIED);
        EXPECT_EQ(payload.ws_status, FpaWsStatus::UNSPECIFIED);
        EXPECT_EQ(payload.ws_conv, FpaWsConv::UNSPECIFIED);
        EXPECT_EQ(payload.markers_status, FpaMarkersStatus::UNSPECIFIED);
        EXPECT_EQ(payload.markers_conv, FpaMarkersConv::UNSPECIFIED);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(FpaTest, FpaTextPayload)
{
    {
        FpaTextPayload payload;
        const char* msg = "$FP,TEXT,1,INFO,Fixposition AG - www.fixposition.com*09\r\n";
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_EQ(payload.level, FpaTextLevel::INFO);
        EXPECT_EQ(std::string(payload.text), "Fixposition AG - www.fixposition.com");
    }
    {
        FpaTextPayload payload;
        const char* msg = "$FP,TEXT,1,NOPE,blabla*09\r\n";
        EXPECT_FALSE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_EQ(payload.level, FpaTextLevel::UNSPECIFIED);
        EXPECT_EQ(payload.text[0], '\0');
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(FpaTest, FpaTfPayload)
{
    {
        FpaTfPayload payload;
        const char* msg =  // clang-format off
            "$FP,TF,2,2233,315835.000000,VRTK,CAM,-0.00000,-0.00000,-0.00000,1.000000,0.000000,0.000000,0.000000*6B\r\n";  // clang-format on
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_EQ(payload.gps_time.week.value, 2233);
        EXPECT_TRUE(payload.gps_time.tow.valid);
        EXPECT_NEAR(payload.gps_time.tow.value, 315835.000000, 1e-9);
        EXPECT_EQ(std::string(payload.frame_a), "VRTK");
        EXPECT_EQ(std::string(payload.frame_b), "CAM");
        EXPECT_TRUE(payload.translation.valid);
        EXPECT_NEAR(payload.translation.values[0], 0.0, 1e-9);
        EXPECT_NEAR(payload.translation.values[1], 0.0, 1e-9);
        EXPECT_NEAR(payload.translation.values[2], 0.0, 1e-9);
        EXPECT_TRUE(payload.orientation.valid);
        EXPECT_NEAR(payload.orientation.values[0], 1.0, 1e-9);
        EXPECT_NEAR(payload.orientation.values[1], 0.0, 1e-9);
        EXPECT_NEAR(payload.orientation.values[2], 0.0, 1e-9);
        EXPECT_NEAR(payload.orientation.values[3], 0.0, 1e-9);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(FpaTest, FpaTpPayload)
{
    {
        FpaTpPayload payload;
        const char* msg = "$FP,TP,1,GNSS1,UTC,USNO,195391,0.000000000000,18*4F\r\n";
        EXPECT_TRUE(payload.SetFromMsg((const uint8_t*)msg, strlen(msg)));
        EXPECT_EQ(std::string(payload.tp_name), "GNSS1");
        EXPECT_EQ(payload.timebase, FpaTimebase::UTC);
        EXPECT_EQ(payload.timeref, FpaTimeref::UTC_USNO);
        EXPECT_TRUE(payload.tp_tow_sec.valid);
        EXPECT_EQ(payload.tp_tow_sec.value, 195391);
        EXPECT_TRUE(payload.tp_tow_psec.valid);
        EXPECT_NEAR(payload.tp_tow_psec.value, 0.0, 1e-10);
        EXPECT_TRUE(payload.gps_leaps.valid);
        EXPECT_EQ(payload.gps_leaps.value, 18);
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
