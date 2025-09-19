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
 * @brief Fixposition SDK: tests for fpsdk::common::parser::nmea
 */

/* LIBC/STL */
#include <cstring>
#include <string>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/nmea.hpp>
#include <fpsdk_common/string.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::parser::nmea;
using namespace fpsdk::common::string;

TEST(ParserNmeaTest, NmeaGetMessageMeta)
{
    // Standard NMEA
    {
        //                 012345678901234567890123456789
        //                  TTFFF PPPPPPPPPPP
        const char* msg = "$GNGGA,123,456,789*xx\r\n";
        NmeaMessageMeta meta;
        EXPECT_TRUE(NmeaGetMessageMeta(meta, (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(meta.talker_), std::string("GN")) << msg;
        EXPECT_EQ(std::string(meta.formatter_), std::string("GGA")) << msg;
        EXPECT_EQ(meta.payload_ix0_, 7) << msg;
        EXPECT_EQ(meta.payload_ix1_, 17) << msg;
    }

    // u-blox proprietary
    {
        //                 012345678901234567890123456789
        //                  TFFF PPPPPPPPPPPPPP
        const char* msg = "$PUBX,nn,123,456,789*xx\r\n";
        NmeaMessageMeta meta;
        EXPECT_TRUE(NmeaGetMessageMeta(meta, (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(meta.talker_), std::string("P")) << msg;
        EXPECT_EQ(std::string(meta.formatter_), std::string("UBX")) << msg;
        EXPECT_EQ(meta.payload_ix0_, 6) << msg;
        EXPECT_EQ(meta.payload_ix1_, 19) << msg;
    }

    // Fixposition proprietary (FP_A). Compare tests for FpaGetMessageMeta
    {
        //                 012345678901234567890123456789
        //                  TT,PPPPPPPPPPPPPPPPPPPPPP
        const char* msg = "$FP,ODOMETRY,n,...,...,...*xx\r\n";
        NmeaMessageMeta meta;
        EXPECT_TRUE(NmeaGetMessageMeta(meta, (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(meta.talker_), std::string("FP")) << msg;
        EXPECT_EQ(std::string(meta.formatter_), std::string("")) << msg;
        EXPECT_EQ(meta.payload_ix0_, 4) << msg;
        EXPECT_EQ(meta.payload_ix1_, 25) << msg;
    }

    // Empty NMEA message
    {
        //                 012345678901234567890123456789
        //                  TFFFF
        const char* msg = "$GPGGA*xx\r\n";
        NmeaMessageMeta meta;
        EXPECT_TRUE(NmeaGetMessageMeta(meta, (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(meta.talker_), std::string("GP")) << msg;
        EXPECT_EQ(std::string(meta.formatter_), std::string("GGA")) << msg;
        EXPECT_EQ(meta.payload_ix0_, 0) << msg;
        EXPECT_EQ(meta.payload_ix1_, 0) << msg;
    }

    // Bad input
    {
        const char* msg = "whateverwhatever";
        NmeaMessageMeta meta;
        EXPECT_FALSE(NmeaGetMessageMeta(meta, NULL, 0)) << msg;
        EXPECT_FALSE(NmeaGetMessageMeta(meta, NULL, -1)) << msg;
        EXPECT_FALSE(NmeaGetMessageMeta(meta, (const uint8_t*)msg, 0)) << msg;
        EXPECT_FALSE(NmeaGetMessageMeta(meta, (const uint8_t*)msg, -1)) << msg;
    }
    {
        const char* msg = "$ABCDEFGHIJKLMNOP";
        NmeaMessageMeta meta;
        EXPECT_FALSE(NmeaGetMessageMeta(meta, (const uint8_t*)msg, std::strlen(msg)));
    }
    {
        const char* msg = "$GNABCDEFGHIJKLMNOPQRSTUVW,123,456,789*xx\r\n";
        NmeaMessageMeta meta;
        EXPECT_FALSE(NmeaGetMessageMeta(meta, (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(meta.talker_), std::string("GN")) << msg;
        EXPECT_EQ(std::string(meta.formatter_), std::string("")) << msg;
        EXPECT_EQ(meta.payload_ix0_, 0) << msg;
        EXPECT_EQ(meta.payload_ix1_, 0) << msg;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNmeaTest, NmeaGetMessageName)
{
    // Standard NMEA
    {
        const char* msg = "$GNGGA,123,456,789*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-GN-GGA")) << msg;
    }

    // u-blox proprietary
    {
        const char* msg = "$PUBX,41,123,456,789*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-PUBX-CONFIG")) << msg;
    }
    {
        const char* msg = "$PUBX,00,123,456,789*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-PUBX-POSITION")) << msg;
    }
    {
        const char* msg = "$PUBX,40,123,456,789*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-PUBX-RATE")) << msg;
    }
    {
        const char* msg = "$PUBX,03,123,456,789*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-PUBX-SVSTATUS")) << msg;
    }
    {
        const char* msg = "$PUBX,04,123,456,789*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-PUBX-TIME")) << msg;
    }
    {
        const char* msg = "$PUBX,99,123,456,789*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-PUBX-99")) << msg;
    }
    {
        const char* msg = "$PUBX,42,123,456,789*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-PUBX-42")) << msg;
    }
    {
        const char* msg = "$PUBX,01,123,456,789*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-PUBX-01")) << msg;
    }
    // Bad u-blox proprietary
    {
        const char* msg = "$PUBXX,00,123,456,789*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-P-UBXX")) << msg;
    }
    {
        const char* msg = "$PVBX,00,123,456,789*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-P-VBX")) << msg;
    }
    {
        const char* msg = "$PUCX,00,123,456,789*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-P-UCX")) << msg;
    }
    {
        const char* msg = "$PUBY,00,123,456,789*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-P-UBY")) << msg;
    }

    // Other NMEA proprietary
    {
        const char* msg = "$PABC,123,456,789*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-P-ABC")) << msg;
    }

    // Fixposition proprietary (FP_A)
    {
        const char* msg = "$FP,ODOMETRY,n,...,...,...*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-FP-?")) << msg;
        // EXPECT_FALSE(NmeaGetMessageName(name, 10, (const uint8_t*)msg, std::strlen(msg))) << msg;
    }
    // Bad FP_A
    {
        const char* msg = "$GP,ODOMETRY,n,...,...,...*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-GP-?")) << msg;
    }
    {
        const char* msg = "$FQ,ODOMETRY,n,...,...,...*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-FQ-?")) << msg;
    }
    {
        const char* msg = "$FPP,ODOMETRY,n,...,...,...*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-FP-P")) << msg;
    }

    // Empty NMEA message
    {
        const char* msg = "$GPGGA*xx\r\n";
        char name[100];
        EXPECT_TRUE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-GP-GGA")) << msg;
    }

    // Bad input
    {
        const char* msg = "whateverwhatever";
        char name[100];
        EXPECT_FALSE(NmeaGetMessageName(name, sizeof(name), NULL, 0)) << msg;
        EXPECT_FALSE(NmeaGetMessageName(name, sizeof(name), NULL, -1)) << msg;
        EXPECT_FALSE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, 0)) << msg;
        EXPECT_FALSE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, -1)) << msg;
        EXPECT_FALSE(NmeaGetMessageName(NULL, 0, (const uint8_t*)msg, sizeof(msg))) << msg;
        EXPECT_FALSE(NmeaGetMessageName(NULL, -1, (const uint8_t*)msg, sizeof(msg))) << msg;
        EXPECT_FALSE(NmeaGetMessageName(name, 0, (const uint8_t*)msg, sizeof(msg))) << msg;
        EXPECT_FALSE(NmeaGetMessageName(name, -1, (const uint8_t*)msg, sizeof(msg))) << msg;
        EXPECT_FALSE(NmeaGetMessageName(NULL, 0, NULL, 0)) << msg;
        EXPECT_FALSE(NmeaGetMessageName(NULL, 0, NULL, -1)) << msg;
    }

    // Too small string is cut
    {
        const char* msg = "$GPGGA*xx\r\n";
        char name[10];
        EXPECT_FALSE(NmeaGetMessageName(name, sizeof(name), (const uint8_t*)msg, std::strlen(msg))) << msg;
        EXPECT_EQ(std::string(name), std::string("NMEA-GP-G")) << msg;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNmeaTest, NmeaTime)
{
    const NmeaTime t1 = { true, 11, 44, 1.0 };
    EXPECT_TRUE(t1.valid);
    EXPECT_EQ(t1.hours, 11);
    EXPECT_EQ(t1.mins, 44);
    EXPECT_NEAR(t1.secs, 1.0, 1e-9);
    const NmeaTime t1copy = t1;
    const NmeaTime t2 = { true, 11, 44, 2.0 };
    const NmeaTime t3 = { true, 11, 44, 2.0 + DBL_EPSILON };
    const NmeaTime t4 = { true, 11, 44, 2.00000001 };
    const NmeaTime t5 = { false, 11, 44, 2.00000001 };
    EXPECT_FALSE(t5.valid);
    EXPECT_EQ(t1, t1copy);
    EXPECT_NE(t1, t2);
    EXPECT_EQ(t2, t3);
    EXPECT_NE(t2, t4);
    EXPECT_NE(t4, t5);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNmeaTest, NmeaDate)
{
    const NmeaDate d1 = { true, 2024, 10, 31 };
    EXPECT_TRUE(d1.valid);
    EXPECT_EQ(d1.years, 2024);
    EXPECT_EQ(d1.months, 10);
    EXPECT_EQ(d1.days, 31);
    const NmeaDate d1copy = { true, 2024, 10, 31 };
    const NmeaDate d2 = { true, 2024, 10, 30 };
    const NmeaDate d3 = { false, 2024, 10, 30 };
    EXPECT_EQ(d1, d1copy);
    EXPECT_NE(d1, d2);
    EXPECT_FALSE(d3.valid);
    EXPECT_NE(d2, d3);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNmeaTest, NmeaGgaPayload)
{
    {
        NmeaGgaPayload gga;
        const char* not_a_gga = "$GNGLL,4724.018931,N,00827.023090,E,110546.800,A,D*4F\r\n";
        EXPECT_FALSE(gga.SetFromMsg((const uint8_t*)not_a_gga, std::strlen(not_a_gga)));
        EXPECT_FALSE(gga.valid_);
        EXPECT_EQ(gga.talker_, NmeaTalkerId::UNSPECIFIED);  // we should not even get the talker
        EXPECT_EQ(gga.formatter_, NmeaFormatter::GGA);
    }
    {
        NmeaGgaPayload gga;
        const char* bad_gga = "$GNGGA,092207.400,4724.017956,N,00827.022383*2E\r\n";
        EXPECT_FALSE(gga.SetFromMsg((const uint8_t*)bad_gga, std::strlen(bad_gga)));
        EXPECT_FALSE(gga.valid_);
        EXPECT_EQ(gga.talker_, NmeaTalkerId::GNSS);  // we should still get the talker
        EXPECT_EQ(gga.formatter_, NmeaFormatter::GGA);
    }
    {
        NmeaGgaPayload gga;
        const char* good_gga = "$GNGGA,092207.400,4724.017956,N,00827.022383,E,2,41,0.43,411.542,M,47.989,M,,*79\r\n";
        EXPECT_TRUE(gga.SetFromMsg((const uint8_t*)good_gga, std::strlen(good_gga)));
        EXPECT_TRUE(gga.valid_);
        EXPECT_EQ(gga.talker_, NmeaTalkerId::GNSS);
        EXPECT_EQ(gga.formatter_, NmeaFormatter::GGA);
        EXPECT_TRUE(gga.time.valid);
        EXPECT_EQ(gga.time.hours, 9);
        EXPECT_EQ(gga.time.mins, 22);
        EXPECT_NEAR(gga.time.secs, 7.4, 1e-9);
        EXPECT_TRUE(gga.llh.latlon_valid);
        EXPECT_TRUE(gga.llh.height_valid);
        EXPECT_NEAR(gga.llh.lat, 47.0 + (24.017956 / 60.0), 1e-12);
        EXPECT_NEAR(gga.llh.lon, 8.0 + (27.022383 / 60.0), 1e-12);
        EXPECT_NEAR(gga.llh.height, 411.542 + 47.989, 1e-9);
        EXPECT_TRUE(gga.height_msl.valid);
        EXPECT_NEAR(gga.height_msl.value, 411.542, 1e-9);
        EXPECT_EQ(gga.quality, NmeaQualityGga::DGNSS);
        EXPECT_TRUE(gga.num_sv.valid);
        EXPECT_EQ(gga.num_sv.value, 41);
        EXPECT_TRUE(gga.hdop.valid);
        EXPECT_NEAR(gga.hdop.value, 0.43, 1e-6);
        EXPECT_FALSE(gga.diff_sta.valid);
        EXPECT_EQ(gga.diff_sta.value, 0);
        EXPECT_FALSE(gga.diff_age.valid);
        EXPECT_EQ(gga.diff_age.value, 0.0);
    }
    {
        NmeaGgaPayload gga;
        const char* rtk_gga =
            "$GNGGA,104022.800,4724.017906,N,00827.021943,E,4,43,0.46,411.424,M,47.990,M,0.8,0000*50\r\n";
        EXPECT_TRUE(gga.SetFromMsg((const uint8_t*)rtk_gga, std::strlen(rtk_gga)));
        EXPECT_TRUE(gga.valid_);
        EXPECT_EQ(gga.talker_, NmeaTalkerId::GNSS);
        EXPECT_EQ(gga.formatter_, NmeaFormatter::GGA);
        EXPECT_TRUE(gga.time.valid);
        EXPECT_EQ(gga.time.hours, 10);
        EXPECT_EQ(gga.time.mins, 40);
        EXPECT_NEAR(gga.time.secs, 22.8, 1e-9);
        EXPECT_TRUE(gga.llh.latlon_valid);
        EXPECT_TRUE(gga.llh.height_valid);
        EXPECT_NEAR(gga.llh.lat, 47.0 + (24.017906 / 60.0), 1e-12);
        EXPECT_NEAR(gga.llh.lon, 8.0 + (27.021943 / 60.0), 1e-12);
        EXPECT_NEAR(gga.llh.height, 411.424 + 47.990, 1e-9);
        EXPECT_TRUE(gga.height_msl.valid);
        EXPECT_NEAR(gga.height_msl.value, 411.424, 1e-9);
        EXPECT_EQ(gga.quality, NmeaQualityGga::RTK_FIXED);
        EXPECT_TRUE(gga.num_sv.valid);
        EXPECT_EQ(gga.num_sv.value, 43);
        EXPECT_TRUE(gga.hdop.valid);
        EXPECT_NEAR(gga.hdop.value, 0.46, 1e-6);
        EXPECT_TRUE(gga.diff_sta.valid);
        EXPECT_EQ(gga.diff_sta.value, 0);
        EXPECT_TRUE(gga.diff_age.valid);
        EXPECT_NEAR(gga.diff_age.value, 0.8, 1e-6);
    }
    {
        NmeaGgaPayload gga;
        const char* good_gga = "$GNGGA,235943.812,,,,,0,00,99.99,,M,,M,,*49\r\n";
        EXPECT_TRUE(gga.SetFromMsg((const uint8_t*)good_gga, std::strlen(good_gga)));
        EXPECT_TRUE(gga.valid_);
        EXPECT_EQ(gga.talker_, NmeaTalkerId::GNSS);
        EXPECT_EQ(gga.formatter_, NmeaFormatter::GGA);
        EXPECT_TRUE(gga.time.valid);
        EXPECT_EQ(gga.time.hours, 23);
        EXPECT_EQ(gga.time.mins, 59);
        EXPECT_NEAR(gga.time.secs, 43.812, 1e-9);
        EXPECT_FALSE(gga.llh.latlon_valid);
        EXPECT_EQ(gga.llh.lat, 0.0);
        EXPECT_EQ(gga.llh.lon, 0.0);
        EXPECT_FALSE(gga.llh.height_valid);
        EXPECT_EQ(gga.llh.height, 0.0);
        EXPECT_FALSE(gga.height_msl.valid);
        EXPECT_EQ(gga.height_msl.value, 0.0);
        EXPECT_EQ(gga.quality, NmeaQualityGga::NOFIX);
        EXPECT_TRUE(gga.num_sv.valid);
        EXPECT_EQ(gga.num_sv.value, 0);
        EXPECT_FALSE(gga.hdop.valid);
        EXPECT_EQ(gga.hdop.value, 0.0);
        EXPECT_FALSE(gga.diff_sta.valid);
        EXPECT_EQ(gga.diff_sta.value, 0);
        EXPECT_FALSE(gga.diff_age.valid);
        EXPECT_EQ(gga.diff_age.value, 0.0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNmeaTest, NmeaGllPayload)
{
    {
        NmeaGllPayload gll;
        const char* good_gll = "$GNGLL,4724.018931,N,00827.023090,E,110546.800,A,D*4F\r\n";
        EXPECT_TRUE(gll.SetFromMsg((const uint8_t*)good_gll, std::strlen(good_gll)));
        EXPECT_TRUE(gll.valid_);
        EXPECT_EQ(gll.talker_, NmeaTalkerId::GNSS);
        EXPECT_EQ(gll.formatter_, NmeaFormatter::GLL);
        EXPECT_TRUE(gll.time.valid);
        EXPECT_EQ(gll.time.hours, 11);
        EXPECT_EQ(gll.time.mins, 5);
        EXPECT_NEAR(gll.time.secs, 46.8, 1e-9);
        EXPECT_TRUE(gll.ll.latlon_valid);
        EXPECT_NEAR(gll.ll.lat, 47.0 + (24.018931 / 60.0), 1e-12);
        EXPECT_NEAR(gll.ll.lon, 8.0 + (27.023090 / 60.0), 1e-12);
        EXPECT_FALSE(gll.ll.height_valid);
        EXPECT_NEAR(gll.ll.height, 0.0, 1e-9);
        EXPECT_EQ(gll.status, NmeaStatusGllRmc::VALID);
        EXPECT_EQ(gll.mode, NmeaModeGllVtg::DGNSS);
    }
    {
        NmeaGllPayload gll;
        const char* gll_coldstart = "$GNGLL,,,,,235943.612,V,N*6B\r\n";
        EXPECT_TRUE(gll.SetFromMsg((const uint8_t*)gll_coldstart, std::strlen(gll_coldstart)));
        EXPECT_TRUE(gll.valid_);
        EXPECT_EQ(gll.talker_, NmeaTalkerId::GNSS);
        EXPECT_EQ(gll.formatter_, NmeaFormatter::GLL);
        EXPECT_TRUE(gll.time.valid);
        EXPECT_EQ(gll.time.hours, 23);
        EXPECT_EQ(gll.time.mins, 59);
        EXPECT_NEAR(gll.time.secs, 43.612, 1e-9);
        EXPECT_FALSE(gll.ll.latlon_valid);
        EXPECT_EQ(gll.ll.lat, 0.0);
        EXPECT_EQ(gll.ll.lon, 0.0);
        EXPECT_FALSE(gll.ll.height_valid);
        EXPECT_EQ(gll.ll.height, 0.0);
        EXPECT_EQ(gll.status, NmeaStatusGllRmc::INVALID);
        EXPECT_EQ(gll.mode, NmeaModeGllVtg::INVALID);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNmeaTest, NmeaRmcPayload)
{
    {
        NmeaRmcPayload rmc;
        const char* good_rmc = "$GNRMC,110546.800,A,4724.018931,N,00827.023090,E,0.015,139.17,231024,,,D,V*3D\r\n";
        EXPECT_TRUE(rmc.SetFromMsg((const uint8_t*)good_rmc, std::strlen(good_rmc)));
        EXPECT_TRUE(rmc.valid_);
        EXPECT_EQ(rmc.talker_, NmeaTalkerId::GNSS);
        EXPECT_EQ(rmc.formatter_, NmeaFormatter::RMC);
        EXPECT_TRUE(rmc.time.valid);
        EXPECT_EQ(rmc.time.hours, 11);
        EXPECT_EQ(rmc.time.mins, 5);
        EXPECT_NEAR(rmc.time.secs, 46.8, 1e-9);
        EXPECT_EQ(rmc.status, NmeaStatusGllRmc::VALID);
        EXPECT_TRUE(rmc.ll.latlon_valid);
        EXPECT_NEAR(rmc.ll.lat, 47.0 + (24.018931 / 60.0), 1e-12);
        EXPECT_NEAR(rmc.ll.lon, 8.0 + (27.023090 / 60.0), 1e-12);
        EXPECT_FALSE(rmc.ll.height_valid);  // no height in RMC
        EXPECT_NEAR(rmc.ll.height, 0.0, 1e-9);
        EXPECT_TRUE(rmc.speed.valid);
        EXPECT_NEAR(rmc.speed.value, 0.015, 1e-6);
        EXPECT_TRUE(rmc.course.valid);
        EXPECT_NEAR(rmc.course.value, 139.17, 1e-3);
        EXPECT_TRUE(rmc.date.valid);
        EXPECT_EQ(rmc.date.years, 2024);
        EXPECT_EQ(rmc.date.months, 10);
        EXPECT_EQ(rmc.date.days, 23);
        EXPECT_EQ(rmc.mode, NmeaModeRmcGns::DGNSS);
        EXPECT_EQ(rmc.navstatus, NmeaNavStatusRmc::NA);
    }
    {
        NmeaRmcPayload rmc;
        const char* rmc_coldstart = "$GNRMC,235943.412,V,,,,,,,050180,,,N,V*28\r\n";
        EXPECT_TRUE(rmc.SetFromMsg((const uint8_t*)rmc_coldstart, std::strlen(rmc_coldstart)));
        EXPECT_TRUE(rmc.valid_);
        EXPECT_EQ(rmc.talker_, NmeaTalkerId::GNSS);
        EXPECT_EQ(rmc.formatter_, NmeaFormatter::RMC);
        EXPECT_TRUE(rmc.time.valid);
        EXPECT_EQ(rmc.time.hours, 23);
        EXPECT_EQ(rmc.time.mins, 59);
        EXPECT_NEAR(rmc.time.secs, 43.412, 1e-9);
        EXPECT_EQ(rmc.status, NmeaStatusGllRmc::INVALID);
        EXPECT_FALSE(rmc.ll.latlon_valid);
        EXPECT_EQ(rmc.ll.lat, 0.0);
        EXPECT_EQ(rmc.ll.lon, 0.0);
        EXPECT_FALSE(rmc.ll.height_valid);
        EXPECT_EQ(rmc.ll.height, 0.0);
        EXPECT_FALSE(rmc.speed.valid);
        EXPECT_EQ(rmc.speed.value, 0.0);
        EXPECT_FALSE(rmc.course.valid);
        EXPECT_EQ(rmc.course.value, 0.0);
        EXPECT_FALSE(rmc.date.valid);
        EXPECT_EQ(rmc.date.years, 1980);
        EXPECT_EQ(rmc.date.months, 1);
        EXPECT_EQ(rmc.date.days, 5);
        EXPECT_EQ(rmc.mode, NmeaModeRmcGns::INVALID);
        EXPECT_EQ(rmc.navstatus, NmeaNavStatusRmc::NA);
    }
    {
        NmeaRmcPayload rmc;
        const char* old_rmc = "$GPRMC,094821.1999,A,4724.0179049,N,00827.0219431,E,0.00151,250.7782,080125,,,R*73\r\n";
        EXPECT_TRUE(rmc.SetFromMsg((const uint8_t*)old_rmc, std::strlen(old_rmc)));
        EXPECT_TRUE(rmc.valid_);
        EXPECT_EQ(rmc.talker_, NmeaTalkerId::GPS_SBAS);
        EXPECT_EQ(rmc.formatter_, NmeaFormatter::RMC);
        EXPECT_TRUE(rmc.time.valid);
        EXPECT_EQ(rmc.time.hours, 9);
        EXPECT_EQ(rmc.time.mins, 48);
        EXPECT_NEAR(rmc.time.secs, 21.1999, 1e-9);
        EXPECT_EQ(rmc.status, NmeaStatusGllRmc::VALID);
        EXPECT_TRUE(rmc.ll.latlon_valid);
        EXPECT_NEAR(rmc.ll.lat, 47.0 + (24.017904 / 60.0), 1e-7);
        EXPECT_NEAR(rmc.ll.lon, 8.0 + (27.021943 / 60.0), 1e-7);
        EXPECT_FALSE(rmc.ll.height_valid);  // no height in RMC
        EXPECT_NEAR(rmc.ll.height, 0.0, 1e-9);
        EXPECT_TRUE(rmc.speed.valid);
        EXPECT_NEAR(rmc.speed.value, 0.00151, 1e-6);
        EXPECT_TRUE(rmc.course.valid);
        EXPECT_NEAR(rmc.course.value, 250.7782, 1e-6);
        EXPECT_TRUE(rmc.date.valid);
        EXPECT_EQ(rmc.date.years, 2025);
        EXPECT_EQ(rmc.date.months, 1);
        EXPECT_EQ(rmc.date.days, 8);
        EXPECT_EQ(rmc.mode, NmeaModeRmcGns::RTK_FIXED);
        EXPECT_EQ(rmc.navstatus, NmeaNavStatusRmc::UNSPECIFIED);  // n/a
    }
    {
        NmeaRmcPayload rmc;
        const char* south_west_rmc =
            "$GNRMC,110546.800,A,4724.018931,S,00827.023090,W,0.015,139.17,231024,,,D,V*3D\r\n";
        EXPECT_TRUE(rmc.SetFromMsg((const uint8_t*)south_west_rmc, std::strlen(south_west_rmc)));
        EXPECT_TRUE(rmc.valid_);
        EXPECT_EQ(rmc.talker_, NmeaTalkerId::GNSS);
        EXPECT_EQ(rmc.formatter_, NmeaFormatter::RMC);
        EXPECT_TRUE(rmc.time.valid);
        EXPECT_EQ(rmc.time.hours, 11);
        EXPECT_EQ(rmc.time.mins, 5);
        EXPECT_NEAR(rmc.time.secs, 46.8, 1e-9);
        EXPECT_EQ(rmc.status, NmeaStatusGllRmc::VALID);
        EXPECT_TRUE(rmc.ll.latlon_valid);
        EXPECT_NEAR(rmc.ll.lat, -(47.0 + (24.018931 / 60.0)), 1e-9);
        EXPECT_NEAR(rmc.ll.lon, -(8.0 + (27.023090 / 60.0)), 1e-9);
        EXPECT_FALSE(rmc.ll.height_valid);  // no height in RMC
        EXPECT_NEAR(rmc.ll.height, 0.0, 1e-9);
        EXPECT_TRUE(rmc.speed.valid);
        EXPECT_NEAR(rmc.speed.value, 0.015, 1e-6);
        EXPECT_TRUE(rmc.course.valid);
        EXPECT_NEAR(rmc.course.value, 139.17, 1e-3);
        EXPECT_TRUE(rmc.date.valid);
        EXPECT_EQ(rmc.date.years, 2024);
        EXPECT_EQ(rmc.date.months, 10);
        EXPECT_EQ(rmc.date.days, 23);
        EXPECT_EQ(rmc.mode, NmeaModeRmcGns::DGNSS);
        EXPECT_EQ(rmc.navstatus, NmeaNavStatusRmc::NA);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNmeaTest, NmeaVtgPayload)
{
    {
        NmeaVtgPayload vtg;
        const char* good_vtg = "$GNVTG,139.17,T,,M,0.015,N,0.027,K,D*2A\r\n";
        EXPECT_TRUE(vtg.SetFromMsg((const uint8_t*)good_vtg, std::strlen(good_vtg)));
        EXPECT_TRUE(vtg.valid_);
        EXPECT_EQ(vtg.talker_, NmeaTalkerId::GNSS);
        EXPECT_EQ(vtg.formatter_, NmeaFormatter::VTG);
        EXPECT_TRUE(vtg.cogt.valid);
        EXPECT_NEAR(vtg.cogt.value, 139.17, 1e-4);
        EXPECT_FALSE(vtg.cogm.valid);
        EXPECT_EQ(vtg.cogm.value, 0.0);
        EXPECT_TRUE(vtg.sogn.valid);
        EXPECT_NEAR(vtg.sogn.value, 0.015, 1e-6);
        EXPECT_TRUE(vtg.sogk.valid);
        EXPECT_NEAR(vtg.sogk.value, 0.027, 1e-6);
    }
    {
        NmeaVtgPayload vtg;
        const char* vtg_coldstart = "$GNVTG,,T,,M,,N,,K,N*32\r\n";
        EXPECT_TRUE(vtg.SetFromMsg((const uint8_t*)vtg_coldstart, std::strlen(vtg_coldstart)));
        EXPECT_TRUE(vtg.valid_);
        EXPECT_EQ(vtg.talker_, NmeaTalkerId::GNSS);
        EXPECT_EQ(vtg.formatter_, NmeaFormatter::VTG);
        EXPECT_FALSE(vtg.cogt.valid);
        EXPECT_EQ(vtg.cogt.value, 0.0);
        EXPECT_FALSE(vtg.cogm.valid);
        EXPECT_EQ(vtg.cogm.value, 0.0);
        EXPECT_FALSE(vtg.sogn.valid);
        EXPECT_EQ(vtg.sogn.value, 0.0);
        EXPECT_FALSE(vtg.sogk.valid);
        EXPECT_EQ(vtg.sogk.value, 0.0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNmeaTest, NmeaGstPayload)
{
    {
        NmeaGstPayload gst;
        const char* good_gst = "$GPGST,132419.0000,,0.0515,0.0188,162.6609,0.0495,0.0236,0.0182*7D\r\n";
        EXPECT_TRUE(gst.SetFromMsg((const uint8_t*)good_gst, std::strlen(good_gst)));
        EXPECT_TRUE(gst.valid_);
        EXPECT_EQ(gst.talker_, NmeaTalkerId::GPS_SBAS);
        EXPECT_EQ(gst.formatter_, NmeaFormatter::GST);
        EXPECT_TRUE(gst.time.valid);
        EXPECT_EQ(gst.time.hours, 13);
        EXPECT_EQ(gst.time.mins, 24);
        EXPECT_NEAR(gst.time.secs, 19.0, 1e-9);
        EXPECT_FALSE(gst.rms_range.valid);
        EXPECT_EQ(gst.rms_range.value, 0.0);
        EXPECT_TRUE(gst.std_major.valid);
        EXPECT_NEAR(gst.std_major.value, 0.0515, 1e-9);
        EXPECT_TRUE(gst.std_minor.valid);
        EXPECT_NEAR(gst.std_minor.value, 0.0188, 1e-9);
        EXPECT_TRUE(gst.angle_major.valid);
        EXPECT_NEAR(gst.angle_major.value, 162.6609, 1e-9);
        EXPECT_TRUE(gst.std_lat.valid);
        EXPECT_NEAR(gst.std_lat.value, 0.0495, 1e-9);
        EXPECT_TRUE(gst.std_lon.valid);
        EXPECT_NEAR(gst.std_lon.value, 0.0236, 1e-9);
        EXPECT_TRUE(gst.std_alt.valid);
        EXPECT_NEAR(gst.std_alt.value, 0.0182, 1e-9);
    }
    {
        NmeaGstPayload gst;
        const char* empty_gst = "$GPGST,,,,,,,,*XX\r\n";
        EXPECT_TRUE(gst.SetFromMsg((const uint8_t*)empty_gst, std::strlen(empty_gst)));
        EXPECT_TRUE(gst.valid_);
        EXPECT_EQ(gst.talker_, NmeaTalkerId::GPS_SBAS);
        EXPECT_EQ(gst.formatter_, NmeaFormatter::GST);
        EXPECT_FALSE(gst.time.valid);
        EXPECT_EQ(gst.time.hours, 0);
        EXPECT_EQ(gst.time.mins, 0);
        EXPECT_EQ(gst.time.secs, 0.0);
        EXPECT_FALSE(gst.rms_range.valid);
        EXPECT_EQ(gst.rms_range.value, 0.0);
        EXPECT_FALSE(gst.std_major.valid);
        EXPECT_EQ(gst.std_major.value, 0.0);
        EXPECT_FALSE(gst.std_minor.valid);
        EXPECT_EQ(gst.std_minor.value, 0.0);
        EXPECT_FALSE(gst.angle_major.valid);
        EXPECT_EQ(gst.angle_major.value, 0.0);
        EXPECT_FALSE(gst.std_lat.valid);
        EXPECT_EQ(gst.std_lat.value, 0.0);
        EXPECT_FALSE(gst.std_lon.valid);
        EXPECT_EQ(gst.std_lon.value, 0.0);
        EXPECT_FALSE(gst.std_alt.valid);
        EXPECT_EQ(gst.std_alt.value, 0.0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNmeaTest, NmeaHdtPayload)
{
    {
        NmeaHdtPayload hdt;
        const char* good_hdt = "$GPHDT,61.7183,T*3F\r\n";
        EXPECT_TRUE(hdt.SetFromMsg((const uint8_t*)good_hdt, std::strlen(good_hdt)));
        EXPECT_TRUE(hdt.valid_);
        EXPECT_EQ(hdt.talker_, NmeaTalkerId::GPS_SBAS);
        EXPECT_EQ(hdt.formatter_, NmeaFormatter::HDT);
        EXPECT_TRUE(hdt.heading.valid);
        EXPECT_NEAR(hdt.heading.value, 61.7183, 1e-9);
    }
    {
        NmeaHdtPayload hdt;
        const char* empty_hdt = "$GPHDT,,*3F\r\n";
        EXPECT_TRUE(hdt.SetFromMsg((const uint8_t*)empty_hdt, std::strlen(empty_hdt)));
        EXPECT_TRUE(hdt.valid_);
        EXPECT_EQ(hdt.talker_, NmeaTalkerId::GPS_SBAS);
        EXPECT_EQ(hdt.formatter_, NmeaFormatter::HDT);
        EXPECT_FALSE(hdt.heading.valid);
        EXPECT_EQ(hdt.heading.value, 0.0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNmeaTest, NmeaZdaPayload)
{
    {
        NmeaZdaPayload zda;
        const char* good_zda = "$GPZDA,090411.0001,10,10,2023,00,00*69\r\n";
        EXPECT_TRUE(zda.SetFromMsg((const uint8_t*)good_zda, std::strlen(good_zda)));
        EXPECT_TRUE(zda.valid_);
        EXPECT_EQ(zda.talker_, NmeaTalkerId::GPS_SBAS);
        EXPECT_EQ(zda.formatter_, NmeaFormatter::ZDA);
        EXPECT_TRUE(zda.time.valid);
        EXPECT_EQ(zda.time.hours, 9);
        EXPECT_EQ(zda.time.mins, 4);
        EXPECT_NEAR(zda.time.secs, 11.0001, 1e-9);
        EXPECT_TRUE(zda.date.valid);
        EXPECT_EQ(zda.date.years, 2023);
        EXPECT_EQ(zda.date.months, 10);
        EXPECT_EQ(zda.date.days, 10);
        EXPECT_TRUE(zda.local_hr.valid);
        EXPECT_EQ(zda.local_hr.value, 0);
        EXPECT_TRUE(zda.local_min.valid);
        EXPECT_EQ(zda.local_min.value, 0);
    }
    {
        NmeaZdaPayload zda;
        const char* empty_zda = "$GPZDA,,,,,,*XX\r\n";
        EXPECT_TRUE(zda.SetFromMsg((const uint8_t*)empty_zda, std::strlen(empty_zda)));
        EXPECT_TRUE(zda.valid_);
        EXPECT_EQ(zda.talker_, NmeaTalkerId::GPS_SBAS);
        EXPECT_EQ(zda.formatter_, NmeaFormatter::ZDA);
        EXPECT_FALSE(zda.time.valid);
        EXPECT_EQ(zda.time.hours, 0);
        EXPECT_EQ(zda.time.mins, 0);
        EXPECT_EQ(zda.time.secs, 0.0);
        EXPECT_FALSE(zda.date.valid);
        EXPECT_EQ(zda.date.years, 0);
        EXPECT_EQ(zda.date.months, 0);
        EXPECT_EQ(zda.date.days, 0);
        EXPECT_FALSE(zda.local_hr.valid);
        EXPECT_EQ(zda.local_hr.value, 0);
        EXPECT_FALSE(zda.local_min.valid);
        EXPECT_EQ(zda.local_min.value, 0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNmeaTest, NmeaGsaPayload)
{
    {
        NmeaGsaPayload gsa;
        const char* good_gsa = "$GNGSA,A,3,03,04,06,07,09,11,19,20,26,30,31,,0.79,0.46,0.64,1*0F\r\n";
        EXPECT_TRUE(gsa.SetFromMsg((const uint8_t*)good_gsa, std::strlen(good_gsa)));
        EXPECT_TRUE(gsa.valid_);
        EXPECT_EQ(gsa.talker_, NmeaTalkerId::GNSS);
        EXPECT_EQ(gsa.formatter_, NmeaFormatter::GSA);
        EXPECT_EQ(gsa.opmode, NmeaOpModeGsa::AUTO);
        EXPECT_EQ(gsa.navmode, NmeaNavModeGsa::FIX3D);
        EXPECT_EQ(gsa.num_sats, 11);
        EXPECT_TRUE(gsa.sats[0].valid);
        EXPECT_EQ(gsa.sats[0].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsa.sats[0].svid, 3);
        EXPECT_TRUE(gsa.sats[1].valid);
        EXPECT_EQ(gsa.sats[1].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsa.sats[1].svid, 4);
        EXPECT_TRUE(gsa.sats[2].valid);
        EXPECT_EQ(gsa.sats[2].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsa.sats[2].svid, 6);
        EXPECT_TRUE(gsa.sats[3].valid);
        EXPECT_EQ(gsa.sats[3].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsa.sats[3].svid, 7);
        EXPECT_TRUE(gsa.sats[4].valid);
        EXPECT_EQ(gsa.sats[4].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsa.sats[4].svid, 9);
        EXPECT_TRUE(gsa.sats[5].valid);
        EXPECT_EQ(gsa.sats[5].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsa.sats[5].svid, 11);
        EXPECT_TRUE(gsa.sats[6].valid);
        EXPECT_EQ(gsa.sats[6].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsa.sats[6].svid, 19);
        EXPECT_TRUE(gsa.sats[7].valid);
        EXPECT_EQ(gsa.sats[7].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsa.sats[7].svid, 20);
        EXPECT_TRUE(gsa.sats[8].valid);
        EXPECT_EQ(gsa.sats[8].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsa.sats[8].svid, 26);
        EXPECT_TRUE(gsa.sats[9].valid);
        EXPECT_EQ(gsa.sats[9].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsa.sats[9].svid, 30);
        EXPECT_TRUE(gsa.sats[10].valid);
        EXPECT_EQ(gsa.sats[10].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsa.sats[10].svid, 31);
        EXPECT_FALSE(gsa.sats[11].valid);
        EXPECT_EQ(gsa.sats[11].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsa.sats[11].svid, 0);
        EXPECT_TRUE(gsa.pdop.valid);
        EXPECT_NEAR(gsa.pdop.value, 0.79, 1e-4);
        EXPECT_TRUE(gsa.hdop.valid);
        EXPECT_NEAR(gsa.hdop.value, 0.46, 1e-4);
        EXPECT_TRUE(gsa.vdop.valid);
        EXPECT_NEAR(gsa.vdop.value, 0.64, 1e-4);
        EXPECT_EQ(gsa.system, NmeaSystemId::GPS_SBAS);
    }
    {
        NmeaGsaPayload gsa;
        const char* gsa_coldstart = "$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,5*37\r\n";
        EXPECT_TRUE(gsa.SetFromMsg((const uint8_t*)gsa_coldstart, std::strlen(gsa_coldstart)));
        EXPECT_TRUE(gsa.valid_);
        EXPECT_EQ(gsa.talker_, NmeaTalkerId::GNSS);
        EXPECT_EQ(gsa.formatter_, NmeaFormatter::GSA);
        EXPECT_EQ(gsa.opmode, NmeaOpModeGsa::AUTO);
        EXPECT_EQ(gsa.navmode, NmeaNavModeGsa::NOFIX);
        EXPECT_EQ(gsa.num_sats, 0);
        EXPECT_FALSE(gsa.sats[0].valid);
        EXPECT_EQ(gsa.sats[0].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsa.sats[0].svid, 0);
        EXPECT_FALSE(gsa.sats[1].valid);
        EXPECT_EQ(gsa.sats[1].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsa.sats[1].svid, 0);
        EXPECT_FALSE(gsa.sats[2].valid);
        EXPECT_EQ(gsa.sats[2].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsa.sats[2].svid, 0);
        EXPECT_FALSE(gsa.sats[3].valid);
        EXPECT_EQ(gsa.sats[3].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsa.sats[3].svid, 0);
        EXPECT_FALSE(gsa.sats[4].valid);
        EXPECT_EQ(gsa.sats[4].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsa.sats[4].svid, 0);
        EXPECT_FALSE(gsa.sats[5].valid);
        EXPECT_EQ(gsa.sats[5].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsa.sats[5].svid, 0);
        EXPECT_FALSE(gsa.sats[6].valid);
        EXPECT_EQ(gsa.sats[6].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsa.sats[6].svid, 0);
        EXPECT_FALSE(gsa.sats[7].valid);
        EXPECT_EQ(gsa.sats[7].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsa.sats[7].svid, 0);
        EXPECT_FALSE(gsa.sats[8].valid);
        EXPECT_EQ(gsa.sats[8].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsa.sats[8].svid, 0);
        EXPECT_FALSE(gsa.sats[9].valid);
        EXPECT_EQ(gsa.sats[9].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsa.sats[9].svid, 0);
        EXPECT_FALSE(gsa.sats[10].valid);
        EXPECT_EQ(gsa.sats[10].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsa.sats[10].svid, 0);
        EXPECT_FALSE(gsa.sats[11].valid);
        EXPECT_EQ(gsa.sats[11].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsa.sats[11].svid, 0);
        EXPECT_FALSE(gsa.pdop.valid);
        EXPECT_EQ(gsa.pdop.value, 0.0);
        EXPECT_FALSE(gsa.hdop.valid);
        EXPECT_EQ(gsa.hdop.value, 0.0);
        EXPECT_FALSE(gsa.vdop.valid);
        EXPECT_EQ(gsa.vdop.value, 0.0);
        EXPECT_EQ(gsa.system, NmeaSystemId::QZSS);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNmeaTest, NmeaGsvPayload)
{
    {
        NmeaGsvPayload gsv;
        const char* gsv_2_of_4 =  // clang-format off
            "$GPGSV,4,2,14,"  "09,84,270,52,"  "11,,,46,"  "16,03,080,,"  "17,03,218,35,"  "1*XX\r\n";  // clang-format on
        EXPECT_TRUE(gsv.SetFromMsg((const uint8_t*)gsv_2_of_4, std::strlen(gsv_2_of_4)));
        EXPECT_TRUE(gsv.valid_);
        EXPECT_EQ(gsv.talker_, NmeaTalkerId::GPS_SBAS);
        EXPECT_EQ(gsv.formatter_, NmeaFormatter::GSV);
        EXPECT_TRUE(gsv.num_msgs.valid);
        EXPECT_EQ(gsv.num_msgs.value, 4);
        EXPECT_TRUE(gsv.msg_num.valid);
        EXPECT_EQ(gsv.msg_num.value, 2);
        EXPECT_TRUE(gsv.tot_num_sat.valid);
        EXPECT_EQ(gsv.tot_num_sat.value, 14);
        EXPECT_TRUE(gsv.azels[0].valid);
        EXPECT_EQ(gsv.num_azels, 3);
        EXPECT_EQ(gsv.azels[0].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsv.azels[0].svid, 9);
        EXPECT_EQ(gsv.azels[0].el, 84);
        EXPECT_EQ(gsv.azels[0].az, 270);
        EXPECT_TRUE(gsv.azels[1].valid);
        EXPECT_EQ(gsv.azels[1].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsv.azels[1].svid, 16);
        EXPECT_EQ(gsv.azels[1].el, 3);
        EXPECT_EQ(gsv.azels[1].az, 80);
        EXPECT_TRUE(gsv.azels[2].valid);
        EXPECT_EQ(gsv.azels[2].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsv.azels[2].svid, 17);
        EXPECT_EQ(gsv.azels[2].el, 3);
        EXPECT_EQ(gsv.azels[2].az, 218);
        EXPECT_FALSE(gsv.azels[3].valid);
        EXPECT_EQ(gsv.azels[3].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.azels[3].svid, 0);
        EXPECT_EQ(gsv.azels[3].el, 0);
        EXPECT_EQ(gsv.azels[3].az, 0);
        EXPECT_EQ(gsv.num_cnos, 3);
        EXPECT_TRUE(gsv.cnos[0].valid);
        EXPECT_EQ(gsv.cnos[0].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsv.cnos[0].svid, 9);
        EXPECT_EQ(gsv.cnos[0].signal, NmeaSignalId::GPS_L1CA);
        EXPECT_EQ(gsv.cnos[0].cno, 52);
        EXPECT_TRUE(gsv.cnos[1].valid);
        EXPECT_EQ(gsv.cnos[1].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsv.cnos[1].svid, 11);
        EXPECT_EQ(gsv.cnos[1].signal, NmeaSignalId::GPS_L1CA);
        EXPECT_EQ(gsv.cnos[1].cno, 46);
        EXPECT_TRUE(gsv.cnos[2].valid);
        EXPECT_EQ(gsv.cnos[2].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsv.cnos[2].svid, 17);
        EXPECT_EQ(gsv.cnos[2].signal, NmeaSignalId::GPS_L1CA);
        EXPECT_EQ(gsv.cnos[2].cno, 35);
        EXPECT_FALSE(gsv.cnos[3].valid);
        EXPECT_EQ(gsv.cnos[3].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[3].svid, 0);
        EXPECT_EQ(gsv.cnos[3].signal, NmeaSignalId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[3].cno, 0);
        EXPECT_EQ(gsv.system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsv.signal, NmeaSignalId::GPS_L1CA);
    }
    {
        NmeaGsvPayload gsv;  // clang-format off
        const char* gsv_4_of_4 = "$GPGSV,4,4,14,"  "31,08,034,46,"  "36,31,149,43,"  "1*62\r\n";  // clang-format on
        EXPECT_TRUE(gsv.SetFromMsg((const uint8_t*)gsv_4_of_4, std::strlen(gsv_4_of_4)));
        EXPECT_TRUE(gsv.valid_);
        EXPECT_EQ(gsv.talker_, NmeaTalkerId::GPS_SBAS);
        EXPECT_EQ(gsv.formatter_, NmeaFormatter::GSV);
        EXPECT_TRUE(gsv.num_msgs.valid);
        EXPECT_EQ(gsv.num_msgs.value, 4);
        EXPECT_TRUE(gsv.msg_num.valid);
        EXPECT_EQ(gsv.msg_num.value, 4);
        EXPECT_TRUE(gsv.tot_num_sat.valid);
        EXPECT_EQ(gsv.tot_num_sat.value, 14);
        EXPECT_EQ(gsv.num_azels, 2);
        EXPECT_TRUE(gsv.azels[0].valid);
        EXPECT_EQ(gsv.azels[0].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsv.azels[0].svid, 31);
        EXPECT_EQ(gsv.azels[0].el, 8);
        EXPECT_EQ(gsv.azels[0].az, 34);
        EXPECT_TRUE(gsv.azels[1].valid);
        EXPECT_EQ(gsv.azels[1].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsv.azels[1].svid, 36);
        EXPECT_EQ(gsv.azels[1].el, 31);
        EXPECT_EQ(gsv.azels[1].az, 149);
        EXPECT_FALSE(gsv.azels[2].valid);
        EXPECT_EQ(gsv.azels[2].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.azels[2].svid, 0);
        EXPECT_EQ(gsv.azels[2].el, 0);
        EXPECT_EQ(gsv.azels[2].az, 0);
        EXPECT_FALSE(gsv.azels[3].valid);
        EXPECT_EQ(gsv.azels[3].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.azels[3].svid, 0);
        EXPECT_EQ(gsv.azels[3].el, 0);
        EXPECT_EQ(gsv.azels[3].az, 0);
        EXPECT_EQ(gsv.num_cnos, 2);
        EXPECT_TRUE(gsv.cnos[0].valid);
        EXPECT_EQ(gsv.cnos[0].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsv.cnos[0].svid, 31);
        EXPECT_EQ(gsv.cnos[0].signal, NmeaSignalId::GPS_L1CA);
        EXPECT_EQ(gsv.cnos[0].cno, 46);
        EXPECT_TRUE(gsv.cnos[1].valid);
        EXPECT_EQ(gsv.cnos[1].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsv.cnos[1].svid, 36);
        EXPECT_EQ(gsv.cnos[1].signal, NmeaSignalId::GPS_L1CA);
        EXPECT_EQ(gsv.cnos[1].cno, 43);
        EXPECT_FALSE(gsv.cnos[2].valid);
        EXPECT_EQ(gsv.cnos[2].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[2].svid, 0);
        EXPECT_EQ(gsv.cnos[2].signal, NmeaSignalId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[2].cno, 0);
        EXPECT_FALSE(gsv.cnos[3].valid);
        EXPECT_EQ(gsv.cnos[3].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[3].svid, 0);
        EXPECT_EQ(gsv.cnos[3].signal, NmeaSignalId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[3].cno, 0);
    }
    {
        NmeaGsvPayload gsv;
        const char* gsv_1_of_1_nosv = "$GAGSV,1,1,00,1*75\r\n";
        EXPECT_TRUE(gsv.SetFromMsg((const uint8_t*)gsv_1_of_1_nosv, std::strlen(gsv_1_of_1_nosv)));
        EXPECT_TRUE(gsv.valid_);
        EXPECT_EQ(gsv.talker_, NmeaTalkerId::GAL);
        EXPECT_EQ(gsv.formatter_, NmeaFormatter::GSV);
        EXPECT_TRUE(gsv.num_msgs.valid);
        EXPECT_EQ(gsv.num_msgs.value, 1);
        EXPECT_TRUE(gsv.msg_num.valid);
        EXPECT_EQ(gsv.msg_num.value, 1);
        EXPECT_TRUE(gsv.tot_num_sat.valid);
        EXPECT_EQ(gsv.tot_num_sat.value, 0);
        EXPECT_EQ(gsv.num_azels, 0);
        EXPECT_EQ(gsv.num_cnos, 0);
        for (int ix = 0; ix < 4; ix++) {
            EXPECT_FALSE(gsv.azels[0].valid);
            EXPECT_EQ(gsv.azels[ix].system, NmeaSystemId::UNSPECIFIED);
            EXPECT_EQ(gsv.azels[ix].svid, 0);
            EXPECT_EQ(gsv.azels[ix].el, 0);
            EXPECT_EQ(gsv.azels[ix].az, 0);
            EXPECT_EQ(gsv.cnos[ix].system, NmeaSystemId::UNSPECIFIED);
            EXPECT_EQ(gsv.cnos[ix].svid, 0);
            EXPECT_EQ(gsv.cnos[ix].signal, NmeaSignalId::UNSPECIFIED);
            EXPECT_EQ(gsv.cnos[ix].cno, 0);
        }
    }
    {
        NmeaGsvPayload gsv;
        const char* bad_gsv_too_many_sv = "$GLGSV,1,1,12,1*75\r\n";
        EXPECT_FALSE(gsv.SetFromMsg((const uint8_t*)bad_gsv_too_many_sv, std::strlen(bad_gsv_too_many_sv)));
        EXPECT_FALSE(gsv.valid_);
        EXPECT_EQ(gsv.talker_, NmeaTalkerId::GLO);
        EXPECT_EQ(gsv.formatter_, NmeaFormatter::GSV);
        EXPECT_TRUE(gsv.num_msgs.valid);
        EXPECT_EQ(gsv.num_msgs.value, 1);
        EXPECT_TRUE(gsv.msg_num.valid);
        EXPECT_EQ(gsv.msg_num.value, 1);
        EXPECT_TRUE(gsv.tot_num_sat.valid);
        EXPECT_EQ(gsv.tot_num_sat.value, 12);
        EXPECT_EQ(gsv.num_azels, 0);
        EXPECT_EQ(gsv.num_cnos, 0);
        for (int ix = 0; ix < 4; ix++) {
            EXPECT_FALSE(gsv.azels[0].valid);
            EXPECT_EQ(gsv.azels[ix].system, NmeaSystemId::UNSPECIFIED);
            EXPECT_EQ(gsv.azels[ix].svid, 0);
            EXPECT_EQ(gsv.azels[ix].el, 0);
            EXPECT_EQ(gsv.azels[ix].az, 0);
            EXPECT_EQ(gsv.cnos[ix].system, NmeaSystemId::UNSPECIFIED);
            EXPECT_EQ(gsv.cnos[ix].svid, 0);
            EXPECT_EQ(gsv.cnos[ix].signal, NmeaSignalId::UNSPECIFIED);
            EXPECT_EQ(gsv.cnos[ix].cno, 0);
        }
    }
    {
        NmeaGsvPayload gsv;
        const char* no_cno =
            "$GPGSV,1,1,03,"
            "11,02,124,,"
            "23,01,241,,"
            "41,02,101,,"
            "0*53..";  // no cnos -> no signal
        EXPECT_TRUE(gsv.SetFromMsg((const uint8_t*)no_cno, std::strlen(no_cno)));
        EXPECT_TRUE(gsv.valid_);
        EXPECT_EQ(gsv.talker_, NmeaTalkerId::GPS_SBAS);
        EXPECT_EQ(gsv.formatter_, NmeaFormatter::GSV);
        EXPECT_TRUE(gsv.num_msgs.valid);
        EXPECT_EQ(gsv.num_msgs.value, 1);
        EXPECT_TRUE(gsv.msg_num.valid);
        EXPECT_EQ(gsv.msg_num.value, 1);
        EXPECT_TRUE(gsv.tot_num_sat.valid);
        EXPECT_EQ(gsv.tot_num_sat.value, 3);
        EXPECT_EQ(gsv.num_azels, 3);
        EXPECT_TRUE(gsv.azels[0].valid);
        EXPECT_EQ(gsv.azels[0].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsv.azels[0].svid, 11);
        EXPECT_EQ(gsv.azels[0].el, 2);
        EXPECT_EQ(gsv.azels[0].az, 124);
        EXPECT_TRUE(gsv.azels[1].valid);
        EXPECT_EQ(gsv.azels[1].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsv.azels[1].svid, 23);
        EXPECT_EQ(gsv.azels[1].el, 1);
        EXPECT_EQ(gsv.azels[1].az, 241);
        EXPECT_TRUE(gsv.azels[2].valid);
        EXPECT_EQ(gsv.azels[2].system, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(gsv.azels[2].svid, 41);
        EXPECT_EQ(gsv.azels[2].el, 2);
        EXPECT_EQ(gsv.azels[2].az, 101);
        EXPECT_FALSE(gsv.azels[3].valid);
        EXPECT_EQ(gsv.azels[3].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.azels[3].svid, 0);
        EXPECT_EQ(gsv.azels[3].el, 0);
        EXPECT_EQ(gsv.azels[3].az, 0);
        EXPECT_EQ(gsv.num_cnos, 0);
        EXPECT_FALSE(gsv.cnos[0].valid);
        EXPECT_EQ(gsv.cnos[0].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[0].svid, 0);
        EXPECT_EQ(gsv.cnos[0].signal, NmeaSignalId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[0].cno, 0);
        EXPECT_FALSE(gsv.cnos[1].valid);
        EXPECT_EQ(gsv.cnos[1].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[1].svid, 0);
        EXPECT_EQ(gsv.cnos[1].signal, NmeaSignalId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[1].cno, 0);
        EXPECT_FALSE(gsv.cnos[2].valid);
        EXPECT_EQ(gsv.cnos[2].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[2].svid, 0);
        EXPECT_EQ(gsv.cnos[2].signal, NmeaSignalId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[2].cno, 0);
        EXPECT_FALSE(gsv.cnos[3].valid);
        EXPECT_EQ(gsv.cnos[3].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[3].svid, 0);
        EXPECT_EQ(gsv.cnos[3].signal, NmeaSignalId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[3].cno, 0);
    }
    {
        NmeaGsvPayload gsv;
        const char* gsv_2_of_3 =  // clang-format off
            "$GBGSV,3,2,09," "25,65,282,09," "26,06,081,14," "35,17,044,08," "40,14,034,08," "B*00\r\n";  // clang-format on
        EXPECT_TRUE(gsv.SetFromMsg((const uint8_t*)gsv_2_of_3, std::strlen(gsv_2_of_3)));
        EXPECT_TRUE(gsv.valid_);
        EXPECT_EQ(gsv.talker_, NmeaTalkerId::BDS);
        EXPECT_EQ(gsv.formatter_, NmeaFormatter::GSV);
        EXPECT_TRUE(gsv.num_msgs.valid);
        EXPECT_EQ(gsv.num_msgs.value, 3);
        EXPECT_TRUE(gsv.msg_num.valid);
        EXPECT_EQ(gsv.msg_num.value, 2);
        EXPECT_TRUE(gsv.tot_num_sat.valid);
        EXPECT_EQ(gsv.tot_num_sat.value, 9);
        EXPECT_EQ(gsv.num_azels, 4);
        EXPECT_TRUE(gsv.azels[0].valid);
        EXPECT_EQ(gsv.azels[0].system, NmeaSystemId::BDS);
        EXPECT_EQ(gsv.azels[0].svid, 25);
        EXPECT_EQ(gsv.azels[0].el, 65);
        EXPECT_EQ(gsv.azels[0].az, 282);
        EXPECT_TRUE(gsv.azels[1].valid);
        EXPECT_EQ(gsv.azels[1].system, NmeaSystemId::BDS);
        EXPECT_EQ(gsv.azels[1].svid, 26);
        EXPECT_EQ(gsv.azels[1].el, 6);
        EXPECT_EQ(gsv.azels[1].az, 81);
        EXPECT_TRUE(gsv.azels[2].valid);
        EXPECT_EQ(gsv.azels[2].system, NmeaSystemId::BDS);
        EXPECT_EQ(gsv.azels[2].svid, 35);
        EXPECT_EQ(gsv.azels[2].el, 17);
        EXPECT_EQ(gsv.azels[2].az, 44);
        EXPECT_TRUE(gsv.azels[3].valid);
        EXPECT_EQ(gsv.azels[3].system, NmeaSystemId::BDS);
        EXPECT_EQ(gsv.azels[3].svid, 40);
        EXPECT_EQ(gsv.azels[3].el, 14);
        EXPECT_EQ(gsv.azels[3].az, 34);
        EXPECT_EQ(gsv.num_cnos, 4);
        EXPECT_TRUE(gsv.cnos[0].valid);
        EXPECT_EQ(gsv.cnos[0].system, NmeaSystemId::BDS);
        EXPECT_EQ(gsv.cnos[0].svid, 25);
        EXPECT_EQ(gsv.cnos[0].signal, NmeaSignalId::BDS_B2ID);
        EXPECT_EQ(gsv.cnos[0].cno, 9);
        EXPECT_TRUE(gsv.cnos[1].valid);
        EXPECT_EQ(gsv.cnos[1].system, NmeaSystemId::BDS);
        EXPECT_EQ(gsv.cnos[1].svid, 26);
        EXPECT_EQ(gsv.cnos[1].signal, NmeaSignalId::BDS_B2ID);
        EXPECT_EQ(gsv.cnos[1].cno, 14);
        EXPECT_TRUE(gsv.cnos[2].valid);
        EXPECT_EQ(gsv.cnos[2].system, NmeaSystemId::BDS);
        EXPECT_EQ(gsv.cnos[2].svid, 35);
        EXPECT_EQ(gsv.cnos[2].signal, NmeaSignalId::BDS_B2ID);
        EXPECT_EQ(gsv.cnos[2].cno, 8);
        EXPECT_TRUE(gsv.cnos[3].valid);
        EXPECT_EQ(gsv.cnos[3].system, NmeaSystemId::BDS);
        EXPECT_EQ(gsv.cnos[3].svid, 40);
        EXPECT_EQ(gsv.cnos[3].signal, NmeaSignalId::BDS_B2ID);
        EXPECT_EQ(gsv.cnos[3].cno, 8);
    }
    {
        NmeaGsvPayload gsv;  // clang-format off
        const char* gsv_3_of_3 = "$GBGSV,3,3,09," "41,43,264,13," "B*3C\r\n";  // clang-format on
        EXPECT_TRUE(gsv.SetFromMsg((const uint8_t*)gsv_3_of_3, std::strlen(gsv_3_of_3)));
        EXPECT_TRUE(gsv.valid_);
        EXPECT_EQ(gsv.talker_, NmeaTalkerId::BDS);
        EXPECT_EQ(gsv.formatter_, NmeaFormatter::GSV);
        EXPECT_TRUE(gsv.num_msgs.valid);
        EXPECT_EQ(gsv.num_msgs.value, 3);
        EXPECT_TRUE(gsv.msg_num.valid);
        EXPECT_EQ(gsv.msg_num.value, 3);
        EXPECT_TRUE(gsv.tot_num_sat.valid);
        EXPECT_EQ(gsv.tot_num_sat.value, 9);
        EXPECT_EQ(gsv.num_azels, 1);
        EXPECT_TRUE(gsv.azels[0].valid);
        EXPECT_EQ(gsv.azels[0].system, NmeaSystemId::BDS);
        EXPECT_EQ(gsv.azels[0].svid, 41);
        EXPECT_EQ(gsv.azels[0].el, 43);
        EXPECT_EQ(gsv.azels[0].az, 264);
        EXPECT_FALSE(gsv.azels[1].valid);
        EXPECT_EQ(gsv.azels[1].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.azels[1].svid, 0);
        EXPECT_EQ(gsv.azels[1].el, 0);
        EXPECT_EQ(gsv.azels[1].az, 0);
        EXPECT_FALSE(gsv.azels[2].valid);
        EXPECT_EQ(gsv.azels[2].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.azels[2].svid, 0);
        EXPECT_EQ(gsv.azels[2].el, 0);
        EXPECT_EQ(gsv.azels[2].az, 0);
        EXPECT_FALSE(gsv.azels[3].valid);
        EXPECT_EQ(gsv.azels[3].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.azels[3].svid, 0);
        EXPECT_EQ(gsv.azels[3].el, 0);
        EXPECT_EQ(gsv.azels[3].az, 0);
        EXPECT_EQ(gsv.num_cnos, 1);
        EXPECT_TRUE(gsv.cnos[0].valid);
        EXPECT_EQ(gsv.cnos[0].system, NmeaSystemId::BDS);
        EXPECT_EQ(gsv.cnos[0].svid, 41);
        EXPECT_EQ(gsv.cnos[0].signal, NmeaSignalId::BDS_B2ID);
        EXPECT_EQ(gsv.cnos[0].cno, 13);
        EXPECT_FALSE(gsv.cnos[1].valid);
        EXPECT_EQ(gsv.cnos[1].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[1].svid, 0);
        EXPECT_EQ(gsv.cnos[1].signal, NmeaSignalId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[1].cno, 0);
        EXPECT_FALSE(gsv.cnos[2].valid);
        EXPECT_EQ(gsv.cnos[2].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[2].svid, 0);
        EXPECT_EQ(gsv.cnos[2].signal, NmeaSignalId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[2].cno, 0);
        EXPECT_FALSE(gsv.cnos[3].valid);
        EXPECT_EQ(gsv.cnos[3].system, NmeaSystemId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[3].svid, 0);
        EXPECT_EQ(gsv.cnos[3].signal, NmeaSignalId::UNSPECIFIED);
        EXPECT_EQ(gsv.cnos[3].cno, 0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNmeaTest, NmeaCollectGsaGsv)
{
    const char* gsa_msgs[] = {
        // clang-format off
        "$GNGSA,A,3," "03,04,06,07,09,11,19,20,26,30,31,,"    "0.79,0.46,0.64,1*XX\r\n",
        "$GNGSA,A,3," "69,71,78,79,80,84,85,86,,,,,"          "0.79,0.46,0.64,2*XX\r\n",
        "$GNGSA,A,3," "02,07,08,12,19,26,29,33,,,,,"          "0.79,0.46,0.64,3*XX\r\n", // (2)
        "$GNGSA,A,3," "06,09,11,12,14,16,21,22,26,28,33,34,"  "0.79,0.46,0.64,4*XX\r\n",
        "$GNGSA,A,3," ",,,,,,,,,,,,"                          "0.79,0.46,0.64,5*XX\r\n",
    };  // clang-format on
    const char* gsv_msgs[] = {
        // clang-format off
        "$GPGSV,4,1,14,"   "03,30,103,46,"  "04,59,056,48,"  "06,62,267,50,"  "07,31,170,46,"   "1*XX\r\n", // (1)
        "$GPGSV,4,2,14,"   "09,84,270,52,"  "11,28,311,46,"  "16,03,080,42,"  "17,03,218,35,"   "1*XX\r\n",
        "$GPGSV,4,3,14,"   "19,14,239,42,"  "20,12,44,,"     "26,07,049,43,"  "30,05,191,36,"   "1*XX\r\n",
        "$GPGSV,4,4,14,"   "31,,,46,"       "36,31,149,,"                                       "1*XX\r\n",
        "$GPGSV,2,1,07,"   "03,30,103,48,"  "04,59,056,52,"  "06,62,267,52,"  "09,84,270,52,"   "8*XX\r\n",
        "$GPGSV,2,2,07,"   "11,28,311,49,"  "26,07,049,48,"  "30,,,39,"                         "8*XX\r\n",
        "$GLGSV,2,1,08,"   "69,36,055,52,"  "71,17,265,45,"  "78,14,023,45,"  "79,28,076,49,"   "1*XX\r\n",
        "$GLGSV,2,2,08,"   "80,12,129,45,"  "84,18,188,33,"  "85,53,243,52,"  "86,32,321,50,"   "1*XX\r\n",
        "$GAGSV,3,1,09,"   "02,06,041,41,"  "07,56,057,47,"  "08,05,057,40,"  "12,16,320,37,"   "7*XX\r\n",
        "$GAGSV,3,2,09,"   "19,19,237,37,"  "26,47,176,45,"  "29,64,237,49,"  "30,03,089,37,"   "7*XX\r\n", // (2)
        "$GAGSV,3,3,09,"   "33,63,276,,"                                                        "7*XX\r\n",
        "$GAGSV,3,1,09,"   "02,06,041,43,"  "07,56,057,50,"  "08,05,057,43,"  "12,16,320,39,"   "1*XX\r\n",
        "$GAGSV,3,2,09,"   "19,19,237,37,"  "26,47,176,49,"  "29,64,237,,"    "30,03,089,41,"   "1*XX\r\n",
        "$GAGSV,3,3,09,"   "33,63,276,51,"                                                      "1*XX\r\n",
        "$GBGSV,5,1,17,"   "06,23,042,42,"  "09,33,050,43,"  "11,37,231,46,"  "12,06,186,37,"   "1*XX\r\n",
        "$GBGSV,5,2,17,"   "14,35,274,45,"  "16,16,037,40,"  "21,57,066,50,"  "22,08,082,43,"   "1*XX\r\n",
        "$GBGSV,5,3,17,"   "26,18,144,42,"  "28,13,324,44,"  "33,10,269,45,"  "34,21,208,44,"   "1*XX\r\n",
        "$GBGSV,5,4,17,"   "36,12,036,43,"  "39,10,035,,"    "42,62,288,50,"  "43,36,260,48,"   "1*XX\r\n",
        "$GBGSV,5,5,17,"   "45,30,085,47,"                                                      "1*XX\r\n",
        "$GBGSV,3,1,11,"   "21,57,066,51,"  "22,08,082,45,"  "26,18,144,44,"  "28,13,324,46,"   "5*XX\r\n",
        "$GBGSV,3,2,11,"   "33,10,269,45,"  "34,21,208,43,"  "36,12,036,46,"  "39,10,035,44,"   "5*XX\r\n",
        "$GBGSV,3,3,11,"   "42,62,288,52,"  "43,36,260,,"    "45,30,085,,"                      "5*XX\r\n",
        "$GQGSV,1,1,00,"                                                                        "1*XX\r\n",
        "$GQGSV,1,1,00,"                                                                        "8*XX\r\n",
    };  // clang-format on
    {
        NmeaCollectGsaGsv coll;
        for (auto& gsa_msg : gsa_msgs) {
            NmeaGsaPayload gsa;
            // fprintf(stderr, "GSA: %s", gsa_msg);
            EXPECT_TRUE(gsa.SetFromMsg((const uint8_t*)gsa_msg, std::strlen(gsa_msg)));
            EXPECT_TRUE(gsa.valid_);
            EXPECT_TRUE(coll.AddGsa(gsa));
        }
        for (auto& gsv_msg : gsv_msgs) {
            NmeaGsvPayload gsv;
            // fprintf(stderr, "GSV: %s", gsv_msg);
            EXPECT_TRUE(gsv.SetFromMsg((const uint8_t*)gsv_msg, std::strlen(gsv_msg)));
            EXPECT_TRUE(gsv.valid_);
            EXPECT_TRUE(coll.AddGsv(gsv));
        }
        coll.Complete();
        // A few checks...
        EXPECT_EQ(coll.sats_.size(), 47);
        EXPECT_EQ(coll.sigs_.size(), 68);
        // (1) 03,30,103,46
        EXPECT_EQ(coll.sats_[0].system_, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(coll.sats_[0].svid_, 3);
        EXPECT_EQ(coll.sats_[0].az_, 103);
        EXPECT_EQ(coll.sats_[0].el_, 30);
        EXPECT_EQ(coll.sigs_[0].system_, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(coll.sigs_[0].svid_, 3);
        EXPECT_EQ(coll.sigs_[0].cno_, 46);
        EXPECT_TRUE(coll.sigs_[0].used_);
        // (2) 26,47,176,45 (used, in GSA) and 30,03,089,37 (not used, not in GSA)
        EXPECT_EQ(coll.sats_[26].system_, NmeaSystemId::GAL);
        EXPECT_EQ(coll.sats_[26].svid_, 26);
        EXPECT_EQ(coll.sats_[26].az_, 176);
        EXPECT_EQ(coll.sats_[26].el_, 47);
        EXPECT_EQ(coll.sigs_[40].system_, NmeaSystemId::GAL);
        EXPECT_EQ(coll.sigs_[40].svid_, 26);
        EXPECT_EQ(coll.sigs_[40].cno_, 45);
        EXPECT_TRUE(coll.sigs_[40].used_);  // used
        EXPECT_EQ(coll.sats_[28].system_, NmeaSystemId::GAL);
        EXPECT_EQ(coll.sats_[28].svid_, 30);
        EXPECT_EQ(coll.sats_[28].az_, 89);
        EXPECT_EQ(coll.sats_[28].el_, 3);
        EXPECT_EQ(coll.sigs_[42].system_, NmeaSystemId::GAL);
        EXPECT_EQ(coll.sigs_[42].svid_, 30);
        EXPECT_EQ(coll.sigs_[42].cno_, 37);
        EXPECT_FALSE(coll.sigs_[42].used_);  // not used
    }
    {
        NmeaCollectGsaGsv coll;
        std::vector<NmeaGsaPayload> gsas;
        std::vector<NmeaGsvPayload> gsvs;
        for (auto& gsa_msg : gsa_msgs) {
            NmeaGsaPayload gsa;
            EXPECT_TRUE(gsa.SetFromMsg((const uint8_t*)gsa_msg, std::strlen(gsa_msg)));
            EXPECT_TRUE(gsa.valid_);
            gsas.push_back(gsa);
        }
        for (auto& gsv_msg : gsv_msgs) {
            NmeaGsvPayload gsv;
            EXPECT_TRUE(gsv.SetFromMsg((const uint8_t*)gsv_msg, std::strlen(gsv_msg)));
            EXPECT_TRUE(gsv.valid_);
            gsvs.push_back(gsv);
        }
        EXPECT_TRUE(coll.AddGsaAndGsv(gsas, gsvs));

        // A few checks...
        EXPECT_EQ(coll.sats_.size(), 47);
        EXPECT_EQ(coll.sigs_.size(), 68);
        // (1) 03,30,103,46
        EXPECT_EQ(coll.sats_[0].system_, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(coll.sats_[0].svid_, 3);
        EXPECT_EQ(coll.sats_[0].az_, 103);
        EXPECT_EQ(coll.sats_[0].el_, 30);
        EXPECT_EQ(coll.sigs_[0].system_, NmeaSystemId::GPS_SBAS);
        EXPECT_EQ(coll.sigs_[0].svid_, 3);
        EXPECT_EQ(coll.sigs_[0].cno_, 46);
        EXPECT_TRUE(coll.sigs_[0].used_);
        // (2) 26,47,176,45 (used, in GSA) and 30,03,089,37 (not used, not in GSA)
        EXPECT_EQ(coll.sats_[26].system_, NmeaSystemId::GAL);
        EXPECT_EQ(coll.sats_[26].svid_, 26);
        EXPECT_EQ(coll.sats_[26].az_, 176);
        EXPECT_EQ(coll.sats_[26].el_, 47);
        EXPECT_EQ(coll.sigs_[40].system_, NmeaSystemId::GAL);
        EXPECT_EQ(coll.sigs_[40].svid_, 26);
        EXPECT_EQ(coll.sigs_[40].cno_, 45);
        EXPECT_TRUE(coll.sigs_[40].used_);  // used
        EXPECT_EQ(coll.sats_[28].system_, NmeaSystemId::GAL);
        EXPECT_EQ(coll.sats_[28].svid_, 30);
        EXPECT_EQ(coll.sats_[28].az_, 89);
        EXPECT_EQ(coll.sats_[28].el_, 3);
        EXPECT_EQ(coll.sigs_[42].system_, NmeaSystemId::GAL);
        EXPECT_EQ(coll.sigs_[42].svid_, 30);
        EXPECT_EQ(coll.sigs_[42].cno_, 37);
        EXPECT_FALSE(coll.sigs_[42].used_);  // not used
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNmeaTest, NmeaMakeMessage)
{
    std::vector<uint8_t> msg;
    EXPECT_TRUE(NmeaMakeMessage(msg, "GNGGA,235943.812,,,,,0,00,99.99,,M,,M,,"));
    EXPECT_EQ(BufToStr(msg), std::string("$GNGGA,235943.812,,,,,0,00,99.99,,M,,M,,*49\r\n"));

    EXPECT_TRUE(NmeaMakeMessage(msg, "Bad$h!7^*\t\\"));
    EXPECT_EQ(BufToStr(msg), std::string("$Bad_h_7____*18\r\n"));
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserNmeaTest, NmeaDecodeMessage)
{
    {
        const char* msg = "$GNGGA,092207.400,4724.017956,N,00827.022383,E,2,41,0.43,411.542,M,47.989,M,,*79\r\n";
        const NmeaPayloadPtr payload = NmeaDecodeMessage((const uint8_t*)msg, std::strlen(msg));
        EXPECT_TRUE(payload);
        EXPECT_EQ(payload->formatter_, NmeaFormatter::GGA);
        const auto gga = dynamic_cast<const NmeaGgaPayload&>(*payload);

        EXPECT_EQ(gga.num_sv.value, 41);
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
