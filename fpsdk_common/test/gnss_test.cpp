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
 * @brief Fixposition SDK: tests for fpsdk::common::gnss
 */

/* LIBC/STL */
#include <map>
#include <set>
#include <unordered_map>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/gnss.hpp>
#include <fpsdk_common/logging.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::gnss;

TEST(GnssTest, FixTypeStr)
{
    // clang-format off
    EXPECT_EQ(std::string(FixTypeStr(FixType::UNKNOWN)),        std::string("UNKNOWN"));
    EXPECT_EQ(std::string(FixTypeStr(FixType::NOFIX)),          std::string("NOFIX"));
    EXPECT_EQ(std::string(FixTypeStr(FixType::DRONLY)),         std::string("DRONLY"));
    EXPECT_EQ(std::string(FixTypeStr(FixType::TIME)),           std::string("TIME"));
    EXPECT_EQ(std::string(FixTypeStr(FixType::SPP_2D)),         std::string("SPP_2D"));
    EXPECT_EQ(std::string(FixTypeStr(FixType::SPP_3D)),         std::string("SPP_3D"));
    EXPECT_EQ(std::string(FixTypeStr(FixType::SPP_3D_DR)),      std::string("SPP_3D_DR"));
    EXPECT_EQ(std::string(FixTypeStr(FixType::RTK_FLOAT)),      std::string("RTK_FLOAT"));
    EXPECT_EQ(std::string(FixTypeStr(FixType::RTK_FIXED)),      std::string("RTK_FIXED"));
    EXPECT_EQ(std::string(FixTypeStr(FixType::RTK_FLOAT_DR)),   std::string("RTK_FLOAT_DR"));
    EXPECT_EQ(std::string(FixTypeStr(FixType::RTK_FIXED_DR)),   std::string("RTK_FIXED_DR"));
    EXPECT_EQ(std::string(FixTypeStr((FixType)99)),             std::string("?")); // NOLINT
    // clang-format on
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(GnssTest, GnssStr)
{
    // clang-format off
    EXPECT_EQ(std::string(GnssStr(Gnss::UNKNOWN)),    std::string("UNKNOWN"));
    EXPECT_EQ(std::string(GnssStr(Gnss::GPS)),        std::string("GPS"));
    EXPECT_EQ(std::string(GnssStr(Gnss::SBAS)),       std::string("SBAS"));
    EXPECT_EQ(std::string(GnssStr(Gnss::GAL)),        std::string("GAL"));
    EXPECT_EQ(std::string(GnssStr(Gnss::BDS)),        std::string("BDS"));
    EXPECT_EQ(std::string(GnssStr(Gnss::QZSS)),       std::string("QZSS"));
    EXPECT_EQ(std::string(GnssStr(Gnss::GLO)),        std::string("GLO"));
    EXPECT_EQ(std::string(GnssStr(Gnss::NAVIC)),      std::string("NAVIC"));
    EXPECT_EQ(std::string(GnssStr((Gnss)99)),         std::string("?")); // NOLINT
    // clang-format on
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(GnssTest, SignalStr)
{
    // clang-format off
    EXPECT_EQ(std::string(SignalStr(Signal::UNKNOWN)),      std::string("UNKNOWN"));
    EXPECT_EQ(std::string(SignalStr(Signal::BDS_B1C)),      std::string("BDS_B1C"));
    EXPECT_EQ(std::string(SignalStr(Signal::BDS_B1I)),      std::string("BDS_B1I"));
    EXPECT_EQ(std::string(SignalStr(Signal::BDS_B2A)),      std::string("BDS_B2A"));
    EXPECT_EQ(std::string(SignalStr(Signal::BDS_B2I)),      std::string("BDS_B2I"));
    EXPECT_EQ(std::string(SignalStr(Signal::GAL_E1)),       std::string("GAL_E1"));
    EXPECT_EQ(std::string(SignalStr(Signal::GAL_E5A)),      std::string("GAL_E5A"));
    EXPECT_EQ(std::string(SignalStr(Signal::GAL_E5B)),      std::string("GAL_E5B"));
    EXPECT_EQ(std::string(SignalStr(Signal::GLO_L1OF)),     std::string("GLO_L1OF"));
    EXPECT_EQ(std::string(SignalStr(Signal::GLO_L2OF)),     std::string("GLO_L2OF"));
    EXPECT_EQ(std::string(SignalStr(Signal::GPS_L1CA)),     std::string("GPS_L1CA"));
    EXPECT_EQ(std::string(SignalStr(Signal::GPS_L2C)),      std::string("GPS_L2C"));
    EXPECT_EQ(std::string(SignalStr(Signal::GPS_L5)),       std::string("GPS_L5"));
    EXPECT_EQ(std::string(SignalStr(Signal::QZSS_L1CA)),    std::string("QZSS_L1CA"));
    EXPECT_EQ(std::string(SignalStr(Signal::QZSS_L1S)),     std::string("QZSS_L1S"));
    EXPECT_EQ(std::string(SignalStr(Signal::QZSS_L2C)),     std::string("QZSS_L2C"));
    EXPECT_EQ(std::string(SignalStr(Signal::QZSS_L5)),      std::string("QZSS_L5"));
    EXPECT_EQ(std::string(SignalStr(Signal::SBAS_L1CA)),    std::string("SBAS_L1CA"));
    EXPECT_EQ(std::string(SignalStr(Signal::NAVIC_L5A)),    std::string("NAVIC_L5A"));
    EXPECT_EQ(std::string(SignalStr((Signal)99)),           std::string("?")); // NOLINT
    // clang-format on
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(GnssTest, BandStr)
{
    // clang-format off
    EXPECT_EQ(std::string(BandStr(Band::UNKNOWN)),  std::string("UNKNOWN"));
    EXPECT_EQ(std::string(BandStr(Band::L1)),       std::string("L1"));
    EXPECT_EQ(std::string(BandStr(Band::L2)),       std::string("L2"));
    EXPECT_EQ(std::string(BandStr(Band::L5)),       std::string("L5"));
    EXPECT_EQ(std::string(BandStr((Band)99)),       std::string("?")); // NOLINT
    // clang-format on
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(GnssTest, Sat)
{
    {
        const Sat sat(Gnss::GPS, 12);
        EXPECT_NE(sat, INVALID_SAT);
        EXPECT_EQ(sat.GetGnss(), Gnss::GPS);
        EXPECT_EQ(sat.GetSvNr(), 12);
    }
    {
        const Sat sat1(Gnss::GPS, 12);
        const Sat sat2(Gnss::GAL, 12);
        const Sat sat3(Gnss::GAL, 12);
        EXPECT_LT(sat1, sat2);
        EXPECT_NE(sat1, sat2);
        EXPECT_EQ(sat2, sat3);
    }
    {
        std::map<Sat, bool> map;
        map.emplace(Sat(Gnss::GAL, 12), true);
        map.emplace(Sat(Gnss::GPS, 12), true);
        EXPECT_EQ(map.begin()->first, Sat(Gnss::GPS, 12));
    }
    {
        std::unordered_map<Sat, bool> umap;
        umap.emplace(Sat(Gnss::GAL, 12), true);
        umap.emplace(Sat(Gnss::GPS, 12), true);
        EXPECT_EQ(umap.size(), 2);
    }
    {
        std::set<Sat> set;
        set.emplace(Sat(Gnss::GAL, 12));
        set.emplace(Sat(Gnss::GPS, 12));
        EXPECT_EQ(set.size(), 2);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(GnssTest, SatSig)
{
    {
        const SatSig satsig(Gnss::GPS, 12, Band::L1, Signal::GPS_L1CA);
        EXPECT_NE(satsig, INVALID_SATSIG);
        EXPECT_EQ(satsig.GetGnss(), Gnss::GPS);
        EXPECT_EQ(satsig.GetSvNr(), 12);
        EXPECT_EQ(satsig.GetBand(), Band::L1);
        EXPECT_EQ(satsig.GetSignal(), Signal::GPS_L1CA);
    }
    {
        const SatSig satsig(Sat(Gnss::GPS, 12), Signal::GPS_L1CA);
        EXPECT_NE(satsig, INVALID_SATSIG);
        EXPECT_EQ(satsig.GetGnss(), Gnss::GPS);
        EXPECT_EQ(satsig.GetSvNr(), 12);
        EXPECT_EQ(satsig.GetBand(), Band::L1);
        EXPECT_EQ(satsig.GetSignal(), Signal::GPS_L1CA);
    }
    {
        EXPECT_EQ(INVALID_SATSIG.GetSat(), INVALID_SAT);
    }
    {
        std::map<SatSig, bool> map;
        map.emplace(SatSig(Gnss::GPS, 12, Band::L1, Signal::GPS_L2C), true);
        map.emplace(SatSig(Gnss::GPS, 12, Band::L1, Signal::GPS_L1CA), true);
        EXPECT_EQ(map.begin()->first, SatSig(Gnss::GPS, 12, Band::L1, Signal::GPS_L1CA));
    }
    {
        std::unordered_map<SatSig, bool> umap;
        umap.emplace(SatSig(Gnss::GPS, 12, Band::L1, Signal::GPS_L2C), true);
        umap.emplace(SatSig(Gnss::GPS, 12, Band::L1, Signal::GPS_L1CA), true);
        EXPECT_EQ(umap.size(), 2);
    }
    {
        std::set<SatSig> set;
        set.emplace(SatSig(Gnss::GPS, 12, Band::L1, Signal::GPS_L2C));
        set.emplace(SatSig(Gnss::GPS, 12, Band::L1, Signal::GPS_L1CA));
        EXPECT_EQ(set.size(), 2);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(GnssTest, SatStr)
{  // clang-format off

    // Valid
    EXPECT_EQ(std::string(Sat(Gnss::GPS,     1).GetStr()), std::string("G01"));
    EXPECT_EQ(std::string(Sat(Gnss::SBAS,   22).GetStr()), std::string("S22"));
    EXPECT_EQ(std::string(Sat(Gnss::GAL,    12).GetStr()), std::string("E12"));
    EXPECT_EQ(std::string(Sat(Gnss::BDS,     5).GetStr()), std::string("C05"));
    EXPECT_EQ(std::string(Sat(Gnss::QZSS,    9).GetStr()), std::string("J09"));
    EXPECT_EQ(std::string(Sat(Gnss::GLO,    15).GetStr()), std::string("R15"));

    // Invalid
    EXPECT_EQ(std::string(Sat(Gnss::GPS,     0).GetStr()), std::string("G??"));
    EXPECT_EQ(std::string(Sat(Gnss::GPS,    33).GetStr()), std::string("G??"));
    EXPECT_EQ(std::string(Sat(Gnss::SBAS,   19).GetStr()), std::string("S??"));
    EXPECT_EQ(std::string(Sat(Gnss::SBAS,   59).GetStr()), std::string("S??"));
    EXPECT_EQ(std::string(Sat(Gnss::GAL,    37).GetStr()), std::string("E??"));
    EXPECT_EQ(std::string(Sat(Gnss::BDS,    64).GetStr()), std::string("C??"));
    EXPECT_EQ(std::string(Sat(Gnss::QZSS,   11).GetStr()), std::string("J??"));
    EXPECT_EQ(std::string(Sat(Gnss::GLO,    33).GetStr()), std::string("R??"));
    EXPECT_EQ(std::string(Sat(Gnss::UNKNOWN, 2).GetStr()), std::string("???"));

    // clang-format on
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(GnssTest, StrSat)
{  // clang-format off

    // Valid
    EXPECT_EQ(Sat("G01"), Sat(Gnss::GPS,   1));
    EXPECT_EQ(Sat("S22"), Sat(Gnss::SBAS, 22));
    EXPECT_EQ(Sat("E12"), Sat(Gnss::GAL,  12));
    EXPECT_EQ(Sat("C05"), Sat(Gnss::BDS,   5));
    EXPECT_EQ(Sat("J09"), Sat(Gnss::QZSS,  9));
    EXPECT_EQ(Sat("R15"), Sat(Gnss::GLO,  15));

    // Invalid
    EXPECT_EQ(Sat("G00"),    INVALID_SAT);
    EXPECT_EQ(Sat("G33"),    INVALID_SAT);
    EXPECT_EQ(Sat("S19"),    INVALID_SAT);
    EXPECT_EQ(Sat("S59"),    INVALID_SAT);
    EXPECT_EQ(Sat("E37"),    INVALID_SAT);
    EXPECT_EQ(Sat("C64"),    INVALID_SAT);
    EXPECT_EQ(Sat("J11"),    INVALID_SAT);
    EXPECT_EQ(Sat("R33"),    INVALID_SAT);
    EXPECT_EQ(Sat("gugugs"), INVALID_SAT);
    EXPECT_EQ(Sat(""),       INVALID_SAT);
    EXPECT_EQ(Sat("G1"),     INVALID_SAT);
    EXPECT_EQ(Sat("E1"),     INVALID_SAT);
    EXPECT_EQ(Sat("C1"),     INVALID_SAT);
    EXPECT_EQ(Sat("J1"),     INVALID_SAT);
    EXPECT_EQ(Sat("R1"),     INVALID_SAT);

    // clang-format on
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(GnssTest, UbxGnssIdToGnss)
{
    EXPECT_EQ(UbxGnssIdToGnss(0), Gnss::GPS);
    EXPECT_EQ(UbxGnssIdToGnss(2), Gnss::GAL);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(GnssTest, UbxGnssIdSvIdToSat)
{
    EXPECT_EQ(UbxGnssIdSvIdToSat(0, 12), Sat(Gnss::GPS, 12));
    EXPECT_EQ(UbxGnssIdSvIdToSat(2, 1), Sat(Gnss::GAL, 1));
    EXPECT_EQ(UbxGnssIdSvIdToSat(1, 123), Sat(Gnss::SBAS, 23));
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(GnssTest, UbxGnssIdSigIdToSignal)
{
    EXPECT_EQ(UbxGnssIdSigIdToSignal(0, 0), Signal::GPS_L1CA);
    EXPECT_EQ(UbxGnssIdSigIdToSignal(2, 5), Signal::GAL_E5B);  // E5 bI
    EXPECT_EQ(UbxGnssIdSigIdToSignal(2, 6), Signal::GAL_E5B);  // E5 bQ
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
