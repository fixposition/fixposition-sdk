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

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/gnss.hpp>
#include <fpsdk_common/logging.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::gnss;

TEST(GnssTest, GnssFixTypeStr)
{
    // clang-format off
    EXPECT_EQ(std::string(GnssFixTypeStr(GnssFixType::UNKNOWN)),        std::string("UNKNOWN"));
    EXPECT_EQ(std::string(GnssFixTypeStr(GnssFixType::NOFIX)),          std::string("NOFIX"));
    EXPECT_EQ(std::string(GnssFixTypeStr(GnssFixType::DRONLY)),         std::string("DRONLY"));
    EXPECT_EQ(std::string(GnssFixTypeStr(GnssFixType::TIME)),           std::string("TIME"));
    EXPECT_EQ(std::string(GnssFixTypeStr(GnssFixType::SPP_2D)),         std::string("SPP_2D"));
    EXPECT_EQ(std::string(GnssFixTypeStr(GnssFixType::SPP_3D)),         std::string("SPP_3D"));
    EXPECT_EQ(std::string(GnssFixTypeStr(GnssFixType::SPP_3D_DR)),      std::string("SPP_3D_DR"));
    EXPECT_EQ(std::string(GnssFixTypeStr(GnssFixType::RTK_FLOAT)),      std::string("RTK_FLOAT"));
    EXPECT_EQ(std::string(GnssFixTypeStr(GnssFixType::RTK_FIXED)),      std::string("RTK_FIXED"));
    EXPECT_EQ(std::string(GnssFixTypeStr(GnssFixType::RTK_FLOAT_DR)),   std::string("RTK_FLOAT_DR"));
    EXPECT_EQ(std::string(GnssFixTypeStr(GnssFixType::RTK_FIXED_DR)),   std::string("RTK_FIXED_DR"));
    EXPECT_EQ(std::string(GnssFixTypeStr((GnssFixType)99)),             std::string("?")); // NOLINT
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
    const Sat sat = GnssSvNrToSat(Gnss::GPS, 12);
    DEBUG("sat=%04x %02x %02x", sat, (int)SatToGnss(sat), (int)SatToSvNr(sat));
    EXPECT_NE(sat, INVALID_SAT);
    EXPECT_EQ(SatToGnss(sat), Gnss::GPS);
    EXPECT_EQ(SatToSvNr(sat), 12);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(GnssTest, SatSig)
{
    const SatSig satsig = GnssSvNrBandSignalToSatSig(Gnss::GPS, 12, Band::L1, Signal::GPS_L1CA);
    DEBUG("satsig=%08x %02x  %02x  %02x %02x", satsig, (int)SatSigToGnss(satsig), (int)SatSigToSvNr(satsig),
        (int)SatSigToBand(satsig), (int)SatSigToSignal(satsig));
    EXPECT_NE(satsig, INVALID_SATSIG);
    EXPECT_EQ(SatSigToGnss(satsig), Gnss::GPS);
    EXPECT_EQ(SatSigToSvNr(satsig), 12);
    EXPECT_EQ(SatSigToBand(satsig), Band::L1);
    EXPECT_EQ(SatSigToSignal(satsig), Signal::GPS_L1CA);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(GnssTest, SatStr)
{
    // Valid
    EXPECT_EQ(std::string(SatStr(GnssSvNrToSat(Gnss::GPS, 1))), std::string("G01"));
    EXPECT_EQ(std::string(SatStr(GnssSvNrToSat(Gnss::SBAS, 22))), std::string("S22"));
    EXPECT_EQ(std::string(SatStr(GnssSvNrToSat(Gnss::GAL, 12))), std::string("E12"));
    EXPECT_EQ(std::string(SatStr(GnssSvNrToSat(Gnss::BDS, 5))), std::string("C05"));
    EXPECT_EQ(std::string(SatStr(GnssSvNrToSat(Gnss::QZSS, 9))), std::string("J09"));
    EXPECT_EQ(std::string(SatStr(GnssSvNrToSat(Gnss::GLO, 15))), std::string("R15"));

    // Invalid
    EXPECT_EQ(std::string(SatStr(GnssSvNrToSat(Gnss::GPS, 0))), std::string("G??"));
    EXPECT_EQ(std::string(SatStr(GnssSvNrToSat(Gnss::GPS, 33))), std::string("G??"));
    EXPECT_EQ(std::string(SatStr(GnssSvNrToSat(Gnss::SBAS, 19))), std::string("S??"));
    EXPECT_EQ(std::string(SatStr(GnssSvNrToSat(Gnss::SBAS, 59))), std::string("S??"));
    EXPECT_EQ(std::string(SatStr(GnssSvNrToSat(Gnss::GAL, 37))), std::string("E??"));
    EXPECT_EQ(std::string(SatStr(GnssSvNrToSat(Gnss::BDS, 64))), std::string("C??"));
    EXPECT_EQ(std::string(SatStr(GnssSvNrToSat(Gnss::QZSS, 11))), std::string("J??"));
    EXPECT_EQ(std::string(SatStr(GnssSvNrToSat(Gnss::GLO, 33))), std::string("R??"));
    EXPECT_EQ(std::string(SatStr(GnssSvNrToSat(Gnss::UNKNOWN, 12))), std::string("???"));
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(GnssTest, StrSat)
{
    // Valid
    EXPECT_EQ(StrSat("G01"), GnssSvNrToSat(Gnss::GPS, 1));
    EXPECT_EQ(StrSat("S22"), GnssSvNrToSat(Gnss::SBAS, 22));
    EXPECT_EQ(StrSat("E12"), GnssSvNrToSat(Gnss::GAL, 12));
    EXPECT_EQ(StrSat("C05"), GnssSvNrToSat(Gnss::BDS, 5));
    EXPECT_EQ(StrSat("J09"), GnssSvNrToSat(Gnss::QZSS, 9));
    EXPECT_EQ(StrSat("R15"), GnssSvNrToSat(Gnss::GLO, 15));

    // Invalid
    EXPECT_EQ(StrSat("G00"), INVALID_SAT);
    EXPECT_EQ(StrSat("G33"), INVALID_SAT);
    EXPECT_EQ(StrSat("S19"), INVALID_SAT);
    EXPECT_EQ(StrSat("S59"), INVALID_SAT);
    EXPECT_EQ(StrSat("E37"), INVALID_SAT);
    EXPECT_EQ(StrSat("C64"), INVALID_SAT);
    EXPECT_EQ(StrSat("J11"), INVALID_SAT);
    EXPECT_EQ(StrSat("R33"), INVALID_SAT);
    EXPECT_EQ(StrSat("gugugs"), INVALID_SAT);
    EXPECT_EQ(StrSat(""), INVALID_SAT);
    EXPECT_EQ(StrSat("G1"), INVALID_SAT);
    EXPECT_EQ(StrSat("E1"), INVALID_SAT);
    EXPECT_EQ(StrSat("C1"), INVALID_SAT);
    EXPECT_EQ(StrSat("J1"), INVALID_SAT);
    EXPECT_EQ(StrSat("R1"), INVALID_SAT);
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
    EXPECT_EQ(UbxGnssIdSvIdToSat(0, 12), GnssSvNrToSat(Gnss::GPS, 12));
    EXPECT_EQ(UbxGnssIdSvIdToSat(2, 1), GnssSvNrToSat(Gnss::GAL, 1));
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
