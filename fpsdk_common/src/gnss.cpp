/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 *
 * Based on work by flipflip (https://github.com/phkehl)
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: GNSS types and utilities
 */

/* LIBC/STL */
#include <cinttypes>
#include <cstdio>

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/gnss.hpp"
#include "fpsdk_common/parser/ubx.hpp"

namespace fpsdk {
namespace common {
namespace gnss {
/* ****************************************************************************************************************** */

const char* GnssFixTypeStr(const GnssFixType fix_type)
{
    switch (fix_type) {  // clang-format off
        case GnssFixType::UNKNOWN:         return "UNKNOWN";
        case GnssFixType::NOFIX:           return "NOFIX";
        case GnssFixType::DRONLY:          return "DRONLY";
        case GnssFixType::TIME:            return "TIME";
        case GnssFixType::SPP_2D:          return "SPP_2D";
        case GnssFixType::SPP_3D:          return "SPP_3D";
        case GnssFixType::SPP_3D_DR:       return "SPP_3D_DR";
        case GnssFixType::RTK_FLOAT:       return "RTK_FLOAT";
        case GnssFixType::RTK_FIXED:       return "RTK_FIXED";
        case GnssFixType::RTK_FLOAT_DR:    return "RTK_FLOAT_DR";
        case GnssFixType::RTK_FIXED_DR:    return "RTK_FIXED_DR";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* GnssStr(const Gnss gnss)
{
    switch (gnss) {  // clang-format off
        case Gnss::UNKNOWN:   return "UNKNOWN";
        case Gnss::GPS:       return "GPS";
        case Gnss::SBAS:      return "SBAS";
        case Gnss::GAL:       return "GAL";
        case Gnss::BDS:       return "BDS";
        case Gnss::QZSS:      return "QZSS";
        case Gnss::GLO:       return "GLO";
        case Gnss::NAVIC:     return "NAVIC";
    }  // clang-format on

    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* SignalStr(const Signal signal, const bool kurz)
{
    switch (signal) {  // clang-format off
        case Signal::UNKNOWN:    return kurz ? "UNKN"  : "UNKNOWN";
        case Signal::BDS_B1C:    return kurz ? "B1C"   : "BDS_B1C";
        case Signal::BDS_B1I:    return kurz ? "B1I"   : "BDS_B1I";
        case Signal::BDS_B2A:    return kurz ? "B2A"   : "BDS_B2A";
        case Signal::BDS_B2I:    return kurz ? "B2I"   : "BDS_B2I";
        case Signal::BDS_B3I:    return kurz ? "B3I"   : "BDS_B3I";
        case Signal::GAL_E1:     return kurz ? "E1"    : "GAL_E1";
        case Signal::GAL_E5A:    return kurz ? "E5A"   : "GAL_E5A";
        case Signal::GAL_E5B:    return kurz ? "E5B"   : "GAL_E5B";
        case Signal::GAL_E6:     return kurz ? "E6"    : "GAL_E6";
        case Signal::GLO_L1OF:   return kurz ? "L1OF"  : "GLO_L1OF";
        case Signal::GLO_L2OF:   return kurz ? "L2OF"  : "GLO_L2OF";
        case Signal::GPS_L1CA:   return kurz ? "L1CA"  : "GPS_L1CA";
        case Signal::GPS_L2C:    return kurz ? "L2C"   : "GPS_L2C";
        case Signal::GPS_L5:     return kurz ? "L5"    : "GPS_L5";
        case Signal::QZSS_L1CA:  return kurz ? "L1CA"  : "QZSS_L1CA";
        case Signal::QZSS_L1S:   return kurz ? "L1S"   : "QZSS_L1S";
        case Signal::QZSS_L2C:   return kurz ? "L2C"   : "QZSS_L2C";
        case Signal::QZSS_L5:    return kurz ? "L5"    : "QZSS_L5";
        case Signal::SBAS_L1CA:  return kurz ? "L1CA"  : "SBAS_L1CA";
        case Signal::NAVIC_L5A:  return kurz ? "L5A"   : "NAVIC_L5A";
    }  // clang-format on

    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* BandStr(const Band band)
{
    switch (band) {  // clang-format off
        case Band::UNKNOWN: return "UNKNOWN";
        case Band::L1:      return "L1";
        case Band::L2:      return "L2";
        case Band::E6:      return "E6";
        case Band::L5:      return "L5";
    }  // clang-format on

    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

Band SignalToBand(const Signal signal)
{
    switch (signal) {  // clang-format off
        case Signal::GPS_L1CA:   /* FALLTHROUGH */
        case Signal::BDS_B1I:    /* FALLTHROUGH */
        case Signal::BDS_B1C:    /* FALLTHROUGH */
        case Signal::GAL_E1:     /* FALLTHROUGH */
        case Signal::GLO_L1OF:   /* FALLTHROUGH */
        case Signal::QZSS_L1CA:  /* FALLTHROUGH */
        case Signal::QZSS_L1S:   /* FALLTHROUGH */
        case Signal::SBAS_L1CA:  return Band::L1;
        case Signal::GPS_L2C:    /* FALLTHROUGH */
        case Signal::BDS_B2I:    /* FALLTHROUGH */
        case Signal::GAL_E5B:    /* FALLTHROUGH */
        case Signal::GLO_L2OF:   /* FALLTHROUGH */
        case Signal::QZSS_L2C:   return Band::L2;
        case Signal::BDS_B3I:    /* FALLTHROUGH */
        case Signal::GAL_E6:     return Band::E6;
        case Signal::BDS_B2A:    /* FALLTHROUGH */
        case Signal::GPS_L5:     /* FALLTHROUGH */
        case Signal::QZSS_L5:    /* FALLTHROUGH */
        case Signal::GAL_E5A:    /* FALLTHROUGH */
        case Signal::NAVIC_L5A:  return Band::L5;
        case Signal::UNKNOWN:    return Band::UNKNOWN;
    }  // clang-format on

    return Band::UNKNOWN;
}

// ---------------------------------------------------------------------------------------------------------------------

const char* SatStr(const Sat sat)
{
    const Gnss gnss = SatToGnss(sat);
    const SvNr svnr = SatToSvNr(sat);

    switch (gnss) {
        case Gnss::GPS: {
            static const std::array<const char*, NUM_GPS> strs =
                /* clang-format off */ {{
                "G01", "G02", "G03", "G04", "G05", "G06", "G07", "G08", "G09", "G10",
                "G11", "G12", "G13", "G14", "G15", "G16", "G17", "G18", "G19", "G20",
                "G21", "G22", "G23", "G24", "G25", "G26", "G27", "G28", "G29", "G30",
                "G31", "G32"
            }};  // clang-format on
            const int ix = (int)svnr - (int)FIRST_GPS;
            if ((ix < 0) || (ix >= (int)strs.size())) {
                return "G??";
            }
            return strs[ix];
        }
        case Gnss::SBAS: {
            static const std::array<const char*, NUM_SBAS> strs =
                /* clang-format off */ {{
                "S20", "S21", "S22", "S23", "S24", "S25", "S26", "S27", "S28", "S29",
                "S30", "S31", "S32", "S33", "S34", "S35", "S36", "S37", "S38", "S39",
                "S40", "S41", "S42", "S43", "S44", "S45", "S46", "S47", "S48", "S49",
                "S50", "S51", "S52", "S53", "S54", "S55", "S56", "S57", "S58"
            }};  // clang-format on
            const int ix = (int)svnr - (int)FIRST_SBAS;
            if ((ix < 0) || (ix >= (int)strs.size())) {
                return "S??";
            }
            return strs[ix];
        }
        case Gnss::GAL: {
            static const std::array<const char*, NUM_GAL> strs =
                /* clang-format off */ {{
                "E01", "E02", "E03", "E04", "E05", "E06", "E07", "E08", "E09", "E10",
                "E11", "E12", "E13", "E14", "E15", "E16", "E17", "E18", "E19", "E20",
                "E21", "E22", "E23", "E24", "E25", "E26", "E27", "E28", "E29", "E30",
                "E31", "E32", "E33", "E34", "E35", "E36"
            }};  // clang-format on
            const int ix = (int)svnr - (int)FIRST_GAL;
            if ((ix < 0) || (ix >= (int)strs.size())) {
                return "E??";
            }
            return strs[ix];
        }
        case Gnss::BDS: {
            static const std::array<const char*, NUM_BDS> strs =
                /* clang-format off */ {{
                "C01", "C02", "C03", "C04", "C05", "C06", "C07", "C08", "C09", "C10",
                "C11", "C12", "C13", "C14", "C15", "C16", "C17", "C18", "C19", "C20",
                "C21", "C22", "C23", "C24", "C25", "C26", "C27", "C28", "C29", "C30",
                "C31", "C32", "C33", "C34", "C35", "C36", "C37", "C38", "C30", "C40",
                "C41", "C42", "C43", "C44", "C45", "C46", "C47", "C48", "C40", "C50",
                "C51", "C52", "C53", "C54", "C55", "C56", "C57", "C58", "C50", "C60",
                "C61", "C62", "C63"
                }};  // clang-format on
            const int ix = (int)svnr - (int)FIRST_BDS;
            if ((ix < 0) || (ix >= (int)strs.size())) {
                return "C??";
            }
            return strs[ix];
        }
        case Gnss::QZSS: {
            static const std::array<const char*, NUM_QZSS> strs = /* clang-format off */ {{
                "J01", "J02", "J03", "J04", "J05", "J06", "J07", "J08", "J09", "J10"
            }};  // clang-format on
            const int ix = (int)svnr - (int)FIRST_QZSS;
            if ((ix < 0) || (ix >= (int)strs.size())) {
                return "J??";
            }
            return strs[ix];
        }
        case Gnss::GLO: {
            static const std::array<const char*, NUM_GLO> strs =
                /* clang-format off */ {{
                "R01", "R02", "R03", "R04", "R05", "R06", "R07", "R08", "R09", "R10",
                "R11", "R12", "R13", "R14", "R15", "R16", "R17", "R18", "R19", "R20",
                "R21", "R22", "R23", "R24", "R25", "R26", "R27", "R28", "R29", "R30",
                "R31", "R32"
            }};  // clang-format on
            const int ix = (int)svnr - (int)FIRST_GLO;
            if ((ix < 0) || (ix >= (int)strs.size())) {
                return "R??";
            }
            return strs[ix];
        }
        case Gnss::NAVIC: {
            static const std::array<const char*, NUM_NAVIC> strs = /* clang-format off */ {{
                "I01", "I02", "I03", "I04", "I05", "I06", "I07", "I08", "I09", "I10",
                "I11", "I12", "I13", "I14"
            }};  // clang-format on
            const int ix = (int)svnr - (int)FIRST_NAVIC;
            if ((ix < 0) || (ix >= (int)strs.size())) {
                return "I??";
            }
            return strs[ix];
        }
        case Gnss::UNKNOWN:
            break;
    }
    return "???";
}

// ---------------------------------------------------------------------------------------------------------------------

Sat StrSat(const char* str)
{
    Sat sat = INVALID_SAT;
    char c = '\0';
    uint8_t n = 0;
    int len = 0;
    if ((std::sscanf(str, "%c%" SCNu8 "%n", &c, &n, &len) == 2) && (len == 3)) {
        // clang-format off
        if      (((uint8_t)c == types::EnumToVal(Gnss::GPS))   && (n >= FIRST_GPS)   && (n < (FIRST_GPS   + NUM_GPS)))   { sat = GnssSvNrToSat(Gnss::GPS,   n); }
        else if (((uint8_t)c == types::EnumToVal(Gnss::SBAS))  && (n >= FIRST_SBAS)  && (n < (FIRST_SBAS  + NUM_SBAS)))  { sat = GnssSvNrToSat(Gnss::SBAS,  n); }
        else if (((uint8_t)c == types::EnumToVal(Gnss::GAL))   && (n >= FIRST_GAL)   && (n < (FIRST_GAL   + NUM_GAL)))   { sat = GnssSvNrToSat(Gnss::GAL,   n); }
        else if (((uint8_t)c == types::EnumToVal(Gnss::BDS))   && (n >= FIRST_BDS)   && (n < (FIRST_BDS   + NUM_BDS)))   { sat = GnssSvNrToSat(Gnss::BDS,   n); }
        else if (((uint8_t)c == types::EnumToVal(Gnss::QZSS))  && (n >= FIRST_QZSS)  && (n < (FIRST_QZSS  + NUM_QZSS)))  { sat = GnssSvNrToSat(Gnss::QZSS,  n); }
        else if (((uint8_t)c == types::EnumToVal(Gnss::GLO))   && (n >= FIRST_GLO)   && (n < (FIRST_GLO   + NUM_GLO)))   { sat = GnssSvNrToSat(Gnss::GLO,   n); }
        else if (((uint8_t)c == types::EnumToVal(Gnss::NAVIC)) && (n >= FIRST_NAVIC) && (n < (FIRST_NAVIC + NUM_NAVIC))) { sat = GnssSvNrToSat(Gnss::NAVIC, n); }
    }
    return sat;
}

// ---------------------------------------------------------------------------------------------------------------------

Gnss UbxGnssIdToGnss(const uint8_t gnssId)
{
    using namespace parser::ubx;
    switch (gnssId) {  // clang-format off
        case UBX_GNSSID_NONE:   return Gnss::UNKNOWN;
        case UBX_GNSSID_GPS:    return Gnss::GPS;
        case UBX_GNSSID_SBAS:   return Gnss::SBAS;
        case UBX_GNSSID_GAL:    return Gnss::GAL;
        case UBX_GNSSID_BDS:    return Gnss::BDS;
        case UBX_GNSSID_QZSS:   return Gnss::QZSS;
        case UBX_GNSSID_GLO:    return Gnss::GLO;
        case UBX_GNSSID_NAVIC:  return Gnss::NAVIC;
    }  // clang-format on
    return Gnss::UNKNOWN;
}

// ---------------------------------------------------------------------------------------------------------------------

Sat UbxGnssIdSvIdToSat(const uint8_t gnssId, const uint8_t svId)
{
    using namespace parser::ubx;
    switch (gnssId) {
        case UBX_GNSSID_GPS:
            if ((svId >= UBX_FIRST_GPS) && (svId < (UBX_FIRST_GPS + UBX_NUM_GPS))) {
                return GnssSvNrToSat(Gnss::GPS, svId - UBX_FIRST_GPS + FIRST_GPS);
            }
            break;
        case UBX_GNSSID_SBAS:
            if ((svId >= UBX_FIRST_SBAS) && (svId < (UBX_FIRST_SBAS + UBX_NUM_SBAS))) {
                return GnssSvNrToSat(Gnss::SBAS, svId - UBX_FIRST_SBAS + FIRST_SBAS);
            }
            break;
        case UBX_GNSSID_GAL:
            if ((svId >= UBX_FIRST_GAL) && (svId < (UBX_FIRST_GAL + UBX_NUM_GAL))) {
                return GnssSvNrToSat(Gnss::GAL, svId - UBX_FIRST_GAL + FIRST_GAL);
            }
            break;
        case UBX_GNSSID_BDS:
            if ((svId >= UBX_FIRST_BDS) && (svId < (UBX_FIRST_BDS + UBX_NUM_BDS))) {
                return GnssSvNrToSat(Gnss::BDS, svId - UBX_FIRST_BDS + FIRST_BDS);
            }
            break;
        case UBX_GNSSID_QZSS:
            if ((svId >= UBX_FIRST_QZSS) && (svId < (UBX_FIRST_QZSS + UBX_NUM_QZSS))) {
                return GnssSvNrToSat(Gnss::QZSS, svId - UBX_FIRST_QZSS + FIRST_QZSS);
            }
            break;
        case UBX_GNSSID_GLO:
            if ((svId >= UBX_FIRST_GLO) && (svId < (UBX_FIRST_GLO + UBX_NUM_GLO))) {
                return GnssSvNrToSat(Gnss::GLO, svId - UBX_FIRST_GLO + FIRST_GLO);
            }
            break;
        case UBX_GNSSID_NAVIC:
            if ((svId >= UBX_FIRST_NAVIC) && (svId < (UBX_FIRST_NAVIC + UBX_NUM_NAVIC))) {
                return GnssSvNrToSat(Gnss::NAVIC, svId - UBX_FIRST_NAVIC + FIRST_NAVIC);
            }
            break;
    }
    return INVALID_SAT;
}

// ---------------------------------------------------------------------------------------------------------------------

Signal UbxGnssIdSigIdToSignal(const uint8_t gnssId, const uint8_t sigId)
{
    using namespace parser::ubx;
    switch (gnssId) {  // clang-format off
        case UBX_GNSSID_GPS:
            switch (sigId) {
                case UBX_SIGID_GPS_L1CA:  return Signal::GPS_L1CA;
                case UBX_SIGID_GPS_L2CL:  /* FALLTHROUGH */
                case UBX_SIGID_GPS_L2CM:  return Signal::GPS_L2C;
                case UBX_SIGID_GPS_L5I:   /* FALLTHROUGH */
                case UBX_SIGID_GPS_L5Q:   return Signal::GPS_L5;
            }
            break;
        case UBX_GNSSID_SBAS:
            switch (sigId) {
                case UBX_SIGID_SBAS_L1CA: return Signal::SBAS_L1CA;;
            }
            break;
        case UBX_GNSSID_GAL :
            switch (sigId) {
                case UBX_SIGID_GAL_E1C:  /* FALLTHROUGH */
                case UBX_SIGID_GAL_E1B:  return Signal::GAL_E1;
                case UBX_SIGID_GAL_E5AI: /* FALLTHROUGH */
                case UBX_SIGID_GAL_E5AQ: return Signal::GAL_E5A;
                case UBX_SIGID_GAL_E5BI: /* FALLTHROUGH */
                case UBX_SIGID_GAL_E5BQ: return Signal::GAL_E5B;
                case UBX_SIGID_GAL_E6B:  /* FALLTHROUGH */
                case UBX_SIGID_GAL_E6C:  /* FALLTHROUGH */
                case UBX_SIGID_GAL_E6A:  return Signal::GAL_E6;

            }
            break;
        case UBX_GNSSID_BDS :
            switch (sigId) {
                case UBX_SIGID_BDS_B1ID1: /* FALLTHROUGH */
                case UBX_SIGID_BDS_B1ID2: return Signal::BDS_B1I;
                case UBX_SIGID_BDS_B1CD:  /* FALLTHROUGH */
                case UBX_SIGID_BDS_B1CP:  return Signal::BDS_B1C;
                case UBX_SIGID_BDS_B2ID1: /* FALLTHROUGH */
                case UBX_SIGID_BDS_B2ID2: return Signal::BDS_B2I;
                case UBX_SIGID_BDS_B3ID1: /* FALLTHROUGH */
                case UBX_SIGID_BDS_B3ID2: return Signal::BDS_B3I;
                case UBX_SIGID_BDS_B2AP:  /* FALLTHROUGH */
                case UBX_SIGID_BDS_B2AD:  return Signal::BDS_B2A;
            }
            break;
        case UBX_GNSSID_QZSS:
            switch (sigId) {
                case UBX_SIGID_QZSS_L1CA: return Signal::QZSS_L1CA;
                case UBX_SIGID_QZSS_L1S : return Signal::QZSS_L1S;
                case UBX_SIGID_QZSS_L2CM: /* FALLTHROUGH */
                case UBX_SIGID_QZSS_L2CL: return Signal::QZSS_L2C;
                case UBX_SIGID_QZSS_L5I: /* FALLTHROUGH */
                case UBX_SIGID_QZSS_L5Q:  return Signal::QZSS_L5;
            }
            break;
        case UBX_GNSSID_GLO :
            switch (sigId) {
                case UBX_SIGID_GLO_L1OF:  return Signal::GLO_L1OF;
                case UBX_SIGID_GLO_L2OF:  return Signal::GLO_L2OF;
            }
            break;
        case UBX_GNSSID_NAVIC:
            switch (sigId) {
                case UBX_SIGID_NAVIC_L5A: return Signal::NAVIC_L5A;
            }
            break;
    }  // clang-format on
    return Signal::UNKNOWN;
}

/* ****************************************************************************************************************** */
}  // namespace gnss
}  // namespace common
}  // namespace fpsdk
