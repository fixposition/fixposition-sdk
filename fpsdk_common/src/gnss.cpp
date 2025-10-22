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

const char* FixTypeStr(const FixType fix_type)
{
    switch (fix_type) {  // clang-format off
        case FixType::UNKNOWN:         return "UNKNOWN";
        case FixType::NOFIX:           return "NOFIX";
        case FixType::DRONLY:          return "DRONLY";
        case FixType::TIME:            return "TIME";
        case FixType::SPP_2D:          return "SPP_2D";
        case FixType::SPP_3D:          return "SPP_3D";
        case FixType::SPP_3D_DR:       return "SPP_3D_DR";
        case FixType::RTK_FLOAT:       return "RTK_FLOAT";
        case FixType::RTK_FIXED:       return "RTK_FIXED";
        case FixType::RTK_FLOAT_DR:    return "RTK_FLOAT_DR";
        case FixType::RTK_FIXED_DR:    return "RTK_FIXED_DR";
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

char GnssChar(const Gnss gnss)
{
    switch (gnss) {  // clang-format off
        case Gnss::UNKNOWN:   return '?';
        case Gnss::GPS:       return 'G';
        case Gnss::SBAS:      return 'S';
        case Gnss::GAL:       return 'E';
        case Gnss::BDS:       return 'C';
        case Gnss::QZSS:      return 'J';
        case Gnss::GLO:       return 'R';
        case Gnss::NAVIC:     return 'I';
    }  // clang-format on

    return '?';
}

// ---------------------------------------------------------------------------------------------------------------------

const char* SignalStr(const Signal signal, const bool kurz)
{
    switch (signal) {  // clang-format off
        case Signal::UNKNOWN:    return kurz ? "UNKN"  : "UNKNOWN";
        //
        case Signal::GPS_L1CA:   return kurz ? "L1CA"  : "GPS_L1CA";
        case Signal::GPS_L2C:    return kurz ? "L2C"   : "GPS_L2C";
        case Signal::GPS_L5:     return kurz ? "L5"    : "GPS_L5";
        //
        case Signal::SBAS_L1CA:  return kurz ? "L1CA"  : "SBAS_L1CA";
        //
        case Signal::GAL_E1:     return kurz ? "E1"    : "GAL_E1";
        case Signal::GAL_E6:     return kurz ? "E6"    : "GAL_E6";
        case Signal::GAL_E5A:    return kurz ? "E5A"   : "GAL_E5A";
        case Signal::GAL_E5B:    return kurz ? "E5B"   : "GAL_E5B";
        //
        case Signal::BDS_B1C:    return kurz ? "B1C"   : "BDS_B1C";
        case Signal::BDS_B1I:    return kurz ? "B1I"   : "BDS_B1I";
        case Signal::BDS_B3I:    return kurz ? "B3I"   : "BDS_B3I";
        case Signal::BDS_B2I:    return kurz ? "B2I"   : "BDS_B2I";
        case Signal::BDS_B2B:    return kurz ? "B2B"   : "BDS_B2B";
        case Signal::BDS_B2A:    return kurz ? "B2A"   : "BDS_B2A";
        //
        case Signal::QZSS_L1CA:  return kurz ? "L1CA"  : "QZSS_L1CA";
        case Signal::QZSS_L1S:   return kurz ? "L1S"   : "QZSS_L1S";
        case Signal::QZSS_L2C:   return kurz ? "L2C"   : "QZSS_L2C";
        case Signal::QZSS_L5:    return kurz ? "L5"    : "QZSS_L5";
        //
        case Signal::GLO_L1OF:   return kurz ? "L1OF"  : "GLO_L1OF";
        case Signal::GLO_L2OF:   return kurz ? "L2OF"  : "GLO_L2OF";
        //
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
        case Band::E6:      return "E6";
        case Band::L2:      return "L2";
        case Band::L5:      return "L5";
    }  // clang-format on

    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* SigUseStr(const SigUse use)
{
    switch (use) {  // clang-format off
        case SigUse::UNKNOWN:   return "UNKNOWN";
        case SigUse::NONE:      return "NONE";
        case SigUse::SEARCH:    return "SEARCH";
        case SigUse::ACQUIRED:  return "ACQUIRED";
        case SigUse::UNUSABLE:  return "UNUSABLE";
        case SigUse::CODELOCK:  return "CODELOCK";
        case SigUse::CARRLOCK:  return "CARRLOCK";
        }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* SigCorrStr(const SigCorr corr)
{
    switch (corr) {  // clang-format off
        case SigCorr::UNKNOWN:    return "UNKNOWN";
        case SigCorr::NONE:       return "NONE";
        case SigCorr::SBAS:       return "SBAS";
        case SigCorr::BDS:        return "BDS";
        case SigCorr::RTCM2:      return "RTCM2";
        case SigCorr::RTCM3_OSR:  return "RTCM3_OSR";
        case SigCorr::RTCM3_SSR:  return "RTCM3_SSR";
        case SigCorr::QZSS_SLAS:  return "QZSS_SLAS";
        case SigCorr::SPARTN:     return "SPARTN";
        }  // clang-format on
    return "?";
}
// ---------------------------------------------------------------------------------------------------------------------

const char* SigIonoStr(const SigIono iono)
{
    switch (iono) {  // clang-format off
        case SigIono::UNKNOWN:    return "UNKNOWN";
        case SigIono::NONE:       return "NONE";
        case SigIono::KLOB_GPS:   return "KLOB_GPS";
        case SigIono::KLOB_BDS:   return "KLOB_BDS";
        case SigIono::SBAS:       return "SBAS";
        case SigIono::DUAL_FREQ:  return "DUAL_FREQ";
        }  // clang-format on
    return "?";
}
// ---------------------------------------------------------------------------------------------------------------------

const char* SigHealthStr(const SigHealth health)
{
    switch (health) {  // clang-format off
        case SigHealth::UNKNOWN:    return "UNKNOWN";
        case SigHealth::HEALTHY:    return "HEALTHY";
        case SigHealth::UNHEALTHY:  return "UNHEALTHY";
    }  // clang-format on
    return "?";
}
// ---------------------------------------------------------------------------------------------------------------------

const char* SatOrbStr(const SatOrb orb)
{
    switch (orb) {  // clang-format off
        case SatOrb::UNKNOWN:  return "UNKNOWN";
        case SatOrb::NONE:     return "NONE";
        case SatOrb::EPH:      return "EPH";
        case SatOrb::ALM:      return "ALM";
        case SatOrb::PRED:     return "PRED";
        case SatOrb::OTHER:    return "OTHER";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

Band SignalToBand(const Signal signal)
{
    switch (signal) {  // clang-format off
        // L1 (E1)
        case Signal::GPS_L1CA:
        case Signal::SBAS_L1CA:  /* FALLTHROUGH */
        case Signal::GAL_E1:     /* FALLTHROUGH */
        case Signal::BDS_B1C:    /* FALLTHROUGH */
        case Signal::BDS_B1I:    /* FALLTHROUGH */
        case Signal::QZSS_L1CA:  /* FALLTHROUGH */
        case Signal::QZSS_L1S:   /* FALLTHROUGH */
        case Signal::GLO_L1OF:   return Band::L1;
        // E6
        case Signal::BDS_B3I:    /* FALLTHROUGH */
        case Signal::GAL_E6:     return Band::E6;
        // L2
        case Signal::GPS_L2C:    /* FALLTHROUGH */
        case Signal::GAL_E5B:    /* FALLTHROUGH */
        case Signal::BDS_B2I:    /* FALLTHROUGH */
        case Signal::BDS_B2B:    /* FALLTHROUGH */
        case Signal::QZSS_L2C:   /* FALLTHROUGH */
        case Signal::GLO_L2OF:   return Band::L2;
        // L5 (E5)
        case Signal::GPS_L5:     /* FALLTHROUGH */
        case Signal::GAL_E5A:    /* FALLTHROUGH */
        case Signal::BDS_B2A:    /* FALLTHROUGH */
        case Signal::QZSS_L5:    /* FALLTHROUGH */
        case Signal::NAVIC_L5A:  return Band::L5;
        case Signal::UNKNOWN:    break;
    }  // clang-format on
    return Band::UNKNOWN;
}

// ---------------------------------------------------------------------------------------------------------------------

Gnss SignalToGnss(const Signal signal)
{
    switch (signal) {  // clang-format off
        case Signal::GPS_L1CA:   /* FALLTHROUGH */
        case Signal::GPS_L2C:    /* FALLTHROUGH */
        case Signal::GPS_L5:     return Gnss::GPS;
        //
        case Signal::SBAS_L1CA:  return Gnss::SBAS;
        //
        case Signal::GAL_E1:     /* FALLTHROUGH */
        case Signal::GAL_E6:     /* FALLTHROUGH */
        case Signal::GAL_E5B:    /* FALLTHROUGH */
        case Signal::GAL_E5A:    return Gnss::GAL;
        //
        case Signal::BDS_B1C:    /* FALLTHROUGH */
        case Signal::BDS_B1I:    /* FALLTHROUGH */
        case Signal::BDS_B3I:    /* FALLTHROUGH */
        case Signal::BDS_B2I:    /* FALLTHROUGH */
        case Signal::BDS_B2B:    /* FALLTHROUGH */
        case Signal::BDS_B2A:    return Gnss::BDS;
        //
        case Signal::QZSS_L1CA:  /* FALLTHROUGH */
        case Signal::QZSS_L1S:   /* FALLTHROUGH */
        case Signal::QZSS_L2C:   /* FALLTHROUGH */
        case Signal::QZSS_L5:    return Gnss::QZSS;
        //
        case Signal::GLO_L1OF:   /* FALLTHROUGH */
        case Signal::GLO_L2OF:   return Gnss::GLO;
        //
        case Signal::NAVIC_L5A:  return Gnss::NAVIC;
        //
        case Signal::UNKNOWN:    break;
    }  // clang-format on
    return Gnss::UNKNOWN;
}

// ---------------------------------------------------------------------------------------------------------------------

static_assert(sizeof(Sat) == sizeof(uint16_t), "");
static_assert(sizeof(SatSig) == sizeof(uint32_t), "");
static_assert(types::EnumToVal(Gnss::UNKNOWN) == 0, "");
static_assert(types::EnumToVal(Band::UNKNOWN) == 0, "");
static_assert(types::EnumToVal(Signal::UNKNOWN) == 0, "");
static_assert(INVALID_SAT.id_ == 0x0000, "");
static_assert(INVALID_SATSIG.id_ == 0x00000000, "");

// ---------------------------------------------------------------------------------------------------------------------

const char* Sat::GetStr() const
{
    const Gnss gnss = GetGnss();
    const SvNr svnr = GetSvNr();

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

Sat::Sat(const char* str)
{
    char c = '\0';
    uint8_t n = 0;
    int len = 0;
    if ((std::sscanf(str, "%c%" SCNu8 "%n", &c, &n, &len) == 2) && (len == 3)) {  // clang-format off
        if      ((c == 'G' /* Gnss::GPS   */) && (n >= FIRST_GPS)   && (n < (FIRST_GPS   + NUM_GPS)))   { *this = { Gnss::GPS,   (SvNr)n }; }
        else if ((c == 'S' /* Gnss::SBAS  */) && (n >= FIRST_SBAS)  && (n < (FIRST_SBAS  + NUM_SBAS)))  { *this = { Gnss::SBAS,  (SvNr)n }; }
        else if ((c == 'E' /* Gnss::GAL   */) && (n >= FIRST_GAL)   && (n < (FIRST_GAL   + NUM_GAL)))   { *this = { Gnss::GAL,   (SvNr)n }; }
        else if ((c == 'C' /* Gnss::BDS   */) && (n >= FIRST_BDS)   && (n < (FIRST_BDS   + NUM_BDS)))   { *this = { Gnss::BDS,   (SvNr)n }; }
        else if ((c == 'J' /* Gnss::QZSS  */) && (n >= FIRST_QZSS)  && (n < (FIRST_QZSS  + NUM_QZSS)))  { *this = { Gnss::QZSS,  (SvNr)n }; }
        else if ((c == 'R' /* Gnss::GLO   */) && (n >= FIRST_GLO)   && (n < (FIRST_GLO   + NUM_GLO)))   { *this = { Gnss::GLO,   (SvNr)n }; }
        else if ((c == 'I' /* Gnss::NAVIC */) && (n >= FIRST_NAVIC) && (n < (FIRST_NAVIC + NUM_NAVIC))) { *this = { Gnss::NAVIC, (SvNr)n }; }
    }  // clang-format on
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
                return { Gnss::GPS, (SvNr)(svId - UBX_FIRST_GPS + FIRST_GPS) };
            }
            break;
        case UBX_GNSSID_SBAS:
            if ((svId >= UBX_FIRST_SBAS) && (svId < (UBX_FIRST_SBAS + UBX_NUM_SBAS))) {
                return { Gnss::SBAS, (SvNr)(svId - UBX_FIRST_SBAS + FIRST_SBAS) };
            }
            break;
        case UBX_GNSSID_GAL:
            if ((svId >= UBX_FIRST_GAL) && (svId < (UBX_FIRST_GAL + UBX_NUM_GAL))) {
                return { Gnss::GAL, (SvNr)(svId - UBX_FIRST_GAL + FIRST_GAL) };
            }
            break;
        case UBX_GNSSID_BDS:
            if ((svId >= UBX_FIRST_BDS) && (svId < (UBX_FIRST_BDS + UBX_NUM_BDS))) {
                return { Gnss::BDS, (SvNr)(svId - UBX_FIRST_BDS + FIRST_BDS) };
            }
            break;
        case UBX_GNSSID_QZSS:
            if ((svId >= UBX_FIRST_QZSS) && (svId < (UBX_FIRST_QZSS + UBX_NUM_QZSS))) {
                return { Gnss::QZSS, (SvNr)(svId - UBX_FIRST_QZSS + FIRST_QZSS) };
            }
            break;
        case UBX_GNSSID_GLO:
            if ((svId >= UBX_FIRST_GLO) && (svId < (UBX_FIRST_GLO + UBX_NUM_GLO))) {
                return { Gnss::GLO, (SvNr)(svId - UBX_FIRST_GLO + FIRST_GLO) };
            }
            break;
        case UBX_GNSSID_NAVIC:
            if ((svId >= UBX_FIRST_NAVIC) && (svId < (UBX_FIRST_NAVIC + UBX_NUM_NAVIC))) {
                return { Gnss::NAVIC, (SvNr)(svId - UBX_FIRST_NAVIC + FIRST_NAVIC) };
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

// ---------------------------------------------------------------------------------------------------------------------

FixType NmeaQualityGgaToFixType(const parser::nmea::NmeaQualityGga quality)
{
    switch (quality) {  // clang-format off
        case parser::nmea::NmeaQualityGga::NOFIX:       /* FALLTHROUGH */
        case parser::nmea::NmeaQualityGga::MANUAL:      return FixType::NOFIX;
        case parser::nmea::NmeaQualityGga::SPP:         /* FALLTHROUGH */
        case parser::nmea::NmeaQualityGga::SIM:         /* FALLTHROUGH */
        case parser::nmea::NmeaQualityGga::DGNSS:       return FixType::SPP_3D;
        case parser::nmea::NmeaQualityGga::PPS:         return FixType::TIME;
        case parser::nmea::NmeaQualityGga::RTK_FIXED:   return FixType::RTK_FIXED;
        case parser::nmea::NmeaQualityGga::RTK_FLOAT:   return FixType::RTK_FLOAT;
        case parser::nmea::NmeaQualityGga::ESTIMATED:   return FixType::DRONLY;
        case parser::nmea::NmeaQualityGga::UNSPECIFIED: break;
    }  // clang-format on
    return FixType::UNKNOWN;
}

FixType NmeaModeRmcGnsToFixType(const parser::nmea::NmeaModeRmcGns mode)
{
    switch (mode) {  // clang-format off
        case parser::nmea::NmeaModeRmcGns::INVALID:     /* FALLTHROUGH */
        case parser::nmea::NmeaModeRmcGns::MANUAL:      return FixType::NOFIX;
        case parser::nmea::NmeaModeRmcGns::AUTONOMOUS:  /* FALLTHROUGH */
        case parser::nmea::NmeaModeRmcGns::DGNSS:       /* FALLTHROUGH */
        case parser::nmea::NmeaModeRmcGns::PRECISE:     /* FALLTHROUGH */
        case parser::nmea::NmeaModeRmcGns::SIM:         return FixType::SPP_3D;
        case parser::nmea::NmeaModeRmcGns::ESTIMATED:   return FixType::DRONLY;
        case parser::nmea::NmeaModeRmcGns::RTK_FIXED:   return FixType::RTK_FIXED;
        case parser::nmea::NmeaModeRmcGns::RTK_FLOAT:   return FixType::RTK_FLOAT;
        case parser::nmea::NmeaModeRmcGns::UNSPECIFIED: break;
    }  // clang-format on
    return FixType::UNKNOWN;
}

FixType NmeaModeGllVtgToFixType(const parser::nmea::NmeaModeGllVtg mode)
{
    switch (mode) {  // clang-format off
        case parser::nmea::NmeaModeGllVtg::INVALID:     /* FALLTHROUGH */
        case parser::nmea::NmeaModeGllVtg::MANUAL:      return FixType::NOFIX;
        case parser::nmea::NmeaModeGllVtg::AUTONOMOUS:  /* FALLTHROUGH */
        case parser::nmea::NmeaModeGllVtg::DGNSS:       /* FALLTHROUGH */
        case parser::nmea::NmeaModeGllVtg::SIM:         return FixType::SPP_3D;
        case parser::nmea::NmeaModeGllVtg::ESTIMATED:   return FixType::DRONLY;
        case parser::nmea::NmeaModeGllVtg::UNSPECIFIED: break;
    }  // clang-format on
    return FixType::UNKNOWN;
}

// ---------------------------------------------------------------------------------------------------------------------

Sat NmeaSystemIdSvIdToSat(
    const parser::nmea::NmeaSystemId systemId, const uint8_t svId, const parser::nmea::NmeaVersion nmeaVersion)
{
    using namespace parser::nmea;

    switch (systemId) {
        case NmeaSystemId::GPS_SBAS:
            if ((svId >= nmeaVersion.svid_min_gps) && (svId <= nmeaVersion.svid_max_gps)) {
                return { Gnss::GPS, (SvNr)(svId - nmeaVersion.svid_min_gps + FIRST_GPS) };
            }
            if ((svId >= nmeaVersion.svid_min_sbas) && (svId <= nmeaVersion.svid_max_sbas)) {
                return { Gnss::SBAS, (SvNr)(svId - nmeaVersion.svid_min_sbas + FIRST_SBAS) };
            }
            break;
        case NmeaSystemId::GLO:
            if ((svId >= nmeaVersion.svid_min_glo) && (svId <= nmeaVersion.svid_max_glo)) {
                return { Gnss::GLO, (SvNr)(svId - nmeaVersion.svid_min_glo + FIRST_GLO) };
            }
            break;
        case NmeaSystemId::GAL:
            if ((svId >= nmeaVersion.svid_min_gal) && (svId <= nmeaVersion.svid_max_gal)) {
                return { Gnss::GAL, (SvNr)(svId - nmeaVersion.svid_min_gal + FIRST_GAL) };
            }
            break;
        case NmeaSystemId::BDS:
            if ((svId >= nmeaVersion.svid_min_bds) && (svId <= nmeaVersion.svid_max_bds)) {
                return { Gnss::BDS, (SvNr)(svId - nmeaVersion.svid_min_bds + FIRST_BDS) };
            }
            break;
        case NmeaSystemId::QZSS:
            if ((svId >= nmeaVersion.svid_min_qzss) && (svId <= nmeaVersion.svid_max_qzss)) {
                return { Gnss::QZSS, (SvNr)(svId - nmeaVersion.svid_min_qzss + FIRST_QZSS) };
            }
            break;
        case NmeaSystemId::NAVIC:
            if ((svId >= nmeaVersion.svid_min_navic) && (svId <= nmeaVersion.svid_max_navic)) {
                return { Gnss::NAVIC, (SvNr)(svId - nmeaVersion.svid_min_navic + FIRST_NAVIC) };
            }
            break;
        case NmeaSystemId::UNSPECIFIED:
            break;
    }
    return INVALID_SAT;
}

// ---------------------------------------------------------------------------------------------------------------------

Signal NmeaSignalIdToSignal(const parser::nmea::NmeaSignalId signalId)
{
    using namespace parser::nmea;
    switch (signalId) {  // clang-format off
        case NmeaSignalId::GPS_L1CA:      return Signal::GPS_L1CA;
        case NmeaSignalId::GPS_L2CL:      /* FALLTHROUGH */
        case NmeaSignalId::GPS_L2CM:      return Signal::GPS_L2C;
        case NmeaSignalId::GPS_L5I:       /* FALLTHROUGH */
        case NmeaSignalId::GPS_L5Q:       return Signal::GPS_L5;
        case NmeaSignalId::GLO_L1OF:      return Signal::GLO_L1OF;
        case NmeaSignalId::GLO_L2OF:      return Signal::GLO_L2OF;
        case NmeaSignalId::GAL_E1:        return Signal::GAL_E1;
        case NmeaSignalId::GAL_E5A:       return Signal::GAL_E5A;
        case NmeaSignalId::GAL_E5B:       return Signal::GAL_E5B;
        case NmeaSignalId::GAL_E6BC:      /* FALLTHROUGH */
        case NmeaSignalId::GAL_E6A:       return Signal::GAL_E6;
        case NmeaSignalId::BDS_B1ID:      return Signal::BDS_B1I;
        case NmeaSignalId::BDS_B2ID:      return Signal::BDS_B2I;
        case NmeaSignalId::BDS_B1C:       return Signal::BDS_B1C;
        case NmeaSignalId::BDS_B2A:       return Signal::BDS_B2A;
        case NmeaSignalId::BDS_B2B:       return Signal::BDS_B2B;
        case NmeaSignalId::QZSS_L1CA:     return Signal::QZSS_L1CA;
        case NmeaSignalId::QZSS_L1S:      return Signal::QZSS_L1S;
        case NmeaSignalId::QZSS_L2CM:     /* FALLTHROUGH */
        case NmeaSignalId::QZSS_L2CL:     return Signal::QZSS_L2C;
        case NmeaSignalId::QZSS_L5I:      /* FALLTHROUGH */
        case NmeaSignalId::QZSS_L5Q:      return Signal::QZSS_L5;
        case NmeaSignalId::NAVIC_L5A:     return Signal::NAVIC_L5A;
        case NmeaSignalId::UNSPECIFIED:   /* FALLTHROUGH */
        case NmeaSignalId::NONE:          break;
    } // clang-format off
    return Signal::UNKNOWN;
}

/* ****************************************************************************************************************** */
}  // namespace gnss
}  // namespace common
}  // namespace fpsdk
