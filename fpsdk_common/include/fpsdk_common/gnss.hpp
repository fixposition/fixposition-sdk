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
 *
 * @page FPSDK_COMMON_GNSS GNSS types and utilities
 *
 * **API**: fpsdk_common/gnss.hpp and fpsdk::common::gnss
 *
 */
#ifndef __FPSDK_COMMON_GNSS_HPP__
#define __FPSDK_COMMON_GNSS_HPP__

/* LIBC/STL */
#include <cstdint>

/* EXTERNAL */

/* PACKAGE */
#include "types.hpp"

namespace fpsdk {
namespace common {
/**
 * @brief GNSS types and utilities
 */
namespace gnss {
/* ****************************************************************************************************************** */

/**
 * @brief GNSS fix types
 */
enum class GnssFixType : uint8_t
{  // clang-format off
    UNKNOWN        =  0,  //!< Unknown fix
    NOFIX          =  1,  //!< No fix
    DRONLY         =  2,  //!< Dead-reckoning only fix
    TIME           =  3,  //!< Time only fix
    SPP_2D         =  4,  //!< 2D fix
    SPP_3D         =  5,  //!< 3D fix
    SPP_3D_DR      =  6,  //!< 3D + dead-reckoning fix
    RTK_FLOAT      =  7,  //!< RTK float fix (implies 3D fix)
    RTK_FIXED      =  8,  //!< RTK fixed fix (implies 3D fix)
    RTK_FLOAT_DR   =  9,  //!< RTK float fix + dead-reckoning (implies RTK_FLOAT fix)
    RTK_FIXED_DR   = 10,  //!< RTK fixed fix + dead-reckoning (implies RTK_FIXED fix)
};  // clang-format on

/**
 * @brief Stringify GNSS fix type
 *
 * @param[in]  fix_type  The fix type
 *
 * @returns a concise and unique string for the fix types, "?" for bad values
 */
const char* GnssFixTypeStr(const GnssFixType fix_type);

/**
 * @brief GNSS
 */
enum class Gnss : uint8_t
{  // clang-format off
    UNKNOWN = 0,    //!< Unknown/unspecified GNSS
    GPS     = 'G',  //!< GPS
    SBAS    = 'S',  //!< SBAS
    GAL     = 'E',  //!< Galileo
    BDS     = 'C',  //!< BeiDou
    QZSS    = 'J',  //!< QZSS
    GLO     = 'R',  //!< GLONASS
    NAVIC   = 'I',  //!< NavIC (IRNSS)
};  // clang-format on

/**
 * @brief Stringify GNSS
 *
 * @param[in]  gnss  The GNSS
 *
 * @returns a concise and unique string for the GNSS, "?" for bad values
 */
const char* GnssStr(const Gnss gnss);

/**
 * @brief Signals
 */
enum class Signal : uint8_t
{  // clang-format off
    UNKNOWN = 0,  //!< Unknown/unspecified signal
    BDS_B1C,      //!< BeiDou B1c signal    (B1 Cp and B1 Cd)
    BDS_B1I,      //!< BeiDou B1I signal    (B1I D1 and B1I D2)
    BDS_B2A,      //!< BeiDou B2a signal    (B2 ap and B2 ad)
    BDS_B2I,      //!< BeiDou B2I signal    (B2I D1 and B2I D2)
    BDS_B3I,      //!< BeiDou B3I signal    (B3I D1 and B3I D2)
    GAL_E1,       //!< Galileo E1 signal    (E1 C and E1 B)
    GAL_E5A,      //!< Galileo E5a signal   (E5 aI and E5 aQ)
    GAL_E5B,      //!< Galileo E5b signal   (E5 bI and E5 bQ)
    GAL_E6,       //!< Galileo E6 signal    (E6A, E6B, adn E6C)
    GLO_L1OF,     //!< GLONASS L1 OF signal
    GLO_L2OF,     //!< GLONASS L2 OF signal
    GPS_L1CA,     //!< GPS L1 C/A signal
    GPS_L2C,      //!< GPS L2 C signal      (L2 CL and L2 CM)
    GPS_L5,       //!< GPS L5 signal        (L5 I and L5 Q)
    QZSS_L1CA,    //!< QZSS L1 C/A signal
    QZSS_L1S,     //!< QZSS L1 S (SAIF) signal
    QZSS_L2C,     //!< QZSS L2 C signal     (L2 CL and L2 CM)
    QZSS_L5,      //!< QZSS L5 signal       (L5 I and L5 Q)
    SBAS_L1CA,    //!< SBAS L1 C/A signal
    NAVIC_L5A,    //!< NavIC L5 A
};  // clang-format on

/**
 * @brief Stringify signal
 *
 * @param[in]  signal  The signal
 * @param[in]  kurz   Get short string (for example, "L1CA" instead of "GPS_L1CA")
 *
 * @returns a concise and unique string for the signal, "?" for bad values
 */
const char* SignalStr(const Signal signal, const bool kurz = false);

/**
 * @brief Frequency bands
 */
enum class Band : uint8_t
{
    UNKNOWN = 0,  //!< Unknown/unspecified band
    L1,           //!< L1 band (~1.5GHz)
    L2,           //!< L2 band (~1.2GHz)
    E6,           //!< E6 band (~1.3GHz)
    L5,           //!< L5 band (~1.1GHz)
};

/**
 * @brief Stringify frequency band
 *
 * @param[in]  band  The frequency band
 *
 * @returns a concise and unique string for the frequency band, "?" for bad values
 */
const char* BandStr(const Band band);

/**
 * @brief Get frequency band for a signal
 *
 * @param[in]  signal   The signal
 *
 * @returns the frequency band for the signal
 */
Band SignalToBand(const Signal signal);

/**
 * @brief Satellite number (within a GNSS)
 */
using SvNr = uint8_t;

// Number of satellites per constellation, see https://igs.org/mgex/constellations/. Satellite numbers are two digits
// satellite numbers as defined by IGS (see RINEX v3.04 section 3.5).
// clang-format off
static constexpr SvNr NUM_GPS      = 32;  //!< Number of GPS satellites (G01-G32, PRN)
static constexpr SvNr NUM_SBAS     = 39;  //!< Number of SBAS satellites (S20-S59, PRN - 100)
static constexpr SvNr NUM_GAL      = 36;  //!< Number of Galileo satellites (E01-E36, PRN)
static constexpr SvNr NUM_BDS      = 63;  //!< Number of BeiDou satellites (C01-C63, PRN)
static constexpr SvNr NUM_QZSS     = 10;  //!< Number of QZSS satellites (J01-J10, PRN - 192)
static constexpr SvNr NUM_GLO      = 32;  //!< Number of GLONASS satellites (R01-R32, slot)
static constexpr SvNr NUM_NAVIC    = 14;  //!< Number of NavIC satellites (I01-I14, PRN)
static constexpr SvNr FIRST_GPS    =  1;  //!< First GPS satellite number
static constexpr SvNr FIRST_SBAS   = 20;  //!< First SBAS satellite number
static constexpr SvNr FIRST_GAL    =  1;  //!< First Galileo satellite number
static constexpr SvNr FIRST_BDS    =  1;  //!< First BeiDou satellite number
static constexpr SvNr FIRST_QZSS   =  1;  //!< First QZSS satellite number
static constexpr SvNr FIRST_GLO    =  1;  //!< First GLONASS satellite number
static constexpr SvNr FIRST_NAVIC  =  1;  //!< First NavIC satellite number
static constexpr SvNr INAVLID_SVNR =  0;  //!< Invalid satellite number (in any GNSS)
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Satellite ("sat"), consisting of GNSS and satellite number, suitable for indexing and sorting
 */
using Sat = uint16_t;

/**
 * @brief Get "sat" from GNSS and satellite ID
 *
 * @note This does not do any range checking and it's up to the user to provide a valid satellite ID for the given GNSS.
 *
 * @param[in]  gnss  GNSS
 * @param[in]  svnr  Satellite ID
 *
 * @returns the "sat"
 */
static constexpr Sat GnssSvNrToSat(const Gnss gnss, const SvNr svnr)
{
    return ((Sat)types::EnumToVal(gnss) << 8) | (Sat)svnr;
}

/**
 * @brief Invalid "sat"
 *
 * @note This is not the only invalid combination of GNSS and satellite number!
 */
static constexpr Sat INVALID_SAT = GnssSvNrToSat(Gnss::UNKNOWN, INAVLID_SVNR);

/**
 * @brief Get GNSS from "sat"
 *
 * @param[in]  sat  The "sat"
 *
 * @returns the GNSS
 */
static constexpr Gnss SatToGnss(const Sat sat)
{
    return (Gnss)((sat >> 8) & 0xff);
}

/**
 * @brief Get satellite nr from "sat"
 *
 * @param[in]  sat  The "sat"
 *
 * @returns the satellite nr
 */
static constexpr SvNr SatToSvNr(const Sat sat)
{
    return sat & 0xff;
}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Satellite plus frequency band and signal ("satsig"), suitable for indexing and sorting
 */
using SatSig = uint32_t;

/**
 * @brief Get "satsig" from components
 *
 * @note This does not do any range checking and it's up to the user to provide a valid satellite ID for the given GNSS.
 *
 * @param[in]  gnss    GNSS
 * @param[in]  svnr    Satellite ID
 * @param[in]  band    Frequency band
 * @param[in]  signal  Signal
 *
 * @returns the "satsig"
 */
static constexpr SatSig GnssSvNrBandSignalToSatSig(
    const Gnss gnss, const SvNr svnr, const Band band, const Signal signal)
{
    return ((SatSig)types::EnumToVal(gnss) << 24) | ((SatSig)svnr << 16) | ((SatSig)band << 8) | (SatSig)signal;
}

/**
 * @brief Invalid "satsig""
 *
 * @note This is not the only invalid combination of GNSS, satellite number, band and signal!
 */
static constexpr SatSig INVALID_SATSIG =
    GnssSvNrBandSignalToSatSig(Gnss::UNKNOWN, INAVLID_SVNR, Band::UNKNOWN, Signal::UNKNOWN);

/**
 * @brief Get GNSS from "satsig"
 *
 * @param[in]  satsig  The "satsig"
 *
 * @returns the GNSS
 */
static constexpr Gnss SatSigToGnss(const SatSig satsig)
{
    return (Gnss)((satsig >> 24) & 0xff);
}

/**
 * @brief Get satellite nr from "satsig"
 *
 * @param[in]  satsig  The "satsig"
 *
 * @returns the satellite nr
 */
static constexpr SvNr SatSigToSvNr(const SatSig satsig)
{
    return (SvNr)((satsig >> 16) & 0xff);
}

/**
 * @brief Get frequency band from "satsig"
 *
 * @param[in]  satsig  The "satsig"
 *
 * @returns the frequency band
 */
static constexpr Band SatSigToBand(const SatSig satsig)
{
    return (Band)((satsig >> 8) & 0xff);
}

/**
 * @brief Get signal from "satsig"
 *
 * @param[in]  satsig  The "satsig"
 *
 * @returns the signal
 */
static constexpr Signal SatSigToSignal(const SatSig satsig)
{
    return (Signal)(satsig & 0xff);
}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Stringify satellite
 *
 * @param[in]  sat  The satellite
 *
 * @returns a unique string identifying the satellite
 */
const char* SatStr(const Sat sat);

/**
 * @brief Satellite from string
 *
 * @param[in]  str  The string ("G03", "R22", "C12", ...)
 *
 * @returns the satellite (INVALID_SAT if str was invalid)
 */
Sat StrSat(const char* str);

/**
 * @brief Convert UBX gnssId to GNSS
 *
 * @param[in]  gnssId  UBX gnssId
 *
 * @returns the GNSS
 */
Gnss UbxGnssIdToGnss(const uint8_t gnssId);

/**
 * @brief Convert UBX gnssId and svId to satellite
 *
 * @param[in]  gnssId   UBX gnssId
 * @param[in]  svId     UBX svId
 *
 * @returns the satellite, INVALID_SAT for invalid gnssId/svId
 */
Sat UbxGnssIdSvIdToSat(const uint8_t gnssId, const uint8_t svId);

/**
 * @brief Convert UBX gnssId and sigId to signal
 *
 * @param[in]  gnssId  UBX gnssId
 * @param[in]  sigId   UBX sigId
 *
 * @returns the signal
 */
Signal UbxGnssIdSigIdToSignal(const uint8_t gnssId, const uint8_t sigId);

/* ****************************************************************************************************************** */
}  // namespace gnss
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_GNSS_HPP__
