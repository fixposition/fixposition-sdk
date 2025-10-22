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
#include "parser/nmea.hpp"
#include "parser/ubx.hpp"
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
enum class FixType : uint8_t
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
const char* FixTypeStr(const FixType fix_type);

/**
 * @brief GNSS
 */
enum class Gnss : uint8_t
{
    UNKNOWN = 0,  //!< Unknown/unspecified GNSS
    GPS,          //!< GPS           (G)
    SBAS,         //!< SBAS          (S)
    GAL,          //!< Galileo       (E)
    BDS,          //!< BeiDou        (C)
    QZSS,         //!< QZSS          (J)
    GLO,          //!< GLONASS       (R)
    NAVIC,        //!< NavIC (IRNSS) (I)
};

/**
 * @brief Stringify GNSS
 *
 * @param[in]  gnss  The GNSS
 *
 * @returns a concise and unique string for the GNSS ("GPS", "GAL", etc.), "?" for bad values
 */
const char* GnssStr(const Gnss gnss);

/**
 * @brief Get GNSS char
 *
 * @param[in]  gnss  The GNSS
 *
 * @returns the character for the GNSS ('G', 'E', etc.), '?' for bad values
 */
char GnssChar(const Gnss gnss);

/**
 * @brief Signals
 */
enum class Signal : uint8_t
{  // clang-format off
    UNKNOWN = 0,  //!< Unknown/unspecified signal
    // GPS
    GPS_L1CA,     //!< [L1]  GPS L1 C/A signal
    GPS_L2C,      //!< [L2]  GPS L2 C signal      (L2 CL and L2 CM)
    GPS_L5,       //!< [L5]  GPS L5 signal        (L5 I and L5 Q)
    // SBAS
    SBAS_L1CA,    //!< [L1]  SBAS L1 C/A signal
    // GAL
    GAL_E1,       //!< [L1]  Galileo E1 signal    (E1 C and E1 B)
    GAL_E6,       //!< [E6]  Galileo E6 signal    (E6A, E6B, and E6C)
    GAL_E5B,      //!< [~L2] Galileo E5b signal   (E5 bI and E5 bQ)
    GAL_E5A,      //!< [L5]  Galileo E5a signal   (E5 aI and E5 aQ)
    // BDS
    BDS_B1C,      //!< [L1]  BeiDou B1c signal    (B1 Cp and B1 Cd)
    BDS_B1I,      //!< [L1]  BeiDou B1I signal    (B1I D1 and B1I D2)
    BDS_B3I,      //!< [E6]  BeiDou B3I signal    (B3I D1 and B3I D2)
    BDS_B2I,      //!< [~L2] BeiDou B2I signal    (B2I D1 and B2I D2)
    BDS_B2B,      //!< [~L2] BeiDou B2b signal
    BDS_B2A,      //!< [L5]  BeiDou B2a signal    (B2 ap and B2 ad)
    // QZSS
    QZSS_L1CA,    //!< [L1]  QZSS L1 C/A signal
    QZSS_L1S,     //!< [L1]  QZSS L1 S (SAIF) signal
    QZSS_L2C,     //!< [L2]  QZSS L2 C signal     (L2 CL and L2 CM)
    QZSS_L5,      //!< [L5]  QZSS L5 signal       (L5 I and L5 Q)
    // GLO
    GLO_L1OF,     //!< [L1]  GLONASS L1 OF signal
    GLO_L2OF,     //!< [L2]  GLONASS L2 OF signal
    // NAVIC
    NAVIC_L5A,    //!< [L5]  NavIC L5 A
};  // clang-format on

/**
 * @brief Stringify signal
 *
 * @param[in]  signal  The signal
 * @param[in]  kurz    Get short string (for example, "L1CA" instead of "GPS_L1CA")
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
    L1,           //!< L1 (E1) band (~1.5GHz)
    E6,           //!< E6 band      (~1.3GHz)
    L2,           //!< L2 band      (~1.2GHz)
    L5,           //!< L5 (E5) band (~1.1GHz)
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
 * @brief Signal use
 */
enum class SigUse : uint8_t
{
    UNKNOWN = 0,  //!< Unknown or unspecified use of the signal
    NONE,         //!< Signal not used
    SEARCH,       //!< Signal is being searched
    ACQUIRED,     //!< Signal was acquired
    UNUSABLE,     //!< Signal tracked but unused
    CODELOCK,     //!< Signal tracked and code locked
    CARRLOCK,     //!< Signal tracked and carrier locked
};

/**
 * @brief Stringify signal use
 *
 * @param[in]  use  The signal use
 *
 * @returns a concise and unique string for the signal use, "?" for bad values
 */
const char* SigUseStr(const SigUse use);

/**
 * @brief Signal correction data availability
 */
enum class SigCorr : uint8_t
{
    UNKNOWN = 0,  //!< Unknown or unspecified corrections
    NONE,         //!< No corrections available
    SBAS,         //!< SBAS (DGNSS) corrections available
    BDS,          //!< BeiDou (DGNSS) corrections available
    RTCM2,        //!< RTCM v2 (DGNSS) corrections available
    RTCM3_OSR,    //!< RTCM v3.x OSR type RTK corrections available
    RTCM3_SSR,    //!< RTCM v3.x SSR type RTK corrections available
    QZSS_SLAS,    //!< QZSS SLAS corrections available
    SPARTN,       //!< SPARTN corrections available
};

/**
 * @brief Stringify signal correction data availability
 *
 * @param[in]  corr  The signal correction data availability
 *
 * @returns a concise and unique string for the signal correction data availability, "?" for bad values
 */
const char* SigCorrStr(const SigCorr corr);

/**
 * @brief Ionosphere corrections
 */
enum class SigIono : uint8_t
{
    UNKNOWN = 0,  //!< Unknown or unspecified corrections
    NONE,         //!< No corrections
    KLOB_GPS,     //!< GPS style Klobuchar corrections
    KLOB_BDS,     //!< BeiDou style Klobuchar corrections
    SBAS,         //!< SBAS corrections
    DUAL_FREQ,    //!< Dual frequency iono-free combination
};

/**
 * @brief Stringify ionosphere corrections
 *
 * @param[in]  iono  The ionosphere corrections
 *
 * @returns a concise and unique string for the ionosphere corrections, "?" for bad values
 */
const char* SigIonoStr(const SigIono iono);

/**
 * @brief Signal health
 */
enum class SigHealth : uint8_t
{
    UNKNOWN = 0,  //!< Unknown or unspecified health
    HEALTHY,      //!< Signal is healthy
    UNHEALTHY,    //!< Signal is unhealthy
};

/**
 * @brief Stringify signal health
 *
 * @param[in]  health  The signal health
 *
 * @returns a concise and unique string for the signal health, "?" for bad values
 */
const char* SigHealthStr(const SigHealth health);

/**
 * @brief Satellite orbit source
 */
enum class SatOrb : uint8_t  // clang-format off
{
    UNKNOWN = 0x00,  //!< Unknown
    NONE    = 0x01,  //!< No orbit available
    EPH     = 0x02,  //!< Ephemeris
    ALM     = 0x04,  //!< Almanac
    PRED    = 0x08,  //!< Predicted orbit
    OTHER   = 0x10,  //!< Other, unspecified orbit source
};  // clang-format on

/**
 * @brief Stringify satellite orbit source
 *
 * @param[in]  orb  The satellite orbit source
 *
 * @returns a concise and unique string for the satellite orbit source, "?" for bad values
 */
const char* SatOrbStr(const SatOrb orb);

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Get frequency band for a signal
 *
 * @note This mapping is in some cases a bit vague and subject to interpretation or preference. See the docu of the
 *       Signal enum.
 *
 * @param[in]  signal  The signal
 *
 * @returns the frequency band for the signal
 */
Band SignalToBand(const Signal signal);

/**
 * @brief Get GNSS for a signal
 *
 * @param[in]  signal  The signal
 *
 * @returns the GNSS for the signal
 */
Gnss SignalToGnss(const Signal signal);

// ---------------------------------------------------------------------------------------------------------------------

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
struct Sat
{  // clang-format off
    /**
     * @brief Default ctor: invalid satellite (INVALID_SAT)
     */
    Sat() = default;

    /**
     * @brief Ctor: from GNSS and satellite number
     *
     * @note This does not do any range checking and it's up to the user to provide a valid satellite ID for the given GNSS.
     *
     * @param[in]  gnss    GNSS
     * @param[in]  svnr    Satellite number
     */
    Sat(const Gnss gnss, const SvNr svnr) : id_ { (uint16_t)(((uint16_t)types::EnumToVal(gnss) << 8) | (uint16_t)svnr) } {}

    /**
     * @brief Ctor: from string
     *
     * @note Invalid strings create an INVALID_SAT
     *
     * @param[in]  str  The string ("G03", "R22", "C12", ...)
     */
    Sat(const char* str);

    /**
     * @brief Get GNSS
     *
     * @returns the GNSS
     */
    inline Gnss GetGnss() const { return (Gnss)((id_ >> 8) & 0xff); }

    /**
     * @brief Get satellite number
     *
     * @returns the satellit nuber
     */
    inline SvNr GetSvNr() const { return id_ & 0xff; }

    /**
     * @brief Stringify satellite
     *
     * @returns a unique string identifying the satellite
     */
    const char* GetStr() const;

    uint16_t id_ = 0x0000;  //!< Internal representation of Gnss+SvNr

    inline bool operator< (const Sat& rhs) const { return id_ <  rhs.id_; } //!< Less-than operator
    inline bool operator==(const Sat& rhs) const { return id_ == rhs.id_; } //!< Equal operator
    inline bool operator!=(const Sat& rhs) const { return id_ != rhs.id_; } //!< Not equal operator
    // See also std::hash<Sat> declaration at the end of this file
};  // clang-format on

/**
 * @brief Invalid "sat"
 *
 * @note This is not the only invalid combination of GNSS and satellite number!
 */
static constexpr Sat INVALID_SAT;

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Satellite plus frequency band and signal ("satsig"), suitable for indexing and sorting
 */
struct SatSig
{  // clang-format off
    /**
     * @brief Default ctor: invalid SatSig (INVALID_SATSIG)
     */
    SatSig() = default;

    /**
     * @brief Ctor: create SatSig
     *
     * @note This does not do any range checking and it's up to the user to provide a valid satellite ID for the given GNSS.
     *
     * @param[in]  gnss    GNSS
     * @param[in]  svnr    Satellite number
     * @param[in]  band    Frequency band
     * @param[in]  signal  Signal
     */
    SatSig(const Gnss gnss, const SvNr svnr, const Band band, const Signal signal) : id_ { (uint32_t)(types::EnumToVal(gnss) << 24) | (uint32_t)(svnr << 16) | (uint32_t)(types::EnumToVal(band) << 8) | (uint32_t)types::EnumToVal(signal) } {}

    /**
     * @brief Ctor: create SatSig
     *
     * @note This does not do any range checking and it's up to the user to provide a valid satellite ID for the given GNSS.
     *
     * @param[in]  sat     Satellite (GNSS + satellite number)
     * @param[in]  signal  Signal (implies band)
     */
    SatSig(const Sat sat, const Signal signal) : id_ { (uint32_t)(((uint32_t)sat.id_ << 16) | ((uint32_t)types::EnumToVal(SignalToBand(signal)) << 8) | ((uint32_t)types::EnumToVal(signal))) } {}

    /**
     * @brief Get satellite
     *
     * @returns the satellite
     */
    inline Sat GetSat() const { return { GetGnss(), GetSvNr() }; }

    /**
     * @brief Get GNSS
     *
     * @returns the GNSS
     */
    inline Gnss GetGnss() const { return (Gnss)((id_ >> 24) & 0xff); }

    /**
     * @brief Get satellite number
     *
     * @returns the satellite number
     */
    inline SvNr GetSvNr() const { return (SvNr)((id_ >> 16) & 0xff); }

    /**
     * @brief Get band
     *
     * @returns the band
     */
    inline Band GetBand() const { return (Band)((id_ >> 8) & 0xff); }

    /**
     * @brief Get signal
     *
     * @returns the signal
     */
    inline Signal GetSignal() const { return (Signal)(id_ & 0xff); }

    uint32_t id_ = 0x00000000;  //!< Internal representation of Gnss+SvNr+Band+Signal

    inline bool operator< (const SatSig& rhs) const { return id_ <  rhs.id_; } //!< Less-than operator
    inline bool operator==(const SatSig& rhs) const { return id_ == rhs.id_; } //!< Equal operator
    inline bool operator!=(const SatSig& rhs) const { return id_ != rhs.id_; } //!< Not equal operator
    // See also std::hash<SatSig> declaration at the end of this file
};  // clang-format on

static constexpr SatSig INVALID_SATSIG;  //!< Invalid SatSig

// ---------------------------------------------------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Convert NMEA GGA quality to fix type
 *
 * @param[in]  quality  NMEA GGA quality
 *
 * @returns the fix type corresponding to the quality
 */
FixType NmeaQualityGgaToFixType(const parser::nmea::NmeaQualityGga quality);

/**
 * @brief Convert NMEA RMC/GNS mode to fix type
 *
 * @param[in]  mode  NMEA RMC/GNS mode
 *
 * @returns the fix type corresponding to the mode
 */
FixType NmeaModeRmcGnsToFixType(const parser::nmea::NmeaModeRmcGns mode);

/**
 * @brief Convert NMEA GLL/VTG mode to fix type
 *
 * @param[in]  mode  NMEA GLL/VTG mode
 *
 * @returns the fix type corresponding to the mode
 */
FixType NmeaModeGllVtgToFixType(const parser::nmea::NmeaModeGllVtg mode);

/**
 * @brief Convert NMEA system ID and satellite number to satellite
 *
 * @param[in]  systemId     NMEA system ID
 * @param[in]  svId         NMEA satellite ID
 * @param[in]  nmeaVersion  NMEA version
 *
 * @returns the satellite, INVALID_SAT for invalid systemId/svId
 */
Sat NmeaSystemIdSvIdToSat(const parser::nmea::NmeaSystemId systemId, const uint8_t svId,
    const parser::nmea::NmeaVersion nmeaVersion = parser::nmea::NmeaVersion::V411);

/**
 * @brief Convert NMEA signal ID to signal
 *
 * @param[in]  signalId  NMEA signal ID
 *
 * @returns the signal
 */
Signal NmeaSignalIdToSignal(const parser::nmea::NmeaSignalId signalId);

/* ****************************************************************************************************************** */
}  // namespace gnss
}  // namespace common
}  // namespace fpsdk

/**
 * @brief Hasher for Sat (e.g. std::unordered_map)
 */
template <>
struct std::hash<fpsdk::common::gnss::Sat>
{
    std::size_t operator()(const fpsdk::common::gnss::Sat& k) const
    {
        return k.id_;
    }  //!< operator
};

/**
 * @brief Hasher for SatSig (e.g. std::unordered_map)
 */
template <>
struct std::hash<fpsdk::common::gnss::SatSig>
{
    std::size_t operator()(const fpsdk::common::gnss::SatSig& k) const
    {
        return k.id_;
    }  //!< operator
};

/* ****************************************************************************************************************** */
#endif  // __FPSDK_COMMON_GNSS_HPP__
