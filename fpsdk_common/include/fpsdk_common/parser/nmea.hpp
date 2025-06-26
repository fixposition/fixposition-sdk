/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 *
 * Based on work by flipflip (https://github.com/phkehl)
 * The information on message structures, IDs, descriptions etc. in this file are from publicly available data, such as:
 * - NMEA 0183 (https://www.nmea.org/)
 * - https://en.wikipedia.org/wiki/NMEA_0183
 * - u-blox ZED-F9P Interface Description (HPG 1.50) (https://www.u-blox.com/en/docs/UBXDOC-963802114-12815),
 *   copyright (c) 2024 u-blox AG
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Parser NMEA routines and types
 */
// clang-format off
/**
 * @page FPSDK_COMMON_PARSER_NMEA Parser NMEA routines and types
 *
 * **API**: fpsdk_common/parser/nmea.hpp and fpsdk::common::parser::nmea
 *
 * @fp_msgspec_begin{NMEA-Protocol}
 *
 * The NMEA framing and behaviour is defined by the NMEA 0183 standard (v4.11 and erratas).
 *
 * In NMEA speak messages are called *Sentences*. Frames (messages) are in this form:
 *
 * <code><b style="color: red;">$</b><b style="color: green;">Talker</b><b style="color: blue;">Formatter</b>,<em>field<sub>1</sub></em>,<em>field<sub>2</sub></em>,…,<em>field<sub>N</sub></em><b style="color: red;">\*CC</b><b style="color: red;">\\r\\n</b></code>
 *
 * Where:
 *
 * - The NMEA style framing:
 *     - <code><b style="color: red;">\$</b></code>
 *       -- Start character ("$", ASCII 36)
 *     - <code><b style="color: red;">\*CC</b></code>
 *       -- Checksum: "\*" (ASCII 42) and two digit XOR value of all payload
 *       characters in captial hexadecimal notation, for example:
 *       "FPX" = <code>'F' ^ 'P' ^ 'X' = 70 ^ 80 ^ 88 = 78 = 0x4e</code> = checksum <code>4E</code>
 *     -  <code><b style="color: red;">\\r\\n</b></code>
 *       -- Sentence termination characters (CR + LF, ASCII 13 + 10)
 * - A <code><b style="color: green;">Talker</b></code> ID -- Two capital characters:
 *     - `GP` -- Talker ID for GPS, also legacy resp. "compatibility"
 *     - `GL` -- Talker ID for GLONASS
 *     - `GA` -- Talker ID for Galileo
 *     - `GB` -- Talker ID for BeiDou
 *     - `GQ` -- Talker ID for QZSS
 *     - `GI` -- Talker ID for NavIC (IRNSS)
 *     - `GN` -- Talker ID for any combination of GNSS
 * - A <code><b style="color: blue;">Formatter</b></code> ID -- Three capital characters, for example:
 *     - `RMC` for the message containing recommended minimum specific GNSS data
 *     - See the NMEA 0183 standard document for an extensive list
 * - Data fields (payload)
 *     - <code><em>field<sub>1</sub></em>,<em>field<sub>2</sub></em>,…,<em>field<sub>N</sub></em></code>
 *       -- The structure of the message data is defined by the <code><b style="color: blue;">Formatter</b></code>.
 *       Each field can contain all printable 7-bit ASCII characters (ASCII 32–126), excluding the
 *       reserved characters `!` (ASCII 33), `$` (ASCII 36), `*` (ASCII 42), `,` (ASCII 44),
 *       `\` (ASCII 92), `~` (ASCII 126).
 * - Field separators
 *     - All fields (identifier, message type, message version, data fields) are separated by a `,` (comma, ASCII 44)
 * - Null fields
 *     - Data fields can be _null_, meaning their value is absent to indicate that no data is
 *         available. The data for null fields is the empty string. For example:
 *         - Definition: <code>…,<em>field<sub>i</sub></em>,<em>field<sub>i+1</sub></em>,<em>field<sub>i+2</sub></em>,…</code>
 *         - Values: <code><em>field<sub>i</sub></em></code> = 123, <code><em>field<sub>i+1</sub></em></code> = _null_,
 *           <code><em>field<sub>i+2</sub></em></code> = 456
 *         - Payload string: <code>…,123,,456,…</code>
 * - Data field types:
 *     - See the NMEA 0183 standard document for specifications
 *
 * @fp_msgspec_end
 *
 */
// clang-format on
#ifndef __FPSDK_COMMON_PARSER_NMEA_HPP__
#define __FPSDK_COMMON_PARSER_NMEA_HPP__

/* LIBC/STL */
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/* EXTERNAL */

/* PACKAGE */

namespace fpsdk {
namespace common {
namespace parser {
/**
 * @brief Parser NMEA routines and types
 */
namespace nmea {
/* ****************************************************************************************************************** */

static constexpr uint8_t NMEA_PREAMBLE = '$';      //!< NMEA framing preamble
static constexpr std::size_t NMEA_FRAME_SIZE = 6;  //!< NMEA frame size ("$*cc\r\n")

//! NMEA message meta data
struct NmeaMessageMeta
{
    char talker_[3] = { 0 };      //!< Talker ID (for example, "GP", "GN" or "P"), nul-terminated string
    char formatter_[20] = { 0 };  //!< Formatter (for example, "GGA", "RMC", or "UBX"), nul-terminated string
    int payload_ix0_ = 0;         //!< Index (offset) for start of payload, 0 if no payload available
    int payload_ix1_ = 0;         //!< Index (offset) for end of payload, 0 if no payload available
};

/**
 * @brief Get NMEA message meta data
 *
 * @param[out]  meta      The meta data
 * @param[in]   msg       Pointer to the NMEA message
 * @param[in]   msg_size  Size of the NMEA message (>= 11)
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a correct NMEA message.
 *
 * @returns true if the meta data was successfully extracted, false otherwise
 */
bool NmeaGetMessageMeta(NmeaMessageMeta& meta, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Get NMEA message name
 *
 * Generates a name (string) in the form "NMEA-TALKER-FORMATTER" (for example, "NMEA-GP-GGA"). Some proprietary messages
 * are recognised, for example, "NMEA-PUBX-POSITION".
 *
 * @param[out] name      String to write the name to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the NMEA message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid NMEA message.
 *
 * @returns true if message name was generated, false if \c name buffer was too small
 */
bool NmeaGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Get NMEA message info
 *
 * This stringifies the content of some NMEA messages, for debugging.
 *
 * @param[out] info      String to write the info to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the NMEA message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid NMEA message.
 *
 * @returns true if message info was generated (even if info is empty), false if \c name buffer was too small
 */
bool NmeaGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Make a NMEA message
 *
 * @param[out]  msg           The message frame
 * @param[in]   payload       The message payload, including the talker ID and formatter
 *                            (up to MAX_NMEA_SIZE - NMEA_FRAME_SIZE bytes, can be empty)
 *
 * @note Illegal (< 0x20 or > 0x7e) or reserved/special ('$', '\', '!', '~', '^' and '*') characters are replaced
 *       by a '_'.
 *
 * @returns true if the message was successfully constructed (\c msg now contains the message),
 *          false if failed contructing the message (payload too large)
 */
bool NmeaMakeMessage(std::vector<uint8_t>& msg, const std::string& payload);

/**
 * @brief Make a NMEA message
 *
 * @param[out]  msg           The message frame
 * @param[in]   payload       The message payload, including the talker ID and formatter
 *                            (up to MAX_NMEA_SIZE - NMEA_FRAME_SIZE bytes, can be empty)
 *
 * @note Illegal (< 0x20 or > 0x7e) or reserved/special ('$', '\', '!', '~', '^' and '*') characters are replaced
 *       by a '_'.
 *
 * @returns true if the message was successfully constructed (\c msg now contains the message),
 *          false if failed contructing the message (payload too large)
 */
bool NmeaMakeMessage(std::string& msg, const std::string& payload);

/**
 * @brief NMEA coordinates (integer degrees, float minutes and a sign for N/S resp. E/W)
 */
struct NmeaCoordinates
{
    /**
     * @brief Constructor
     *
     * @param[in]  degs    Decimal degrees
     * @param[in]  digits  Number of digits (0-12), param clamped to range
     */
    NmeaCoordinates(const double degs, const int digits = 5);

    int deg_;     //!< Integer degrees value, >= 0
    double min_;  //!< Fractional minutes value, >= 0.0
    bool sign_;   //!< false for negative (S or W), true for positive (N or E)
};

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief NMEA talker IDs
 */
enum class NmeaTalkerId : int
{  // clang-format off
    UNSPECIFIED = '!',  //!< Unspecified
    PROPRIETARY = 'x',  //!< Proprietary
    GPS_SBAS    = 'P',  //!< GPS and/or SBAS
    GLO         = 'L',  //!< GLONASS
    GAL         = 'A',  //!< GALILEO
    BDS         = 'B',  //!< BeiDou
    NAVIC       = 'I',  //!< NavIC
    QZSS        = 'Q',  //!< QZSS
    GNSS        = 'N',  //!< GNSS (multi-constellation)
};  // clang-format on

/**
 * @brief Stringify NMEA talker ID
 *
 * @param[in]  talker  The NMEA talker ID
 *
 * @returns the stringification of the NMEA talker ID
 */
const char* NmeaTalkerIdStr(const NmeaTalkerId talker);

/**
 * @brief NMEA formatter
 */
enum class NmeaFormatter
{
    UNSPECIFIED,  //!< Unspecified
    GGA,          //!< Formatter GGA (NmeaGgaPayload)
    GLL,          //!< Formatter GLL (NmeaGllPayload)
    RMC,          //!< Formatter RMC (NmeaRmcPayload)
    VTG,          //!< Formatter VTG (NmeaVtgPayload)
    GST,          //!< Formatter GST (NmeaGstPayload)
    HDT,          //!< Formatter HDT (NmeaHdtPayload)
    ZDA,          //!< Formatter ZDA (NmeaZdaPayload)
    GSA,          //!< Formatter GSA (NmeaGsaPayload)
    GSV,          //!< Formatter GSV (NmeaGsvPayload)
};

/**
 * @brief Stringify NMEA formatter
 *
 * @param[in]  formatter  The NMEA formatter
 *
 * @returns the stringification of the NMEA formatter
 */
const char* NmeaFormatterStr(const NmeaFormatter formatter);

/**
 * @brief NMEA-Gx-GGA quality indicator
 */
enum class NmeaQualityGga : int
{  // clang-format off
    UNSPECIFIED = '!',  //!< Unspecified
    NOFIX       = '0',  //!< No fix
    SPP         = '1',  //!< Autonomous GNSS fix
    DGNSS       = '2',  //!< Differential GPS fix (e.g. with SBAS)
    PPS         = '3',  //!< PPS mode
    RTK_FIXED   = '4',  //!< RTK fixed
    RTK_FLOAT   = '5',  //!< RTK float
    ESTIMATED   = '6',  //!< Estimated (dead reckoning only)
    MANUAL      = '7',  //!< Manual input mode
    SIM         = '8',  //!< Simulator
};  // clang-format on

/**
 * @brief Stringify NMEA-Gx-GGA quality indicator
 *
 * @param[in]  qual  The NMEA-Gx-GGA quality indicator
 *
 * @returns the stringification of the NMEA-Gx-GGA quality indicator
 */
const char* NmeaQualityGgaStr(const NmeaQualityGga qual);

/**
 * @brief NMEA-Gx-GLL and NMEA-Gx-RMC status
 *
 * @note Do not use <, >, >=, <= operators on this!
 */
enum class NmeaStatusGllRmc : int
{  // clang-format off
    UNSPECIFIED  = '!',  //!< Unspecified
    INVALID      = 'V',  //!< Data invalid
    VALID        = 'A',  //!< Data valid
 // DIFFERENTIAL = 'D',  // @todo another possible value?
};  // clang-format on

/**
 * @brief Stringify NMEA-Gx-GLL and NMEA-Gx-RMC status
 *
 * @param[in]  status  The NMEA-Gx-GLL and NMEA-Gx-RMC status
 *
 * @returns the stringification of the NMEA-Gx-GLL and NMEA-Gx-RMC status
 */
const char* NmeaStatusGllRmcStr(const NmeaStatusGllRmc status);

/**
 * @brief NMEA-Gx-GLL and NMEA-Gx-VTG pos mode
 *
 * @note Do not use <, >, >=, <= operators on this!
 */
enum class NmeaModeGllVtg : int
{  // clang-format off
    UNSPECIFIED = '!',  //!< Unspecified
    INVALID     = 'N',  //!< Invalid (no fix)
    AUTONOMOUS  = 'A',  //!< Autonomous mode (SPP)
    DGNSS       = 'D',  //!< Differential GNSS fix
    ESTIMATED   = 'E',  //!< Estimated (dead reckoning only)
    MANUAL      = 'M',  //!< Manual input mode
    SIM         = 'S',  //!< Simulator mode
};  // clang-format on

/**
 * @brief Stringify NMEA-Gx-GLL and NMEA-Gx-VTG pos mode
 *
 * @param[in]  mode  The NMEA-Gx-GLL and NMEA-Gx-VTG pos mode
 *
 * @returns the stringification of the NMEA-Gx-GLL and NMEA-Gx-VTG pos mode
 */
const char* NmeaModeGllVtgStr(const NmeaModeGllVtg mode);

/**
 * @brief NMEA-Gx-RMC and NMEA-Gx-GNS pos mode
 *
 * @note Do not use <, >, >=, <= operators on this!
 */
enum class NmeaModeRmcGns : int
{  // clang-format off
    UNSPECIFIED = '!',  //!< Unspecified
    INVALID     = 'N',  //!< Invalid (no fix)
    AUTONOMOUS  = 'A',  //!< Autonomous mode (SPP)
    DGNSS       = 'D',  //!< Differential GNSS fix
    ESTIMATED   = 'E',  //!< Estimated (dead reckoning only)
    RTK_FIXED   = 'R',  //!< RTK fixed
    RTK_FLOAT   = 'F',  //!< RTK float
    PRECISE     = 'P',  //!< Precise
    MANUAL      = 'M',  //!< Manual input mode
    SIM         = 'S',  //!< Simulator mode
};  // clang-format on

/**
 * @brief Stringify NMEA-Gx-RMC and NMEA-Gx-GNS pos mode
 *
 * @param[in]  mode  The NMEA-Gx-RMC and NMEA-Gx-GNS pos mode
 *
 * @returns the stringification of the NMEA-Gx-RMC and NMEA-Gx-GNS pos mode
 */
const char* NmeaModeRmcGnsStr(const NmeaModeRmcGns mode);

/**
 * @brief NMEA-Gx-RMC navigational status
 *
 * @note Do not use <, >, >=, <= operators on this!
 */
enum class NmeaNavStatusRmc : int
{  // clang-format off
    UNSPECIFIED = '!',  //!< Unspecified
    SAFE        = 'S',  //!< Safe
    CAUTION     = 'C',  //!< Caution
    UNSAFE      = 'U',  //!< Unsafe
    NA          = 'V',  //!< Equipment does not provide navigational status
};  // clang-format on

/**
 * @brief Stringify NMEA-Gx-RMC navigational status
 *
 * @param[in]  navstatus  The NMEA-Gx-RMC navigational status
 *
 * @returns the stringification of the NMEA-Gx-RMC navigational status
 */
const char* NmeaNavStatusRmcStr(const NmeaNavStatusRmc navstatus);

/**
 * @brief NMEA-Gx-GNS operation mode
 *
 * @note Do not use <, >, >=, <= operators on this!
 */
enum class NmeaOpModeGsa : int
{  // clang-format off
    UNSPECIFIED = '!',  //!< Unspecified
    MANUAL      = 'M',  //!< Manual
    AUTO        = 'A',  //!< Automatic
};  // clang-format on

/**
 * @brief Stringify NMEA-Gx-GNS operation mode
 *
 * @param[in]  opmode  The NMEA-Gx-GNS operation mode
 *
 * @returns the stringification of the NMEA-Gx-GNS operation mode
 */
const char* NmeaOpModeGsaStr(const NmeaOpModeGsa opmode);

/**
 * @brief NMEA-Gx-GNS nav mode
 */
enum class NmeaNavModeGsa : int
{  // clang-format off
    UNSPECIFIED = '!',  //!< Unspecified
    NOFIX       = '1',  //!< No fix
    FIX2D       = '2',  //!< 2D fix
    FIX3D       = '3',  //!< 3D fix
};  // clang-format on

/**
 * @brief Stringify NMEA-Gx-GNS nav mode
 *
 * @param[in]  navmode  The NMEA-Gx-GNS nav mode
 *
 * @returns the stringification of the NMEA-Gx-GNS nav mode
 */
const char* NmeaNavModeGsaStr(const NmeaNavModeGsa navmode);

/**
 * @brief NMEA system IDs
 *
 * @note Do not use <, >, >=, <= operators on this!
 */
enum class NmeaSystemId : int
{  // clang-format off
    UNSPECIFIED = '!',  //!< Unspecified
    GPS_SBAS    = '1',  //!< GPS and/or SBAS
    GLO         = '2',  //!< GLONASS
    GAL         = '3',  //!< Galileo
    BDS         = '4',  //!< BeiDou
    QZSS        = '5',  //!< QZSS
    NAVIC       = '6',  //!< NavIC
};  // clang-format on

/**
 * @brief Stringify NMEA system ID
 *
 * @param[in]  system  The NMEA system ID
 *
 * @returns the stringification of the NMEA system ID
 */
const char* NmeaSystemIdStr(const NmeaSystemId system);

/**
 * @brief NMEA signal IDs
 *
 * @note Do not use <, >, >=, <= operators on this!
 */
enum class NmeaSignalId : int
{  // clang-format off
    UNSPECIFIED = 0x000 + '!',  //!< Unspecified
    NONE        = 0x000 + '0',  //!< None
    // GPS (SBAS)
    GPS_L1CA    = 0x100 + '1',  //!< GPS L1 C/A or SBAS L1 C/A
    GPS_L2CL    = 0x100 + '6',  //!< GPS L2 CL
    GPS_L2CM    = 0x100 + '5',  //!< GPS L2 CM
    GPS_L5I     = 0x100 + '7',  //!< GPS L5 I
    GPS_L5Q     = 0x100 + '8',  //!< GPS L5 Q
    // GAL
    GAL_E1      = 0x300 + '7',  //!< Galileo E1
    GAL_E5A     = 0x300 + '1',  //!< Galileo E5 A
    GAL_E5B     = 0x300 + '2',  //!< Galileo E5 B
    GAL_E6BC    = 0x300 + '3',  //!< Galileo E6 B/C
    GAL_E6A     = 0x300 + '4',  //!< Galileo E6 A
    // BDS
    BDS_B1ID    = 0x400 + '1',  //!< BeiDou B1I D
    BDS_B2ID    = 0x400 + 'B',  //!< BeiDou B2I D
    BDS_B1C     = 0x400 + '3',  //!< BeiDou B1 C
    BDS_B2A     = 0x400 + '5',  //!< BeiDou B2 a
    BDS_B2B     = 0x400 + '6',  //!< BeiDou B2 b
    // QZSS
    QZSS_L1CA   = 0x500 + '1',  //!< QZSS L1 C/A
    QZSS_L1S    = 0x500 + '4',  //!< QZSS L1S
    QZSS_L2CM   = 0x500 + '5',  //!< QZSS L2 CM
    QZSS_L2CL   = 0x500 + '6',  //!< QZSS L2 CL
    QZSS_L5I    = 0x500 + '7',  //!< QZSS L5 I
    QZSS_L5Q    = 0x500 + '8',  //!< QZSS L5 Q
    // GLO
    GLO_L1OF    = 0x200 + '1',  //!< GLONASS L1 OF
    GLO_L2OF    = 0x200 + '3',  //!< GLONASS L2 OF
    // NAVIC
    NAVIC_L5A   = 0x600 + '5',  //!< NavIC L5 A
};  // clang-format on

/**
 * @brief Stringify NMEA signal IDs
 *
 * @param[in]  signal  The NMEA signal IDs
 *
 * @returns the stringification of the NMEA signal IDs
 */
const char* NmeaSignalIdStr(const NmeaSignalId signal);

/**
 * @brief NMEA time (hour, minutes, seconds)
 */
struct NmeaTime
{
    bool valid = false;  //!< Data is valid
    int hours = 0;       //!< Hours
    int mins = 0;        //!< Minutes
    double secs = 0.0;   //!< Seconds

    bool operator==(const NmeaTime& rhs) const;  //!< Equal
    bool operator!=(const NmeaTime& rhs) const;  //!< Not equal
};

/**
 * @brief NMEA date (year, month, day)
 */
struct NmeaDate
{
    bool valid = false;  //!< Data is valid
    int years = 0;       //!< Year
    int months = 0;      //!< Month
    int days = 0.0;      //!< Day

    bool operator==(const NmeaDate& rhs) const;  //!< Equal
    bool operator!=(const NmeaDate& rhs) const;  //!< Not equal
};

/**
 * @brief NMEA geodetic position
 */
struct NmeaLlh
{
    bool latlon_valid = false;  //!< Latitude/longitude is valid
    double lat = 0.0;           //!< Latitude [deg], >= 0.0 East, < 0.0 West
    double lon = 0.0;           //!< Longitude [deg], >= 0.0 North, < 0.0 South
    bool height_valid = false;  //!< Height is valid
    double height = 0.0;        //!< Ellipsoidal (!) height [m]
};

/**
 * @brief NMEA satellite (used, e.g. in GSA)
 */
struct NmeaSat
{
    bool valid = false;                               //!< Data is valid
    NmeaSystemId system = NmeaSystemId::UNSPECIFIED;  //!< System ID
    int svid = 0;                                     //!< Satellite ID (numbering cf. NMEA 0183)
};

/**
 * @brief Constants for different versions of NMEA
 */
struct NmeaVersion
{
    const int svid_min_gps = -1;            //!< Min GPS satellite ID
    const int svid_max_gps = -1;            //!< Max GPS satellite ID
    const int svid_min_sbas = -1;           //!< Min SBAS satellite ID
    const int svid_max_sbas = -1;           //!< Max SBAS satellite ID
    const int svid_min_gal = -1;            //!< Min Galileo satellite ID
    const int svid_max_gal = -1;            //!< Max Galileo satellite ID
    const int svid_min_bds = -1;            //!< Min BeiDou satellite ID
    const int svid_max_bds = -1;            //!< Max BeiDou satellite ID
    const int svid_min_glo = -1;            //!< Min GLONASS satellite ID
    const int svid_max_glo = -1;            //!< Max GLONASS satellite ID
    const int svid_min_qzss = -1;           //!< Min QZSS satellite ID, -1 if not available
    const int svid_max_qzss = -1;           //!< Max QZSS satellite ID, -1 if not available
    const int svid_min_navic = -1;          //!< Min NavIC satellite ID, -1 if not available
    const int svid_max_navic = -1;          //!< Max NavIC satellite ID, -1 if not available
    static const NmeaVersion V410;          //!< Value for NMEA v4.10
    static const NmeaVersion V410_UBX_EXT;  //!< Value for NMEA v4.10 extended (u-blox flavour)
    static const NmeaVersion V411;          //!< Value for NMEA v4.11
};

/**
 * NMEA satellite position (GSA)
 */
struct NmeaAzEl
{
    bool valid = false;                               //!< Data is valid
    NmeaSystemId system = NmeaSystemId::UNSPECIFIED;  //!< System ID
    int svid = 0;                                     //!< Satellite ID (numbering cf. NMEA 0183)
    int el = 0;                                       //!< Elevation [deg] (-90..90)
    int az = 0;                                       //!< Azimuth [deg] (0..360)
};

/**
 * NMEA signal levels (GSA)
 */
struct NmeaCno
{
    bool valid = false;                               //!< Data valid
    NmeaSystemId system = NmeaSystemId::UNSPECIFIED;  //!< System ID
    int svid = 0;                                     //!< Satellite ID (numbering cf. NMEA 0183)
    NmeaSignalId signal = NmeaSignalId::UNSPECIFIED;  //!< Signal ID
    int cno = 0;                                      //!< Signal level [dBHz]
};

/**
 * @brief NMEA integer value
 */
struct NmeaInt
{
    bool valid = false;  //!< Data is valid
    int value = 0;       //!< Value
};

/**
 * @brief NMEA float value
 */
struct NmeaFloat
{
    bool valid = false;  //!< Data is valid
    double value = 0;    //!< Value
};

/**
 * @brief NMEA payload base class
 */
struct NmeaPayload
{
    NmeaTalkerId talker_ = NmeaTalkerId::UNSPECIFIED;       //!< Talker
    NmeaFormatter formatter_ = NmeaFormatter::UNSPECIFIED;  //!< Formatter
    bool valid_ = false;               //!< Payload successfully decoded (true), or not (yet) decoded (false)
    virtual ~NmeaPayload() = default;  //!< Virtual dtor for polymorphism

    /**
     * @brief Set data from message
     *
     * @param[in]  msg       Pointer to the NMEA message
     * @param[in]  msg_size  Size of the NMEA message (>= 11)
     *
     * @returns true if sentence payload was correct and all data could be extracted (fields are now valid), or false
     *          otherwise (fields are now invalid)
     */
    virtual bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) = 0;

    /**
     * @brief Set data from message
     *
     * @param[in]  buf  The NMEA message data
     *
     * @returns true if message payload was correct and all data could be extracted (fields are now valid), or false
     *          otherwise (fields are now invalid)
     */
    inline bool SetFromBuf(const std::vector<uint8_t>& buf)
    {
        return SetFromMsg(buf.data(), buf.size());
    }
};

/**
 * @brief NMEA-Gx-GGA message payload
 */
struct NmeaGgaPayload : public NmeaPayload
{
    NmeaTime time;                                         //!< Time
    NmeaLlh llh;                                           //!< Position (with ellipsoidal height)
    NmeaFloat height_msl;                                  //!< Orthomeric height [m]
    NmeaQualityGga quality = NmeaQualityGga::UNSPECIFIED;  //!< Fix quality
    NmeaInt num_sv;                                        //!< Number of satellites used (may be limited to 12)
    NmeaFloat hdop;                                        //!< Horizontal dilution of precision (only with valid fix)
    NmeaFloat diff_age;                                    //!< Differential data age (optional, NMEA 4.11 only)
    NmeaInt diff_sta;                                      //!< Differential station ID (optional, NMEA 4.11 only)

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* FORMATTER = "GGA";  //!< Formatter
};

/**
 * @brief NMEA-Gx-GLL message payload
 */
struct NmeaGllPayload : public NmeaPayload
{
    NmeaLlh ll;                                               //!< Position (no height)
    NmeaTime time;                                            //!< Time
    NmeaStatusGllRmc status = NmeaStatusGllRmc::UNSPECIFIED;  //!< Positioning system status
    NmeaModeGllVtg mode = NmeaModeGllVtg::UNSPECIFIED;        //!< Positioning system mode

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* FORMATTER = "GLL";  //!< Formatter
};

/**
 * @brief NMEA-Gx-RMC message payload
 */
struct NmeaRmcPayload : public NmeaPayload
{
    NmeaTime time;                                               //!< Time
    NmeaStatusGllRmc status = NmeaStatusGllRmc::UNSPECIFIED;     //!< Positioning system status
    NmeaLlh ll;                                                  //!< Position (no height)
    NmeaFloat speed;                                             //!< Speed over ground [knots]
    NmeaFloat course;                                            //!< Course over ground w.r.t. True North [deg]
    NmeaDate date;                                               //!< Date
    NmeaModeRmcGns mode = NmeaModeRmcGns::UNSPECIFIED;           //!< Positioning system mode indicator
    NmeaNavStatusRmc navstatus = NmeaNavStatusRmc::UNSPECIFIED;  //!< Navigational status (optional)

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* FORMATTER = "RMC";  //!< Formatter
};

/**
 * @brief NMEA-Gx-VTG message payload
 */
struct NmeaVtgPayload : public NmeaPayload
{
    NmeaFloat cogt;  //!< Course over ground (true) [deg]
    NmeaFloat cogm;  //!< Course over ground (magnetic) [deg], not typically available
    NmeaFloat sogn;  //!< Speed over ground [knots]
    NmeaFloat sogk;  //!< Speed over ground [km/h]
    NmeaModeGllVtg mode = NmeaModeGllVtg::UNSPECIFIED;  //!< Positioning system mode

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* FORMATTER = "VTG";  //!< Talker
};

/**
 * @brief NMEA-Gx-GST message payload
 */
struct NmeaGstPayload : public NmeaPayload
{
    NmeaTime time;          //!< Time
    NmeaFloat rms_range;    //!< RMS value of the standard deviation of the range inputs to the navigation process
    NmeaFloat std_major;    //!< Standard deviation of semi-major axis of error ellipse
    NmeaFloat std_minor;    //!< Standard deviation of semi-minor axis of error ellipse
    NmeaFloat angle_major;  //!< Angle of semi-major axis of error ellipse from true North
    NmeaFloat std_lat;      //!< Standard deviation of latitude error
    NmeaFloat std_lon;      //!< Standard deviation of longitude error
    NmeaFloat std_alt;      //!< Standard deviation of altitude error

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* FORMATTER = "GST";  //!< Formatter
};

/**
 * @brief NMEA-Gx-HDT message payload
 */
struct NmeaHdtPayload : public NmeaPayload
{
    NmeaFloat heading;  //!< True heading

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* FORMATTER = "HDT";  //!< Formatter
};

/**
 * @brief NMEA-Gx-ZDA message payload
 */
struct NmeaZdaPayload : public NmeaPayload
{
    NmeaTime time;      //!< Time
    NmeaDate date;      //!< Date
    NmeaInt local_hr;   //!< Local zone hours, always 00 (= UTC)
    NmeaInt local_min;  //!< Local zone minutes, always 00 (= UTC)

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* FORMATTER = "ZDA";  //!< Formatter
};

/**
 * @brief NMEA-Gx-GSA message payload (NMEA 4.11 only!)
 */
struct NmeaGsaPayload : public NmeaPayload
{
    NmeaOpModeGsa opmode = NmeaOpModeGsa::UNSPECIFIED;     //!< Operation mode
    NmeaNavModeGsa navmode = NmeaNavModeGsa::UNSPECIFIED;  //!< Nav mode
    std::array<NmeaSat, 12> sats;                          //!< Satellites, valid ones are 0..(num_sats-1)
    int num_sats = 0;                                      //!< Number of valid sats (the first n of sats[])
    NmeaFloat pdop;                                        //!< PDOP
    NmeaFloat hdop;                                        //!< HDOP
    NmeaFloat vdop;                                        //!< VDOP
    NmeaSystemId system = NmeaSystemId::UNSPECIFIED;       //!< System ID

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* FORMATTER = "GSA";  //!< Formatter
};

/**
 * @brief NMEA-Gx-GSV message payload (NMEA 4.11 only!)
 */
struct NmeaGsvPayload : public NmeaPayload
{
    NmeaInt num_msgs;               //!< Number of messages in this GSV sequence (for this signal ID)
    NmeaInt msg_num;                //!< Message number in sequence (1...num_msgs)
    NmeaInt tot_num_sat;            //!< Number of sat/sig info in the whole sequence of GSV messages
    std::array<NmeaAzEl, 4> azels;  //!< Satellite positions, valid ones are 0..(num_sats-1)
    int num_azels = 0;              //!< Number of valid satellite positions (the first n of azels[])
    std::array<NmeaCno, 4> cnos;    //!< Signal levels, valid ones are 0..(num_sats-1)
    int num_cnos = 0;               //!< Number of valid signal levels (the first n of azels[])
    NmeaSystemId system;            //!< System ID
    NmeaSignalId signal;            //!< Signal ID

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* FORMATTER = "GSV";  //!< Formatter
};

// ---------------------------------------------------------------------------------------------------------------------

//! Pointer to NMEA payload
using NmeaPayloadPtr = std::unique_ptr<NmeaPayload>;

/**
 * @brief Decode NMEA message
 *
 * @param[in]  msg       Pointer to the NMEA message
 * @param[in]  msg_size  Size of the NMEA message (>= 11)
 *
 * @returns an instance of the decoded payload on success, nullptr otherwise (bad or unknown message)
 */
NmeaPayloadPtr NmeaDecodeMessage(const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Decode NMEA message
 *
 * @param[in]  msg  The NMEA message data
 *
 * @returns an instance of the decoded payload on success, nullptr otherwise (bad or unknown message)
 */
inline NmeaPayloadPtr NmeaDecodeMessage(const std::vector<uint8_t>& msg)
{
    return NmeaDecodeMessage(msg.data(), msg.size());
}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Collector for NMEA-Gx-GSA and NMEA-Gx-GPA
 */
struct NmeaCollectGsaGsv
{
    /**
     * @brief Satellite info
     */
    struct Sat
    {
        NmeaSystemId system_ = NmeaSystemId::UNSPECIFIED;  //!< System ID
        int svid_ = 0;                                     //!< Satellite ID (numbering cf. NMEA 0183)
        int az_ = 0;                                       //!< Azimuth [deg] (0..360)
        int el_ = 0;                                       //!< Elevation [deg] (-90..90)
    };

    /**
     * Signal info
     */
    struct Sig
    {
        NmeaSystemId system_ = NmeaSystemId::UNSPECIFIED;  //!< System ID
        int svid_ = 0;                                     //!< Satellite ID (numbering cf. NMEA 0183)
        NmeaSignalId signal_ = NmeaSignalId::UNSPECIFIED;  //!< Signal ID
        double cno_ = 0.0;                                 //!< Signal level [dBHz]
        bool used_ = false;                                //!< Signal is used in navigation
    };

    std::vector<Sat> sats_;  //!< Collected satellite info
    std::vector<Sig> sigs_;  //!< Collected signal info

    /**
     * @brief Add NMEA-GN-GSA message to collection
     *
     * These must be provided in order and completely, and before the GSV messages.
     *
     * @param[in]  gsa  Decoded message payload
     *
     * @returns true if the message was accepted, false otherwise
     */
    bool AddGsa(const NmeaGsaPayload& gsa);

    /**
     * @brief Add NMEA-Gx-GSV message to collection
     *
     * These must be provided in order and completely, and after the GSA messages.
     *
     * @param[in]  gsv  Decoded message payload
     *
     * @returns true if the message was accepted, false otherwise
     */
    bool AddGsv(const NmeaGsvPayload& gsv);

    /**
     * @brief Complete collection after feeding all GSA and GSV messages
     */
    void Complete();

    /**
     * @brief Add NMEA-GN-GSA and NMEA-Gx-GSV messages to collection
     *
     * Does all of AddGsa(), AddGsv() and Complete() in one call.
     *
     * @param[in]  gsas  All decoded message payloads, complete and in order
     * @param[in]  gsvs  All decoded message payloads, complete and in order
     *
     * @returns true if all messages were collected successfully
     */
    bool AddGsaAndGsv(const std::vector<NmeaGsaPayload>& gsas, const std::vector<NmeaGsvPayload>& gsvs);

   private:
    std::vector<NmeaSat> gsa_sats_;  //!< Satellites used
};

/* ****************************************************************************************************************** */
}  // namespace nmea
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_PARSER_NMEA_HPP__
