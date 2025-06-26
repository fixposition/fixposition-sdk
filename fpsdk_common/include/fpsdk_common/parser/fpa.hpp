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
 * @brief Fixposition SDK: Parser FP_A routines and types
 */
// clang-format off
/**
 * @page FPSDK_COMMON_PARSER_FPA Parser FP_A routines and types
 *
 * **API**: fpsdk_common/parser/fpa.hpp and fpsdk::common::parser::fpa
 *
 * @fp_msgspec_begin{FP_A-Protocol}
 *
 * Messages are in this form:
 *
 * <code><b style="color: red;">$</b><b style="color: green;">FP</b>,<b style="color: blue;">msg_type</b>,<b style="color: blue;">msg_version</b>,<em>field<sub>3</sub></em>,<em>field<sub>4</sub></em>,…,<em>field<sub>N</sub></em><b style="color: red;">*CC</b><b style="color: red;">\\r\\n</b></code>
 *
 * Where:
 *
 * - The FP_A style framing:
 *     - <code><b style="color: red;">\$</b></code>
 *       -- Start character ("$", ASCII 36)
 *     - <code><b style="color: red;">\*CC</b></code>
 *       -- Checksum: "*" (ASCII 42) and two digit XOR value of all payload
 *       characters in captial hexadecimal notation, for example:
 *       "FPX" = <code>'F' ^ 'P' ^ 'X' = 70 ^ 80 ^ 88 = 78 = 0x4e</code> = checksum <code>4E</code>
 *     - <code><b style="color: red;">\\r\\n</b></code> -- Message termination characters (CR + LF, ASCII 13 + 10)
 * - A Fixposition identifier:
 *     - <code><b style="color: green;">FP</b></code>
 *        -- Fixposition ASCII message identifier, "FP" (ASCII 70 + 80)
 * - Fixposition message type and version:
 *     - <code><b style="color: blue;">msg_type</b></code> (= <code><em>field<sub>1</sub></em></code>)
 *       -- Message type, all capital letters (ASCII 65--90)
 *     - <code><b style="color: blue;">msg_version</b></code> (= <code><em>field<sub>2</sub></em></code>)
 *       -- Message version, decimal number (letters 0--9, ASCII 48--57), range 1--…
 * - Data fields (payload)
 *     - <code><em>field<sub>3</sub></em>,<em>field<sub>4</sub></em>,…,<em>field<sub>N</sub></em></code>
 *       -- The structure of the message data is defined by the <code><b style="color: blue;">msg_type</b></code>
 *       and <code><b style="color: blue;">version</b></code>.
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
 *     - _Numeric_: Decimal integer number, one or more digits (0-9) and optional leading "-" sign
 *     - _Float (.x)_: Decimal floating point number, one or more digits (0-9) and optional leading "-" sign, with
 *       _x_ digits fractional part separated by a dot (".")
 *     - _Float (x)_: Decimal floating point number with _x_ significant digits, optional leading "-", optional fractional
 *       part separated by a dot (".")
 *     -  _String_: String of allowed payload characters (but not the `,` field separator)
 *     -  ...
 *     -  ...
 *
 * @fp_msgspec_end
 *
 */
// clang-format on
#ifndef __FPSDK_COMMON_PARSER_FPA_HPP__
#define __FPSDK_COMMON_PARSER_FPA_HPP__

/* LIBC/STL */
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/* EXTERNAL */

/* PACKAGE */
#include "../logging.hpp"
#include "../types.hpp"
#include "types.hpp"

namespace fpsdk {
namespace common {
namespace parser {
/**
 * @brief Parser FP_A routines and types
 */
namespace fpa {
/* ****************************************************************************************************************** */

static constexpr uint8_t FP_A_PREAMBLE = '$';  //!< FP_A framing preamble
static constexpr int FP_A_FRAME_SIZE = 6;      //!< FP_A frame size ("$*cc\r\n")

//! FP_A message meta data
struct FpaMessageMeta
{
    char msg_type_[40] = { 0 };  //!< Message type (for example, "ODOMETRY"), nul-terminated string
    int msg_version_ = 0;        //!< Message version (for example, 1), < 0 if unknown
    int payload_ix0_ = 0;        //!< Index (offset) for start of payload, 0 if no payload available
    int payload_ix1_ = 0;        //!< Index (offset) for end of payload, 0 if no payload available
};

/**
 * @brief Get FP_A message meta data
 *
 * @param[out]  meta      The meta data
 * @param[in]   msg       Pointer to the FP_A message
 * @param[in]   msg_size  Size of the FP_A message (>= 11)
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a correct FP_A message.
 *
 * @returns true if the meta data was successfully extracted, false otherwise
 */
bool FpaGetMessageMeta(FpaMessageMeta& meta, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Get FP_A message name
 *
 * Generates a name (string) in the form "FP_A-MESSAGEID" (for example, "FP_A-ODOMETRY").
 *
 * @param[out]  name      String to write the name to
 * @param[in]   size      Size of \c name (incl. nul termination)
 * @param[in]   msg       Pointer to the FP_A message
 * @param[in]   msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid FP_A or FP_A message.
 *
 * @returns true if message name was generated, false if \c name buffer was too small
 */
bool FpaGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Get FP_A message info
 *
 * This stringifies the content of some FP_A messages, for debugging.
 *
 * @param[out] info      String to write the info to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the FP_A message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid FP_A message.
 *
 * @returns true if message info was generated (even if info is empty), false if \c name buffer was too small
 */
bool FpaGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief FP_A fusion initialisation status
 */
enum class FpaInitStatus : int
{  // clang-format off
    UNSPECIFIED              = '!',  //!< Unspecified
    NOT_INIT                 = '0',  //!< Not initialised
    LOCAL_INIT               = '1',  //!< Locally initialised
    GLOBAL_INIT              = '2',  //!< Globally initialised
};  // clang-format on

/**
 * @brief Stringify initialisation status
 *
 * @param[in]  status  The initialisation status
 *
 * @returns the stringification of the initialisation status
 */
const char* FpaInitStatusStr(const FpaInitStatus status);

/**
 * @brief FP_A legacy fusion status
 */
enum class FpaFusionStatusLegacy : int
{  // clang-format off
    UNSPECIFIED              = '!',  //!< Unspecified
    NONE                     = '0',  //!< Not started
    VISION                   = '1',  //!< Vision only
    VIO                      = '2',  //!< Visual-inertial fusion
    IMU_GNSS                 = '3',  //!< Inertial-GNSS fusion
    VIO_GNSS                 = '4'   //!< Visual-inertial-GNSS fusion
};  // clang-format on

/**
 * @brief Stringify legacy fusion status
 *
 * @param[in]  status  The legacy fusion status
 *
 * @returns the stringification of the legacy fusion status
 */
const char* FpaFusionStatusLegacyStr(const FpaFusionStatusLegacy status);

/**
 * @brief FP_A fusion measurement status
 */
enum class FpaMeasStatus : int
{  // clang-format off
    UNSPECIFIED              = '!',  //!< Unspecified
    NOT_USED                 = '0',  //!< Not used
    USED                     = '1',  //!< Used
    DEGRADED                 = '2',  //!< Degraded
};  // clang-format on

/**
 * @brief Stringify fusion measurement status
 *
 * @param[in]  status  The fusion measurement status
 *
 * @returns the stringification of the fusion measurement status
 */
const char* FpaMeasStatusStr(const FpaMeasStatus status);

/**
 * @brief FP_A IMU bias status
 */
enum class FpaImuStatus : int
{  // clang-format off
    UNSPECIFIED              = '!',  //!< Unspecified
    NOT_CONVERGED            = '0',  //!< Not converged
    WARMSTARTED              = '1',  //!< Warmstarted
    ROUGH_CONVERGED          = '2',  //!< Rough convergence
    FINE_CONVERGED           = '3',  //!< Fine convergence
};  // clang-format on

/**
 * @brief Stringify IMU bias status
 *
 * @param[in]  status  The IMU bias status
 *
 * @returns the stringification of the IMU bias status
 */
const char* FpaImuStatusStr(const FpaImuStatus status);

/**
 * @brief FP_A legacy IMU bias status
 */
enum class FpaImuStatusLegacy : int
{  // clang-format off
    UNSPECIFIED              = '!',  //!< Unspecified
    NOT_CONVERGED            = '0',  //!< Not converged
    CONVERGED                = '1',  //!< Converged
};  // clang-format on

/**
 * @brief Stringify legacy IMU bias status
 *
 * @param[in]  status  The legacy IMU bias status
 *
 * @returns the stringification of the legacy IMU bias status
 */
const char* FpaImuStatusLegacyStr(const FpaImuStatusLegacy status);

/**
 * @brief FP_A IMU variance
 */
enum class FpaImuNoise : int
{  // clang-format off
    UNSPECIFIED              = '!',  //!< Unspecified
    LOW_NOISE                = '1',  //!< Low noise
    MEDIUM_NOISE             = '2',  //!< Medium noise
    HIGH_NOISE               = '3',  //!< High noise
    RESERVED4                = '4',  //!< Reserved
    RESERVED5                = '5',  //!< Reserved
    RESERVED6                = '6',  //!< Reserved
    RESERVED7                = '7',  //!< Reserved
};  // clang-format on

/**
 * @brief Stringify IMU variance
 *
 * @param[in]  noise  The IMU variance
 *
 * @returns the stringification of the IMU variance
 */
const char* FpaImuNoiseStr(const FpaImuNoise noise);

/**
 * @brief FP_A IMU accelerometer and gyroscope convergence
 */
enum class FpaImuConv : int
{  // clang-format off
    UNSPECIFIED              = '!',  //!< Unspecified
    RESERVED0                = '0',  //!< Reserved
    WAIT_IMU_MEAS            = '1',  //!< Awaiting IMU measurements
    WAIT_GLOBAL_MEAS         = '2',  //!< Insufficient global measurements
    WAIT_MOTION              = '3',  //!< Insufficient motion
    CONVERGING               = '4',  //!< Converging
    RESERVED5                = '5',  //!< Reserved
    RESERVED6                = '6',  //!< Reserved
    IDLE                     = '7',  //!< Idle
};  // clang-format on

/**
 * @brief Stringify IMU accelerometer and gyroscope convergence
 *
 * @param[in]  conv  The IMU accelerometer and gyroscope convergence
 *
 * @returns the stringification of the IMU accelerometer and gyroscope convergence
 */
const char* FpaImuConvStr(const FpaImuConv conv);

/**
 * @brief FP_A GNSS fix status
 */
enum class FpaGnssStatus : int
{  // clang-format off
    UNSPECIFIED              = '!',  //!< Unspecified
    NO_FIX                   = '0',  //!< No fix
    SPP                      = '1',  //!< Single-point positioning (SPP)
    RTK_MB                   = '2',  //!< RTK moving baseline
    RESERVED3                = '3',  //!< Reserved
    RESERVED4                = '4',  //!< Reserved
    RTK_FLOAT                = '5',  //!< RTK float
    RESERVED6                = '6',  //!< Reserved
    RESERVED7                = '7',  //!< Reserved
    RTK_FIXED                = '8',  //!< RTK fixed
    RESERVED9                = '9',  //!< Reserved
};  // clang-format on

/**
 * @brief Stringify GNSS fix status
 *
 * @param[in]  status  TheGNSS fix status
 *
 * @returns the stringification of the GNSS fix status
 */
const char* FpaGnssStatusStr(const FpaGnssStatus status);

/**
 * @brief FP_A GNSS correction status
 */
enum class FpaCorrStatus : int
{  // clang-format off
    UNSPECIFIED              = '!',  //!< Unspecified
    WAITING_FUSION           = '0',  //!< Waiting fusion
    NO_GNSS                  = '1',  //!< No GNSS available
    NO_CORR                  = '2',  //!< No corrections used
    LIMITED_CORR             = '3',  //!< Limited corrections used: station data & at least one of the constellations (both bands) among the valid rover measurements
    OLD_CORR                 = '4',  //!< Corrections are too old (TBD)
    GOOD_CORR                = '5',  //!< Sufficient corrections used: station data and corrections for ALL bands among the valid rover measurements
    RESERVED6                = '6',  //!< Reserved
    RESERVED7                = '7',  //!< Reserved
    RESERVED8                = '8',  //!< Reserved
    RESERVED9                = '9',  //!< Reserved
};  // clang-format on

/**
 * @brief Stringify GNSS correction status
 *
 * @param[in]  status  The GNSS correction status
 *
 * @returns the stringification of the GNSS correction status
 */
const char* FpaCorrStatusStr(const FpaCorrStatus status);

/**
 * @brief FP_A baseline status
 */
enum class FpaBaselineStatus : int
{  // clang-format off
    UNSPECIFIED              = '!',  //!< Unspecified
    WAITING_FUSION           = '0',  //!< Waiting fusion
    NO_FIX                   = '1',  //!< Not available or no fix
    FAILING                  = '2',  //!< Failing
    PASSING                  = '3',  //!< Passing
};  // clang-format on

/**
 * @brief Stringify baseline status
 *
 * @param[in]  status  The baseline status
 *
 * @returns the stringification of the baseline status
 */
const char* FpaBaselineStatusStr(const FpaBaselineStatus status);

/**
 * @brief FP_A camera status
 */
enum class FpaCamStatus : int
{  // clang-format off
    UNSPECIFIED              = '!',  //!< Unspecified
    CAM_UNAVL                = '0',  //!< Camera not available
    BAD_FEAT                 = '1',  //!< Camera available, but not usable  (e.g. too dark)
    RESERVED2                = '2',  //!< Reserved
    RESERVED3                = '3',  //!< Reserved
    RESERVED4                = '4',  //!< Reserved
    GOOD                     = '5',  //!< Camera working and available
};  // clang-format on

/**
 * @brief Stringify camera status
 *
 * @param[in]  status  The camera status
 *
 * @returns the stringification of the camera status
 */
const char* FpaCamStatusStr(const FpaCamStatus status);

/**
 * @brief FP_A wheelspeed status
 */
enum class FpaWsStatus : int
{  // clang-format off
    UNSPECIFIED              = '!',  //!< Unspecified
    NOT_ENABLED              = '0',  //!< No wheelspeed enabled
    MISS_MEAS                = '1',  //!< Missing wheelspeed measurements
    NONE_CONVERGED           = '2',  //!< At least one wheelspeed enabled, no wheelspeed converged
    ONE_CONVERGED            = '3',  //!< At least one wheelspeed enabled and at least one converged
    ALL_CONVERGED            = '4',  //!< At least one wheelspeed enabled and all converged
};  // clang-format on

/**
 * @brief Stringify wheelspeed status
 *
 * @param[in]  status  The wheelspeed status
 *
 * @returns the stringification of the wheelspeed status
 */
const char* FpaWsStatusStr(const FpaWsStatus status);

/**
 * @brief FP_A legacy wheelspeed status
 */
enum class FpaWsStatusLegacy : int
{  // clang-format off
    UNSPECIFIED              = '!', //!< Unspecified
    NOT_ENABLED              = '-', //!< No wheelspeed enabled
    NONE_CONVERGED           = '0', //!< None converged
    ONE_OR_MORE_CONVERGED    = '1', //!< At least one converged
};  // clang-format on

/**
 * @brief Stringify legacy wheelspeed status
 *
 * @param[in]  status  The legacy wheelspeed status
 *
 * @returns the stringification of the legacy wheelspeed status
 */
const char* FpaWsStatusLegacyStr(const FpaWsStatusLegacy status);

/**
 * @brief FP_A wheelspeed convergence status
 */
enum class FpaWsConv : int
{  // clang-format off
    UNSPECIFIED              = '!',  //!< Unspecified
    WAIT_FUSION              = '0',  //!< Awaiting Fusion
    WAIT_WS_MEAS             = '1',  //!< Missing wheelspeed measurements
    WAIT_GLOBAL_MEAS         = '2',  //!< Insufficient global measurements
    WAIT_MOTION              = '3',  //!< Insufficient motion
    WAIT_IMU_BIAS            = '4',  //!< Insufficient imu bias convergence
    CONVERGING               = '5',  //!< Converging
    IDLE                     = '6',  //!< Idle
};  // clang-format on

/**
 * @brief Stringify wheelspeed convergence status
 *
 * @param[in]  status  The wheelspeed convergence status
 *
 * @returns the stringification of the wheelspeed convergence status
 */
const char* FpaWsConvStr(const FpaWsConv status);

/**
 * @brief FP_A markers status
 */
enum class FpaMarkersStatus : int
{  // clang-format off
    UNSPECIFIED              = '!',  //!< Unspecified
    NOT_ENABLED              = '0',  //!< No markers available
    NONE_CONVERGED           = '1',  //!< Markers available
    ONE_CONVERGED            = '2',  //!< Markers available and used
    ALL_CONVERGED            = '3',  //!< All markers available and used
};  // clang-format on

/**
 * @brief Stringify markers status
 *
 * @param[in]  status  The markers status
 *
 * @returns the stringification of the markers status
 */
const char* FpaMarkersStatusStr(const FpaMarkersStatus status);

/**
 * @brief FP_A markers convergence status
 */
enum class FpaMarkersConv : int
{  // clang-format off
    UNSPECIFIED              = '!',  //!< Unspecified
    WAIT_FUSION              = '0',  //!< Awaiting Fusion
    WAIT_MARKER_MEAS         = '1',  //!< Waiting marker measurements
    WAIT_GLOBAL_MEAS         = '2',  //!< Insufficient global measurements
    CONVERGING               = '3',  //!< Converging
    IDLE                     = '4',  //!< Idle
};  // clang-format on

/**
 * @brief Stringify markers convergence status
 *
 * @param[in]  conv  The markers convergence status
 *
 * @returns the stringification of the markers convergence status
 */
const char* FpaMarkersConvStr(const FpaMarkersConv conv);

/**
 * @brief FP_A GNSS fix type
 */
enum class FpaGnssFix : int
{  // clang-format off
    UNSPECIFIED              = '!',  //!< Unspecified
    UNKNOWN                  = '0',  //!< Unknown
    NOFIX                    = '1',  //!< No fix
    DRONLY                   = '2',  //!< Dead-reckoning only
    TIME                     = '3',  //!< Time-only fix
    S2D                      = '4',  //!< Single 2D fix
    S3D                      = '5',  //!< Single 3D fix
    S3D_DR                   = '6',  //!< Single 3D fix with dead-reckoning
    RTK_FLOAT                = '7',  //!< RTK float fix
    RTK_FIXED                = '8',  //!< RTK fixed fix
};  // clang-format on

/**
 * @brief Stringify fix type
 *
 * @param[in] fix  The fix type
 *
 * @returns the stringification of the fix type ("S3D", "RTK_FLOAT", etc.)
 */
const char* FpaGnssFixStr(const FpaGnssFix fix);

/**
 * @brief FP_A epoch type
 */
enum class FpaEpoch : int
{
    UNSPECIFIED = 0,  //!< Unspecified
    GNSS1,            //!< GNSS 1
    GNSS2,            //!< GNSS 2
    GNSS,             //!< Combined GNSS
    FUSION,           //!< Fusion
};

/**
 * @brief Stringify epoch type
 *
 * @param[in]  epoch  The epoch type
 *
 * @returns the stringification of the epoch type ("FUSION", "GNSS1", etc.)
 */
const char* FpaEpochStr(const FpaEpoch epoch);

/**
 * @brief FP_A GNSS antenna state
 */
enum class FpaAntState : int
{
    UNSPECIFIED = 0,  //!< Unspecified
    OPEN,             //!< No antenna detected (or connected via DC block)
    OK,               //!< Antenna detected and good
    SHORT,            //!< Antenna short circuit detected
};

/**
 * @brief Stringify antenna state
 *
 * @param[in]  state  The antenna state
 *
 * @returns the stringification of the antenna state ("OPEN", "OK", etc.)
 */
const char* FpaAntStateStr(const FpaAntState state);

/**
 * @brief FP_A GNSS antenna power
 */
enum class FpaAntPower : int
{
    UNSPECIFIED = 0,  //!< Unspecified
    ON,               //!< Antenna power supply is on
    OFF,              //!< Antenna power supply is off
};

/**
 * @brief Stringify antenna power
 *
 * @param[in]  power  The antenna power
 *
 * @returns the stringification of the antenna power ("ON", "OFF", etc.)
 */
const char* FpaAntPowerStr(const FpaAntPower power);

/**
 * @brief FP_A text levels
 */
enum class FpaTextLevel : int
{  // clang-format off
    UNSPECIFIED  = 0,                                                 //!< Unspecified
    ERROR        = types::EnumToVal(logging::LoggingLevel::ERROR),    //!< Error
    WARNING      = types::EnumToVal(logging::LoggingLevel::WARNING),  //!< Warning
    INFO         = types::EnumToVal(logging::LoggingLevel::INFO),     //!< Info
    DEBUG        = types::EnumToVal(logging::LoggingLevel::DEBUG),    //!< Debug
};  // clang-format on

/**
 * @brief Stringify text level
 *
 * @param[in]  level  The text level
 *
 * @returns the stringification of the text level ("ERROR", "INFO", etc.)
 */
const char* FpaTextLevelStr(const FpaTextLevel level);

/**
 * @brief FP_A time base
 */
enum class FpaTimebase : int
{  // clang-format off
    UNSPECIFIED = 0,  //!< Unspecified
    NONE,             //!< None
    GNSS,             //!< GNSS
    UTC,              //!< UTC
};  // clang-format on

/**
 * @brief Stringify time base
 *
 * @param[in]  base  The time base
 *
 * @returns the stringification of the time base ("GNSS", "UTC", etc.)
 */
const char* FpaTimebaseStr(const FpaTimebase base);

/**
 * @brief FP_A time reference
 */
enum class FpaTimeref : int
{  // clang-format off
    UNSPECIFIED = 0,  //!< Unspecified
    UTC_NONE,         //!< UTC: None (UTC params not yet known)
    UTC_CRL,          //!< UTC: Communications Research Laboratory (CRL), Japan
    UTC_NIST,         //!< UTC: National Institute of Standards and Technology (NIST)
    UTC_USNO,         //!< UTC: U.S. Naval Observatory (USNO)
    UTC_BIPM,         //!< UTC: International Bureau of Weights and Measures (BIPM)
    UTC_EU,           //!< UTC: European laboratories
    UTC_SU,           //!< UTC: Former Soviet Union (SU)
    UTC_NTSC,         //!< UTC: National Time Service Center (NTSC), China
    GNSS_GPS,         //!< GNSS: GPS
    GNSS_GAL,         //!< GNSS: Galileo
    GNSS_BDS,         //!< GNSS: BeiDou
    GNSS_GLO,         //!< GNSS: GLONASS
    GNSS_NVC,         //!< GNSS: NavIC
    OTHER,            //!< other/unknown GNSS/UTC
};  // clang-format on

/**
 * @brief Stringify time reference
 *
 * @param[in]  ref  The time reference
 *
 * @returns the stringification of the time reference ("UTC_NONE", "GNSS_GPS", etc.)
 */
const char* FpaTimerefStr(const FpaTimeref ref);

/**
 * @brief FP_A integer value
 */
struct FpaInt
{
    bool valid = false;  //!< Data is valid
    int value = 0;       //!< Value
};

/**
 * @brief FP_A float value
 */
struct FpaFloat
{
    bool valid = false;  //!< Data is valid
    double value = 0.0;  //!< Value
};

/**
 * @brief FP_A three float values
 */
struct FpaFloat3
{
    bool valid = false;                                    //!< Data is valid
    std::array<double, 3> values = { { 0.0, 0.0, 0.0 } };  //!< Values
};

/**
 * @brief FP_A four float values
 */
struct FpaFloat4
{
    bool valid = false;                                         //!< Data is valid
    std::array<double, 4> values = { { 0.0, 0.0, 0.0, 0.0 } };  //!< Values
};

/**
 * @brief FP_A six float values
 */
struct FpaFloat6
{
    bool valid = false;                                                   //!< Data is valid
    std::array<double, 6> values = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };  //!< Values
};

/**
 * @brief FP_A GPS time
 */
struct FpaGpsTime
{
    FpaInt week;   //!< GPS week number, range 0-9999, or null if time not (yet) available
    FpaFloat tow;  //!< GPS time of week, range 0.000-604799.999999, or null if time not (yet) available

    bool operator==(const FpaGpsTime& rhs) const;  //!< Equal (epsilon = 1e-6)
    bool operator!=(const FpaGpsTime& rhs) const;  //!< Not equal (epsilon = 1e-6)
    bool operator<(const FpaGpsTime& rhs) const;   //!< Less than
    bool operator>(const FpaGpsTime& rhs) const;   //!< Greater than
    bool operator<=(const FpaGpsTime& rhs) const;  //!< Less or equal than
    bool operator>=(const FpaGpsTime& rhs) const;  //!< Greater or equal than
};

/**
 * @brief FP_A message types
 */
enum class FpaMessageType
{
    UNSPECIFIED,  //!< Unspecified
    EOE,          //!< FpaEoePayload
    GNSSANT,      //!< FpaGnssantPayload
    GNSSCORR,     //!< FpaGnsscorrPayload
    RAWIMU,       //!< FpaRawimuPayload
    CORRIMU,      //!< FpaCorrimuPayload
    IMUBIAS,      //!< FpaImubiasPayload
    LLH,          //!< FpaLlhPayload
    ODOMETRY,     //!< FpaOdometryPayload
    ODOMENU,      //!< FpaOdomenuPayload
    ODOMSH,       //!< FpaOdomshPayload
    ODOMSTATUS,   //!< FpaOdomstatusPayload
    TEXT,         //!< FpaTextPayload
    TF,           //!< FpaTfPayload
    TP,           //!< FpaTpPayload
    VERSION,      //!< FpaVersionPayload
};

/**
 * @brief FP_A payload base class
 */
struct FpaPayload
{
    bool valid_ = false;  //!< Payload successfully decoded (true), or not (yet) decoded (false)
    FpaMessageType msg_type_ = FpaMessageType::UNSPECIFIED;  //!< Message (payload) type
    virtual ~FpaPayload() = default;                         //!< Virtual dtor for polymorphism

    /**
     * @brief Set data from message
     *
     * @param[in]  msg       Pointer to the FP_A message
     * @param[in]  msg_size  Size of the FP_A message (>= 11)
     *
     * @returns true if message payload was correct and all data could be extracted (fields are now valid), or false
     *          otherwise (fields are now invalid)
     */
    virtual bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) = 0;

    /**
     * @brief Set data from message
     *
     * @param[in]  buf  The FP_A message data
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
 * @brief FP_A-EOE (version 1) message payload
 */
struct FpaEoePayload : public FpaPayload
{
    FpaGpsTime gps_time;                     //!< Time
    FpaEpoch epoch = FpaEpoch::UNSPECIFIED;  //!< Indicates which epoch ended ("FUSION", "GNSS", "GNSS1", "GNSS2")

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* MSG_NAME = "FP_A-EOE";  //!< Message name
};

/**
 * @brief FP_A-GNSSANT (version 1) message payload
 */
struct FpaGnssantPayload : public FpaPayload
{  // clang-format off
    FpaGpsTime   gps_time;    //!< Time
    FpaAntState  gnss1_state = FpaAntState::UNSPECIFIED;  //!< GNSS1 antenna state
    FpaAntPower  gnss1_power = FpaAntPower::UNSPECIFIED;  //!< GNSS1 antenna power
    FpaInt       gnss1_age;   //!< Time since gnss1_state or gnss1_power information last changed, 0-604800
    FpaAntState  gnss2_state = FpaAntState::UNSPECIFIED;  //!< GNSS2 antenna state
    FpaAntPower  gnss2_power = FpaAntPower::UNSPECIFIED;  //!< GNSS2 antenna power
    FpaInt       gnss2_age;   //!< Time since gnss2_state or gnss2_power information last changed, 0-604800
    // clang-format on

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* MSG_NAME = "FP_A-GNSSANT";  //!< Message name
};

/**
 * @brief FP_A-GNSSCORR (version 1) message payload
 */
struct FpaGnsscorrPayload : public FpaPayload
{  // clang-format off
    FpaGpsTime  gps_time;          //!< Time
    FpaGnssFix  gnss1_fix = FpaGnssFix::UNSPECIFIED;  //!< Fix status of GNSS1
    FpaInt      gnss1_nsig_l1;     //!< Number of L1 signals with correction data tracked by GNSS1 receiver, range 0-100
    FpaInt      gnss1_nsig_l2;     //!< Number of L2 signals with correction data tracked by GNSS1 receiver, range 0-100
    FpaGnssFix  gnss2_fix = FpaGnssFix::UNSPECIFIED;  //!< Fix status of GNSS2 receiver
    FpaInt      gnss2_nsig_l1;     //!< Number of L1 signals with correction data tracked by GNSS2 receiver, range 0-100
    FpaInt      gnss2_nsig_l2;     //!< Number of L2 signals with correction data tracked by GNSS2 receiver, range 0-100
    FpaFloat    corr_latency;      //!< Average correction data latency (10s window), range 0.0-99.9
    FpaFloat    corr_update_rate;  //!< Average correction update rate (10s window), range 0.0-0.0
    FpaFloat    corr_data_rate;    //!< Average correction data rate (10s window), range 0.0-50.0
    FpaFloat    corr_msg_rate;     //!< Average correction message rate (10s window), range 0-50
    FpaInt      sta_id;            //!< Correction station ID, range 0--4095
    FpaFloat3   sta_llh;           //!< Correction station latitude/longitude/height [deg, deg, m]
    FpaInt      sta_dist;          //!< Correction station baseline length (approximate 3d distance), range 0-100000
    // clang-format on

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* MSG_NAME = "FP_A-GNSSCORR";  //!< Message name
};

/**
 * @brief FP_A-...IMU (versions 1 and 2) messages payload
 */
struct FpaImuPayload : public FpaPayload
{
    // clang-format off
    FpaGpsTime   gps_time;          //!< Time
    FpaFloat3    acc;               //!< Raw acceleration in output frame, X/Y/Z components
    FpaFloat3    rot;               //!< Raw angular velocity in output frame, X/Y/Z components
    bool         bias_comp = false; //!< Signal is bias compensated (true) or not (false), always false for RAWIMU, may be true for CORRIMU
    FpaImuStatus imu_status = FpaImuStatus::UNSPECIFIED;  //!< IMU bias status (only available for version 2 messages)
    // clang-format on
};

/**
 * @brief FP_A-RAWIMU (version 1) message payload
 */
struct FpaRawimuPayload : public FpaImuPayload
{
    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* MSG_NAME = "FP_A-RAWIMU";  //!< Message name
};

/**
 * @brief FP_A-CORRIMU (version 1) message payload
 */
struct FpaCorrimuPayload : public FpaImuPayload
{
    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* MSG_NAME = "FP_A-CORRIMU";  //!< Message name
};

/**
 * @brief FP_A-IMUBIAS (version 1) message payload
 */
struct FpaImubiasPayload : public FpaPayload
{  // clang-format off
    FpaGpsTime     gps_time;      //!< Time
    FpaMeasStatus  fusion_imu;    //!< Fusion measurement status: IMU
    FpaImuStatus   imu_status;    //!< IMU bias status
    FpaImuNoise    imu_noise;     //!< IMU variance status
    FpaImuConv     imu_conv;      //!< IMU convergence status
    FpaFloat3      bias_acc;      //!< Accelerometer bias, X/Y/Z components
    FpaFloat3      bias_gyr;      //!< Gyroscope bias, X/Y/Z components
    FpaFloat3      bias_cov_acc;  //!< Accelerometer bias covariance, X/Y/Z components
    FpaFloat3      bias_cov_gyr;  //!< Gyroscope bias covariance, X/Y/Z components
    // clang-format on

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* MSG_NAME = "FP_A-IMUBIAS";  //!< Message name
};

/**
 * @brief FP_A-LLH (version 1) message payload
 */
struct FpaLlhPayload : public FpaPayload
{  // clang-format off
    FpaGpsTime gps_time;  //!< Time
    FpaFloat3 llh;        //!< Latitude/longitude/height [deg, deg, m]
    FpaFloat6 cov_enu;    //!< Position covariance in ENU, EE/NN/UU/EN/NU/EU components [m^2]
    // clang-format on

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* MSG_NAME = "FP_A-LLH";  //!< Message name
};

/**
 * @brief FP_A-ODOM... (version 1) messages payload
 */
struct FpaOdomPayload : public FpaPayload
{
    // clang-format off
    FpaGpsTime             gps_time;             //!< Time
    FpaFloat3              pos;                  //!< Position, X/Y/Z components
    FpaFloat4              orientation;          //!< Quaternion, W/X/Y/Z components
    FpaFloat3              vel;                  //!< Velocity, X/Y/Z components
    FpaFloat3              rot;                  //!< Bias corrected angular velocity, X/Y/Z components
    FpaFloat3              acc;                  //!< Bias corrected acceleration, X/Y/Z components
    FpaFusionStatusLegacy  fusion_status;        //!< Fustion status
    FpaImuStatusLegacy     imu_bias_status;      //!< IMU bias status
    FpaGnssFix             gnss1_fix;            //!< Fix status of GNSS1 receiver
    FpaGnssFix             gnss2_fix;            //!< Fix status of GNSS2 receiver
    FpaWsStatusLegacy      wheelspeed_status;    //!< Wheelspeed status
    FpaFloat6              pos_cov;              //!< Position covariance, XX/YY/ZZ/XY/YZ/XZ components
    FpaFloat6              orientation_cov;      //!< Orientation covariance, XX/YY/ZZ/XY/YZ/XZ components
    FpaFloat6              vel_cov;              //!< Velocity covariance, XX/YY/ZZ/XY/YZ/XZ components
    char                   version[100] = { 0 }; //!< Software version string
    // clang-format on

    virtual bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) = 0;
};

/**
 * @brief FP_A-ODOMETRY (version 2) messages payload (ECEF)
 */
struct FpaOdometryPayload : public FpaOdomPayload
{
    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* MSG_NAME = "FP_A-ODOMETRY";  //!< Message name
};

/**
 * @brief FP_A-ODOMENU (version 1) messages payload (ENU0)
 */
struct FpaOdomenuPayload : public FpaOdomPayload
{
    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* MSG_NAME = "FP_A-ODOMENU";  //!< Message name
};

/**
 * @brief FP_A-ODOMSH (version 1) messages payload (ECEF)
 */
struct FpaOdomshPayload : public FpaOdomPayload
{
    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* MSG_NAME = "FP_A-ODOMSH";  //!< Message name
};

/**
 * @brief FP_A-ODOMSTATUS (version 1) messages payload
 */
struct FpaOdomstatusPayload : public FpaPayload
{  // clang-format off
    FpaGpsTime         gps_time;         //!< Time
    FpaInitStatus      init_status;      //!< Fusion init status
    FpaMeasStatus      fusion_imu;       //!< Fusion measurement status: IMU
    FpaMeasStatus      fusion_gnss1;     //!< Fusion measurement status: GNSS 1
    FpaMeasStatus      fusion_gnss2;     //!< Fusion measurement status: GNSS 2
    FpaMeasStatus      fusion_corr;      //!< Fusion measurement status: GNSS corrections
    FpaMeasStatus      fusion_cam1;      //!< Fusion measurement status: camera
    FpaMeasStatus      fusion_ws;        //!< Fusion measurement status: wheelspeed
    FpaMeasStatus      fusion_markers;   //!< Fusion measurement status: markers
    FpaImuStatus       imu_status;       //!< IMU bias status
    FpaImuNoise        imu_noise;        //!< IMU variance status
    FpaImuConv         imu_conv;         //!< IMU convergence status
    FpaGnssStatus      gnss1_status;     //!< GNSS 1 status
    FpaGnssStatus      gnss2_status;     //!< GNSS 2 status
    FpaBaselineStatus  baseline_status;  //!< Baseline status
    FpaCorrStatus      corr_status;      //!< GNSS correction status
    FpaCamStatus       cam1_status;      //!< Camera 1 status
    FpaWsStatus        ws_status;        //!< Wheelspeed status
    FpaWsConv          ws_conv;          //!< Wheelspeed convergence status
    FpaMarkersStatus   markers_status;   //!< Marker status
    FpaMarkersConv     markers_conv;     //!< Marker convergence status
    // clang-format on

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* MSG_NAME = "FP_A-ODOMSTATUS";  //!< Message name
};

/**
 * @brief FP_A-TEXT (version 1) message payload
 */
struct FpaTextPayload : public FpaPayload
{  // clang-format off
    FpaTextLevel  level;                        //!< Level
    char          text[MAX_FP_A_SIZE] = { 0 };  //!< Text (nul-terminated string)
    // clang-format on

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* MSG_NAME = "FP_A-TEXT";  //!< Message name
};

/**
 * @brief FP_A-TF (version 2) message payload
 */
struct FpaTfPayload : public FpaPayload
{  // clang-format off
    FpaGpsTime gps_time;             //!< Time
    char       frame_a[10] = { 0 };  //!< Target frame (nul-terminated string)
    char       frame_b[10] = { 0 };  //!< Initial frame (nul-terminated string)
    FpaFloat3  translation;          //!< Translation, X/Y/Z components
    FpaFloat4  orientation;          //!< Rotation in quaternion, W/X/Y/Z components
    // clang-format on

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* MSG_NAME = "FP_A-TF";  //!< Message name
};

/**
 * @brief FP_A-TP (versions 1 and 2) message payload
 */
struct FpaTpPayload : public FpaPayload
{  // clang-format off
    char        tp_name[10] = { 0 };                  //!< Timepulse name (nul-terminated string)
    FpaTimebase timebase = FpaTimebase::UNSPECIFIED;  //!< Time base
    FpaTimeref  timeref = FpaTimeref::UNSPECIFIED;    //!< Time reference
    FpaInt      tp_week;                              //!< Timepulse week number (only since version 2 of the message)
    FpaInt      tp_tow_sec;                           //!< Timepulse time seconds of week, integer second part (0--604799)
    FpaFloat    tp_tow_psec;                          //!< Timepulse time seconds of week, sub-second part (0.000000000000--0.999999999999)
    FpaInt      gps_leaps;                            //!< GPS leapseconds
    // clang-format on

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* MSG_NAME = "FP_A-TP";  //!< Message name
};

/**
 * @brief FP_A-VERSION (version 1) message payload
 */
struct FpaVersionPayload : public FpaPayload
{  // clang-format off
    char sw_version[100]    = { 0 }; //!< Software version string
    char hw_name[100]       = { 0 }; //!< Hardware name
    char hw_ver[100]        = { 0 }; //!< Hardware version string
    char hw_uid[100]        = { 0 }; //!< Hardware UID
    char product_model[100] = { 0 }; //!< Product model
    // clang-format on

    bool SetFromMsg(const uint8_t* msg, const std::size_t msg_size) final;

    static constexpr const char* MSG_NAME = "FP_A-VERSION";  //!< Message name
};

// ---------------------------------------------------------------------------------------------------------------------

//! Pointer to FP_A payload
using FpaPayloadPtr = std::unique_ptr<FpaPayload>;

/**
 * @brief Decode FP_A message
 *
 * @param[in]  msg       Pointer to the FP_A message
 * @param[in]  msg_size  Size of the FP_A message (>= 11)
 *
 * @returns an instance of the decoded payload on success, nullptr otherwise (bad or unknown message)
 */
FpaPayloadPtr FpaDecodeMessage(const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Decode FP_A message
 *
 * @param[in]  msg  The FP_A message data
 *
 * @returns an instance of the decoded payload on success, nullptr otherwise (bad or unknown message)
 */
inline FpaPayloadPtr FpaDecodeMessage(const std::vector<uint8_t>& msg)
{
    return FpaDecodeMessage(msg.data(), msg.size());
}

/* ****************************************************************************************************************** */
}  // namespace fpa
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_PARSER_FPA_HPP__
