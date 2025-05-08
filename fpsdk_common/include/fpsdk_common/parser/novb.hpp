/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 *
 * Parts copyright (c) 2020 NovAtel Inc. See the LICENSE file.
 * Based on work by flipflip (https://github.com/phkehl)
 * The information on message structures, IDs, descriptions etc. in this file are from publicly available data, such as:
 * - https://github.com/novatel/novatel_oem7_driver/tree/master
 * - https://docs.novatel.com/OEM7/Content/Messages/Binary.htm
 * - https://docs.novatel.com/OEM7/Content/Messages/Description_of_Short_Headers.htm
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Parser NOV_B routines and types
 *
 * @page FPSDK_COMMON_PARSER_NOVB Parser NOV_B routines and types
 *
 * **API**: fpsdk_common/parser/novb.hpp and fpsdk::common::parser::novb
 *
 */
#ifndef __FPSDK_COMMON_PARSER_NOVB_HPP__
#define __FPSDK_COMMON_PARSER_NOVB_HPP__

/* LIBC/STL */
#include <cstddef>
#include <cstdint>

/* EXTERNAL */

/* PACKAGE */

namespace fpsdk {
namespace common {
namespace parser {
/**
 * @brief Parser NOV_B routines and types
 */
namespace novb {
/* ****************************************************************************************************************** */

// Doxygen is easily confused.. :-/
//! Message struct that must be packed
#ifndef _DOXYGEN_
#  define NOV_B_PACKED __attribute__((packed))
#else
#  define NOV_B_PACKED /* packed */
#endif

static constexpr uint8_t NOV_B_SYNC_1 = 0xaa;             //!< Sync char 1
static constexpr uint8_t NOV_B_SYNC_2 = 0x44;             //!< Sync char 2
static constexpr uint8_t NOV_B_SYNC_3_LONG = 0x12;        //!< Sync char 3 (long header)
static constexpr uint8_t NOV_B_SYNC_3_SHORT = 0x13;       //!< Sync char 3 (short header)
static constexpr std::size_t NOV_B_HEAD_SIZE_LONG = 28;   //!< Size of the NOV_B long header (NovbLongHeader)
static constexpr std::size_t NOV_B_HEAD_SIZE_SHORT = 12;  //!< Size of the NOV_B short header (NovbShortHeader)

/**
 * @brief Get message ID
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid NOV_B message.
 *
 * @returns message ID
 */
constexpr uint16_t NovbMsgId(const uint8_t* msg)
{
    return (((uint16_t)((uint8_t*)msg)[5] << 8) | (uint16_t)((uint8_t*)msg)[4]);
}

/**
 * @brief Check if message has the long header
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid NOV_B message.
 *
 * @returns true if the message has the long header (NovbLongHeader)
 */
constexpr bool NovbIsLongHeader(const uint8_t* msg)
{
    return msg[2] == NOV_B_SYNC_3_LONG;
}

/**
 * @brief Check if message has the short header
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid NOV_B message.
 *
 * @returns true if the message has the short header (NovbShortHeader)
 */
constexpr bool NovbIsShortHeader(const uint8_t* msg)
{
    return msg[2] == NOV_B_SYNC_3_SHORT;
}

/**
 * @brief Get NOV_B message name
 *
 * Generates a name (string) in the form "NOV_B-NAME", where NAME is a suitable stringifications of the
 * message ID if known (for example, "NOV_B-BESTGNSSPOS", respectively "%05u" formatted message ID if unknown (for
 * example, "NOV_B-MSG01234").
 *
 * @param[out] name      String to write the name to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the NOV_B message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid NOV_B message.
 *
 * @returns true if message name was generated, false if \c name buffer was too small
 */
bool NovbGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Get NOV_B message info
 *
 * This stringifies the content of some NOV_B messages, for debugging.
 *
 * @param[out] info      String to write the info to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the NOV_B message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid NOV_B message.
 *
 * @returns true if message info was generated (even if info is empty), false if \c name buffer was too small
 */
bool NovbGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @name NOV_B messages (names and IDs)
 *
 * @{
 */
// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_NOVB_MESSAGES}
static constexpr uint16_t    NOV_B_BESTGNSSPOS_MSGID        =  1429;                    //!< NOV_B-BESTGNSSPOS message ID
static constexpr const char* NOV_B_BESTGNSSPOS_STRID        = "NOV_B-BESTGNSSPOS";      //!< NOV_B-BESTGNSSPOS message name
static constexpr uint16_t    NOV_B_BESTPOS_MSGID            =    42;                    //!< NOV_B-BESTPOS message ID
static constexpr const char* NOV_B_BESTPOS_STRID            = "NOV_B-BESTPOS";          //!< NOV_B-BESTPOS message name
static constexpr uint16_t    NOV_B_BESTUTM_MSGID            =   726;                    //!< NOV_B-BESTUTM message ID
static constexpr const char* NOV_B_BESTUTM_STRID            = "NOV_B-BESTUTM";          //!< NOV_B-BESTUTM message name
static constexpr uint16_t    NOV_B_BESTVEL_MSGID            =    99;                    //!< NOV_B-BESTVEL message ID
static constexpr const char* NOV_B_BESTVEL_STRID            = "NOV_B-BESTVEL";          //!< NOV_B-BESTVEL message name
static constexpr uint16_t    NOV_B_BESTXYZ_MSGID            =   241;                    //!< NOV_B-BESTXYZ message ID
static constexpr const char* NOV_B_BESTXYZ_STRID            = "NOV_B-BESTXYZ";          //!< NOV_B-BESTXYZ message name
static constexpr uint16_t    NOV_B_CORRIMUS_MSGID           =  2264;                    //!< NOV_B-CORRIMUS message ID
static constexpr const char* NOV_B_CORRIMUS_STRID           = "NOV_B-CORRIMUS";         //!< NOV_B-CORRIMUS message name
static constexpr uint16_t    NOV_B_HEADING2_MSGID           =  1335;                    //!< NOV_B-HEADING2 message ID
static constexpr const char* NOV_B_HEADING2_STRID           = "NOV_B-HEADING2";         //!< NOV_B-HEADING2 message name
static constexpr uint16_t    NOV_B_IMURATECORRIMUS_MSGID    =  1362;                    //!< NOV_B-IMURATECORRIMUS message ID
static constexpr const char* NOV_B_IMURATECORRIMUS_STRID    = "NOV_B-IMURATECORRIMUS";  //!< NOV_B-IMURATECORRIMUS message name
static constexpr uint16_t    NOV_B_INSCONFIG_MSGID          =  1945;                    //!< NOV_B-INSCONFIG message ID
static constexpr const char* NOV_B_INSCONFIG_STRID          = "NOV_B-INSCONFIG";        //!< NOV_B-INSCONFIG message name
static constexpr uint16_t    NOV_B_INSPVA_MSGID             =   507;                    //!< NOV_B-INSPVA message ID
static constexpr const char* NOV_B_INSPVA_STRID             = "NOV_B-INSPVA";           //!< NOV_B-INSPVA message name
static constexpr uint16_t    NOV_B_INSPVAS_MSGID            =   508;                    //!< NOV_B-INSPVAS message ID
static constexpr const char* NOV_B_INSPVAS_STRID            = "NOV_B-INSPVAS";          //!< NOV_B-INSPVAS message name
static constexpr uint16_t    NOV_B_INSPVAX_MSGID            =  1465;                    //!< NOV_B-INSPVAX message ID
static constexpr const char* NOV_B_INSPVAX_STRID            = "NOV_B-INSPVAX";          //!< NOV_B-INSPVAX message name
static constexpr uint16_t    NOV_B_INSSTDEV_MSGID           =  2051;                    //!< NOV_B-INSSTDEV message ID
static constexpr const char* NOV_B_INSSTDEV_STRID           = "NOV_B-INSSTDEV";         //!< NOV_B-INSSTDEV message name
static constexpr uint16_t    NOV_B_PSRDOP2_MSGID            =  1163;                    //!< NOV_B-PSRDOP2 message ID
static constexpr const char* NOV_B_PSRDOP2_STRID            = "NOV_B-PSRDOP2";          //!< NOV_B-PSRDOP2 message name
static constexpr uint16_t    NOV_B_RAWDMI_MSGID             =  2269;                    //!< NOV_B-RAWDMI message ID
static constexpr const char* NOV_B_RAWDMI_STRID             = "NOV_B-RAWDMI";           //!< NOV_B-RAWDMI message name
static constexpr uint16_t    NOV_B_RAWIMU_MSGID             =   268;                    //!< NOV_B-RAWIMU message ID
static constexpr const char* NOV_B_RAWIMU_STRID             = "NOV_B-RAWIMU";           //!< NOV_B-RAWIMU message name
static constexpr uint16_t    NOV_B_RAWIMUSX_MSGID           =  1462;                    //!< NOV_B-RAWIMUSX message ID
static constexpr const char* NOV_B_RAWIMUSX_STRID           = "NOV_B-RAWIMUSX";         //!< NOV_B-RAWIMUSX message name
static constexpr uint16_t    NOV_B_RXSTATUS_MSGID           =    93;                    //!< NOV_B-RXSTATUS message ID
static constexpr const char* NOV_B_RXSTATUS_STRID           = "NOV_B-RXSTATUS";         //!< NOV_B-RXSTATUS message name
static constexpr uint16_t    NOV_B_TIME_MSGID               =   101;                    //!< NOV_B-TIME message ID
static constexpr const char* NOV_B_TIME_STRID               = "NOV_B-TIME";             //!< NOV_B-TIME message name
// @fp_codegen_end{FPSDK_COMMON_PARSER_NOVB_MESSAGES}
// clang-format on
///@}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief NOV_B frame long header
 *
 * See https://docs.novatel.com/OEM7/Content/Messages/Binary.htm
 */
struct NovbLongHeader
{  // clang-format off
    uint8_t  sync1;             //!< = #NOV_B_SYNC_1
    uint8_t  sync2;             //!< = #NOV_B_SYNC_2
    uint8_t  sync3;             //!< = #NOV_B_SYNC_3_LONG
    uint8_t  header_length;     //!< = sizeof(NovbLongHeader)
    uint16_t message_id;        //!< Message ID
    uint8_t  message_type;      //!< Message type (see #NovbMsgTypeSource)
    uint8_t  port_address;      //!< Port address (see #NovbPortAddr)
    uint16_t message_length;    //!< Message (payload) length
    uint16_t sequence;          //!< Sequence for groups of same messages
    uint8_t  idle_time;         //!< CPU idle [%]
    uint8_t  time_status;       //!< Time quality indicator (see #NovbTimeStatus)
    uint16_t gps_week;          //!< GPS week number
    int32_t  gps_milliseconds;  //!< GPS time of week [ms]
    uint32_t receiver_status;   //!< Receiver status flags
    uint16_t reserved;          //!< Reserved
    uint16_t receiver_version;  //!< Receiver sw version
};  // clang-format on

static_assert(sizeof(NovbLongHeader) == NOV_B_HEAD_SIZE_LONG, "");

/**
 * @brief NOV_B frame short header
 *
 * See https://docs.novatel.com/OEM7/Content/Messages/Description_of_Short_Headers.htm
 */
struct NovbShortHeader
{  // clang-format off
    uint8_t  sync1;             //!< = #NOV_B_SYNC_1
    uint8_t  sync2;             //!< = #NOV_B_SYNC_2
    uint8_t  sync3;             //!< = #NOV_B_SYNC_3_LONG
    uint8_t  message_length;    //!< Message (payload) length
    uint16_t message_id;        //!< Message ID
    uint16_t gps_week;          //!< GPS week number
    int32_t  gps_milliseconds;  //!< GPS time of week [ms]
};  // clang-format on

static_assert(sizeof(NovbShortHeader) == NOV_B_HEAD_SIZE_SHORT, "");

enum class NovbMsgTypeSource : uint8_t;  // forward declaration

/**
 * @brief Union of NOV_B long and short header
 */
struct NovbHeader
{
    union
    {
        NovbLongHeader long_header;    //!< Long header
        NovbShortHeader short_header;  //!< Short header
    };
    /**
     * @brief Check for long vs short header
     *
     * @returns true if long_header is valid, false if short_header is valid
     */
    bool IsLongHeader() const;
    /**
     * @brief Get message type source
     *
     * @returns the message type source (NovbMsgTypeSource::_MASK if unknown)
     */
    NovbMsgTypeSource Source() const;
};

static_assert(sizeof(NovbHeader) == sizeof(NovbLongHeader), "");

/**
 * @brief Message type measurement source (bits 4..0 of NovbLongHeader.message_type)
 */
enum class NovbMsgTypeSource : uint8_t
{  // clang-format off
    PRIMARY   = 0b00000000,  //!< Primary antenna
    SECONDARY = 0b00000001,  //!< Secondary antenna
    _MASK     = 0b00011111,  //!< Mask for the source part of the NovbLongHeader.message_type field
};  // clang-format on

/**
 * @brief Stringify message type measurement source
 *
 * @param[in]  message_type  Message type
 *
 * @returns a unique string for the message type measurement source
 */
const char* NovbMsgTypeSourceStr(const uint8_t message_type);

/**
 * @brief Message type format (bits 6..5 of NovbLongHeader.message_type)
 */
enum class NovbMsgTypeFormat : uint8_t
{  // clang-format off
    BINARY      = 0b00000000,  //!< Binary
    ASCII       = 0b00100000,  //!< ASCII
    AASCII_NMEA = 0b01000000,  //!< Abbreviated ASCII, NMEA
    RESERVED    = 0b01100000,  //!< Reserved
    _MASK       = 0b01100000,  //!< Mask for the format part of the NovbLongHeader.message_type field
};  // clang-format on

/**
 * @brief Message type format (bit 7 of NovbLongHeader.message_type)
 */
enum class NovbMsgTypeResp : uint8_t
{  // clang-format off
    ORIGINAL = 0b00000000,  //!< Primary GNSS receiver
    RESPONSE = 0b10000000,  //!< Secondary GNSS receiver
    _MASK    = 0b10000000,  //!< Mask for the response part of the NovbLongHeader.message_type field
};  // clang-format on

/**
 * @brief Message time status (NovbLongHeader.time_status)
 *
 * See https://docs.novatel.com/OEM7/Content/Messages/GPS_Reference_Time_Statu.htm#Table_GPSReferenceTimeStatus
 */
enum class NovbTimeStatus : uint8_t
{  // clang-format off
    UNKNOWN            =  20,  //!< Time validity is unknown
    APPROXIMATE        =  60,  //!< Time is set approximately
    COARSEADJUSTING    =  80,  //!< Time is approaching coarse precision
    COARSE             = 100,  //!< This time is valid to coarse precision
    COARSESTEERING     = 120,  //!< Time is coarse set and is being steered
    FREEWHEELING       = 130,  //!< Position is lost and the range bias cannot be calculated
    FINEADJUSTING      = 140,  //!< Time is adjusting to fine precision
    FINE               = 160,  //!< Time has fine precision
    FINEBACKUPSTEERING = 170,  //!< Time is fine set and is being steered by the backup system
    FINESTEERING       = 180,  //!< Time is fine set and is being steered
    SATTIME            = 200,  //!< Time from satellite. Only used in logs containing satellite data, such as eph or alm
};  // clang-format on

/**
 * @brief Stringify time status
 *
 * @param[in]  time_status  The time status
 *
 * @returns a unique string for the time status
 */
const char* NovbTimeStatusStr(const NovbTimeStatus time_status);

/**
 * @brief Message port address (NovbLongHeader.port_address)
 *
 * See https://docs.novatel.com/OEM7/Content/Messages/Binary.htm#Table_DetailedPortIdentifier
 */
enum class NovbPortAddr : uint8_t
{  // clang-format off
    NO_PORTS  = 0x00,  //!< No ports specified
    ALL_PORTS = 0x80,  //!< All virtual ports for all ports
    THISPORT  = 0xc0,  //!< Current COM port
    // There are many more...
};  // clang-format on

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Solution status
 *
 * See https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm#SolutionStatus
 */
enum class NovbGnssSolStat : uint32_t
{  // clang-format off
    SOL_COMPUTED     =  0,  //!< Solution computed
    INSUFFICIENT_OBS =  1,  //!< Insufficient observations
    NO_CONVERGENCE   =  2,  //!< No convergence
    SINGULARITY      =  3,  //!< Singularity at parameters matrix
    COV_TRACE        =  4,  //!< Covariance trace exceeds maximum (trace > 1000 m)
    TEST_DIST        =  5,  //!< Test distance exceeded (maximum of 3 rejections if distance >10 km)
    COLD_START       =  6,  //!< Not yet converged from cold start
    V_H_LIMIT        =  7,  //!< Height or velocity limits exceeded (in accordance with export licensing restrictions)
    VARIANCE         =  8,  //!< Variance exceeds limits
    RESIDUALS        =  9,  //!< Residuals are too large
    // There are some more...
};  // clang-format on

/**
 * @brief Extended solution status (bits)
 *
 * See https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm#ExtendedSolutionStatus
 */
enum class NovbGnssSolExtStat : uint8_t
{  // clang-format off
    SOL_VERIFIED = 0b00000001,  //!< Solution verified
    // There are more...
};  // clang-format on

/**
 * @brief Solution source (bits)
 *
 * See https://docs.novatel.com/OEM7/Content/Logs/HEADING2.htm#SolutionSource
 */
enum class NovbSolSource : uint8_t
{  // clang-format off
    PRIMARY   = 0b00000100,  //!< Primary antenna
    SECONDARY = 0b00001000,  //!< Secondary antenna
    _MASK     = 0b00001100,  //!< Mask
};  // clang-format on

/**
 * @brief Datum ID
 */
enum class NovbDatumId : uint32_t
{  // clang-format off
    WGS84 = 61,  //!< WGS-84 datum
    USER  = 63,  //!< User-defined datum
};  // clang-format on

/**
 * @brief Position or velocity type
 *
 * See https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm#Position_VelocityType
 */
enum class NovbPosOrVelType : uint32_t
{  // clang-format off
    NONE                      =  0,  //!< No solution
    FIXEDPOS                  =  1,  //!< Position has been fixed by the FIX position command or by position averaging.
    FIXEDHEIGHT               =  2,  //!< Position has been fixed by the FIX height or FIX auto command or by position averaging
    DOPPLER_VELOCITY          =  8,  //!< Velocity computed using instantaneous Doppler
    SINGLE                    = 16,  //!< Solution calculated using only data supplied by the GNSS satellites
    PSRDIFF                   = 17,  //!< Solution calculated using pseudorange differential (DGPS, DGNSS) corrections
    WAAS                      = 18,  //!< Solution calculated using corrections from an SBAS satellite
    PROPAGATED                = 19,  //!< Propagated by a Kalman filter without new observations
    L1_FLOAT                  = 32,  //!< Single-frequency RTK solution with unresolved, float carrier phase ambiguities
    NARROW_FLOAT              = 34,  //!< Multi-frequency RTK solution with unresolved, float carrier phase ambiguities
    L1_INT                    = 48,  //!< Single-frequency RTK solution with carrier phase ambiguities resolved to integers
    WIDE_INT                  = 49,  //!< Multi-frequency RTK solution with carrier phase ambiguities resolved to wide-lane integers
    NARROW_INT                = 50,  //!< Multi-frequency RTK solution with carrier phase ambiguities resolved to narrow-lane integers
    RTK_DIRECT_INS            = 51,  //!< RTK status where the RTK filter is directly initialized from the INS filter
    INS_SBAS                  = 52,  //!< INS position, where the last applied position update used a GNSS solution computed using corrections from an SBAS (WAAS) solution
    INS_PSRSP                 = 53,  //!< INS position, where the last applied position update used a single point GNSS (SINGLE) solution
    INS_PSRDIFF               = 54,  //!< INS position, where the last applied position update used a pseudorange differential GNSS (PSRDIFF) solution
    INS_RTKFLOAT              = 55,  //!< INS position, where the last applied position update used a floating ambiguity RTK (L1_FLOAT or NARROW_FLOAT) solution
    INS_RTKFIXED              = 56,  //!< INS position, where the last applied position update used a fixed integer ambiguity RTK (L1_INT, WIDE_INT or NARROW_INT) solution
    PPP_CONVERGING            = 68,  //!< Converging TerraStar-C, TerraStar-C PRO or TerraStar-X solution
    PPP                       = 69,  //!< Converged TerraStar-C, TerraStar-C PRO or TerraStar-X solution
    OPERATIONAL               = 70,  //!< Solution accuracy is within UAL operational limit
    WARNING                   = 71,  //!< Solution accuracy is outside UAL operational limit but within warning limit
    OUT_OF_BOUNDS             = 72,  //!< Solution accuracy is outside UAL limits
    INS_PPP_CONVERGING        = 73,  //!< INS position, where the last applied position update used a converging TerraStar-C, TerraStar-C PRO or TerraStar-X PPP (PPP_CONVERGING) solution
    INS_PPP                   = 74,  //!< INS position, where the last applied position update used a converged TerraStar-C, TerraStar-C PRO or TerraStar-X PPP (PPP) solution
    PPP_BASIC_CONVERGING      = 77,  //!< Converging TerraStar-L solution
    PPP_BASIC                 = 78,  //!< Converged TerraStar-L solution
    INS_PPP_BASIC_CONVERGING  = 79,  //!< INS position, where the last applied position update used a converging TerraStar-L PPP (PPP_BASIC) solution
    INS_PPP_BASIC             = 80,  //!< INS position, where the last applied position update used a converged TerraStar-L PPP (PPP_BASIC) solution
};  // clang-format on

/**
 * @brief GPS and GLONASS signals used (bits)
 *
 * See https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm#GPS_GLONASSSignalUsedMask
 */
enum class NovbSigUsedGpsGlo : uint8_t
{  // clang-format off
    GPS_L1     = 0x01,  //!< GPS L1 used in solution
    GPS_L2     = 0x02,  //!< GPS L2 used in solution
    GPS_L5     = 0x04,  //!< GPS L5 used in solution
    GLONASS_L1 = 0x10,  //!< GLONASS L1 used in solution
    GLONASS_L2 = 0x20,  //!< GLONASS L2 used in solution
    GLONASS_L5 = 0x40,  //!< GLONASS L3 used in solution
};  // clang-format on

/**
 * @brief Gelileo and BeiDou signals used (bits)
 *
 * See https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm#Galileo_BeiDouSignalUsedMask
 */
enum class NovbSigUsedGalBds : uint8_t
{  // clang-format off
    GALILEO_L1     = 0x01,  //!< Galileo E1 used in Solution
    GALILEO_L2     = 0x02,  //!< Galileo E5a used in Solution
    GALILEO_L5     = 0x04,  //!< Galileo E5b used in Solution
    GALILEO_ALTBOC = 0x08,  //!< Galileo ALTBOC used in Solution
    BEIDOU_L1      = 0x10,  //!< BeiDou B1 used in Solution (B1I and B1C)
    BEIDOU_L2      = 0x20,  //!< BeiDou B2 used in Solution (B2I, B2a and B2b)
    BEIDOU_L5      = 0x40,  //!< BeiDou B3 used in Solution (B3I)
    GALILEO_E6     = 0x80,  //!< Galileo E6 used in Solution (E6B and E6C)
};  // clang-format on

/**
 * @brief Intertial solution status
 *
 * See https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSATT.htm#InertialSolutionStatus
 */
enum class NovbInsSolStatus : uint32_t
{  // clang-format off
    INS_INACTIVE                 =  0,  //!< IMU logs are present, but the alignment routine has not started; INS is inactive
    INS_ALIGNING                 =  1,  //!< INS is in alignment mode
    INS_HIGH_VARIANCE            =  2,  //!< The INS solution uncertainty contains outliers and the solution may be outside specifications
    INS_SOLUTION_GOOD            =  3,  //!< The INS filter is in navigation mode and the INS solution is good
    INS_SOLUTION_FREE            =  6,  //!< The INS Filter is in navigation mode and the GNSS solution is suspected to be in error
    INS_ALIGNMENT_COMPLETE       =  7,  //!< The INS filter is in navigation mode, but not enough vehicle dynamics have been experienced for the system to be within specifications
    DETERMINING_ORIENTATION      =  8,  //!< INS is determining the IMU axis aligned with gravity
    WAITING_INITIAL_POS          =  9,  //!< The INS filter has determined the IMU orientation and is awaiting an initial position estimate to begin the alignment process
    WAITING_AZIMUTH              = 10,  //!< The INS filer has orientation, initial biases, initial position and valid roll/pitch estimated
    INITIALIZING_BIASES          = 11,  //!< The INS filter is estimating initial biases during the first 10 seconds of stationary data
    MOTION_DETECT                = 12,  //!< The INS filter has not completely aligned, but has detected motion
    WAITING_ALIGNMENTORIENTATION = 14,  //!< The INS filter is waiting to start alignment
};  // clang-format on

/**
 * @brief Extended intertial solution status (bits)
 *
 * See https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSATTX.htm#ExtendedSolutionStatus
 */
enum class NovbInsSolExtStatus : uint32_t
{  // clang-format off
    POSITION_UPDATE               = 0x00000001,  //!< Position update
    PHASE_UPDATE                  = 0x00000002,  //!< Phase update
    ZERO_VELOCITY_UPDATE          = 0x00000004,  //!< Zero velocity update
    WHEEL_SENSOR_UPDATE           = 0x00000008,  //!< Wheel sensor update
    HEADING_UPDATE                = 0x00000010,  //!< ALIGN (heading) update
    EXTERNAL_POSITION_UPDATE      = 0x00000020,  //!< External position update
    INS_SOLUTION_CONVERGENCE      = 0x00000040,  //!< INS solution convergence flag
    DOPPLER_UPDATE                = 0x00000080,  //!< Doppler update
    PSEUDORANGE_UPDATE            = 0x00000100,  //!< Pseudorange update
    VELOCITY_UPDATE               = 0x00000200,  //!< Velocity update
    DR_UPDATE                     = 0x00000800,  //!< Dead reckoning update
    PHASE_WINDUP_UPDATE           = 0x00001000,  //!< Phase wind up update
    COURSE_OVER_GROUND_UPDATE     = 0x00002000,  //!< Course over ground update
    EXTERNAL_VELOCITY_UPDATE      = 0x00004000,  //!< External velocity update
    EXTERNAL_ATTITUDE_UPDATE      = 0x00008000,  //!< External attitude update
    EXTERNAL_HEADING_UPDATE       = 0x00010000,  //!< External heading update
    EXTERNAL_HEIGHT_UPDATE        = 0x00020000,  //!< External height update
    SECONDARY_INS_USED            = 0x00400000,  //!< Secondary INS solution
    TURN_ON_BIAS_ESTIMATED        = 0x01000000,  //!< Turn on biases estimated
    ALIGNMENT_DIRECTION_VERIFIED  = 0x02000000,  //!< Alignment direction verified
    ALIGNMENT_INDICATION_1        = 0x04000000,  //!< Alignment indication 1
    ALIGNMENT_INDICATION_2        = 0x08000000,  //!< Alignment indication 2
    ALIGNMENT_INDICATION_3        = 0x10000000,  //!< Alignment indication 3
    NVM_SEED_INDICATION_1         = 0x20000000,  //!< NVM seed indication 1
    NVM_SEED_INDICATION_2         = 0x40000000,  //!< NVM seed indication 2
    NVM_SEED_INDICATION_3         = 0x80000000,  //!< NVM seed indication 3
};  // clang-format on

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief NOV_B-RAWDMI payload
 *
 * See https://docs.novatel.com/OEM7/Content/SPAN_Logs/RAWDMI.htm
 */
struct NovbRawdmi
{  // clang-format off
    int32_t  dmi1;            //!< DMI1 value (RC wheel or FR wheel)
    int32_t  dmi2;            //!< DMI2 value (FL wheel or YW sensor)
    int32_t  dmi3;            //!< DMI3 value (RR wheel)
    int32_t  dmi4;            //!< DMI4 value (RL wheel)
    uint32_t dmi1_valid : 1;  //!< DMI1 value valid flag (1 = dmi1 value is valid)
    uint32_t dmi2_valid : 1;  //!< DMI2 value valid flag (1 = dmi2 value is valid)
    uint32_t dmi3_valid : 1;  //!< DMI3 value valid flag (1 = dmi3 value is valid)
    uint32_t dmi4_valid : 1;  //!< DMI4 value valid flag (1 = dmi4 value is valid)
    uint32_t dmi1_type  : 7;  //!< DMI1 value type (0 = linear speed, 1 = angular velocity)
    uint32_t dmi2_type  : 7;  //!< DMI2 value type (0 = linear speed, 1 = angular velocity)
    uint32_t dmi3_type  : 7;  //!< DMI3 value type (0 = linear speed, 1 = angular velocity)
    uint32_t dmi4_type  : 7;  //!< DMI4 value type (0 = linear speed, 1 = angular velocity)
};  // clang-format on

static_assert(sizeof(NovbRawdmi) == 20, "");

/**
 * @brief NOV_B-RAWIMU payload
 *
 * See https://docs.novatel.com/OEM7/Content/SPAN_Logs/RAWIMU.htm
 */
struct NOV_B_PACKED NovbRawimu
{  // clang-format off
    uint32_t week;      //!< GNSS week number
    double   seconds;   //!< GNSS time of week [s]
    uint32_t imu_stat;  //!< The status of the IMU
    int32_t  z_accel;   //!< Change in velocity count along z axis
    int32_t  y_accel;   //!< Change in velocity count along y axis
    int32_t  x_accel;   //!< Change in velocity count along x axis
    int32_t  z_gyro;    //!< Change in angle count around z axis
    int32_t  y_gyro;    //!< Change in angle count around y axis
    int32_t  x_gyro;    //!< Change in angle count around x axis
};  // clang-format on

static_assert(sizeof(NovbRawimu) == 40, "");

/**
 * @brief NOV_B-BESTGNSSPOS payload
 *
 * See https://docs.novatel.com/OEM7/Content/SPAN_Logs/BESTGNSSPOS.htm
 */
struct NovbBestgnsspos
{  // clang-format off
    uint32_t sol_stat;                 //!< See #NovbGnssSolStat
    uint32_t pos_type;                 //!< See #NovbPosOrVelType
    double   lat;                      //!< @todo document
    double   lon;                      //!< @todo document
    double   hgt;                      //!< @todo document
    float    undulation;               //!< @todo document
    uint32_t datum_id;                 //!< See #NovbDatumId
    float    lat_stdev;                //!< @todo document
    float    lon_stdev;                //!< @todo document
    float    hgt_stdev;                //!< @todo document
    uint8_t  stn_id[4];                //!< @todo document
    float    diff_age;                 //!< @todo document
    float    sol_age;                  //!< @todo document
    uint8_t  num_svs;                  //!< @todo document
    uint8_t  num_sol_svs;              //!< @todo document
    uint8_t  num_sol_l1_svs;           //!< @todo document
    uint8_t  num_sol_multi_svs;        //!< @todo document
    uint8_t  reserved;                 //!< @todo document
    uint8_t  ext_sol_stat;             //!< See #NovbGnssSolExtStat
    uint8_t  galileo_beidou_sig_mask;  //!< See #NovbSigUsedGalBds
    uint8_t  gps_glonass_sig_mask;     //!< See #NovbSigUsedGpsGlo
};  // clang-format on

static_assert(sizeof(NovbBestgnsspos) == 72, "");

/**
 * @brief NOV_B-BESTVEL payload
 *
 * See https://docs.novatel.com/OEM7/Content/Logs/BESTVEL.htm
 */
struct NOV_B_PACKED NovbBestvel
{  // clang-format off
    uint32_t sol_stat;                 //!< See #NovbGnssSolStat
    uint32_t vel_type;                 //!< See #NovbPosOrVelType
    float    latency;                  //!< @todo document
    float    age;                      //!< @todo document
    double   hor_spd;                  //!< @todo document
    double   trk_gnd;                  //!< @todo document
    double   vert_spd;                 //!< @todo document
    uint32_t reserved;                 //!< @todo document
};  // clang-format on

static_assert(sizeof(NovbBestvel) == 44, "");

/**
 * @brief NOV_B-HEADING2 payload
 *
 * See https://docs.novatel.com/OEM7/Content/Logs/HEADING2.htm
 */
struct NovbHeading2
{  // clang-format off
    uint32_t sol_status;               //!< See #NovbGnssSolStat
    uint32_t pos_type;                 //!< See #NovbPosOrVelType
    float    length;                   //!< @todo document
    float    heading;                  //!< @todo document
    float    pitch;                    //!< @todo document
    float    reserved;                 //!< @todo document
    float    heading_stdev;            //!< @todo document
    float    pitch_stdev;              //!< @todo document
    uint8_t  rover_stn_id[4];          //!< @todo document
    uint8_t  master_stn_id[4];         //!< @todo document
    uint8_t  num_sv_tracked;           //!< @todo document
    uint8_t  num_sv_in_sol;            //!< @todo document
    uint8_t  num_sv_obs;               //!< @todo document
    uint8_t  num_sv_multi;             //!< @todo document
    uint8_t  sol_source;               //!< See #NovbSolSource
    uint8_t  ext_sol_status;           //!< See #NovbGnssSolExtStat
    uint8_t  galileo_beidou_sig_mask;  //!< See #NovbSigUsedGalBds
    uint8_t  gps_glonass_sig_mask;     //!< See #NovbSigUsedGpsGlo
};  // clang-format on

static_assert(sizeof(NovbHeading2) == 48, "");

/**
 * @brief NOV_B-INSPVAX payload
 *
 * See https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSPVAX.htm
 */
struct NOV_B_PACKED NovbInspvax
{  // clang-format off
    uint32_t ins_status;            //!< See #NovbInsSolStatus
    uint32_t pos_type;              //!< See #NovbPosOrVelType
    double   latitude;              //!< @todo document
    double   longitude;             //!< @todo document
    double   height;                //!< @todo document
    float    undulation;            //!< @todo document
    double   north_velocity;        //!< @todo document
    double   east_velocity;         //!< @todo document
    double   up_velocity;           //!< @todo document
    double   roll;                  //!< @todo document
    double   pitch;                 //!< @todo document
    double   azimuth;               //!< @todo document
    float    latitude_stdev;        //!< @todo document
    float    longitude_stdev;       //!< @todo document
    float    height_stdev;          //!< @todo document
    float    north_velocity_stdev;  //!< @todo document
    float    east_velocity_stdev;   //!< @todo document
    float    up_velocity_stdev;     //!< @todo document
    float    roll_stdev;            //!< @todo document
    float    pitch_stdev;           //!< @todo document
    float    azimuth_stdev;         //!< @todo document
    uint32_t extended_status;       //!< See #NovbInsSolExtStatus
    uint16_t time_since_update;     //!< @todo document
};  // clang-format on

static_assert(sizeof(NovbInspvax) == 126, "");

#undef NOV_B_PACKED

/* ****************************************************************************************************************** */
}  // namespace novb
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_PARSER_NOVB_HPP__
