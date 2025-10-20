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
 * - RTCM (https://www.rtcm.org/publications)
 * - https://www.use-snip.com/kb/knowledge-base/rtcm-3-message-list/
 * - Forums, stackoverflow, et al.
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Parser RTCM3 routines and types
 *
 * @page FPSDK_COMMON_PARSER_RTCM3 Parser RTCM3 routines and types
 *
 * **API**: fpsdk_common/parser/rtcm3.hpp and fpsdk::common::parser::rtcm3
 *
 */
#ifndef __FPSDK_COMMON_PARSER_RTCM3_HPP__
#define __FPSDK_COMMON_PARSER_RTCM3_HPP__

/* LIBC/STL */
#include <cstdint>

/* EXTERNAL */

/* PACKAGE */

namespace fpsdk {
namespace common {
namespace parser {
/**
 * @brief Parser RTCM3 routines and types
 */
namespace rtcm3 {
/* ****************************************************************************************************************** */

static constexpr uint8_t RTCM3_PREAMBLE = 0xd3;                         //!< RTCM3 frame preamble
static constexpr std::size_t RTCM3_HEAD_SIZE = 3;                       //!< Size of RTCM3 header (in bytes)
static constexpr std::size_t RTCM3_FRAME_SIZE = (RTCM3_HEAD_SIZE + 3);  //!< Size of RTCM3 frame

/**
 * @brief Get RTCM3 message type (DF002, 12 bits, unsigned)
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid RTCM3 message.
 *
 * @returns the RTCM3 message type
 */
constexpr uint16_t Rtcm3Type(const uint8_t* msg)
{
    return (((uint16_t)(((uint8_t*)(msg))[RTCM3_HEAD_SIZE + 0]) << 4) |
            (uint16_t)((((uint8_t*)(msg))[RTCM3_HEAD_SIZE + 1] >> 4) & 0x0f));
}

/**
 * @brief Get sub-type for a RTCM3 message (like the type 4072)
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid RTCM3 message with a
 *       sub-type DF002 as the first field.
 *
 * @returns the sub-type of a RTCM3 message
 */
constexpr uint16_t Rtcm3SubType(const uint8_t* msg)
{
    return (
        ((uint16_t)(((uint8_t*)(msg))[RTCM3_HEAD_SIZE + 1] & 0x0f) << 8) | (((uint8_t*)(msg))[RTCM3_HEAD_SIZE + 2]));
}

/**
 * @brief Get RTCM3 message name
 *
 * Generates a name (string) in the form "RTCM-TYPENNNN_P", where NNNN is the message type and _P a suffix for some
 * proprietary messages (for example, RTCM3-TYPE4072_0 for sub-type 0 of that message).
 *
 * @param[out] name      String to write the name to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the RTCM3 message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid RTCM3 message.
 *
 * @returns true if message name was generated, false if \c name buffer was too small
 */
bool Rtcm3GetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Get RTCM3 message info
 *
 * This stringifies the content of some RTCM3 messages, for debugging.
 *
 * @param[out] info      String to write the info to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the RTCM3 message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid RTCM3 message.
 *
 * @returns true if message info was generated (even if info is empty), false if \c name buffer was too small
 */
bool Rtcm3GetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Get description for a RTCM3 message type
 *
 * @param[in]  type     Message type
 * @param[in]  subtype  Message sub-type (if appropriate)
 *
 * @return the description of the message type, if available, NULL otherwise
 */
const char* Rtcm3GetTypeDesc(const uint16_t type, const uint16_t subtype = 0xffff);

/**
 * @brief Get RTCM3 unsigned integer
 *
 * @param[in]  data  Pointer to the start of the RTCM3 data (at offset RTCM3_HEAD_SIZE into the message)
 * @param[in]  offs  Offset in [bits] to the start of the value
 * @param[in]  size  Size in [bits] of the value
 *
 * @returns the unsigned integer value (zero padded 64-bit value)
 */
uint64_t Rtcm3GetUnsigned(const uint8_t* data, const std::size_t offs, const std::size_t size);

/**
 * @brief Get RTCM3 signed integer
 *
 * @param[in]  data  Pointer to the start of the RTCM3 data (at offset RTCM3_HEAD_SIZE into the message)
 * @param[in]  offs  Offset in [bits] to the start of the value
 * @param[in]  size  Size in [bits] of the value
 *
 * @returns the signed integer value (sign-extended 64-bit value)
 */
int64_t Rtcm3GetSigned(const uint8_t* data, const std::size_t offs, const std::size_t size);

/**
 * @brief Count number of set bits
 *
 * @param[in]  mask  The value to count the number of set bits in
 *
 * @returns the number of set (non-zero) bits in the value
 */
std::size_t Rtcm3CountBits(const uint64_t mask);

/**
 * @brief Set RTCM3 unsigned integer
 *
 * @param[in,out]  data  Pointer to the start of the RTCM3 data (at offset RTCM3_HEAD_SIZE into the message)
 * @param[in]      offs  Offset in [bits] to the start of the value
 * @param[in]      size  Size in [bits] of the value
 * @param[in]      value The value to set
 */
void Rtcm3SetUnsigned(uint8_t* data, const std::size_t offs, const std::size_t size, const uint64_t value);

/**
 * @brief Get RTCM3 signed integer
 *
 * @param[in,out]  data  Pointer to the start of the RTCM3 data (at offset RTCM3_HEAD_SIZE into the message)
 * @param[in]      offs  Offset in [bits] to the start of the value
 * @param[in]      size  Size in [bits] of the value
 * @param[in]      value The value to set
 */
void Rtcm3SetSigned(uint8_t* data, const std::size_t offs, const std::size_t size, const int64_t value);

/**
 * @brief Antenna reference point
 */
struct Rtcm3Arp
{
    // clang-format off
    int    ref_sta_id_ = -1;   //!< Reference station ID
    double ecef_x_     = 0.0;  //!< ECEF X [m]
    double ecef_y_     = 0.0;  //!< ECEF Y [m]
    double ecef_z_     = 0.0;  //!< ECEF Z [m]
    int    phy_sta_id_ = -1;   //!< Physical station ID           (only type 1032)
    double ant_height_ = 0.0;  //!< Antenna height [m]            (only type 1006)
    int    gps_ind_    = -1;   //!< GPS indicator                 (only type 1005/1006)
    int    glo_ind_    = -1;   //!< GLONASS indicator             (only type 1005/1006)
    int    gal_ind_    = -1;   //!< Galileo indicator             (only type 1005/1006)
    int    ref_ind_    = -1;   //!< Reference station indicator   (only type 1005/1006)
    int    osc_ind_    = -1;   //!< Oscillator indicator          (only type 1005/1006)
    int    cyc_ind_    = -1;   //!< Quarter cycle indicator       (only type 1005/1006)
};

/**
 * @brief Get ARP from message types 1005, 1006 or 1032
 *
 * @param[in]   msg  The raw RTCM3 message
 * @param[out]  arp  The header information
 *
 * @returns true if the ARP was successfully extracted, false otherwise
 */
bool Rtcm3GetArp(const uint8_t* msg, Rtcm3Arp& arp);

/**
 * @brief Antenna info
 */
struct Rtcm3Ant  // clang-format off
{
    int     ref_sta_id_;       //!< @todo documentation
    char    ant_desc_[32];     //!< @todo documentation
    char    ant_serial_[32];   //!< @todo documentation
    uint8_t ant_setup_id_;     //!< @todo documentation
    char    rx_type_[32];      //!< @todo documentation
    char    rx_fw_[32];        //!< @todo documentation
    char    rx_serial_[32];    //!< @todo documentation
};  // clang-format on

/**
 * @brief Get (some) antenna info from  message type 1007, 1008 or 1033
 *
 * @param[in]   msg  The raw RTCM3 message
 * @param[out]  ant  The antenna information
 *
 * @returns true if the antenna info was successfully extracted, false otherwise
 */
bool Rtcm3GetAnt(const uint8_t* msg, Rtcm3Ant& ant);

/**
 * @brief RTCM3 MSM messages GNSS
 */
enum class Rtcm3MsmGnss : uint16_t
{  // clang-format off
    GPS    =  70,  //!< GPS
    GLO    =  80,  //!< GLONASS
    GAL    =  90,  //!< Galileo
    SBAS   = 100,  //!< SBAS
    QZSS   = 110,  //!< QZSS
    BDS    = 120,  //!< BeiDou
    NAVIC  = 130,  //!< NavIC
};  // clang-format on

/**
 * @brief RTCM3 MSM messages type
 */
enum class Rtcm3MsmType : uint16_t
{
    MSM1 = 1,  //!< Type 1 (C)
    MSM2 = 2,  //!< Type 2 (L)
    MSM3 = 3,  //!< Type 3 (C, L)
    MSM4 = 4,  //!< Type 4 (full C, full L, S)
    MSM5 = 5,  //!< Type 5 (full C, full L, S, D)
    MSM6 = 6,  //!< Type 6 (ext full C, ext full L, S)
    MSM7 = 7,  //!< Type 7 (ext full C, ext full L, S, D)
};

/**
 * @brief RTMC3 message type to MSM GNSS and type
 *
 * @param[in]   msg_type  The RTMC3 message type
 * @param[out]  gnss      The GNSS (or NULL)
 * @param[out]  msm       The MSM type (number) (or NULL)
 *
 * @returns true if msgType was a valid and known RTCM3 MSM message
 */
bool Rtcm3TypeToMsm(uint16_t msg_type, Rtcm3MsmGnss& gnss, Rtcm3MsmType& msm);

/**
 * @brief RTCM3 MSM messages common header
 */
struct Rtcm3MsmHeader
{
    Rtcm3MsmGnss gnss_;  //!< GNSS
    Rtcm3MsmType msm_;   //!< MSM

    uint16_t msg_type_;    //!< Message number (DF002, uint12)
    uint16_t ref_sta_id_;  //!< Reference station ID (DF003, uint12)
    union
    {
        double any_tow_;   //!< Any time of week [s]
        double gps_tow_;   //!< GPS time of week [s] (DF004 uint30 [ms])
        double sbas_tow_;  //!< SBAS time of week [s] (DF004 uint30 [ms])
        double glo_tow_;   //!< GLONASS time of week [s] (DF416 uint3 [d], DF034 uint27 [ms])
        double gal_tow_;   //!< Galileo time of week [s] (DF248 uint30 [ms])
        double qzss_tow_;  //!< QZSS time of week [s] (DF428 uint30 [ms])
        double bds_tow_;   //!< BeiDou time of week [s] (DF427 uint30 [ms])
    };
    bool multi_msg_bit_;    //!< Multiple message bit (DF393, bit(1))
    uint8_t iods_;          //!< IODS, issue of data station (DF409, uint3)
    uint8_t clk_steering_;  //!< Clock steering indicator (DF411, uint2)
    uint8_t ext_clock_;     //!< External clock indicator (DF412, uint2)
    bool smooth_;           //!< GNSS divergence-free smoothing indicator (DF417, bit(1))
    uint8_t smooth_int_;    //!< GNSS smoothing interval (DF418, bit(3))
    uint64_t sat_mask_;     //!< GNSS satellite mask (DF394, bit(64))
    uint64_t sig_mask_;     //!< GNSS signal mask (DF395, bit(64))
    uint64_t cell_mask_;    //!< GNSS cell mask (DF396, bit(64))

    int num_sat_;   //!< Number of satellites (in satMask)
    int num_sig_;   //!< Number of signals (in sigMask)
    int num_cell_;  //!< Number of cells (in cellMask)
};

/**
 * @brief Extract RTCM3 MSM message common header
 *
 * @param[in]   msg     The raw RTCM3 message
 * @param[out]  header  The header information
 *
 * @returns true if the header was successfully extracted, false otherwise
 */
bool Rtcm3GetMsmHeader(const uint8_t* msg, Rtcm3MsmHeader& header);

/**
 * @name RTCM3 messages (names and IDs)
 *
 * @{
 */
// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_RTCM3_MESSAGES}
static constexpr uint16_t    RTCM3_TYPE1001_MSGID           = 1001;                     //!< RTCM3-TYPE1001 message ID
static constexpr const char* RTCM3_TYPE1001_STRID           = "RTCM3-TYPE1001";         //!< RTCM3-TYPE1001 message name
static constexpr uint16_t    RTCM3_TYPE1002_MSGID           = 1002;                     //!< RTCM3-TYPE1002 message ID
static constexpr const char* RTCM3_TYPE1002_STRID           = "RTCM3-TYPE1002";         //!< RTCM3-TYPE1002 message name
static constexpr uint16_t    RTCM3_TYPE1003_MSGID           = 1003;                     //!< RTCM3-TYPE1003 message ID
static constexpr const char* RTCM3_TYPE1003_STRID           = "RTCM3-TYPE1003";         //!< RTCM3-TYPE1003 message name
static constexpr uint16_t    RTCM3_TYPE1004_MSGID           = 1004;                     //!< RTCM3-TYPE1004 message ID
static constexpr const char* RTCM3_TYPE1004_STRID           = "RTCM3-TYPE1004";         //!< RTCM3-TYPE1004 message name
static constexpr uint16_t    RTCM3_TYPE1005_MSGID           = 1005;                     //!< RTCM3-TYPE1005 message ID
static constexpr const char* RTCM3_TYPE1005_STRID           = "RTCM3-TYPE1005";         //!< RTCM3-TYPE1005 message name
static constexpr uint16_t    RTCM3_TYPE1006_MSGID           = 1006;                     //!< RTCM3-TYPE1006 message ID
static constexpr const char* RTCM3_TYPE1006_STRID           = "RTCM3-TYPE1006";         //!< RTCM3-TYPE1006 message name
static constexpr uint16_t    RTCM3_TYPE1007_MSGID           = 1007;                     //!< RTCM3-TYPE1007 message ID
static constexpr const char* RTCM3_TYPE1007_STRID           = "RTCM3-TYPE1007";         //!< RTCM3-TYPE1007 message name
static constexpr uint16_t    RTCM3_TYPE1008_MSGID           = 1008;                     //!< RTCM3-TYPE1008 message ID
static constexpr const char* RTCM3_TYPE1008_STRID           = "RTCM3-TYPE1008";         //!< RTCM3-TYPE1008 message name
static constexpr uint16_t    RTCM3_TYPE1009_MSGID           = 1009;                     //!< RTCM3-TYPE1009 message ID
static constexpr const char* RTCM3_TYPE1009_STRID           = "RTCM3-TYPE1009";         //!< RTCM3-TYPE1009 message name
static constexpr uint16_t    RTCM3_TYPE1010_MSGID           = 1010;                     //!< RTCM3-TYPE1010 message ID
static constexpr const char* RTCM3_TYPE1010_STRID           = "RTCM3-TYPE1010";         //!< RTCM3-TYPE1010 message name
static constexpr uint16_t    RTCM3_TYPE1011_MSGID           = 1011;                     //!< RTCM3-TYPE1011 message ID
static constexpr const char* RTCM3_TYPE1011_STRID           = "RTCM3-TYPE1011";         //!< RTCM3-TYPE1011 message name
static constexpr uint16_t    RTCM3_TYPE1012_MSGID           = 1012;                     //!< RTCM3-TYPE1012 message ID
static constexpr const char* RTCM3_TYPE1012_STRID           = "RTCM3-TYPE1012";         //!< RTCM3-TYPE1012 message name
static constexpr uint16_t    RTCM3_TYPE1030_MSGID           = 1030;                     //!< RTCM3-TYPE1030 message ID
static constexpr const char* RTCM3_TYPE1030_STRID           = "RTCM3-TYPE1030";         //!< RTCM3-TYPE1030 message name
static constexpr uint16_t    RTCM3_TYPE1031_MSGID           = 1031;                     //!< RTCM3-TYPE1031 message ID
static constexpr const char* RTCM3_TYPE1031_STRID           = "RTCM3-TYPE1031";         //!< RTCM3-TYPE1031 message name
static constexpr uint16_t    RTCM3_TYPE1032_MSGID           = 1032;                     //!< RTCM3-TYPE1032 message ID
static constexpr const char* RTCM3_TYPE1032_STRID           = "RTCM3-TYPE1032";         //!< RTCM3-TYPE1032 message name
static constexpr uint16_t    RTCM3_TYPE1033_MSGID           = 1033;                     //!< RTCM3-TYPE1033 message ID
static constexpr const char* RTCM3_TYPE1033_STRID           = "RTCM3-TYPE1033";         //!< RTCM3-TYPE1033 message name
static constexpr uint16_t    RTCM3_TYPE1230_MSGID           = 1230;                     //!< RTCM3-TYPE1230 message ID
static constexpr const char* RTCM3_TYPE1230_STRID           = "RTCM3-TYPE1230";         //!< RTCM3-TYPE1230 message name
static constexpr uint16_t    RTCM3_TYPE1071_MSGID           = 1071;                     //!< RTCM3-TYPE1071 message ID
static constexpr const char* RTCM3_TYPE1071_STRID           = "RTCM3-TYPE1071";         //!< RTCM3-TYPE1071 message name
static constexpr uint16_t    RTCM3_TYPE1072_MSGID           = 1072;                     //!< RTCM3-TYPE1072 message ID
static constexpr const char* RTCM3_TYPE1072_STRID           = "RTCM3-TYPE1072";         //!< RTCM3-TYPE1072 message name
static constexpr uint16_t    RTCM3_TYPE1073_MSGID           = 1073;                     //!< RTCM3-TYPE1073 message ID
static constexpr const char* RTCM3_TYPE1073_STRID           = "RTCM3-TYPE1073";         //!< RTCM3-TYPE1073 message name
static constexpr uint16_t    RTCM3_TYPE1074_MSGID           = 1074;                     //!< RTCM3-TYPE1074 message ID
static constexpr const char* RTCM3_TYPE1074_STRID           = "RTCM3-TYPE1074";         //!< RTCM3-TYPE1074 message name
static constexpr uint16_t    RTCM3_TYPE1075_MSGID           = 1075;                     //!< RTCM3-TYPE1075 message ID
static constexpr const char* RTCM3_TYPE1075_STRID           = "RTCM3-TYPE1075";         //!< RTCM3-TYPE1075 message name
static constexpr uint16_t    RTCM3_TYPE1076_MSGID           = 1076;                     //!< RTCM3-TYPE1076 message ID
static constexpr const char* RTCM3_TYPE1076_STRID           = "RTCM3-TYPE1076";         //!< RTCM3-TYPE1076 message name
static constexpr uint16_t    RTCM3_TYPE1077_MSGID           = 1077;                     //!< RTCM3-TYPE1077 message ID
static constexpr const char* RTCM3_TYPE1077_STRID           = "RTCM3-TYPE1077";         //!< RTCM3-TYPE1077 message name
static constexpr uint16_t    RTCM3_TYPE1081_MSGID           = 1081;                     //!< RTCM3-TYPE1081 message ID
static constexpr const char* RTCM3_TYPE1081_STRID           = "RTCM3-TYPE1081";         //!< RTCM3-TYPE1081 message name
static constexpr uint16_t    RTCM3_TYPE1082_MSGID           = 1082;                     //!< RTCM3-TYPE1082 message ID
static constexpr const char* RTCM3_TYPE1082_STRID           = "RTCM3-TYPE1082";         //!< RTCM3-TYPE1082 message name
static constexpr uint16_t    RTCM3_TYPE1083_MSGID           = 1083;                     //!< RTCM3-TYPE1083 message ID
static constexpr const char* RTCM3_TYPE1083_STRID           = "RTCM3-TYPE1083";         //!< RTCM3-TYPE1083 message name
static constexpr uint16_t    RTCM3_TYPE1084_MSGID           = 1084;                     //!< RTCM3-TYPE1084 message ID
static constexpr const char* RTCM3_TYPE1084_STRID           = "RTCM3-TYPE1084";         //!< RTCM3-TYPE1084 message name
static constexpr uint16_t    RTCM3_TYPE1085_MSGID           = 1085;                     //!< RTCM3-TYPE1085 message ID
static constexpr const char* RTCM3_TYPE1085_STRID           = "RTCM3-TYPE1085";         //!< RTCM3-TYPE1085 message name
static constexpr uint16_t    RTCM3_TYPE1086_MSGID           = 1086;                     //!< RTCM3-TYPE1086 message ID
static constexpr const char* RTCM3_TYPE1086_STRID           = "RTCM3-TYPE1086";         //!< RTCM3-TYPE1086 message name
static constexpr uint16_t    RTCM3_TYPE1087_MSGID           = 1087;                     //!< RTCM3-TYPE1087 message ID
static constexpr const char* RTCM3_TYPE1087_STRID           = "RTCM3-TYPE1087";         //!< RTCM3-TYPE1087 message name
static constexpr uint16_t    RTCM3_TYPE1091_MSGID           = 1091;                     //!< RTCM3-TYPE1091 message ID
static constexpr const char* RTCM3_TYPE1091_STRID           = "RTCM3-TYPE1091";         //!< RTCM3-TYPE1091 message name
static constexpr uint16_t    RTCM3_TYPE1092_MSGID           = 1092;                     //!< RTCM3-TYPE1092 message ID
static constexpr const char* RTCM3_TYPE1092_STRID           = "RTCM3-TYPE1092";         //!< RTCM3-TYPE1092 message name
static constexpr uint16_t    RTCM3_TYPE1093_MSGID           = 1093;                     //!< RTCM3-TYPE1093 message ID
static constexpr const char* RTCM3_TYPE1093_STRID           = "RTCM3-TYPE1093";         //!< RTCM3-TYPE1093 message name
static constexpr uint16_t    RTCM3_TYPE1094_MSGID           = 1094;                     //!< RTCM3-TYPE1094 message ID
static constexpr const char* RTCM3_TYPE1094_STRID           = "RTCM3-TYPE1094";         //!< RTCM3-TYPE1094 message name
static constexpr uint16_t    RTCM3_TYPE1095_MSGID           = 1095;                     //!< RTCM3-TYPE1095 message ID
static constexpr const char* RTCM3_TYPE1095_STRID           = "RTCM3-TYPE1095";         //!< RTCM3-TYPE1095 message name
static constexpr uint16_t    RTCM3_TYPE1096_MSGID           = 1096;                     //!< RTCM3-TYPE1096 message ID
static constexpr const char* RTCM3_TYPE1096_STRID           = "RTCM3-TYPE1096";         //!< RTCM3-TYPE1096 message name
static constexpr uint16_t    RTCM3_TYPE1097_MSGID           = 1097;                     //!< RTCM3-TYPE1097 message ID
static constexpr const char* RTCM3_TYPE1097_STRID           = "RTCM3-TYPE1097";         //!< RTCM3-TYPE1097 message name
static constexpr uint16_t    RTCM3_TYPE1101_MSGID           = 1101;                     //!< RTCM3-TYPE1101 message ID
static constexpr const char* RTCM3_TYPE1101_STRID           = "RTCM3-TYPE1101";         //!< RTCM3-TYPE1101 message name
static constexpr uint16_t    RTCM3_TYPE1102_MSGID           = 1102;                     //!< RTCM3-TYPE1102 message ID
static constexpr const char* RTCM3_TYPE1102_STRID           = "RTCM3-TYPE1102";         //!< RTCM3-TYPE1102 message name
static constexpr uint16_t    RTCM3_TYPE1103_MSGID           = 1103;                     //!< RTCM3-TYPE1103 message ID
static constexpr const char* RTCM3_TYPE1103_STRID           = "RTCM3-TYPE1103";         //!< RTCM3-TYPE1103 message name
static constexpr uint16_t    RTCM3_TYPE1104_MSGID           = 1104;                     //!< RTCM3-TYPE1104 message ID
static constexpr const char* RTCM3_TYPE1104_STRID           = "RTCM3-TYPE1104";         //!< RTCM3-TYPE1104 message name
static constexpr uint16_t    RTCM3_TYPE1105_MSGID           = 1105;                     //!< RTCM3-TYPE1105 message ID
static constexpr const char* RTCM3_TYPE1105_STRID           = "RTCM3-TYPE1105";         //!< RTCM3-TYPE1105 message name
static constexpr uint16_t    RTCM3_TYPE1106_MSGID           = 1106;                     //!< RTCM3-TYPE1106 message ID
static constexpr const char* RTCM3_TYPE1106_STRID           = "RTCM3-TYPE1106";         //!< RTCM3-TYPE1106 message name
static constexpr uint16_t    RTCM3_TYPE1107_MSGID           = 1107;                     //!< RTCM3-TYPE1107 message ID
static constexpr const char* RTCM3_TYPE1107_STRID           = "RTCM3-TYPE1107";         //!< RTCM3-TYPE1107 message name
static constexpr uint16_t    RTCM3_TYPE1111_MSGID           = 1111;                     //!< RTCM3-TYPE1111 message ID
static constexpr const char* RTCM3_TYPE1111_STRID           = "RTCM3-TYPE1111";         //!< RTCM3-TYPE1111 message name
static constexpr uint16_t    RTCM3_TYPE1112_MSGID           = 1112;                     //!< RTCM3-TYPE1112 message ID
static constexpr const char* RTCM3_TYPE1112_STRID           = "RTCM3-TYPE1112";         //!< RTCM3-TYPE1112 message name
static constexpr uint16_t    RTCM3_TYPE1113_MSGID           = 1113;                     //!< RTCM3-TYPE1113 message ID
static constexpr const char* RTCM3_TYPE1113_STRID           = "RTCM3-TYPE1113";         //!< RTCM3-TYPE1113 message name
static constexpr uint16_t    RTCM3_TYPE1114_MSGID           = 1114;                     //!< RTCM3-TYPE1114 message ID
static constexpr const char* RTCM3_TYPE1114_STRID           = "RTCM3-TYPE1114";         //!< RTCM3-TYPE1114 message name
static constexpr uint16_t    RTCM3_TYPE1115_MSGID           = 1115;                     //!< RTCM3-TYPE1115 message ID
static constexpr const char* RTCM3_TYPE1115_STRID           = "RTCM3-TYPE1115";         //!< RTCM3-TYPE1115 message name
static constexpr uint16_t    RTCM3_TYPE1116_MSGID           = 1116;                     //!< RTCM3-TYPE1116 message ID
static constexpr const char* RTCM3_TYPE1116_STRID           = "RTCM3-TYPE1116";         //!< RTCM3-TYPE1116 message name
static constexpr uint16_t    RTCM3_TYPE1117_MSGID           = 1117;                     //!< RTCM3-TYPE1117 message ID
static constexpr const char* RTCM3_TYPE1117_STRID           = "RTCM3-TYPE1117";         //!< RTCM3-TYPE1117 message name
static constexpr uint16_t    RTCM3_TYPE1121_MSGID           = 1121;                     //!< RTCM3-TYPE1121 message ID
static constexpr const char* RTCM3_TYPE1121_STRID           = "RTCM3-TYPE1121";         //!< RTCM3-TYPE1121 message name
static constexpr uint16_t    RTCM3_TYPE1122_MSGID           = 1122;                     //!< RTCM3-TYPE1122 message ID
static constexpr const char* RTCM3_TYPE1122_STRID           = "RTCM3-TYPE1122";         //!< RTCM3-TYPE1122 message name
static constexpr uint16_t    RTCM3_TYPE1123_MSGID           = 1123;                     //!< RTCM3-TYPE1123 message ID
static constexpr const char* RTCM3_TYPE1123_STRID           = "RTCM3-TYPE1123";         //!< RTCM3-TYPE1123 message name
static constexpr uint16_t    RTCM3_TYPE1124_MSGID           = 1124;                     //!< RTCM3-TYPE1124 message ID
static constexpr const char* RTCM3_TYPE1124_STRID           = "RTCM3-TYPE1124";         //!< RTCM3-TYPE1124 message name
static constexpr uint16_t    RTCM3_TYPE1125_MSGID           = 1125;                     //!< RTCM3-TYPE1125 message ID
static constexpr const char* RTCM3_TYPE1125_STRID           = "RTCM3-TYPE1125";         //!< RTCM3-TYPE1125 message name
static constexpr uint16_t    RTCM3_TYPE1126_MSGID           = 1126;                     //!< RTCM3-TYPE1126 message ID
static constexpr const char* RTCM3_TYPE1126_STRID           = "RTCM3-TYPE1126";         //!< RTCM3-TYPE1126 message name
static constexpr uint16_t    RTCM3_TYPE1127_MSGID           = 1127;                     //!< RTCM3-TYPE1127 message ID
static constexpr const char* RTCM3_TYPE1127_STRID           = "RTCM3-TYPE1127";         //!< RTCM3-TYPE1127 message name
static constexpr uint16_t    RTCM3_TYPE1131_MSGID           = 1131;                     //!< RTCM3-TYPE1131 message ID
static constexpr const char* RTCM3_TYPE1131_STRID           = "RTCM3-TYPE1131";         //!< RTCM3-TYPE1131 message name
static constexpr uint16_t    RTCM3_TYPE1132_MSGID           = 1132;                     //!< RTCM3-TYPE1132 message ID
static constexpr const char* RTCM3_TYPE1132_STRID           = "RTCM3-TYPE1132";         //!< RTCM3-TYPE1132 message name
static constexpr uint16_t    RTCM3_TYPE1133_MSGID           = 1133;                     //!< RTCM3-TYPE1133 message ID
static constexpr const char* RTCM3_TYPE1133_STRID           = "RTCM3-TYPE1133";         //!< RTCM3-TYPE1133 message name
static constexpr uint16_t    RTCM3_TYPE1134_MSGID           = 1134;                     //!< RTCM3-TYPE1134 message ID
static constexpr const char* RTCM3_TYPE1134_STRID           = "RTCM3-TYPE1134";         //!< RTCM3-TYPE1134 message name
static constexpr uint16_t    RTCM3_TYPE1135_MSGID           = 1135;                     //!< RTCM3-TYPE1135 message ID
static constexpr const char* RTCM3_TYPE1135_STRID           = "RTCM3-TYPE1135";         //!< RTCM3-TYPE1135 message name
static constexpr uint16_t    RTCM3_TYPE1136_MSGID           = 1136;                     //!< RTCM3-TYPE1136 message ID
static constexpr const char* RTCM3_TYPE1136_STRID           = "RTCM3-TYPE1136";         //!< RTCM3-TYPE1136 message name
static constexpr uint16_t    RTCM3_TYPE1137_MSGID           = 1137;                     //!< RTCM3-TYPE1137 message ID
static constexpr const char* RTCM3_TYPE1137_STRID           = "RTCM3-TYPE1137";         //!< RTCM3-TYPE1137 message name
static constexpr uint16_t    RTCM3_TYPE1019_MSGID           = 1019;                     //!< RTCM3-TYPE1019 message ID
static constexpr const char* RTCM3_TYPE1019_STRID           = "RTCM3-TYPE1019";         //!< RTCM3-TYPE1019 message name
static constexpr uint16_t    RTCM3_TYPE1020_MSGID           = 1020;                     //!< RTCM3-TYPE1020 message ID
static constexpr const char* RTCM3_TYPE1020_STRID           = "RTCM3-TYPE1020";         //!< RTCM3-TYPE1020 message name
static constexpr uint16_t    RTCM3_TYPE1041_MSGID           = 1041;                     //!< RTCM3-TYPE1041 message ID
static constexpr const char* RTCM3_TYPE1041_STRID           = "RTCM3-TYPE1041";         //!< RTCM3-TYPE1041 message name
static constexpr uint16_t    RTCM3_TYPE1042_MSGID           = 1042;                     //!< RTCM3-TYPE1042 message ID
static constexpr const char* RTCM3_TYPE1042_STRID           = "RTCM3-TYPE1042";         //!< RTCM3-TYPE1042 message name
static constexpr uint16_t    RTCM3_TYPE1044_MSGID           = 1044;                     //!< RTCM3-TYPE1044 message ID
static constexpr const char* RTCM3_TYPE1044_STRID           = "RTCM3-TYPE1044";         //!< RTCM3-TYPE1044 message name
static constexpr uint16_t    RTCM3_TYPE1045_MSGID           = 1045;                     //!< RTCM3-TYPE1045 message ID
static constexpr const char* RTCM3_TYPE1045_STRID           = "RTCM3-TYPE1045";         //!< RTCM3-TYPE1045 message name
static constexpr uint16_t    RTCM3_TYPE1046_MSGID           = 1046;                     //!< RTCM3-TYPE1046 message ID
static constexpr const char* RTCM3_TYPE1046_STRID           = "RTCM3-TYPE1046";         //!< RTCM3-TYPE1046 message name
static constexpr uint16_t    RTCM3_TYPE4050_MSGID           = 4050;                     //!< RTCM3-TYPE4050 message ID
static constexpr const char* RTCM3_TYPE4050_STRID           = "RTCM3-TYPE4050";         //!< RTCM3-TYPE4050 message name
static constexpr uint16_t    RTCM3_TYPE4051_MSGID           = 4051;                     //!< RTCM3-TYPE4051 message ID
static constexpr const char* RTCM3_TYPE4051_STRID           = "RTCM3-TYPE4051";         //!< RTCM3-TYPE4051 message name
static constexpr uint16_t    RTCM3_TYPE4052_MSGID           = 4052;                     //!< RTCM3-TYPE4052 message ID
static constexpr const char* RTCM3_TYPE4052_STRID           = "RTCM3-TYPE4052";         //!< RTCM3-TYPE4052 message name
static constexpr uint16_t    RTCM3_TYPE4053_MSGID           = 4053;                     //!< RTCM3-TYPE4053 message ID
static constexpr const char* RTCM3_TYPE4053_STRID           = "RTCM3-TYPE4053";         //!< RTCM3-TYPE4053 message name
static constexpr uint16_t    RTCM3_TYPE4054_MSGID           = 4054;                     //!< RTCM3-TYPE4054 message ID
static constexpr const char* RTCM3_TYPE4054_STRID           = "RTCM3-TYPE4054";         //!< RTCM3-TYPE4054 message name
static constexpr uint16_t    RTCM3_TYPE4055_MSGID           = 4055;                     //!< RTCM3-TYPE4055 message ID
static constexpr const char* RTCM3_TYPE4055_STRID           = "RTCM3-TYPE4055";         //!< RTCM3-TYPE4055 message name
static constexpr uint16_t    RTCM3_TYPE4056_MSGID           = 4056;                     //!< RTCM3-TYPE4056 message ID
static constexpr const char* RTCM3_TYPE4056_STRID           = "RTCM3-TYPE4056";         //!< RTCM3-TYPE4056 message name
static constexpr uint16_t    RTCM3_TYPE4057_MSGID           = 4057;                     //!< RTCM3-TYPE4057 message ID
static constexpr const char* RTCM3_TYPE4057_STRID           = "RTCM3-TYPE4057";         //!< RTCM3-TYPE4057 message name
static constexpr uint16_t    RTCM3_TYPE4058_MSGID           = 4058;                     //!< RTCM3-TYPE4058 message ID
static constexpr const char* RTCM3_TYPE4058_STRID           = "RTCM3-TYPE4058";         //!< RTCM3-TYPE4058 message name
static constexpr uint16_t    RTCM3_TYPE4059_MSGID           = 4059;                     //!< RTCM3-TYPE4059 message ID
static constexpr const char* RTCM3_TYPE4059_STRID           = "RTCM3-TYPE4059";         //!< RTCM3-TYPE4059 message name
static constexpr uint16_t    RTCM3_TYPE4060_MSGID           = 4060;                     //!< RTCM3-TYPE4060 message ID
static constexpr const char* RTCM3_TYPE4060_STRID           = "RTCM3-TYPE4060";         //!< RTCM3-TYPE4060 message name
static constexpr uint16_t    RTCM3_TYPE4061_MSGID           = 4061;                     //!< RTCM3-TYPE4061 message ID
static constexpr const char* RTCM3_TYPE4061_STRID           = "RTCM3-TYPE4061";         //!< RTCM3-TYPE4061 message name
static constexpr uint16_t    RTCM3_TYPE4062_MSGID           = 4062;                     //!< RTCM3-TYPE4062 message ID
static constexpr const char* RTCM3_TYPE4062_STRID           = "RTCM3-TYPE4062";         //!< RTCM3-TYPE4062 message name
static constexpr uint16_t    RTCM3_TYPE4063_MSGID           = 4063;                     //!< RTCM3-TYPE4063 message ID
static constexpr const char* RTCM3_TYPE4063_STRID           = "RTCM3-TYPE4063";         //!< RTCM3-TYPE4063 message name
static constexpr uint16_t    RTCM3_TYPE4064_MSGID           = 4064;                     //!< RTCM3-TYPE4064 message ID
static constexpr const char* RTCM3_TYPE4064_STRID           = "RTCM3-TYPE4064";         //!< RTCM3-TYPE4064 message name
static constexpr uint16_t    RTCM3_TYPE4065_MSGID           = 4065;                     //!< RTCM3-TYPE4065 message ID
static constexpr const char* RTCM3_TYPE4065_STRID           = "RTCM3-TYPE4065";         //!< RTCM3-TYPE4065 message name
static constexpr uint16_t    RTCM3_TYPE4066_MSGID           = 4066;                     //!< RTCM3-TYPE4066 message ID
static constexpr const char* RTCM3_TYPE4066_STRID           = "RTCM3-TYPE4066";         //!< RTCM3-TYPE4066 message name
static constexpr uint16_t    RTCM3_TYPE4067_MSGID           = 4067;                     //!< RTCM3-TYPE4067 message ID
static constexpr const char* RTCM3_TYPE4067_STRID           = "RTCM3-TYPE4067";         //!< RTCM3-TYPE4067 message name
static constexpr uint16_t    RTCM3_TYPE4068_MSGID           = 4068;                     //!< RTCM3-TYPE4068 message ID
static constexpr const char* RTCM3_TYPE4068_STRID           = "RTCM3-TYPE4068";         //!< RTCM3-TYPE4068 message name
static constexpr uint16_t    RTCM3_TYPE4069_MSGID           = 4069;                     //!< RTCM3-TYPE4069 message ID
static constexpr const char* RTCM3_TYPE4069_STRID           = "RTCM3-TYPE4069";         //!< RTCM3-TYPE4069 message name
static constexpr uint16_t    RTCM3_TYPE4070_MSGID           = 4070;                     //!< RTCM3-TYPE4070 message ID
static constexpr const char* RTCM3_TYPE4070_STRID           = "RTCM3-TYPE4070";         //!< RTCM3-TYPE4070 message name
static constexpr uint16_t    RTCM3_TYPE4071_MSGID           = 4071;                     //!< RTCM3-TYPE4071 message ID
static constexpr const char* RTCM3_TYPE4071_STRID           = "RTCM3-TYPE4071";         //!< RTCM3-TYPE4071 message name
static constexpr uint16_t    RTCM3_TYPE4072_MSGID           = 4072;                     //!< RTCM3-TYPE4072 message ID
static constexpr const char* RTCM3_TYPE4072_STRID           = "RTCM3-TYPE4072";         //!< RTCM3-TYPE4072 message name
static constexpr uint16_t    RTCM3_TYPE4073_MSGID           = 4073;                     //!< RTCM3-TYPE4073 message ID
static constexpr const char* RTCM3_TYPE4073_STRID           = "RTCM3-TYPE4073";         //!< RTCM3-TYPE4073 message name
static constexpr uint16_t    RTCM3_TYPE4074_MSGID           = 4074;                     //!< RTCM3-TYPE4074 message ID
static constexpr const char* RTCM3_TYPE4074_STRID           = "RTCM3-TYPE4074";         //!< RTCM3-TYPE4074 message name
static constexpr uint16_t    RTCM3_TYPE4075_MSGID           = 4075;                     //!< RTCM3-TYPE4075 message ID
static constexpr const char* RTCM3_TYPE4075_STRID           = "RTCM3-TYPE4075";         //!< RTCM3-TYPE4075 message name
static constexpr uint16_t    RTCM3_TYPE4076_MSGID           = 4076;                     //!< RTCM3-TYPE4076 message ID
static constexpr const char* RTCM3_TYPE4076_STRID           = "RTCM3-TYPE4076";         //!< RTCM3-TYPE4076 message name
static constexpr uint16_t    RTCM3_TYPE4077_MSGID           = 4077;                     //!< RTCM3-TYPE4077 message ID
static constexpr const char* RTCM3_TYPE4077_STRID           = "RTCM3-TYPE4077";         //!< RTCM3-TYPE4077 message name
static constexpr uint16_t    RTCM3_TYPE4078_MSGID           = 4078;                     //!< RTCM3-TYPE4078 message ID
static constexpr const char* RTCM3_TYPE4078_STRID           = "RTCM3-TYPE4078";         //!< RTCM3-TYPE4078 message name
static constexpr uint16_t    RTCM3_TYPE4079_MSGID           = 4079;                     //!< RTCM3-TYPE4079 message ID
static constexpr const char* RTCM3_TYPE4079_STRID           = "RTCM3-TYPE4079";         //!< RTCM3-TYPE4079 message name
static constexpr uint16_t    RTCM3_TYPE4080_MSGID           = 4080;                     //!< RTCM3-TYPE4080 message ID
static constexpr const char* RTCM3_TYPE4080_STRID           = "RTCM3-TYPE4080";         //!< RTCM3-TYPE4080 message name
static constexpr uint16_t    RTCM3_TYPE4081_MSGID           = 4081;                     //!< RTCM3-TYPE4081 message ID
static constexpr const char* RTCM3_TYPE4081_STRID           = "RTCM3-TYPE4081";         //!< RTCM3-TYPE4081 message name
static constexpr uint16_t    RTCM3_TYPE4082_MSGID           = 4082;                     //!< RTCM3-TYPE4082 message ID
static constexpr const char* RTCM3_TYPE4082_STRID           = "RTCM3-TYPE4082";         //!< RTCM3-TYPE4082 message name
static constexpr uint16_t    RTCM3_TYPE4083_MSGID           = 4083;                     //!< RTCM3-TYPE4083 message ID
static constexpr const char* RTCM3_TYPE4083_STRID           = "RTCM3-TYPE4083";         //!< RTCM3-TYPE4083 message name
static constexpr uint16_t    RTCM3_TYPE4084_MSGID           = 4084;                     //!< RTCM3-TYPE4084 message ID
static constexpr const char* RTCM3_TYPE4084_STRID           = "RTCM3-TYPE4084";         //!< RTCM3-TYPE4084 message name
static constexpr uint16_t    RTCM3_TYPE4085_MSGID           = 4085;                     //!< RTCM3-TYPE4085 message ID
static constexpr const char* RTCM3_TYPE4085_STRID           = "RTCM3-TYPE4085";         //!< RTCM3-TYPE4085 message name
static constexpr uint16_t    RTCM3_TYPE4086_MSGID           = 4086;                     //!< RTCM3-TYPE4086 message ID
static constexpr const char* RTCM3_TYPE4086_STRID           = "RTCM3-TYPE4086";         //!< RTCM3-TYPE4086 message name
static constexpr uint16_t    RTCM3_TYPE4087_MSGID           = 4087;                     //!< RTCM3-TYPE4087 message ID
static constexpr const char* RTCM3_TYPE4087_STRID           = "RTCM3-TYPE4087";         //!< RTCM3-TYPE4087 message name
static constexpr uint16_t    RTCM3_TYPE4088_MSGID           = 4088;                     //!< RTCM3-TYPE4088 message ID
static constexpr const char* RTCM3_TYPE4088_STRID           = "RTCM3-TYPE4088";         //!< RTCM3-TYPE4088 message name
static constexpr uint16_t    RTCM3_TYPE4089_MSGID           = 4089;                     //!< RTCM3-TYPE4089 message ID
static constexpr const char* RTCM3_TYPE4089_STRID           = "RTCM3-TYPE4089";         //!< RTCM3-TYPE4089 message name
static constexpr uint16_t    RTCM3_TYPE4090_MSGID           = 4090;                     //!< RTCM3-TYPE4090 message ID
static constexpr const char* RTCM3_TYPE4090_STRID           = "RTCM3-TYPE4090";         //!< RTCM3-TYPE4090 message name
static constexpr uint16_t    RTCM3_TYPE4091_MSGID           = 4091;                     //!< RTCM3-TYPE4091 message ID
static constexpr const char* RTCM3_TYPE4091_STRID           = "RTCM3-TYPE4091";         //!< RTCM3-TYPE4091 message name
static constexpr uint16_t    RTCM3_TYPE4092_MSGID           = 4092;                     //!< RTCM3-TYPE4092 message ID
static constexpr const char* RTCM3_TYPE4092_STRID           = "RTCM3-TYPE4092";         //!< RTCM3-TYPE4092 message name
static constexpr uint16_t    RTCM3_TYPE4093_MSGID           = 4093;                     //!< RTCM3-TYPE4093 message ID
static constexpr const char* RTCM3_TYPE4093_STRID           = "RTCM3-TYPE4093";         //!< RTCM3-TYPE4093 message name
static constexpr uint16_t    RTCM3_TYPE4094_MSGID           = 4094;                     //!< RTCM3-TYPE4094 message ID
static constexpr const char* RTCM3_TYPE4094_STRID           = "RTCM3-TYPE4094";         //!< RTCM3-TYPE4094 message name
static constexpr uint16_t    RTCM3_TYPE4095_MSGID           = 4095;                     //!< RTCM3-TYPE4095 message ID
static constexpr const char* RTCM3_TYPE4095_STRID           = "RTCM3-TYPE4095";         //!< RTCM3-TYPE4095 message name
static constexpr uint16_t    RTCM3_TYPE4072_0_SUBID         = 0;                        //!< RTCM3-TYPE4072_0 message ID
static constexpr const char* RTCM3_TYPE4072_0_STRID         = "RTCM3-TYPE4072_0";       //!< RTCM3-TYPE4072_0 message name
static constexpr uint16_t    RTCM3_TYPE4072_1_SUBID         = 1;                        //!< RTCM3-TYPE4072_1 message ID
static constexpr const char* RTCM3_TYPE4072_1_STRID         = "RTCM3-TYPE4072_1";       //!< RTCM3-TYPE4072_1 message name
// @fp_codegen_end{FPSDK_COMMON_PARSER_RTCM3_MESSAGES}
// clang-format on

///@}

/* ****************************************************************************************************************** */
}  // namespace rtcm3
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_PARSER_RTCM3_HPP__
