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
 * - Secure Position Augmentation for Real-Time Navigation (SPARTN) Interface Control Document
 *   (https://www.spartnformat.org)
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Parser SPARTN routines and types
 *
 * @page FPSDK_COMMON_PARSER_SPARTN Parser SPARTN routines and types
 *
 * **API**: fpsdk_common/parser/spartn.hpp and fpsdk::common::parser::spartn
 *
 */
#ifndef __FPSDK_COMMON_PARSER_SPARTN_HPP__
#define __FPSDK_COMMON_PARSER_SPARTN_HPP__

/* LIBC/STL */
#include <cstdint>

/* EXTERNAL */

/* PACKAGE */

namespace fpsdk {
namespace common {
namespace parser {
/**
 * @brief Parser SPARTN routines and types
 */
namespace spartn {
/* ****************************************************************************************************************** */

static constexpr uint8_t SPARTN_PREAMBLE = 0x73;        //!< SPARTN protocol preamble
static constexpr std::size_t SPARTN_MIN_HEAD_SIZE = 8;  //!< Minimal header size of a SPARTN message

/**
 * @brief Get SPARTN message type
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid SPARTN message.
 *
 * @returns the SPARTN message type
 */
constexpr uint8_t SpartnType(const uint8_t* msg)
{
    return (msg[1] & 0xfe) >> 1;
}

/**
 * @brief Get SPARTN message sub-type
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid SPARTN message.
 *
 * @returns the SPARTN message sub-type
 */
constexpr uint8_t SpartnSubType(const uint8_t* msg)
{
    return (msg[4] & 0xf0) >> 4;
}

/**
 * @brief Get SPARTN message name
 *
 * Generates a name (string) in the form "SPARTN-TYPEN_S", where N is the message type and S is the sub-type. For
 * example: "SPARTN-TYPE1_2".
 *
 * @param[out] name      String to write the name to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the SPARTN message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid SPARTN message.
 *
 * @returns true if message name was generated, false if \c name buffer was too small
 */
bool SpartnGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Get SPARTN message info
 *
 * This stringifies the content of some SPARTN messages, for debugging.
 *
 * @param[out] info      String to write the info to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the SPARTN message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid SPARTN message.
 *
 * @returns true if message info was generated (even if info is empty), false if \c name buffer was too small
 */
bool SpartnGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Get description for a SPARTN message type
 *
 * @param[in]  type     Message type
 * @param[in]  subtype  Message sub-type (if appropriate)
 *
 * @return the description of the message type, if available, NULL otherwise
 */
const char* SpartnGetTypeDesc(const uint16_t type, const uint16_t subtype = 0xffff);

/**
 * @name SPARTN messages (names and IDs)
 *
 * @{
 */
// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_SPARTN_MESSAGES}
static constexpr uint16_t    SPARTN_OCB_MSGID               =    0;                     //!< SPARTN-OCB message ID
static constexpr const char* SPARTN_OCB_STRID               = "SPARTN-OCB";             //!< SPARTN-OCB message name
static constexpr uint16_t    SPARTN_HPAC_MSGID              =    1;                     //!< SPARTN-HPAC message ID
static constexpr const char* SPARTN_HPAC_STRID              = "SPARTN-HPAC";            //!< SPARTN-HPAC message name
static constexpr uint16_t    SPARTN_GAD_MSGID               =    2;                     //!< SPARTN-GAD message ID
static constexpr const char* SPARTN_GAD_STRID               = "SPARTN-GAD";             //!< SPARTN-GAD message name
static constexpr uint16_t    SPARTN_BPAC_MSGID              =    3;                     //!< SPARTN-BPAC message ID
static constexpr const char* SPARTN_BPAC_STRID              = "SPARTN-BPAC";            //!< SPARTN-BPAC message name
static constexpr uint16_t    SPARTN_EAS_MSGID               =    4;                     //!< SPARTN-EAS message ID
static constexpr const char* SPARTN_EAS_STRID               = "SPARTN-EAS";             //!< SPARTN-EAS message name
static constexpr uint16_t    SPARTN_PROP_MSGID              =  120;                     //!< SPARTN-PROP message ID
static constexpr const char* SPARTN_PROP_STRID              = "SPARTN-PROP";            //!< SPARTN-PROP message name
static constexpr uint16_t    SPARTN_OCB_GPS_SUBID           = 0;                        //!< SPARTN-OCB-GPS message ID
static constexpr const char* SPARTN_OCB_GPS_STRID           = "SPARTN-OCB-GPS";         //!< SPARTN-OCB-GPS message name
static constexpr uint16_t    SPARTN_OCB_GLO_SUBID           = 1;                        //!< SPARTN-OCB-GLO message ID
static constexpr const char* SPARTN_OCB_GLO_STRID           = "SPARTN-OCB-GLO";         //!< SPARTN-OCB-GLO message name
static constexpr uint16_t    SPARTN_OCB_GAL_SUBID           = 2;                        //!< SPARTN-OCB-GAL message ID
static constexpr const char* SPARTN_OCB_GAL_STRID           = "SPARTN-OCB-GAL";         //!< SPARTN-OCB-GAL message name
static constexpr uint16_t    SPARTN_OCB_BDS_SUBID           = 3;                        //!< SPARTN-OCB-BDS message ID
static constexpr const char* SPARTN_OCB_BDS_STRID           = "SPARTN-OCB-BDS";         //!< SPARTN-OCB-BDS message name
static constexpr uint16_t    SPARTN_OCB_QZSS_SUBID          = 4;                        //!< SPARTN-OCB-QZSS message ID
static constexpr const char* SPARTN_OCB_QZSS_STRID          = "SPARTN-OCB-QZSS";        //!< SPARTN-OCB-QZSS message name
static constexpr uint16_t    SPARTN_HPAC_GPS_SUBID          = 0;                        //!< SPARTN-HPAC-GPS message ID
static constexpr const char* SPARTN_HPAC_GPS_STRID          = "SPARTN-HPAC-GPS";        //!< SPARTN-HPAC-GPS message name
static constexpr uint16_t    SPARTN_HPAC_GLO_SUBID          = 1;                        //!< SPARTN-HPAC-GLO message ID
static constexpr const char* SPARTN_HPAC_GLO_STRID          = "SPARTN-HPAC-GLO";        //!< SPARTN-HPAC-GLO message name
static constexpr uint16_t    SPARTN_HPAC_GAL_SUBID          = 2;                        //!< SPARTN-HPAC-GAL message ID
static constexpr const char* SPARTN_HPAC_GAL_STRID          = "SPARTN-HPAC-GAL";        //!< SPARTN-HPAC-GAL message name
static constexpr uint16_t    SPARTN_HPAC_BDS_SUBID          = 3;                        //!< SPARTN-HPAC-BDS message ID
static constexpr const char* SPARTN_HPAC_BDS_STRID          = "SPARTN-HPAC-BDS";        //!< SPARTN-HPAC-BDS message name
static constexpr uint16_t    SPARTN_HPAC_QZSS_SUBID         = 4;                        //!< SPARTN-HPAC-QZSS message ID
static constexpr const char* SPARTN_HPAC_QZSS_STRID         = "SPARTN-HPAC-QZSS";       //!< SPARTN-HPAC-QZSS message name
static constexpr uint16_t    SPARTN_GAD_GAD_SUBID           = 0;                        //!< SPARTN-GAD-GAD message ID
static constexpr const char* SPARTN_GAD_GAD_STRID           = "SPARTN-GAD-GAD";         //!< SPARTN-GAD-GAD message name
static constexpr uint16_t    SPARTN_BPAC_POLY_SUBID         = 0;                        //!< SPARTN-BPAC-POLY message ID
static constexpr const char* SPARTN_BPAC_POLY_STRID         = "SPARTN-BPAC-POLY";       //!< SPARTN-BPAC-POLY message name
static constexpr uint16_t    SPARTN_EAS_KEY_SUBID           = 0;                        //!< SPARTN-EAS-KEY message ID
static constexpr const char* SPARTN_EAS_KEY_STRID           = "SPARTN-EAS-KEY";         //!< SPARTN-EAS-KEY message name
static constexpr uint16_t    SPARTN_EAS_GROUP_SUBID         = 1;                        //!< SPARTN-EAS-GROUP message ID
static constexpr const char* SPARTN_EAS_GROUP_STRID         = "SPARTN-EAS-GROUP";       //!< SPARTN-EAS-GROUP message name
static constexpr uint16_t    SPARTN_PROP_EST_SUBID          = 0;                        //!< SPARTN-PROP-EST message ID
static constexpr const char* SPARTN_PROP_EST_STRID          = "SPARTN-PROP-EST";        //!< SPARTN-PROP-EST message name
static constexpr uint16_t    SPARTN_PROP_UBLOX_SUBID        = 1;                        //!< SPARTN-PROP-UBLOX message ID
static constexpr const char* SPARTN_PROP_UBLOX_STRID        = "SPARTN-PROP-UBLOX";      //!< SPARTN-PROP-UBLOX message name
static constexpr uint16_t    SPARTN_PROP_SWIFT_SUBID        = 2;                        //!< SPARTN-PROP-SWIFT message ID
static constexpr const char* SPARTN_PROP_SWIFT_STRID        = "SPARTN-PROP-SWIFT";      //!< SPARTN-PROP-SWIFT message name
// @fp_codegen_end{FPSDK_COMMON_PARSER_SPARTN_MESSAGES}
// clang-format on

///@}

/* ****************************************************************************************************************** */
}  // namespace spartn
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_PARSER_SPARTN_HPP__
