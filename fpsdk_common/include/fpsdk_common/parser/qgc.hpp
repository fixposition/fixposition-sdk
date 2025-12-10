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
 * - LG290P&LGx80P Series, GNSS Protocol Specification, copyright Quectel Wireless Solutions Co
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Parser QGC routines and types
 *
 * @page FPSDK_COMMON_PARSER_QGC Parser QGC routines and types
 *
 * **API**: fpsdk_common/parser/qgc.hpp and fpsdk::common::parser::qgc
 *
 */
#ifndef __FPSDK_COMMON_PARSER_QGC_HPP__
#define __FPSDK_COMMON_PARSER_QGC_HPP__

/* LIBC/STL */
#include <cstdint>

/* EXTERNAL */

/* PACKAGE */

namespace fpsdk {
namespace common {
namespace parser {
/**
 * @brief Parser QGC routines and types
 */
namespace qgc {
/* ****************************************************************************************************************** */

static constexpr uint8_t QGC_SYNC_1 = 0x51;       //!< Sync char 1 ('Q')
static constexpr uint8_t QGC_SYNC_2 = 0x47;       //!< Sync char 2 ('G')
static constexpr std::size_t QGC_HEAD_SIZE = 6;   //!< Size of the QGC header
static constexpr std::size_t QGC_FRAME_SIZE = 8;  //!< Size (in bytes) of QGC frame

/**
 * @brief Get group ID from message
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. This is meant for use as a helper in other functions. Checks on the
 *       \c msg and its data should be carried out there.
 *
 * @returns the QGC group ID
 */
constexpr uint8_t QgcGrpId(const uint8_t* msg)
{
    return (((uint8_t*)(msg))[2]);
}

/**
 * @brief Get message ID from message
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. This is meant for use as a helper in other functions. Checks on the
 *       \c msg and its data should be carried out there.
 *
 * @returns the QGC message ID
 */
constexpr uint8_t QgcMsgId(const uint8_t* msg)
{
    return (((uint8_t*)(msg))[3]);
}

/**
 * @brief Get QGC message name
 *
 * Generates a name (string) in the form "QGC-GRPID-MSGID", where GRPID and MSGID are suitable stringifications of the
 * class ID and message ID if known (for example, "QGC-RAW-HASE6", respectively %02X formatted IDs if unknown (for
 * example, "QGC-0A-FF").
 *
 * @param[out] name      String to write the name to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the QGC message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid QGC message.
 *
 * @returns true if message name was generated, false if \c name buffer was too small
 */
bool QgcGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Get QGC message info
 *
 * This stringifies the content of some QGC messages, for debugging.
 *
 * @param[out] info      String to write the info to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the QGC message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid QGC message.
 *
 * @returns true if message info was generated (even if info is empty), false if \c name buffer was too small
 */
bool QgcGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @name QGC groups and messages (names and IDs)
 *
 * @{
 */
// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_QGC_GROUPS}
static constexpr uint8_t     QGC_RAW_GRPID                  = 0x0a;                     //!< QGC-RAW group ID
static constexpr const char* QGC_RAW_STRID                  = "QGC-RAW";                //!< QGC-RAW group name
// @fp_codegen_end{FPSDK_COMMON_PARSER_QGC_GROUPS}
// clang-format on

// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_QGC_MESSAGES}
static constexpr uint8_t     QGC_RAW_PPPB2B_MSGID           = 0xb2;                     //!< QGC-RAW-PPPB2B message ID
static constexpr const char* QGC_RAW_PPPB2B_STRID           = "QGC-RAW-PPPB2B";         //!< QGC-RAW-PPPB2B message name
static constexpr uint8_t     QGC_RAW_QZSSL6_MSGID           = 0xb6;                     //!< QGC-RAW-QZSSL6 message ID
static constexpr const char* QGC_RAW_QZSSL6_STRID           = "QGC-RAW-QZSSL6";         //!< QGC-RAW-QZSSL6 message name
static constexpr uint8_t     QGC_RAW_HASE6_MSGID            = 0xe6;                     //!< QGC-RAW-HASE6 message ID
static constexpr const char* QGC_RAW_HASE6_STRID            = "QGC-RAW-HASE6";          //!< QGC-RAW-HASE6 message name
// @fp_codegen_end{FPSDK_COMMON_PARSER_QGC_MESSAGES}
// clang-format on
///@}

/* ****************************************************************************************************************** */
}  // namespace qgc
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_PARSER_QGC_HPP__
