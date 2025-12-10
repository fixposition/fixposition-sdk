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
 * @brief Fixposition SDK: Parser
 *
 * @page FPSDK_COMMON_PARSER_TYPES Parser common types
 *
 * **API**: fpsdk_common/parser/types.hpp and fpsdk::common::parser
 *
 */
#ifndef __FPSDK_COMMON_PARSER_TYPES_HPP__
#define __FPSDK_COMMON_PARSER_TYPES_HPP__

/* LIBC/STL */
#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

/* EXTERNAL */

/* PACKAGE */

namespace fpsdk {
namespace common {
namespace parser {
/* ****************************************************************************************************************** */

/**
 * @brief Protocols (message types), see also @ref FPSDK_COMMON_PARSER_NAMING
 */
enum class Protocol : int
{
    // See FP_PARSER_NAMEING for naming constraints!
    FP_A = 1,  //!< FP_A (Fixposition proprietary ASCII)                                         (#PROTOCOL_NAME_FP_A)
    FP_B,      //!< FP_B (Fixposition proprietary binary)                                        (#PROTOCOL_NAME_FP_B)
    NMEA,      //!< NMEA                                                                         (#PROTOCOL_NAME_NMEA)
    UBX,       //!< UBX (u-blox proprietary binary)                                              (#PROTOCOL_NAME_UBX)
    RTCM3,     //!< RTCM3                                                                        (#PROTOCOL_NAME_RTCM3)
    UNI_B,     //!< UNI_B (Unicore proprietary binary)                                           (#PROTOCOL_NAME_UNI_B)
    NOV_B,     //!< NOV_B (NovAtel proprietary binary, long or short header)                     (#PROTOCOL_NAME_NOV_B)
    SBF,       //!< SBF (Septentrio binary format)                                               (#PROTOCOL_NAME_SBF)
    QGC,       //!< QGC (Quectel proprietary binary)                                             (#PROTOCOL_NAME_QGC)
    SPARTN,    //!< SPARTN                                                                       (#PROTOCOL_NAME_SPARTN)
    OTHER,     //!< Other "message" (unknown or corrupt message, spurious data, line noise, ...) (#PROTOCOL_NAME_OTHER)
};

/**
 * @name Protocol names
 * @{
 */
// clang-format off
    // See FP_PARSER_NAMEING for naming constraints!
static constexpr const char* PROTOCOL_NAME_FP_A   = "FP_A";    //!< Name (label) for Protocol::FP_A
static constexpr const char* PROTOCOL_NAME_FP_B   = "FP_B";    //!< Name (label) for Protocol::FP_B
static constexpr const char* PROTOCOL_NAME_NMEA   = "NMEA";    //!< Name (label) for Protocol::NMEA
static constexpr const char* PROTOCOL_NAME_UBX    = "UBX";     //!< Name (label) for Protocol::UBX
static constexpr const char* PROTOCOL_NAME_RTCM3  = "RTCM3";   //!< Name (label) for Protocol::RTCM3
static constexpr const char* PROTOCOL_NAME_UNI_B  = "UNI_B";   //!< Name (label) for Protocol::UNI_B
static constexpr const char* PROTOCOL_NAME_NOV_B  = "NOV_B";   //!< Name (label) for Protocol::NOV_B
static constexpr const char* PROTOCOL_NAME_SBF    = "SBF";     //!< Name (label) for Protocol::SBF
static constexpr const char* PROTOCOL_NAME_QGC    = "QGC";     //!< Name (label) for Protocol::QGC
static constexpr const char* PROTOCOL_NAME_SPARTN = "SPARTN";  //!< Name (label) for Protocol::SPARTN
static constexpr const char* PROTOCOL_NAME_OTHER  = "OTHER";   //!< Name (label) for Protocol::OTHER
// clang-format on
///@}

/**
 * @brief Stringify Protocol
 *
 * @param[in]  proto  The protocol
 *
 * @returns a unique string for the type, for example "UBX" (PROTOCOL_NAME_UBX) for Protocol::UBX
 */
const char* ProtocolStr(const Protocol proto);

/**
 * @brief Get Protocol from name
 *
 * @param[in]  name  The protocol name
 *
 * @returns the Protocol enum, for example Protocol::UBX for "UBX" (PROTOCOL_NAME_UBX), and for any unknown string
 *          Protocol::OTHER
 */
Protocol StrProtocol(const char* name);

/**
 * @brief Message frame output by the Parser
 */
struct ParserMsg
{
    // clang-format off
    Protocol             proto_ = Protocol::OTHER; //!< Protocol (message type)
    std::vector<uint8_t> data_;     //!< Message data
    std::string          name_;     //!< Name of the message
    mutable std::string  info_;     //!< Message (debug) info, default empty, call MakeInfo() to fill
    uint64_t             seq_ = 0;  //!< Message counter (starting at 1)
    // clang-format on

    /**
     * @brief Make info_ field
     *
     * Fills the info field, unless it's not empty. Depending on the message, this may or may not put something
     * useful into the info_ field.
     */
    void MakeInfo() const;

    /**
     * @brief Get message raw data
     *
     * @returns a pointer to the message raw data
     */ // clang-format off
    inline const uint8_t* Data() const { return data_.data(); }  // clang-format on

    /**
     * @brief Get message raw size
     *
     * @returns the message raw size
     */ // clang-format off
    inline std::size_t Size() const { return data_.size(); }  // clang-format on
};

/**
 * @brief Parser statistics
 */
struct ParserStats
{
    uint64_t n_msgs_ = 0;    //!< Number of messages parsed
    uint64_t s_msgs_ = 0;    //!< Total size of messages parsed
    uint64_t n_fpa_ = 0;     //!< Number of Protocol::FP_A messages
    uint64_t s_fpa_ = 0;     //!< Total size of Protocol::FP_A messages
    uint64_t n_fpb_ = 0;     //!< Number of Protocol::FP_B messages
    uint64_t s_fpb_ = 0;     //!< Total size of Protocol::FP_B messages
    uint64_t n_nmea_ = 0;    //!< Number of Protocol::NMEA messages
    uint64_t s_nmea_ = 0;    //!< Total size of Protocol::NMEA messages
    uint64_t n_ubx_ = 0;     //!< Number of Protocol::UBX messages
    uint64_t s_ubx_ = 0;     //!< Total size of Protocol::UBX messages
    uint64_t n_rtcm3_ = 0;   //!< Number of Protocol::RTCM3 messages
    uint64_t s_rtcm3_ = 0;   //!< Total size of Protocol::RTCM3 messages
    uint64_t n_unib_ = 0;    //!< Number of Protocol::UNI_B messages
    uint64_t s_unib_ = 0;    //!< Total size of Protocol::UNI_B messages
    uint64_t n_novb_ = 0;    //!< Number of Protocol::NOV_B messages
    uint64_t s_novb_ = 0;    //!< Total size of Protocol::NOV_B messages
    uint64_t n_sbf_ = 0;     //!< Number of Protocol::SBF messages
    uint64_t s_sbf_ = 0;     //!< Total size of Protocol::SBF messages
    uint64_t n_qgc_ = 0;     //!< Number of Protocol::QGC messages
    uint64_t s_qgc_ = 0;     //!< Total size of Protocol::QGC messages
    uint64_t n_spartn_ = 0;  //!< Number of Protocol::SPARTN messages
    uint64_t s_spartn_ = 0;  //!< Total size of Protocol::SPARTN messages
    uint64_t n_other_ = 0;   //!< Number of Protocol::OTHER messages
    uint64_t s_other_ = 0;   //!< Total size of Protocol::OTHER messages

    /**
     * @brief Update stats
     *
     * @param[in]  msg  Message to account for
     */
    void Update(const ParserMsg& msg);
};

/**
 * @name Parser constraints
 * @{
 */
// clang-format off
static constexpr std::size_t MAX_ADD_SIZE  = 32 * 1024;  //!< Max size for Parser::Add() that is guaranteed to work
static constexpr std::size_t MAX_NAME_SIZE     =   100;  //!< Maximum size of message name string (incl. nul termination)
static constexpr std::size_t MAX_INFO_SIZE     =   400;  //!< Maximum size of message info string (incl. nul termination)
static constexpr std::size_t MAX_FP_A_SIZE     =   400;  //!< Maximum FP_A message size (must be the same as MAX_NMEA_SIZE)
static constexpr std::size_t MAX_FP_B_SIZE     =  4096;  //!< Maximum FP_B message size
static constexpr std::size_t MAX_NMEA_SIZE     =   400;  //!< Maximum NMEA message size (standard NMEA would be 82!)
static constexpr std::size_t MAX_UBX_SIZE      =  4096;  //!< Maximum UBX message size
static constexpr std::size_t MAX_RTCM3_SIZE    =  1028;  //!< Maximum RTCM3 message size
static constexpr std::size_t MAX_SPARTN_SIZE   =  1110;  //!< Maximum SPARTN message size
static constexpr std::size_t MAX_NOV_B_SIZE    =  4096;  //!< Maximum NOV_B message size
static constexpr std::size_t MAX_UNI_B_SIZE    =  4096;  //!< Maximum UNI_B message size
static constexpr std::size_t MAX_SBF_SIZE      =  4608;  //!< Maximum SBF message size
static constexpr std::size_t MAX_QGC_SIZE      =  4608;  //!< Maximum QGC message size
static constexpr std::size_t MAX_OTHER_SIZE    =   256;  //!< Maximum OTHER message size
static constexpr std::size_t MAX_ANY_SIZE      =  std::max({ MAX_NMEA_SIZE, MAX_FP_A_SIZE, MAX_FP_B_SIZE, MAX_UBX_SIZE,
        MAX_RTCM3_SIZE, MAX_SPARTN_SIZE, MAX_NOV_B_SIZE, MAX_UNI_B_SIZE, MAX_SBF_SIZE, MAX_QGC_SIZE, MAX_OTHER_SIZE });  //!< The largest of the above
static constexpr std::size_t MIN_ANY_SIZE      =  std::min({ MAX_NMEA_SIZE, MAX_FP_A_SIZE, MAX_FP_B_SIZE, MAX_UBX_SIZE,
        MAX_RTCM3_SIZE, MAX_SPARTN_SIZE, MAX_NOV_B_SIZE, MAX_UNI_B_SIZE, MAX_SBF_SIZE, MAX_QGC_SIZE, MAX_OTHER_SIZE });  //!< The smallest of the above
// clang-format on
///@}

// Assertions on some constraints we rely on
static_assert(MAX_ADD_SIZE > (4 * MAX_ANY_SIZE), "");  // Should be fairly large...
static_assert(MAX_NMEA_SIZE == MAX_FP_A_SIZE, "");     // As documented above
static_assert(MIN_ANY_SIZE >= 256, "");                // Something reasonable...
static_assert(MIN_ANY_SIZE < 1024, "");                // Something reasonable...
static_assert(MAX_ANY_SIZE >= 2048, "");               // Something reasonable...
static_assert(MAX_ANY_SIZE <= 10240, "");              // Something reasonable...
static_assert(MAX_RTCM3_SIZE < (0x3ff + 6), "");       // 10-bits payload size + frame size
static_assert(MAX_SPARTN_SIZE < (0x3ff + 88), "");     // 10-bits payload size + max. frame size

/* ****************************************************************************************************************** */
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_PARSER_TYPES_HPP__
