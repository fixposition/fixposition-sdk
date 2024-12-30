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
 * - Unicore Standard Position Protocol, copyright (c) 2009-2023, Unicore Communications, Inc.
 * - Unicore High Precision Commands and Logs, Copyright (c) 2009-2021, Unicore Communications, Inc.
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Parser UNI_B routines and types
 *
 * @page FPSDK_COMMON_PARSER_UNIB Parser UNI_B routines and types
 *
 * **API**: fpsdk_common/parser/unib.hpp and fpsdk::common::parser::unib
 *
 */
#ifndef __FPSDK_COMMON_PARSER_UNIB_HPP__
#define __FPSDK_COMMON_PARSER_UNIB_HPP__

/* LIBC/STL */
#include <cstdint>

/* EXTERNAL */

/* PACKAGE */
#include "novb.hpp"

namespace fpsdk {
namespace common {
namespace parser {
/**
 * @brief Parser UNI_B routines and types
 */
namespace unib {
/* ****************************************************************************************************************** */

static constexpr uint8_t UNI_B_SYNC_1 = 0xaa;       //!< Sync char 1
static constexpr uint8_t UNI_B_SYNC_2 = 0x44;       //!< Sync char 2
static constexpr uint8_t UNI_B_SYNC_3 = 0xb5;       //!< Sync char 3
static constexpr std::size_t UNI_B_HEAD_SIZE = 24;  //!< Size of the UNI_B header (UnibHeader)

/**
 * @brief Get message ID
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid UNI_B message.
 *
 * @returns the message ID
 */
constexpr uint16_t UnibMsgId(const uint8_t* msg)
{
    return (((uint16_t)((uint8_t*)msg)[5] << 8) | (uint16_t)((uint8_t*)msg)[4]);
}

/**
 * @brief Get message size
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid UNI_B message.
 *
 * @returns the message size
 */
constexpr uint16_t UnibMsgSize(const uint8_t* msg)
{
    return (((uint16_t)((uint8_t*)msg)[7] << 8) | (uint16_t)((uint8_t*)msg)[6]);
}

/**
 * @brief Get message version
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid UNI_B message.
 *
 * @returns the message version
 */
constexpr uint8_t UNI_B_MSGVER(const uint8_t* msg)
{
    return msg[20];
}

/**
 * @brief Get UNI_B message name
 *
 * Generates a name (string) in the form "UNI_B-NAME", where NAME is a suitable stringifications of the
 * message ID if known (for example, "UNI_B-BESTNAV", respectively "%05u" formatted message ID if unknown (for
 * example, "UNI_B-MSG01234").
 *
 * @param[out] name      String to write the name to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the UNI_B message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid UNI_B message.
 *
 * @returns true if message name was generated, false if \c name buffer was too small
 */
bool UnibGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Get UNI_B message info
 *
 * This stringifies the content of some UNI_B messages, for debugging.
 *
 * @param[out] info      String to write the info to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the UNI_B message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid UNI_B message.
 *
 * @returns true if message info was generated (even if info is empty), false if \c name buffer was too small
 */
bool UnibGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @name UNI_B messages (names and IDs)
 *
 * @{
 */
// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_UNIB_MESSAGES}
static constexpr uint16_t    UNI_B_VERSION_MSGID            =    37;                    //!< UNI_B-VERSION message ID
static constexpr const char* UNI_B_VERSION_STRID            = "UNI_B-VERSION";          //!< UNI_B-VERSION message name
static constexpr uint16_t    UNI_B_OBSVM_MSGID              =    12;                    //!< UNI_B-OBSVM message ID
static constexpr const char* UNI_B_OBSVM_STRID              = "UNI_B-OBSVM";            //!< UNI_B-OBSVM message name
static constexpr uint16_t    UNI_B_OBSVH_MSGID              =    13;                    //!< UNI_B-OBSVH message ID
static constexpr const char* UNI_B_OBSVH_STRID              = "UNI_B-OBSVH";            //!< UNI_B-OBSVH message name
static constexpr uint16_t    UNI_B_OBSVMCMP_MSGID           =   138;                    //!< UNI_B-OBSVMCMP message ID
static constexpr const char* UNI_B_OBSVMCMP_STRID           = "UNI_B-OBSVMCMP";         //!< UNI_B-OBSVMCMP message name
static constexpr uint16_t    UNI_B_OBSVHCMP_MSGID           =   139;                    //!< UNI_B-OBSVHCMP message ID
static constexpr const char* UNI_B_OBSVHCMP_STRID           = "UNI_B-OBSVHCMP";         //!< UNI_B-OBSVHCMP message name
static constexpr uint16_t    UNI_B_OBSVBASE_MSGID           =   284;                    //!< UNI_B-OBSVBASE message ID
static constexpr const char* UNI_B_OBSVBASE_STRID           = "UNI_B-OBSVBASE";         //!< UNI_B-OBSVBASE message name
static constexpr uint16_t    UNI_B_BASEINFO_MSGID           =   176;                    //!< UNI_B-BASEINFO message ID
static constexpr const char* UNI_B_BASEINFO_STRID           = "UNI_B-BASEINFO";         //!< UNI_B-BASEINFO message name
static constexpr uint16_t    UNI_B_GPSION_MSGID             =     8;                    //!< UNI_B-GPSION message ID
static constexpr const char* UNI_B_GPSION_STRID             = "UNI_B-GPSION";           //!< UNI_B-GPSION message name
static constexpr uint16_t    UNI_B_BD3ION_MSGID             =    21;                    //!< UNI_B-BD3ION message ID
static constexpr const char* UNI_B_BD3ION_STRID             = "UNI_B-BD3ION";           //!< UNI_B-BD3ION message name
static constexpr uint16_t    UNI_B_BDSION_MSGID             =     4;                    //!< UNI_B-BDSION message ID
static constexpr const char* UNI_B_BDSION_STRID             = "UNI_B-BDSION";           //!< UNI_B-BDSION message name
static constexpr uint16_t    UNI_B_GALION_MSGID             =     9;                    //!< UNI_B-GALION message ID
static constexpr const char* UNI_B_GALION_STRID             = "UNI_B-GALION";           //!< UNI_B-GALION message name
static constexpr uint16_t    UNI_B_GPSUTC_MSGID             =    19;                    //!< UNI_B-GPSUTC message ID
static constexpr const char* UNI_B_GPSUTC_STRID             = "UNI_B-GPSUTC";           //!< UNI_B-GPSUTC message name
static constexpr uint16_t    UNI_B_BD3UTC_MSGID             =    22;                    //!< UNI_B-BD3UTC message ID
static constexpr const char* UNI_B_BD3UTC_STRID             = "UNI_B-BD3UTC";           //!< UNI_B-BD3UTC message name
static constexpr uint16_t    UNI_B_BDSUTC_MSGID             =  2012;                    //!< UNI_B-BDSUTC message ID
static constexpr const char* UNI_B_BDSUTC_STRID             = "UNI_B-BDSUTC";           //!< UNI_B-BDSUTC message name
static constexpr uint16_t    UNI_B_GALUTC_MSGID             =    20;                    //!< UNI_B-GALUTC message ID
static constexpr const char* UNI_B_GALUTC_STRID             = "UNI_B-GALUTC";           //!< UNI_B-GALUTC message name
static constexpr uint16_t    UNI_B_GPSEPH_MSGID             =   106;                    //!< UNI_B-GPSEPH message ID
static constexpr const char* UNI_B_GPSEPH_STRID             = "UNI_B-GPSEPH";           //!< UNI_B-GPSEPH message name
static constexpr uint16_t    UNI_B_QZSSEPH_MSGID            =   110;                    //!< UNI_B-QZSSEPH message ID
static constexpr const char* UNI_B_QZSSEPH_STRID            = "UNI_B-QZSSEPH";          //!< UNI_B-QZSSEPH message name
static constexpr uint16_t    UNI_B_BD3EPH_MSGID             =  2999;                    //!< UNI_B-BD3EPH message ID
static constexpr const char* UNI_B_BD3EPH_STRID             = "UNI_B-BD3EPH";           //!< UNI_B-BD3EPH message name
static constexpr uint16_t    UNI_B_BDSEPH_MSGID             =   108;                    //!< UNI_B-BDSEPH message ID
static constexpr const char* UNI_B_BDSEPH_STRID             = "UNI_B-BDSEPH";           //!< UNI_B-BDSEPH message name
static constexpr uint16_t    UNI_B_GLOEPH_MSGID             =   107;                    //!< UNI_B-GLOEPH message ID
static constexpr const char* UNI_B_GLOEPH_STRID             = "UNI_B-GLOEPH";           //!< UNI_B-GLOEPH message name
static constexpr uint16_t    UNI_B_GALEPH_MSGID             =   109;                    //!< UNI_B-GALEPH message ID
static constexpr const char* UNI_B_GALEPH_STRID             = "UNI_B-GALEPH";           //!< UNI_B-GALEPH message name
static constexpr uint16_t    UNI_B_AGRIC_MSGID              =  1276;                    //!< UNI_B-AGRIC message ID
static constexpr const char* UNI_B_AGRIC_STRID              = "UNI_B-AGRIC";            //!< UNI_B-AGRIC message name
static constexpr uint16_t    UNI_B_PVTSLN_MSGID             =  1021;                    //!< UNI_B-PVTSLN message ID
static constexpr const char* UNI_B_PVTSLN_STRID             = "UNI_B-PVTSLN";           //!< UNI_B-PVTSLN message name
static constexpr uint16_t    UNI_B_BESTNAV_MSGID            =  2118;                    //!< UNI_B-BESTNAV message ID
static constexpr const char* UNI_B_BESTNAV_STRID            = "UNI_B-BESTNAV";          //!< UNI_B-BESTNAV message name
static constexpr uint16_t    UNI_B_BESTNAVXYZ_MSGID         =   240;                    //!< UNI_B-BESTNAVXYZ message ID
static constexpr const char* UNI_B_BESTNAVXYZ_STRID         = "UNI_B-BESTNAVXYZ";       //!< UNI_B-BESTNAVXYZ message name
static constexpr uint16_t    UNI_B_BESTNAVH_MSGID           =  2119;                    //!< UNI_B-BESTNAVH message ID
static constexpr const char* UNI_B_BESTNAVH_STRID           = "UNI_B-BESTNAVH";         //!< UNI_B-BESTNAVH message name
static constexpr uint16_t    UNI_B_BESTNAVXYZH_MSGID        =   242;                    //!< UNI_B-BESTNAVXYZH message ID
static constexpr const char* UNI_B_BESTNAVXYZH_STRID        = "UNI_B-BESTNAVXYZH";      //!< UNI_B-BESTNAVXYZH message name
static constexpr uint16_t    UNI_B_BESTSAT_MSGID            =  1041;                    //!< UNI_B-BESTSAT message ID
static constexpr const char* UNI_B_BESTSAT_STRID            = "UNI_B-BESTSAT";          //!< UNI_B-BESTSAT message name
static constexpr uint16_t    UNI_B_ADRNAV_MSGID             =   142;                    //!< UNI_B-ADRNAV message ID
static constexpr const char* UNI_B_ADRNAV_STRID             = "UNI_B-ADRNAV";           //!< UNI_B-ADRNAV message name
static constexpr uint16_t    UNI_B_ADRNAVH_MSGID            =  2117;                    //!< UNI_B-ADRNAVH message ID
static constexpr const char* UNI_B_ADRNAVH_STRID            = "UNI_B-ADRNAVH";          //!< UNI_B-ADRNAVH message name
static constexpr uint16_t    UNI_B_PPPNAV_MSGID             =  1026;                    //!< UNI_B-PPPNAV message ID
static constexpr const char* UNI_B_PPPNAV_STRID             = "UNI_B-PPPNAV";           //!< UNI_B-PPPNAV message name
static constexpr uint16_t    UNI_B_SPPNAV_MSGID             =    46;                    //!< UNI_B-SPPNAV message ID
static constexpr const char* UNI_B_SPPNAV_STRID             = "UNI_B-SPPNAV";           //!< UNI_B-SPPNAV message name
static constexpr uint16_t    UNI_B_SPPNAVH_MSGID            =  2116;                    //!< UNI_B-SPPNAVH message ID
static constexpr const char* UNI_B_SPPNAVH_STRID            = "UNI_B-SPPNAVH";          //!< UNI_B-SPPNAVH message name
static constexpr uint16_t    UNI_B_STADOP_MSGID             =   954;                    //!< UNI_B-STADOP message ID
static constexpr const char* UNI_B_STADOP_STRID             = "UNI_B-STADOP";           //!< UNI_B-STADOP message name
static constexpr uint16_t    UNI_B_STADOPH_MSGID            =  2122;                    //!< UNI_B-STADOPH message ID
static constexpr const char* UNI_B_STADOPH_STRID            = "UNI_B-STADOPH";          //!< UNI_B-STADOPH message name
static constexpr uint16_t    UNI_B_ADRDOP_MSGID             =   953;                    //!< UNI_B-ADRDOP message ID
static constexpr const char* UNI_B_ADRDOP_STRID             = "UNI_B-ADRDOP";           //!< UNI_B-ADRDOP message name
static constexpr uint16_t    UNI_B_ADRDOPH_MSGID            =  2121;                    //!< UNI_B-ADRDOPH message ID
static constexpr const char* UNI_B_ADRDOPH_STRID            = "UNI_B-ADRDOPH";          //!< UNI_B-ADRDOPH message name
static constexpr uint16_t    UNI_B_PPPDOP_MSGID             =  1025;                    //!< UNI_B-PPPDOP message ID
static constexpr const char* UNI_B_PPPDOP_STRID             = "UNI_B-PPPDOP";           //!< UNI_B-PPPDOP message name
static constexpr uint16_t    UNI_B_SPPDOP_MSGID             =   173;                    //!< UNI_B-SPPDOP message ID
static constexpr const char* UNI_B_SPPDOP_STRID             = "UNI_B-SPPDOP";           //!< UNI_B-SPPDOP message name
static constexpr uint16_t    UNI_B_SPPDOPH_MSGID            =  2120;                    //!< UNI_B-SPPDOPH message ID
static constexpr const char* UNI_B_SPPDOPH_STRID            = "UNI_B-SPPDOPH";          //!< UNI_B-SPPDOPH message name
static constexpr uint16_t    UNI_B_SATSINFO_MSGID           =  2124;                    //!< UNI_B-SATSINFO message ID
static constexpr const char* UNI_B_SATSINFO_STRID           = "UNI_B-SATSINFO";         //!< UNI_B-SATSINFO message name
static constexpr uint16_t    UNI_B_BASEPOS_MSGID            =    49;                    //!< UNI_B-BASEPOS message ID
static constexpr const char* UNI_B_BASEPOS_STRID            = "UNI_B-BASEPOS";          //!< UNI_B-BASEPOS message name
static constexpr uint16_t    UNI_B_SATELLITE_MSGID          =  1042;                    //!< UNI_B-SATELLITE message ID
static constexpr const char* UNI_B_SATELLITE_STRID          = "UNI_B-SATELLITE";        //!< UNI_B-SATELLITE message name
static constexpr uint16_t    UNI_B_SATECEF_MSGID            =  2115;                    //!< UNI_B-SATECEF message ID
static constexpr const char* UNI_B_SATECEF_STRID            = "UNI_B-SATECEF";          //!< UNI_B-SATECEF message name
static constexpr uint16_t    UNI_B_RECTIME_MSGID            =   102;                    //!< UNI_B-RECTIME message ID
static constexpr const char* UNI_B_RECTIME_STRID            = "UNI_B-RECTIME";          //!< UNI_B-RECTIME message name
static constexpr uint16_t    UNI_B_NOVHEADING_MSGID         =   972;                    //!< UNI_B-NOVHEADING message ID
static constexpr const char* UNI_B_NOVHEADING_STRID         = "UNI_B-NOVHEADING";       //!< UNI_B-NOVHEADING message name
static constexpr uint16_t    UNI_B_NOVHEADING2_MSGID        =  1331;                    //!< UNI_B-NOVHEADING2 message ID
static constexpr const char* UNI_B_NOVHEADING2_STRID        = "UNI_B-NOVHEADING2";      //!< UNI_B-NOVHEADING2 message name
static constexpr uint16_t    UNI_B_RTKSTATUS_MSGID          =   509;                    //!< UNI_B-RTKSTATUS message ID
static constexpr const char* UNI_B_RTKSTATUS_STRID          = "UNI_B-RTKSTATUS";        //!< UNI_B-RTKSTATUS message name
static constexpr uint16_t    UNI_B_AGNSSSTATUS_MSGID        =   512;                    //!< UNI_B-AGNSSSTATUS message ID
static constexpr const char* UNI_B_AGNSSSTATUS_STRID        = "UNI_B-AGNSSSTATUS";      //!< UNI_B-AGNSSSTATUS message name
static constexpr uint16_t    UNI_B_RTCSTATUS_MSGID          =   510;                    //!< UNI_B-RTCSTATUS message ID
static constexpr const char* UNI_B_RTCSTATUS_STRID          = "UNI_B-RTCSTATUS";        //!< UNI_B-RTCSTATUS message name
static constexpr uint16_t    UNI_B_JAMSTATUS_MSGID          =   511;                    //!< UNI_B-JAMSTATUS message ID
static constexpr const char* UNI_B_JAMSTATUS_STRID          = "UNI_B-JAMSTATUS";        //!< UNI_B-JAMSTATUS message name
static constexpr uint16_t    UNI_B_FREQJAMSTATUS_MSGID      =   519;                    //!< UNI_B-FREQJAMSTATUS message ID
static constexpr const char* UNI_B_FREQJAMSTATUS_STRID      = "UNI_B-FREQJAMSTATUS";    //!< UNI_B-FREQJAMSTATUS message name
static constexpr uint16_t    UNI_B_RTCMSTATUS_MSGID         =  2125;                    //!< UNI_B-RTCMSTATUS message ID
static constexpr const char* UNI_B_RTCMSTATUS_STRID         = "UNI_B-RTCMSTATUS";       //!< UNI_B-RTCMSTATUS message name
static constexpr uint16_t    UNI_B_HWSTATUS_MSGID           =   218;                    //!< UNI_B-HWSTATUS message ID
static constexpr const char* UNI_B_HWSTATUS_STRID           = "UNI_B-HWSTATUS";         //!< UNI_B-HWSTATUS message name
static constexpr uint16_t    UNI_B_PPPSTATUS_MSGID          =  9000;                    //!< UNI_B-PPPSTATUS message ID
static constexpr const char* UNI_B_PPPSTATUS_STRID          = "UNI_B-PPPSTATUS";        //!< UNI_B-PPPSTATUS message name
static constexpr uint16_t    UNI_B_AGC_MSGID                =   220;                    //!< UNI_B-AGC message ID
static constexpr const char* UNI_B_AGC_STRID                = "UNI_B-AGC";              //!< UNI_B-AGC message name
static constexpr uint16_t    UNI_B_INFOPART1_MSGID          =  1019;                    //!< UNI_B-INFOPART1 message ID
static constexpr const char* UNI_B_INFOPART1_STRID          = "UNI_B-INFOPART1";        //!< UNI_B-INFOPART1 message name
static constexpr uint16_t    UNI_B_INFOPART2_MSGID          =  1020;                    //!< UNI_B-INFOPART2 message ID
static constexpr const char* UNI_B_INFOPART2_STRID          = "UNI_B-INFOPART2";        //!< UNI_B-INFOPART2 message name
static constexpr uint16_t    UNI_B_MSPOS_MSGID              =   520;                    //!< UNI_B-MSPOS message ID
static constexpr const char* UNI_B_MSPOS_STRID              = "UNI_B-MSPOS";            //!< UNI_B-MSPOS message name
static constexpr uint16_t    UNI_B_TROPINFO_MSGID           =  2318;                    //!< UNI_B-TROPINFO message ID
static constexpr const char* UNI_B_TROPINFO_STRID           = "UNI_B-TROPINFO";         //!< UNI_B-TROPINFO message name
static constexpr uint16_t    UNI_B_PTOBSINFO_MSGID          =   221;                    //!< UNI_B-PTOBSINFO message ID
static constexpr const char* UNI_B_PTOBSINFO_STRID          = "UNI_B-PTOBSINFO";        //!< UNI_B-PTOBSINFO message name
static constexpr uint16_t    UNI_B_PPPB2INFO1_MSGID         =  2302;                    //!< UNI_B-PPPB2INFO1 message ID
static constexpr const char* UNI_B_PPPB2INFO1_STRID         = "UNI_B-PPPB2INFO1";       //!< UNI_B-PPPB2INFO1 message name
static constexpr uint16_t    UNI_B_PPPB2INFO2_MSGID         =  2304;                    //!< UNI_B-PPPB2INFO2 message ID
static constexpr const char* UNI_B_PPPB2INFO2_STRID         = "UNI_B-PPPB2INFO2";       //!< UNI_B-PPPB2INFO2 message name
static constexpr uint16_t    UNI_B_PPPB2INFO3_MSGID         =  2306;                    //!< UNI_B-PPPB2INFO3 message ID
static constexpr const char* UNI_B_PPPB2INFO3_STRID         = "UNI_B-PPPB2INFO3";       //!< UNI_B-PPPB2INFO3 message name
static constexpr uint16_t    UNI_B_PPPB2INFO4_MSGID         =  2308;                    //!< UNI_B-PPPB2INFO4 message ID
static constexpr const char* UNI_B_PPPB2INFO4_STRID         = "UNI_B-PPPB2INFO4";       //!< UNI_B-PPPB2INFO4 message name
static constexpr uint16_t    UNI_B_PPPB2INFO5_MSGID         =  2310;                    //!< UNI_B-PPPB2INFO5 message ID
static constexpr const char* UNI_B_PPPB2INFO5_STRID         = "UNI_B-PPPB2INFO5";       //!< UNI_B-PPPB2INFO5 message name
static constexpr uint16_t    UNI_B_PPPB2INFO6_MSGID         =  2312;                    //!< UNI_B-PPPB2INFO6 message ID
static constexpr const char* UNI_B_PPPB2INFO6_STRID         = "UNI_B-PPPB2INFO6";       //!< UNI_B-PPPB2INFO6 message name
static constexpr uint16_t    UNI_B_PPPB2INFO7_MSGID         =  2314;                    //!< UNI_B-PPPB2INFO7 message ID
static constexpr const char* UNI_B_PPPB2INFO7_STRID         = "UNI_B-PPPB2INFO7";       //!< UNI_B-PPPB2INFO7 message name
// @fp_codegen_end{FPSDK_COMMON_PARSER_UNIB_MESSAGES}
// clang-format on
///@}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief UNI_B frame long header
 */
struct UnibHeader
{  // clang-format off
    uint8_t  sync1;             //!< = UNI_B_SYNC_1
    uint8_t  sync2;             //!< = UNI_B_SYNC_2
    uint8_t  sync3;             //!< = UNI_B_SYNC_3_LONG
    uint8_t  cpu_idle;          //!< CPU idle [%] (0-100), 1s average
    uint16_t message_id;        //!< Message ID
    uint16_t message_size;      //!< Size of message (payload?) TODO
    uint8_t  time_ref;          //!< Time reference (UnibTimeRef)
    uint8_t  time_status;       //!< Time status (UnibTimeStatus)
    uint16_t wno;               //!< Week number [-]
    uint32_t tow_ms;            //!< Time of week [ms]
    uint32_t reserved;          //!< Reserved
    uint8_t  version;           //!< Unicore format version (TODO message (payload) version?)
    uint8_t  leap_sec;          //!< Leap second (TODO number of leap seconds?)
    uint16_t output_delay;      //!< Output delay (w.r.t. measurement time)
};  // clang-format on

static_assert(sizeof(UnibHeader) == UNI_B_HEAD_SIZE, "");

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Time reference
 */
enum class UnibTimeRef : uint8_t
{  // clang-format off
    GPST      = 0,  //!< GPS
    BDST      = 1,  //!< BeiDou
};  // clang-format on

/**
 * @brief Stringify time reference
 *
 * @param[in]  time_ref  The time reference
 *
 * @returns a unique string for the time reference
 */
const char* UnibTimeRefStr(const UnibTimeRef time_ref);

/**
 * @brief Time status
 */
enum class UnibTimeStatus : uint8_t
{  // clang-format off
    UNKNOWN   = static_cast<uint8_t>(novb::NovbTimeStatus::UNKNOWN),  //!< Unknown, precise time not known
    FINE      = static_cast<uint8_t>(novb::NovbTimeStatus::FINE),     //!< Precise time known
};  // clang-format on

/**
 * @brief Stringify time status
 *
 * @param[in]  time_status  The time status
 *
 * @returns a unique string for the time status
 */
const char* UnibTimeStatusStr(const UnibTimeStatus time_status);

/**
 * @brief Product model
 */
enum class UnibProductModel : uint32_t
{  // clang-format off
    UNKNOWN   =  0,  //!< Unknown
    UB4B0     =  1,  //!< UB4B0
    UM4B0     =  2,  //!< UM4B0
    UM480     =  3,  //!< UM480
    UM440     =  4,  //!< UM440
    UM482     =  5,  //!< UM482
    UM442     =  6,  //!< UM442
    UB482     =  7,  //!< UB482
    UT4B0     =  8,  //!< UT4B0
    UB362L    = 10,  //!< UB362L
    UB4B0M    = 11,  //!< UB4B0M
    UB4B0J    = 12,  //!< UB4B0J
    UM482L    = 13,  //!< UM482L
    UM4B0L    = 14,  //!< UM4B0L
    CLAP_B    = 16,  //!< CLAP-B
    UM982     = 17,  //!< UM982
    UM980     = 18,  //!< UM980
    UM960     = 19,  //!< UM960
    UM980A    = 21,  //!< UM980A
    CLAP_C    = 23,  //!< CLAP-C
    UM960L    = 24,  //!< UM960L
    UM981     = 26,  //!< UM981
};  // clang-format on

/**
 * @brief Stringify UNI_B-VERSION.product_model
 *
 * @param[in]  model  The model value
 *
 * @returns a short and unique string identifying the receiver model
 */
const char* UnibProductModelStr(const UnibProductModel model);

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief UNI_B-VERSION payload
 */
struct UnibVersionPayload
{  // clang-format off
    uint32_t product_model;          //!< Product model (UnibProductModel)
    uint8_t  firmware_version[33];   //!< Firmware version (nul-terminated string)
    uint8_t  auth_type[129];         //!< Authorization type (nul-terminated string)
    uint8_t  psn[66];                //!< PN and SN (nul-terminated string)
    uint8_t  board_id[33];           //!< Board ID (nul-terminated string)
    uint8_t  firmware_date[43];      //!< Firmware date (yyyy/mm/dd, nul-terminated string)
};  // clang-format on

static_assert(sizeof(UnibVersionPayload) == 308, "");

/* ****************************************************************************************************************** */
}  // namespace unib
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_PARSER_UNIB_HPP__
