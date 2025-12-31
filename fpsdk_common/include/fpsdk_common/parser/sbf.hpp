/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 *
 * The information on message structures, IDs, descriptions etc. in this file are from publicly available data, such as:
 * - mosaic-G5 Reference Guide, copyright 2000-2025 Septentrio NV/SA, part of HEXAGON
 *   (direct download link from https://www.ardusimple.com/how-to-configure-septentrio-mosaic-g5/#documentation)
 * - sbf2asc source code, (c) Copyright 2002-2015 Septentrio NV/SA
 * - https://github.com/septentrio-gnss/septentrio_gnss_driver
 * - https://github.com/rtklibexplorer/RTKLIB
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Parser SBF routines and types
 *
 * @page FPSDK_COMMON_PARSER_SBF Parser SBF routines and types
 *
 * **API**: fpsdk_common/parser/sbf.hpp and fpsdk::common::parser::sbf
 *
 */
#ifndef __FPSDK_COMMON_PARSER_SBF_HPP__
#define __FPSDK_COMMON_PARSER_SBF_HPP__

/* LIBC/STL */
#include <cstdint>
#include <limits>

/* EXTERNAL */

/* PACKAGE */

namespace fpsdk {
namespace common {
namespace parser {
/**
 * @brief Parser SBF routines and types
 */
namespace sbf {
/* ****************************************************************************************************************** */

static constexpr uint8_t SBF_SYNC_1 = '$';       //!< Sync char 1 (like NMEA...)
static constexpr uint8_t SBF_SYNC_2 = '@';       //!< Sync char 2
static constexpr std::size_t SBF_HEAD_SIZE = 8;  //!< Size of the SBF header (SbfHeader)

/**
 * @brief Get block type (message ID w/o revision)
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid SBF message.
 *
 * @returns block type
 */
constexpr uint16_t SbfBlockType(const uint8_t* msg)
{
    return (((uint16_t)((uint8_t*)msg)[5] << 8) | (uint16_t)((uint8_t*)msg)[4]) & 0x1fff;
}

/**
 * @brief Get block revision
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid SBF message.
 *
 * @returns block type
 */
constexpr uint8_t SbfBlockRev(const uint8_t* msg)
{
    return ((uint8_t*)msg)[5] >> 5;
}

/**
 * @brief Get message size
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid SBF message.
 *
 * @returns the message size
 */
constexpr uint16_t SbfMsgSize(const uint8_t* msg)
{
    return (((uint16_t)((uint8_t*)msg)[7] << 8) | (uint16_t)((uint8_t*)msg)[6]);
}

/**
 * @brief Get SBF message name
 *
 * Generates a name (string) in the form "SBF-NAME", where NAME is a suitable stringifications of the
 * message ID if known (for example, "SBF-NAVCART", respectively "%05u" formatted message ID if unknown (for
 * example, "SBF-BLOCK01234").
 *
 * @param[out] name      String to write the name to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the SBF message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid SBF message.
 *
 * @returns true if message name was generated, false if \c name buffer was too small
 */
bool SbfGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Get SBF message info
 *
 * This stringifies the content of some SBF messages, for debugging.
 *
 * @param[out] info      String to write the info to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the SBF message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid SBF message.
 *
 * @returns true if message info was generated (even if info is empty), false if \c name buffer was too small
 */
bool SbfGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Get description for a SBF message
 *
 * @param[in]  type     Message block type
 *
 * @return the description of the message type, if available, NULL otherwise
 */
const char* SbfGetTypeDesc(const uint16_t type);

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @name SBF messages (names and IDs)
 *
 * @{
 */
// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_SBF_MESSAGES}
static constexpr uint16_t    SBF_MEASEPOCH_MSGID              =  4027;                         //!< SBF-MEASEPOCH message ID
static constexpr const char* SBF_MEASEPOCH_STRID              = "SBF-MEASEPOCH";               //!< SBF-MEASEPOCH message name
static constexpr uint16_t    SBF_MEASEXTRA_MSGID              =  4000;                         //!< SBF-MEASEXTRA message ID
static constexpr const char* SBF_MEASEXTRA_STRID              = "SBF-MEASEXTRA";               //!< SBF-MEASEXTRA message name
static constexpr uint16_t    SBF_ENDOFMEAS_MSGID              =  5922;                         //!< SBF-ENDOFMEAS message ID
static constexpr const char* SBF_ENDOFMEAS_STRID              = "SBF-ENDOFMEAS";               //!< SBF-ENDOFMEAS message name
static constexpr uint16_t    SBF_GPSRAWCA_MSGID               =  4017;                         //!< SBF-GPSRAWCA message ID
static constexpr const char* SBF_GPSRAWCA_STRID               = "SBF-GPSRAWCA";                //!< SBF-GPSRAWCA message name
static constexpr uint16_t    SBF_GPSRAWL2C_MSGID              =  4018;                         //!< SBF-GPSRAWL2C message ID
static constexpr const char* SBF_GPSRAWL2C_STRID              = "SBF-GPSRAWL2C";               //!< SBF-GPSRAWL2C message name
static constexpr uint16_t    SBF_GPSRAWL5_MSGID               =  4019;                         //!< SBF-GPSRAWL5 message ID
static constexpr const char* SBF_GPSRAWL5_STRID               = "SBF-GPSRAWL5";                //!< SBF-GPSRAWL5 message name
static constexpr uint16_t    SBF_GPSRAWL1C_MSGID              =  4221;                         //!< SBF-GPSRAWL1C message ID
static constexpr const char* SBF_GPSRAWL1C_STRID              = "SBF-GPSRAWL1C";               //!< SBF-GPSRAWL1C message name
static constexpr uint16_t    SBF_GLORAWCA_MSGID               =  4026;                         //!< SBF-GLORAWCA message ID
static constexpr const char* SBF_GLORAWCA_STRID               = "SBF-GLORAWCA";                //!< SBF-GLORAWCA message name
static constexpr uint16_t    SBF_GALRAWFNAV_MSGID             =  4022;                         //!< SBF-GALRAWFNAV message ID
static constexpr const char* SBF_GALRAWFNAV_STRID             = "SBF-GALRAWFNAV";              //!< SBF-GALRAWFNAV message name
static constexpr uint16_t    SBF_GALRAWINAV_MSGID             =  4023;                         //!< SBF-GALRAWINAV message ID
static constexpr const char* SBF_GALRAWINAV_STRID             = "SBF-GALRAWINAV";              //!< SBF-GALRAWINAV message name
static constexpr uint16_t    SBF_GALRAWCNAV_MSGID             =  4024;                         //!< SBF-GALRAWCNAV message ID
static constexpr const char* SBF_GALRAWCNAV_STRID             = "SBF-GALRAWCNAV";              //!< SBF-GALRAWCNAV message name
static constexpr uint16_t    SBF_GEORAWL1_MSGID               =  4020;                         //!< SBF-GEORAWL1 message ID
static constexpr const char* SBF_GEORAWL1_STRID               = "SBF-GEORAWL1";                //!< SBF-GEORAWL1 message name
static constexpr uint16_t    SBF_GEORAWL5_MSGID               =  4021;                         //!< SBF-GEORAWL5 message ID
static constexpr const char* SBF_GEORAWL5_STRID               = "SBF-GEORAWL5";                //!< SBF-GEORAWL5 message name
static constexpr uint16_t    SBF_BDSRAW_MSGID                 =  4047;                         //!< SBF-BDSRAW message ID
static constexpr const char* SBF_BDSRAW_STRID                 = "SBF-BDSRAW";                  //!< SBF-BDSRAW message name
static constexpr uint16_t    SBF_BDSRAWB1C_MSGID              =  4218;                         //!< SBF-BDSRAWB1C message ID
static constexpr const char* SBF_BDSRAWB1C_STRID              = "SBF-BDSRAWB1C";               //!< SBF-BDSRAWB1C message name
static constexpr uint16_t    SBF_BDSRAWB2A_MSGID              =  4219;                         //!< SBF-BDSRAWB2A message ID
static constexpr const char* SBF_BDSRAWB2A_STRID              = "SBF-BDSRAWB2A";               //!< SBF-BDSRAWB2A message name
static constexpr uint16_t    SBF_BDSRAWB2B_MSGID              =  4242;                         //!< SBF-BDSRAWB2B message ID
static constexpr const char* SBF_BDSRAWB2B_STRID              = "SBF-BDSRAWB2B";               //!< SBF-BDSRAWB2B message name
static constexpr uint16_t    SBF_QZSRAWL1CA_MSGID             =  4066;                         //!< SBF-QZSRAWL1CA message ID
static constexpr const char* SBF_QZSRAWL1CA_STRID             = "SBF-QZSRAWL1CA";              //!< SBF-QZSRAWL1CA message name
static constexpr uint16_t    SBF_QZSRAWL2C_MSGID              =  4067;                         //!< SBF-QZSRAWL2C message ID
static constexpr const char* SBF_QZSRAWL2C_STRID              = "SBF-QZSRAWL2C";               //!< SBF-QZSRAWL2C message name
static constexpr uint16_t    SBF_QZSRAWL5_MSGID               =  4068;                         //!< SBF-QZSRAWL5 message ID
static constexpr const char* SBF_QZSRAWL5_STRID               = "SBF-QZSRAWL5";                //!< SBF-QZSRAWL5 message name
static constexpr uint16_t    SBF_QZSRAWL6D_MSGID              =  4270;                         //!< SBF-QZSRAWL6D message ID
static constexpr const char* SBF_QZSRAWL6D_STRID              = "SBF-QZSRAWL6D";               //!< SBF-QZSRAWL6D message name
static constexpr uint16_t    SBF_QZSRAWL6E_MSGID              =  4271;                         //!< SBF-QZSRAWL6E message ID
static constexpr const char* SBF_QZSRAWL6E_STRID              = "SBF-QZSRAWL6E";               //!< SBF-QZSRAWL6E message name
static constexpr uint16_t    SBF_QZSRAWL1C_MSGID              =  4227;                         //!< SBF-QZSRAWL1C message ID
static constexpr const char* SBF_QZSRAWL1C_STRID              = "SBF-QZSRAWL1C";               //!< SBF-QZSRAWL1C message name
static constexpr uint16_t    SBF_QZSRAWL1S_MSGID              =  4228;                         //!< SBF-QZSRAWL1S message ID
static constexpr const char* SBF_QZSRAWL1S_STRID              = "SBF-QZSRAWL1S";               //!< SBF-QZSRAWL1S message name
static constexpr uint16_t    SBF_QZSRAWL5S_MSGID              =  4246;                         //!< SBF-QZSRAWL5S message ID
static constexpr const char* SBF_QZSRAWL5S_STRID              = "SBF-QZSRAWL5S";               //!< SBF-QZSRAWL5S message name
static constexpr uint16_t    SBF_NAVICRAW_MSGID               =  4093;                         //!< SBF-NAVICRAW message ID
static constexpr const char* SBF_NAVICRAW_STRID               = "SBF-NAVICRAW";                //!< SBF-NAVICRAW message name
static constexpr uint16_t    SBF_GPSNAV_MSGID                 =  5891;                         //!< SBF-GPSNAV message ID
static constexpr const char* SBF_GPSNAV_STRID                 = "SBF-GPSNAV";                  //!< SBF-GPSNAV message name
static constexpr uint16_t    SBF_GPSALM_MSGID                 =  5892;                         //!< SBF-GPSALM message ID
static constexpr const char* SBF_GPSALM_STRID                 = "SBF-GPSALM";                  //!< SBF-GPSALM message name
static constexpr uint16_t    SBF_GPSION_MSGID                 =  5893;                         //!< SBF-GPSION message ID
static constexpr const char* SBF_GPSION_STRID                 = "SBF-GPSION";                  //!< SBF-GPSION message name
static constexpr uint16_t    SBF_GPSUTC_MSGID                 =  5894;                         //!< SBF-GPSUTC message ID
static constexpr const char* SBF_GPSUTC_STRID                 = "SBF-GPSUTC";                  //!< SBF-GPSUTC message name
static constexpr uint16_t    SBF_GPSCNAV_MSGID                =  4042;                         //!< SBF-GPSCNAV message ID
static constexpr const char* SBF_GPSCNAV_STRID                = "SBF-GPSCNAV";                 //!< SBF-GPSCNAV message name
static constexpr uint16_t    SBF_GLONAV_MSGID                 =  4004;                         //!< SBF-GLONAV message ID
static constexpr const char* SBF_GLONAV_STRID                 = "SBF-GLONAV";                  //!< SBF-GLONAV message name
static constexpr uint16_t    SBF_GLOALM_MSGID                 =  4005;                         //!< SBF-GLOALM message ID
static constexpr const char* SBF_GLOALM_STRID                 = "SBF-GLOALM";                  //!< SBF-GLOALM message name
static constexpr uint16_t    SBF_GLOTIME_MSGID                =  4036;                         //!< SBF-GLOTIME message ID
static constexpr const char* SBF_GLOTIME_STRID                = "SBF-GLOTIME";                 //!< SBF-GLOTIME message name
static constexpr uint16_t    SBF_GALNAV_MSGID                 =  4002;                         //!< SBF-GALNAV message ID
static constexpr const char* SBF_GALNAV_STRID                 = "SBF-GALNAV";                  //!< SBF-GALNAV message name
static constexpr uint16_t    SBF_GALALM_MSGID                 =  4003;                         //!< SBF-GALALM message ID
static constexpr const char* SBF_GALALM_STRID                 = "SBF-GALALM";                  //!< SBF-GALALM message name
static constexpr uint16_t    SBF_GALION_MSGID                 =  4030;                         //!< SBF-GALION message ID
static constexpr const char* SBF_GALION_STRID                 = "SBF-GALION";                  //!< SBF-GALION message name
static constexpr uint16_t    SBF_GALUTC_MSGID                 =  4031;                         //!< SBF-GALUTC message ID
static constexpr const char* SBF_GALUTC_STRID                 = "SBF-GALUTC";                  //!< SBF-GALUTC message name
static constexpr uint16_t    SBF_GALGSTGPS_MSGID              =  4032;                         //!< SBF-GALGSTGPS message ID
static constexpr const char* SBF_GALGSTGPS_STRID              = "SBF-GALGSTGPS";               //!< SBF-GALGSTGPS message name
static constexpr uint16_t    SBF_GALSARRLM_MSGID              =  4034;                         //!< SBF-GALSARRLM message ID
static constexpr const char* SBF_GALSARRLM_STRID              = "SBF-GALSARRLM";               //!< SBF-GALSARRLM message name
static constexpr uint16_t    SBF_BDSNAV_MSGID                 =  4081;                         //!< SBF-BDSNAV message ID
static constexpr const char* SBF_BDSNAV_STRID                 = "SBF-BDSNAV";                  //!< SBF-BDSNAV message name
static constexpr uint16_t    SBF_BDSCNAV1_MSGID               =  4251;                         //!< SBF-BDSCNAV1 message ID
static constexpr const char* SBF_BDSCNAV1_STRID               = "SBF-BDSCNAV1";                //!< SBF-BDSCNAV1 message name
static constexpr uint16_t    SBF_BDSCNAV2_MSGID               =  4252;                         //!< SBF-BDSCNAV2 message ID
static constexpr const char* SBF_BDSCNAV2_STRID               = "SBF-BDSCNAV2";                //!< SBF-BDSCNAV2 message name
static constexpr uint16_t    SBF_BDSCNAV3_MSGID               =  4253;                         //!< SBF-BDSCNAV3 message ID
static constexpr const char* SBF_BDSCNAV3_STRID               = "SBF-BDSCNAV3";                //!< SBF-BDSCNAV3 message name
static constexpr uint16_t    SBF_BDSALM_MSGID                 =  4119;                         //!< SBF-BDSALM message ID
static constexpr const char* SBF_BDSALM_STRID                 = "SBF-BDSALM";                  //!< SBF-BDSALM message name
static constexpr uint16_t    SBF_BDSION_MSGID                 =  4120;                         //!< SBF-BDSION message ID
static constexpr const char* SBF_BDSION_STRID                 = "SBF-BDSION";                  //!< SBF-BDSION message name
static constexpr uint16_t    SBF_BDSUTC_MSGID                 =  4121;                         //!< SBF-BDSUTC message ID
static constexpr const char* SBF_BDSUTC_STRID                 = "SBF-BDSUTC";                  //!< SBF-BDSUTC message name
static constexpr uint16_t    SBF_QZSNAV_MSGID                 =  4095;                         //!< SBF-QZSNAV message ID
static constexpr const char* SBF_QZSNAV_STRID                 = "SBF-QZSNAV";                  //!< SBF-QZSNAV message name
static constexpr uint16_t    SBF_QZSALM_MSGID                 =  4116;                         //!< SBF-QZSALM message ID
static constexpr const char* SBF_QZSALM_STRID                 = "SBF-QZSALM";                  //!< SBF-QZSALM message name
static constexpr uint16_t    SBF_NAVICLNAV_MSGID              =  4254;                         //!< SBF-NAVICLNAV message ID
static constexpr const char* SBF_NAVICLNAV_STRID              = "SBF-NAVICLNAV";               //!< SBF-NAVICLNAV message name
static constexpr uint16_t    SBF_GEONAV_MSGID                 =  5896;                         //!< SBF-GEONAV message ID
static constexpr const char* SBF_GEONAV_STRID                 = "SBF-GEONAV";                  //!< SBF-GEONAV message name
static constexpr uint16_t    SBF_GEOALM_MSGID                 =  5897;                         //!< SBF-GEOALM message ID
static constexpr const char* SBF_GEOALM_STRID                 = "SBF-GEOALM";                  //!< SBF-GEOALM message name
static constexpr uint16_t    SBF_PVTCARTESIAN_MSGID           =  4006;                         //!< SBF-PVTCARTESIAN message ID
static constexpr const char* SBF_PVTCARTESIAN_STRID           = "SBF-PVTCARTESIAN";            //!< SBF-PVTCARTESIAN message name
static constexpr uint16_t    SBF_PVTGEODETIC_MSGID            =  4007;                         //!< SBF-PVTGEODETIC message ID
static constexpr const char* SBF_PVTGEODETIC_STRID            = "SBF-PVTGEODETIC";             //!< SBF-PVTGEODETIC message name
static constexpr uint16_t    SBF_POSCOVCARTESIAN_MSGID        =  5905;                         //!< SBF-POSCOVCARTESIAN message ID
static constexpr const char* SBF_POSCOVCARTESIAN_STRID        = "SBF-POSCOVCARTESIAN";         //!< SBF-POSCOVCARTESIAN message name
static constexpr uint16_t    SBF_POSCOVGEODETIC_MSGID         =  5906;                         //!< SBF-POSCOVGEODETIC message ID
static constexpr const char* SBF_POSCOVGEODETIC_STRID         = "SBF-POSCOVGEODETIC";          //!< SBF-POSCOVGEODETIC message name
static constexpr uint16_t    SBF_VELCOVCARTESIAN_MSGID        =  5907;                         //!< SBF-VELCOVCARTESIAN message ID
static constexpr const char* SBF_VELCOVCARTESIAN_STRID        = "SBF-VELCOVCARTESIAN";         //!< SBF-VELCOVCARTESIAN message name
static constexpr uint16_t    SBF_VELCOVGEODETIC_MSGID         =  5908;                         //!< SBF-VELCOVGEODETIC message ID
static constexpr const char* SBF_VELCOVGEODETIC_STRID         = "SBF-VELCOVGEODETIC";          //!< SBF-VELCOVGEODETIC message name
static constexpr uint16_t    SBF_DOP_MSGID                    =  4001;                         //!< SBF-DOP message ID
static constexpr const char* SBF_DOP_STRID                    = "SBF-DOP";                     //!< SBF-DOP message name
static constexpr uint16_t    SBF_BASEVECTORCART_MSGID         =  4043;                         //!< SBF-BASEVECTORCART message ID
static constexpr const char* SBF_BASEVECTORCART_STRID         = "SBF-BASEVECTORCART";          //!< SBF-BASEVECTORCART message name
static constexpr uint16_t    SBF_BASEVECTORGEOD_MSGID         =  4028;                         //!< SBF-BASEVECTORGEOD message ID
static constexpr const char* SBF_BASEVECTORGEOD_STRID         = "SBF-BASEVECTORGEOD";          //!< SBF-BASEVECTORGEOD message name
static constexpr uint16_t    SBF_PVTSUPPORT_MSGID             =  4076;                         //!< SBF-PVTSUPPORT message ID
static constexpr const char* SBF_PVTSUPPORT_STRID             = "SBF-PVTSUPPORT";              //!< SBF-PVTSUPPORT message name
static constexpr uint16_t    SBF_PVTSUPPORTA_MSGID            =  4079;                         //!< SBF-PVTSUPPORTA message ID
static constexpr const char* SBF_PVTSUPPORTA_STRID            = "SBF-PVTSUPPORTA";             //!< SBF-PVTSUPPORTA message name
static constexpr uint16_t    SBF_ENDOFPVT_MSGID               =  5921;                         //!< SBF-ENDOFPVT message ID
static constexpr const char* SBF_ENDOFPVT_STRID               = "SBF-ENDOFPVT";                //!< SBF-ENDOFPVT message name
static constexpr uint16_t    SBF_NAVCART_MSGID                =  4272;                         //!< SBF-NAVCART message ID
static constexpr const char* SBF_NAVCART_STRID                = "SBF-NAVCART";                 //!< SBF-NAVCART message name
static constexpr uint16_t    SBF_ATTEULER_MSGID               =  5938;                         //!< SBF-ATTEULER message ID
static constexpr const char* SBF_ATTEULER_STRID               = "SBF-ATTEULER";                //!< SBF-ATTEULER message name
static constexpr uint16_t    SBF_ATTCOVEULER_MSGID            =  5939;                         //!< SBF-ATTCOVEULER message ID
static constexpr const char* SBF_ATTCOVEULER_STRID            = "SBF-ATTCOVEULER";             //!< SBF-ATTCOVEULER message name
static constexpr uint16_t    SBF_AUXANTPOSITIONS_MSGID        =  5942;                         //!< SBF-AUXANTPOSITIONS message ID
static constexpr const char* SBF_AUXANTPOSITIONS_STRID        = "SBF-AUXANTPOSITIONS";         //!< SBF-AUXANTPOSITIONS message name
static constexpr uint16_t    SBF_ENDOFATT_MSGID               =  5943;                         //!< SBF-ENDOFATT message ID
static constexpr const char* SBF_ENDOFATT_STRID               = "SBF-ENDOFATT";                //!< SBF-ENDOFATT message name
static constexpr uint16_t    SBF_RECEIVERTIME_MSGID           =  5914;                         //!< SBF-RECEIVERTIME message ID
static constexpr const char* SBF_RECEIVERTIME_STRID           = "SBF-RECEIVERTIME";            //!< SBF-RECEIVERTIME message name
static constexpr uint16_t    SBF_XPPSOFFSET_MSGID             =  5911;                         //!< SBF-XPPSOFFSET message ID
static constexpr const char* SBF_XPPSOFFSET_STRID             = "SBF-XPPSOFFSET";              //!< SBF-XPPSOFFSET message name
static constexpr uint16_t    SBF_EXTEVENT_MSGID               =  5924;                         //!< SBF-EXTEVENT message ID
static constexpr const char* SBF_EXTEVENT_STRID               = "SBF-EXTEVENT";                //!< SBF-EXTEVENT message name
static constexpr uint16_t    SBF_EXTEVENTPVTCARTESIAN_MSGID   =  4037;                         //!< SBF-EXTEVENTPVTCARTESIAN message ID
static constexpr const char* SBF_EXTEVENTPVTCARTESIAN_STRID   = "SBF-EXTEVENTPVTCARTESIAN";    //!< SBF-EXTEVENTPVTCARTESIAN message name
static constexpr uint16_t    SBF_EXTEVENTPVTGEODETIC_MSGID    =  4038;                         //!< SBF-EXTEVENTPVTGEODETIC message ID
static constexpr const char* SBF_EXTEVENTPVTGEODETIC_STRID    = "SBF-EXTEVENTPVTGEODETIC";     //!< SBF-EXTEVENTPVTGEODETIC message name
static constexpr uint16_t    SBF_EXTEVENTBASEVECTGEOD_MSGID   =  4217;                         //!< SBF-EXTEVENTBASEVECTGEOD message ID
static constexpr const char* SBF_EXTEVENTBASEVECTGEOD_STRID   = "SBF-EXTEVENTBASEVECTGEOD";    //!< SBF-EXTEVENTBASEVECTGEOD message name
static constexpr uint16_t    SBF_EXTEVENTATTEULER_MSGID       =  4237;                         //!< SBF-EXTEVENTATTEULER message ID
static constexpr const char* SBF_EXTEVENTATTEULER_STRID       = "SBF-EXTEVENTATTEULER";        //!< SBF-EXTEVENTATTEULER message name
static constexpr uint16_t    SBF_DIFFCORRIN_MSGID             =  5919;                         //!< SBF-DIFFCORRIN message ID
static constexpr const char* SBF_DIFFCORRIN_STRID             = "SBF-DIFFCORRIN";              //!< SBF-DIFFCORRIN message name
static constexpr uint16_t    SBF_BASESTATION_MSGID            =  5949;                         //!< SBF-BASESTATION message ID
static constexpr const char* SBF_BASESTATION_STRID            = "SBF-BASESTATION";             //!< SBF-BASESTATION message name
static constexpr uint16_t    SBF_LBANDTRACKERSTATUS_MSGID     =  4201;                         //!< SBF-LBANDTRACKERSTATUS message ID
static constexpr const char* SBF_LBANDTRACKERSTATUS_STRID     = "SBF-LBANDTRACKERSTATUS";      //!< SBF-LBANDTRACKERSTATUS message name
static constexpr uint16_t    SBF_LBANDRAW_MSGID               =  4212;                         //!< SBF-LBANDRAW message ID
static constexpr const char* SBF_LBANDRAW_STRID               = "SBF-LBANDRAW";                //!< SBF-LBANDRAW message name
static constexpr uint16_t    SBF_CHANNELSTATUS_MSGID          =  4013;                         //!< SBF-CHANNELSTATUS message ID
static constexpr const char* SBF_CHANNELSTATUS_STRID          = "SBF-CHANNELSTATUS";           //!< SBF-CHANNELSTATUS message name
static constexpr uint16_t    SBF_RECEIVERSTATUS_MSGID         =  4014;                         //!< SBF-RECEIVERSTATUS message ID
static constexpr const char* SBF_RECEIVERSTATUS_STRID         = "SBF-RECEIVERSTATUS";          //!< SBF-RECEIVERSTATUS message name
static constexpr uint16_t    SBF_SATVISIBILITY_MSGID          =  4012;                         //!< SBF-SATVISIBILITY message ID
static constexpr const char* SBF_SATVISIBILITY_STRID          = "SBF-SATVISIBILITY";           //!< SBF-SATVISIBILITY message name
static constexpr uint16_t    SBF_INPUTLINK_MSGID              =  4090;                         //!< SBF-INPUTLINK message ID
static constexpr const char* SBF_INPUTLINK_STRID              = "SBF-INPUTLINK";               //!< SBF-INPUTLINK message name
static constexpr uint16_t    SBF_OUTPUTLINK_MSGID             =  4091;                         //!< SBF-OUTPUTLINK message ID
static constexpr const char* SBF_OUTPUTLINK_STRID             = "SBF-OUTPUTLINK";              //!< SBF-OUTPUTLINK message name
static constexpr uint16_t    SBF_QUALITYIND_MSGID             =  4082;                         //!< SBF-QUALITYIND message ID
static constexpr const char* SBF_QUALITYIND_STRID             = "SBF-QUALITYIND";              //!< SBF-QUALITYIND message name
static constexpr uint16_t    SBF_DISKSTATUS_MSGID             =  4059;                         //!< SBF-DISKSTATUS message ID
static constexpr const char* SBF_DISKSTATUS_STRID             = "SBF-DISKSTATUS";              //!< SBF-DISKSTATUS message name
static constexpr uint16_t    SBF_RFSTATUS_MSGID               =  4092;                         //!< SBF-RFSTATUS message ID
static constexpr const char* SBF_RFSTATUS_STRID               = "SBF-RFSTATUS";                //!< SBF-RFSTATUS message name
static constexpr uint16_t    SBF_GALAUTHSTATUS_MSGID          =  4245;                         //!< SBF-GALAUTHSTATUS message ID
static constexpr const char* SBF_GALAUTHSTATUS_STRID          = "SBF-GALAUTHSTATUS";           //!< SBF-GALAUTHSTATUS message name
static constexpr uint16_t    SBF_RECEIVERSETUP_MSGID          =  5902;                         //!< SBF-RECEIVERSETUP message ID
static constexpr const char* SBF_RECEIVERSETUP_STRID          = "SBF-RECEIVERSETUP";           //!< SBF-RECEIVERSETUP message name
static constexpr uint16_t    SBF_RXMESSAGE_MSGID              =  4103;                         //!< SBF-RXMESSAGE message ID
static constexpr const char* SBF_RXMESSAGE_STRID              = "SBF-RXMESSAGE";               //!< SBF-RXMESSAGE message name
static constexpr uint16_t    SBF_COMMANDS_MSGID               =  4015;                         //!< SBF-COMMANDS message ID
static constexpr const char* SBF_COMMANDS_STRID               = "SBF-COMMANDS";                //!< SBF-COMMANDS message name
static constexpr uint16_t    SBF_COMMENT_MSGID                =  5936;                         //!< SBF-COMMENT message ID
static constexpr const char* SBF_COMMENT_STRID                = "SBF-COMMENT";                 //!< SBF-COMMENT message name
static constexpr uint16_t    SBF_BBSAMPLES_MSGID              =  4040;                         //!< SBF-BBSAMPLES message ID
static constexpr const char* SBF_BBSAMPLES_STRID              = "SBF-BBSAMPLES";               //!< SBF-BBSAMPLES message name
static constexpr uint16_t    SBF_ASCIIIN_MSGID                =  4075;                         //!< SBF-ASCIIIN message ID
static constexpr const char* SBF_ASCIIIN_STRID                = "SBF-ASCIIIN";                 //!< SBF-ASCIIIN message name
// @fp_codegen_end{FPSDK_COMMON_PARSER_SBF_MESSAGES}
// clang-format on
///@}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief SBF block header
 */
struct SbfHeader
{  // clang-format off
    uint8_t  sync1;             //!< = SBF_SYNC_1
    uint8_t  sync2;             //!< = SBF_SYNC_2
    uint16_t id;                //!< Block (message) type (bits 0..12) and version (bits 13..15)
    uint16_t crc;               //!< CRC
    uint16_t length;            //!< Block (payload) size
};  // clang-format on

static_assert(sizeof(SbfHeader) == SBF_HEAD_SIZE, "");

// ---------------------------------------------------------------------------------------------------------------------

// Doxygen is easily confused.. :-/
//! Message struct that must be packed
#ifndef _DOXYGEN_
#  define SFB_PACKED __attribute__((packed))
#else
#  define SFB_PACKED /* packed */
#endif

// clang-format off
static constexpr int8_t   SBF_DONOTUSE_INT8   = std::numeric_limits<int8_t>::min();                                     //!< Do-not-use value for int8_t
static constexpr int16_t  SBF_DONOTUSE_INT16  = std::numeric_limits<int16_t>::min();                                    //!< Do-not-use value for int16_t
static constexpr int32_t  SBF_DONOTUSE_INT32  = std::numeric_limits<int32_t>::min();                                    //!< Do-not-use value for int32_t
static constexpr int8_t   SBF_DONOTUSE_UINT8  = std::numeric_limits<uint8_t>::min();                                    //!< Do-not-use value for uint8_t
static constexpr uint16_t SBF_DONOTUSE_UINT16 = std::numeric_limits<uint16_t>::max();                                   //!< Do-not-use value for uint16_t
static constexpr uint32_t SBF_DONOTUSE_UINT32 = std::numeric_limits<uint32_t>::max();                                   //!< Do-not-use value for uint32_t
static constexpr float    SBF_DONOTUSE_FLOAT  = -2e10f;                                                                 //!< Do-not-use value for float
static constexpr double   SBF_DONOTUSE_DOUBLE = -2e10;                                                                  //!< Do-not-use value for double

static constexpr bool SbfDoNotUse(const int8_t   val) { return val == SBF_DONOTUSE_INT8;   }                           //!< Check if value is do-not-use
static constexpr bool SbfDoNotUse(const int16_t  val) { return val == SBF_DONOTUSE_INT16;  }                           //!< Check if value is do-not-use
static constexpr bool SbfDoNotUse(const int32_t  val) { return val == SBF_DONOTUSE_INT32;  }                           //!< Check if value is do-not-use
static constexpr bool SbfDoNotUse(const uint8_t  val) { return val == SBF_DONOTUSE_UINT8;  }                           //!< Check if value is do-not-use
static constexpr bool SbfDoNotUse(const uint16_t val) { return val == SBF_DONOTUSE_UINT16; }                           //!< Check if value is do-not-use
static constexpr bool SbfDoNotUse(const uint32_t val) { return val == SBF_DONOTUSE_UINT32; }                           //!< Check if value is do-not-use
static constexpr bool SbfDoNotUse(const double   val) { return val == SBF_DONOTUSE_FLOAT;  }                           //!< Check if value is do-not-use
static constexpr bool SbfDoNotUse(const float    val) { return val == SBF_DONOTUSE_DOUBLE; }                           //!< Check if value is do-not-use
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

// clang-format off
static constexpr uint8_t  SBF_MODEPVT_SOL(const uint8_t mode)                   { return mode & 0x0f; }                 //!< GNSS PVT mode
static constexpr uint8_t  SBF_MODEPVT_SOL_NO_GNSS_PVT                           =  0;                                   //!< No GNSS PVT available (the Error field indicates the cause of the absence of the PVT solution)
static constexpr uint8_t  SBF_MODEPVT_SOL_STANDALONE_PVT                        =  1;                                   //!< Stand-Alone PVT
static constexpr uint8_t  SBF_MODEPVT_SOL_DIFFERENTIAL_PVT                      =  2;                                   //!< Differential PVT
static constexpr uint8_t  SBF_MODEPVT_SOL_FIXED_LOCATION                        =  3;                                   //!< Fixed location
static constexpr uint8_t  SBF_MODEPVT_SOL_RTK_FIXED                             =  4;                                   //!< RTK with fixed ambiguities
static constexpr uint8_t  SBF_MODEPVT_SOL_RTK_FLOAT                             =  5;                                   //!< RTK with float ambiguities
static constexpr uint8_t  SBF_MODEPVT_SOL_SBAS_AIDED_PVT                        =  6;                                   //!< SBAS aided PVT
static constexpr uint8_t  SBF_MODEPVT_SOL_MOVINGBASE_RTK_FIXED                  =  7;                                   //!< moving-base RTK with fixed ambiguities
static constexpr uint8_t  SBF_MODEPVT_SOL_MOVINGBASE_RTK_FLOAT                  =  8;                                   //!< moving-base RTK with float ambiguities
static constexpr uint8_t  SBF_MODEPVT_SOL_RESERVED0                             =  9;                                   //!< Reserved
static constexpr uint8_t  SBF_MODEPVT_SOL_PPP                                   = 10;                                   //!< Precise point positioning (PPP)
static constexpr uint8_t  SBF_MODEPVT_SOL_RESERVED1                             = 12;                                   //!< Reserved
static constexpr bool     SBF_MODEPVT_IS2D(const uint8_t mode)                  { return (mode & 0x80) == 0x80; }       //!< true = 2D mode, false = 3D mode
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

// clang-format off
static constexpr uint8_t  SBF_ERRORPVT_NO_ERROR                                 =  0;                                   //!< No Error
static constexpr uint8_t  SBF_ERRORPVT_NOT_ENOUGH_MEAS                          =  1;                                   //!< Not enough measurements
static constexpr uint8_t  SBF_ERRORPVT_NOT_ENOUGH_EPH                           =  2;                                   //!< Not enough ephemerides available
static constexpr uint8_t  SBF_ERRORPVT_DOP_TOO_LARGE                            =  3;                                   //!< DOP too large (larger than 15)
static constexpr uint8_t  SBF_ERRORPVT_RESIDUALS_TOO_LARGE                      =  4;                                   //!< Sum of squared residuals too large
static constexpr uint8_t  SBF_ERRORPVT_NO_CONVERGENCE                           =  5;                                   //!< No convergence
static constexpr uint8_t  SBF_ERRORPVT_TOO_MANY_OUTLIERS                        =  6;                                   //!< Not enough measurements after outlier rejection
static constexpr uint8_t  SBF_ERRORPVT_POS_OUTPUT_PROHIB                        =  7;                                   //!< Position output prohibited due to export laws
static constexpr uint8_t  SBF_ERRORPVT_NOT_ENOUGH_CORR                          =  8;                                   //!< Not enough differential corrections available
static constexpr uint8_t  SBF_ERRORPVT_NO_BASE_COORD                            =  9;                                   //!< Base station coordinates unavailable
static constexpr uint8_t  SBF_ERRORPVT_AMBIGUITIES_UNFIXED                      = 10;                                   //!< Ambiguities not fixed and user requested to only output RTK-fixed positions
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

// clang-format off
static constexpr uint8_t  SBF_TIMESYSTEM_GPS                                    =   0;                                  //!< GPS time
static constexpr uint8_t  SBF_TIMESYSTEM_GAL                                    =   1;                                  //!< Galileo time
static constexpr uint8_t  SBF_TIMESYSTEM_GLO                                    =   3;                                  //!< GLONASS time
static constexpr uint8_t  SBF_TIMESYSTEM_BDS                                    =   4;                                  //!< BeiDou time
static constexpr uint8_t  SBF_TIMESYSTEM_QZSS                                   =   5;                                  //!< QZSS time
static constexpr uint8_t  SBF_TIMESYSTEM_FUGRO                                  = 100;                                  //!< Fugro AtomiChron time
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

// clang-format off
static constexpr uint8_t  SBF_DATUM_WGS84_ITRS                                  =   0;                                  //!< WGS84/ITRS
static constexpr uint8_t  SBF_DATUM_DGNSS_BASE                                  =  19;                                  //!< Datum equal to that used by the DGNSS/RTK base station
static constexpr uint8_t  SBF_DATUM_ETRS89                                      =  30;                                  //!< ETRS89 (ETRF2000 realization)
static constexpr uint8_t  SBF_DATUM_NAD83                                       =  31;                                  //!< NAD83(2011), North American Datum (2011)
static constexpr uint8_t  SBF_DATUM_NAD83_PA11                                  =  32;                                  //!< NAD83(PA11), North American Datum, Pacific plate (2011)
static constexpr uint8_t  SBF_DATUM_NAD83_MA11                                  =  33;                                  //!< NAD83(MA11), North American Datum, Marianas plate (2011)
static constexpr uint8_t  SBF_DATUM_QDA94                                       =  34;                                  //!< GDA94(2010), Geocentric Datum of Australia (2010)
static constexpr uint8_t  SBF_DATUM_QDA2020                                     =  35;                                  //!< GDA2020, Geocentric Datum of Australia 2020
static constexpr uint8_t  SBF_DATUM_JGD2011                                     =  36;                                  //!< JGD2011, Japanese Geodetic Datum 2011
static constexpr uint8_t  SBF_DATUM_USER1                                       = 250;                                  //!< First user-defined datum
static constexpr uint8_t  SBF_DATUM_USER2                                       = 251;                                  //!< Second user-defined datum
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

// clang-format off
static constexpr bool     SBF_WACORRINFO_ORBCLK_USED(const uint8_t info)        { return (info & 0x01) == 0x01; }       //!< Set if orbit and satellite clock correction information is used
static constexpr bool     SBF_WACORRINFO_RANGECORR_USED(const uint8_t info)     { return (info & 0x02) == 0x02; }       //!< Set if range correction information is used
static constexpr bool     SBF_WACORRINFO_IONO_USED(const uint8_t info)          { return (info & 0x04) == 0x04; }       //!< Set if ionospheric information is used
static constexpr bool     SBF_WACORRINFO_ORBACC_USED(const uint8_t info)        { return (info & 0x08) == 0x08; }       //!< Set if orbit accuracy information is used (UERE/SISA)
static constexpr bool     SBF_WACORRINFO_DO229_USED(const uint8_t info)         { return (info & 0x10) == 0x10; }       //!< Set if DO229 Precision Approach mode is active
static constexpr uint8_t  SBF_WACORRINFO_CORRTYPE(const uint8_t info)           { return (info >> 4) & 0x03; }          //!< Which corrections have been applied
static constexpr uint8_t  SBF_WACORRINFO_CORRTYPE_UNKNOWN                       = 0;                                    //!< Unknown or not in differential positioning mode
static constexpr uint8_t  SBF_WACORRINFO_CORRTYPE_CORS                          = 1;                                    //!< Corrections from a physical base
static constexpr uint8_t  SBF_WACORRINFO_CORRTYPE_VRS                           = 2;                                    //!< Corrections from a virtual base (VRS)
static constexpr uint8_t  SBF_WACORRINFO_CORRTYPE_SSR                           = 3;                                    //!< SSR corrections
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

// clang-format off
static constexpr uint8_t  SBF_ALERTFLAG_RAIM(const uint8_t flag)                { return flag & 0x03; }                 //!< RAIM integrity flag
static constexpr uint8_t  SBF_ALERTFLAG_RAIM_NOTACTIVE                          = 0;                                    //!< RAIM not active
static constexpr uint8_t  SBF_ALERTFLAG_RAIM_SUCESSFUL                          = 1;                                    //!< RAIM integrity test successful
static constexpr uint8_t  SBF_ALERTFLAG_RAIM_FAILED                             = 2;                                    //!< RAIM integrity test failed
static constexpr bool     SBF_ALERTFLAG_GALHPCAFAIL(const uint8_t flag)         { return (flag & 0x04) == 0x04; }       //!< Galileo GPCA failed
static constexpr bool     SBF_ALERTFLAG_GALIONOSTORM(const uint8_t flag)        { return (flag & 0x08) == 0x08; }       //!< Galileo ionospheric storm flag
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

// clang-format off
static constexpr uint16_t SBF_PPPINFO_AGEOFLASTSEED(const uint16_t flag)        { return flag & 0x0fff; }               //!< Age of last seed [s]
static constexpr uint8_t  SBF_PPPINFO_TYPEOFLASTSEED(const uint16_t flag)       { return (flag >> 13) & 0x03; }         //!< Type of last seed
static constexpr uint8_t  SBF_PPPINFO_TYPEOFLASTSEED_NONE                       = 0;                                    //!< Not seeded
static constexpr uint8_t  SBF_PPPINFO_TYPEOFLASTSEED_MANUAL                     = 1;                                    //!< Manual seed
static constexpr uint8_t  SBF_PPPINFO_TYPEOFLASTSEED_DGNSS                      = 2;                                    //!< Seeded from DGNSS
static constexpr uint8_t  SBF_PPPINFO_TYPEOFLASTSEED_RTKFIXED                   = 3;                                    //!< Seeded from RTK fixed
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

// clang-format off
static constexpr bool     SBF_MISC_BASELINEARP(const uint8_t flag)              { return (flag & 0x01) == 0x01; }       //!< Baseline points to base station ARP
static constexpr bool     SBF_MISC_ROVERBASEOFFS(const uint8_t flag)            { return (flag & 0x02) == 0x02; }       //!< Rover phase center offset compensated
static constexpr uint8_t  SBF_MISC_MARKER(const uint8_t flag)                   { return (flag >> 6) & 0x03; }          //!< Marker position
static constexpr uint8_t  SBF_MISC_MARKER_UNKNOWN                               = 0;                                    //!< Unknown
static constexpr uint8_t  SBF_MISC_MARKER_ARPZERO                               = 1;                                    //!< ARP-to-marker offset is zero
static constexpr uint8_t  SBF_MISC_MARKER_ARPNONZERO                            = 2;                                    //!< ARP-to-marker offset is not zero
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

// clang-format off
static constexpr uint8_t  SBF_MODEATT_NO_ATT                                    =  0;                                   //!< No attitude
static constexpr uint8_t  SBF_MODEATT_HDG_PITCH_FLOAT                           =  1;                                   //!< Heading and pitch from float ambiguities
static constexpr uint8_t  SBF_MODEATT_HDG_PITCH_FIXED                           =  2;                                   //!< Heading and pitch from fixed ambiguities
static constexpr uint8_t  SBF_MODEATT_HDG_PITCH_ROLL_FLOAT                      =  3;                                   //!< Heading, pitch and roll from float ambiguities
static constexpr uint8_t  SBF_MODEATT_HDG_PITCH_ROLL_FIXED                      =  4;                                   //!< Heading, pitch and roll from fixed ambiguities
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

// clang-format off
static constexpr uint8_t  SBF_ERRORATT_AUX1(const uint8_t mode)                 { return mode & 0x03; }                 //!< Error code for Main-Aux1 baseline
static constexpr uint8_t  SBF_ERRORATT_AUX1_NO_ERROR                            =  0;                                   //!< No error
static constexpr uint8_t  SBF_ERRORATT_AUX1_NOT_ENOUGH_MEAS                     =  1;                                   //!< Not enough measurements
static constexpr uint8_t  SBF_ERRORATT_AUX1_RESERVED0                           =  2;                                   //!< Reserved
static constexpr uint8_t  SBF_ERRORATT_AUX1_RESERVED1                           =  3;                                   //!< Reserved
static constexpr uint8_t  SBF_ERRORATT_AUX2(const uint8_t mode)                 { return (mode >> 2) & 0x03; }          //!< Error code for Main-Aux2 baseline
static constexpr uint8_t  SBF_ERRORATT_AUX2_NO_ERROR                            =  0;                                   //!< No error
static constexpr uint8_t  SBF_ERRORATT_AUX2_NOT_ENOUGH_MEAS                     =  1;                                   //!< Not enough measurements
static constexpr uint8_t  SBF_ERRORATT_AUX2_RESERVED0                           =  2;                                   //!< Reserved
static constexpr uint8_t  SBF_ERRORATT_AUX2_RESERVED1                           =  3;                                   //!< Reserved
static constexpr bool     SBF_ERRORATT_NO_ATT(const uint8_t mode)               { return (mode & 0x80) == 0x80; }       //!< GNSS-based attitude not requested by user
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief SBF PVTGeodetic payload (rev 2)
 */
struct SFB_PACKED SbfPvtGeodeticRev2
{  // clang-format off
    uint32_t TOW;            //!< GPS time of week [ms]
    uint16_t WNc;            //!< GPS week number
    uint8_t  Mode;           //!< -> SBF_MODEPVT_...
    uint8_t  Error;          //!< -> SBF_ERRORPVT_...
    double   Latitude;       //!< Latitude [rad]
    double   Longitude;      //!< Longitude [rad]
    double   Height;         //!< Ellips. height [m]
    float    Undulation;     //!< Geoid undilation [m]
    float    Vn;             //!< Velocity north [m/s]
    float    Ve;             //!< Velocity east [m/s]
    float    Vu;             //!< Velocity up [m/s]
    float    COG;            //!< Course over ground [deg]
    double   RxClkBias;      //!< Receiver clock bias [ms]
    float    RxClkDrift;     //!< Receiver clock drift [ppm]
    uint8_t  TimeSystem;     //!< -> SBF_TIMESYSTEM_...
    uint8_t  Datum;          //!< -> SBF_DATUM_...
    uint8_t  NrSV;           //!< Number of satellites used
    uint8_t  WACorrInfo;     //!< Applied corrections (bitfield)
    uint16_t ReferenceID;    //!< Reference station ID
    uint16_t MeanCorrAge;    //!< Mean age of corrections [0.01s]
    uint32_t SignalInfo;     //!< Used signals (bitmask)
    uint8_t  AlertFlag;      //!< -> SBF_ALERTFLAG_...
    // Rev 1
    uint8_t  NrBases;        //!< Number of basestations used
    uint16_t PPPInfo;        //!< -> SBF_PPPINFO_...
    // Rev 2
    uint16_t Latency;        //!< Latency [0.0001s]
    uint16_t HAccuracy;      //!< 2DRMS horizontal accuracy estimate [0.01m]
    uint16_t VAccuracy;      //!< 2-sigma vertical accuracy estimate [0.01m]
    uint8_t  Misc;           //!< -> SBF_MISC_...
    uint8_t  padding[1];     //!< Padding
};  // clang-format on

static_assert(sizeof(SbfPvtGeodeticRev2) == 88, "");

// clang-format off
static constexpr std::size_t SBF_PVTGEODETIC_REV2_SIZE = sizeof(SbfPvtGeodeticRev2) + SBF_HEAD_SIZE;  //!< Size of PVTGeodetic rev 2 message
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief SBF PVTCartesian payload (rev 2)
 */
struct SFB_PACKED SbfPvtCartesianRev2
{  // clang-format off
    uint32_t TOW;            //!< GPS time of week [ms]
    uint16_t WNc;            //!< GPS week number
    uint8_t  Mode;           //!< -> SBF_MODEPVT_...
    uint8_t  Error;          //!< -> SBF_ERRORPVT_...
    double   X;              //!< X coordinate in coordinate frame specified by Datum [m]
    double   Y;              //!< Y coordinate in coordinate frame specified by Datum [m]
    double   Z;              //!< Z coordinate in coordinate frame specified by Datum [m]
    float    Undulation;     //!< Geoid undilation [m]
    float    Vx;             //!< Velocity X [m/s]
    float    Vy;             //!< Velocity Y [m/s]
    float    Vz;             //!< Velocity Z [m/s]
    float    COG;            //!< Course over ground [deg]
    double   RxClkBias;      //!< Receiver clock bias [ms]
    float    RxClkDrift;     //!< Receiver clock drift [ppm]
    uint8_t  TimeSystem;     //!< -> SBF_TIMESYSTEM_...
    uint8_t  Datum;          //!< -> SBF_DATUM_...
    uint8_t  NrSV;           //!< Number of satellites used
    uint8_t  WACorrInfo;     //!< Applied corrections (bitfield)
    uint16_t ReferenceID;    //!< Reference station ID
    uint16_t MeanCorrAge;    //!< Mean age of corrections [0.01s]
    uint32_t SignalInfo;     //!< Used signals (bitmask)
    uint8_t  AlertFlag;      //!< -> SBF_ALERTFLAG_...
    // Rev 1
    uint8_t  NrBases;        //!< Number of basestations used
    uint16_t PPPInfo;        //!< -> SBF_PPPINFO_...
    // Rev 2
    uint16_t Latency;        //!< Latency [0.0001s]
    uint16_t HAccuracy;      //!< 2DRMS horizontal accuracy estimate [0.01m]
    uint16_t VAccuracy;      //!< 2-sigma vertical accuracy estimate [0.01m]
    uint8_t  Misc;           //!< -> SBF_MISC_...
    uint8_t  padding[1];     //!< Padding
};  // clang-format on

static_assert(sizeof(SbfPvtCartesianRev2) == 88, "");

// clang-format off
static constexpr std::size_t SBF_PVTCARTESIAN_REV2_SIZE = sizeof(SbfPvtCartesianRev2) + SBF_HEAD_SIZE;  //!< Size of PVTCartesian rev 2 message
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief SBF NavCart payload (rev 0)
 */
struct SFB_PACKED SbfNavCartRev0
{  // clang-format off
    uint32_t TOW;            //!< GPS time of week [ms]
    uint16_t WNc;            //!< GPS week number
    uint8_t  Mode;           //!< -> SBF_PVTMODE_...
    uint8_t  Error;          //!< -> SBF_PVTERROR_
    double   X;              //!< X coordinate in coordinate frame specified by Datum [m]
    double   Y;              //!< Y coordinate in coordinate frame specified by Datum [m]
    double   Z;              //!< Z coordinate in coordinate frame specified by Datum [m]
    float    Undulation;     //!< Geoid undilation [m]
    float    Vx;             //!< Velocity X [m/s]
    float    Vy;             //!< Velocity Y [m/s]
    float    Vz;             //!< Velocity Z [m/s]
    float    COG;            //!< Course over ground [deg]
    double   RxClkBias;      //!< Receiver clock bias [ms]
    float    RxClkDrift;     //!< Receiver clock drift [ppm]
    uint8_t  TimeSystem;     //!< -> SBF_TIMESYSTEM_...
    uint8_t  Datum;          //!< -> SBF_DATUM_...
    uint8_t  NrSV;           //!< Number of satellites used
    uint8_t  WACorrInfo;     //!< Applied corrections (bitfield)
    uint16_t ReferenceID;    //!< Reference station ID
    uint16_t MeanCorrAge;    //!< Mean age of corrections [0.01s]
    uint64_t SignalInfo;     //!< Used signals (bitmask)
    uint8_t  AlertFlag;      //!< -> SBF_ALERTFLAG_...
    uint8_t  NrBases;        //!< Number of basestations used
    uint16_t PPPInfo;        //!< -> SBF_PPPINFO_...
    uint16_t Latency;        //!< Latency [0.0001s]
    uint16_t PosHAcc;        //!< 2DRMS horizontal position accuracy estimate [0.01m]
    uint16_t PosVAcc;        //!< 2-sigma vertical position accuracy estimate [0.01m]
    uint16_t VelHAcc;        //!< 2DRMS horizontal velocity accuracy estimate [0.01m]
    uint16_t VelVacc;        //!< 2-sigma vertical velocity accuracy estimate [0.01m]
    uint8_t  Misc;           //!< -> SBF_MISC_...
    uint8_t  Reserved;       //!< Reserved
    uint16_t ModeAtt;        //!< -> SBF_MODEATT_...
    uint8_t  ErrorAtt;       //!< -> SBF_ERRORATT_...
    uint8_t  NrSVAtt;        //!< Number of satellites used for attitude
    float    Heading;        //!< Heading [deg]
    float    Pitch;          //!< Pitch [deg]
    float    Roll;           //!< Roll [deg]
    uint16_t HeadingAcc;     //!< 2-sigma heading accuracy estimate [0.001deg]
    uint16_t PitchAcc;       //!< 2-sigma pitch accuracy estimate [0.001deg]
    uint16_t RollAcc;        //!< 2-sigma roll accuracy estimate [0.001deg]
    uint16_t PDOP;           //!< PDOP [0.01]
    int8_t   UTCHour;        //!< UTC hours
    int8_t   UTCMin;         //!< UTC minutes
    int16_t  UTCmsec;        //!< UTC seconds [0.001s]
    int8_t   UTCYear;        //!< UTC hours
    int8_t   UTCMonth;       //!< UTC hours
    int8_t   UTCDay;         //!< UTC day
    uint8_t  Padding[1];     //!< Padding
};  // clang-format on

static_assert(sizeof(SbfNavCartRev0) == 128, "");

// clang-format off
static constexpr std::size_t SBF_NAVCART_REV0_SIZE = sizeof(SbfNavCartRev0) + SBF_HEAD_SIZE;  //!< Size of NavCart rev 0 message
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief SBF MeasEpoch payload (rev 1)
 */
struct SFB_PACKED MeasEpochRev1
{  // clang-format off
    uint32_t TOW;            //!< GPS time of week [ms]
    uint16_t WNc;            //!< GPS week number
    uint8_t  N1;             //!< Number of MeasEpochChannelType1Rev0 sub-blocks in this MeasEpoch block
    uint8_t  SB1Length;      //!< Length of a MeasEpochChannelType1 sub-block, excluding the nested MeasEpochChannelType2Rev0 sub-blocks
    uint8_t  SB2Length;      //!< Length of a MeasEpochChannelType2 sub-block
    uint8_t  CommonFlags;    //!< Bit field containing flags common to all measurements
    uint8_t  CumClkJumps;    //!< Cumulative millisecond clock jumps since start-up
    uint8_t  Reserved;       //!< Reserved
};  // clang-format on

static_assert(sizeof(MeasEpochRev1) == 12, "");

/**
 * @brief SBF MeasEpochChannelType1Rev0 (rev 0) sub-block
 */
struct MeasEpochChannelType1Rev0
{  // clang-format off
    uint8_t  RxChannel;      //!< Receiver channel
    uint8_t  Type;           //!< -> MEASEPOCH_CHANNEL_TYPE_...
    uint8_t  SVID;           //!< Satellite ID
    uint8_t  Misc;           //!< @todo document
    uint32_t CodeLSB;        //!< @todo document
    int32_t  Doppler;        //!< @todo document
    uint16_t CarrierLSB;     //!< @todo document
    int8_t   CarrierMSB;     //!< @todo document
    uint8_t  CN0;            //!< C/N0 [0.25dBHz]
    uint16_t LockTime;       //!< Lock time [s]
    uint8_t  ObsInfo;        //!< -> MEASEPOCH_CHANNEL_OBSINFO_...
    uint8_t  N2;             //!< Number of MeasEpochChannelType2 sub-blocks contained in this MeasEpochChannelType1 sub-block
};  // clang-format on

static_assert(sizeof(MeasEpochChannelType1Rev0) == 20, "");

/**
 * @brief SBF MeasEpochChannelType2Rev0 sub-block
 */
struct MeasEpochChannelType2Rev0
{  // clang-format off
    uint8_t  Type;           //!< -> SBF_MEASEPOCH_CHANNEL_TYPE_...
    uint8_t  LockTime;       //!< Lock time [s]
    uint8_t  CN0;            //!< C/N0 [0.25dBHz]
    uint8_t  OffsetsMSB;     //!< @todo document
    int8_t   CarrierMSB;     //!< @todo document
    uint8_t  ObsInfo;        //!< -> SBF_MEASEPOCH_CHANNEL_OBSINFO_...
    uint16_t CodeOffsetLSB;  //!< @todo document
    uint16_t CarrierLSB;     //!< @todo document
    uint16_t DopplerOffsetLSB; //!< @todo document
};  // clang-format on

static_assert(sizeof(MeasEpochChannelType2Rev0) == 12, "");

// clang-format off
static constexpr std::size_t SBF_MEASEPOCH_REV1_MIN_SIZE = sizeof(MeasEpochRev1) + SBF_HEAD_SIZE;  //!< Minimal size of MeasEpoch message
// clang-format on

// clang-format off
static constexpr uint8_t  SBF_MEASEPOCH_CHANNEL_TYPE_SIGIDXLO(const uint8_t type) { return type & 0x1f; }               //!< Signal number (part 1)
static constexpr uint8_t  SBF_MEASEPOCH_CHANNEL_TYPE_ANTID(const uint8_t type)  { return (type >> 5) & 0x07; }          //!< Antenna ID
static constexpr uint8_t  SBF_MEASEPOCH_CHANNEL_TYPE_ANTID_MAIN                 =  0;                                   //!< Main antenna
static constexpr uint8_t  SBF_MEASEPOCH_CHANNEL_TYPE_ANTID_AUX1                 =  1;                                   //!< Aux1 antenna
static constexpr uint8_t  SBF_MEASEPOCH_CHANNEL_TYPE_ANTID_AUX2                 =  2;                                   //!< Aux2 antenna
static constexpr uint8_t  SBF_MEASEPOCH_CHANNEL_OBSINFO_SIGIDXHI(const uint8_t info){ return (info >> 3) & 0x1f; }      //!< Signal number (part 2)
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief SBF ChannelStatus payload (rev 0)
 */
struct SFB_PACKED ChannelStatusRev0
{  // clang-format off
    uint32_t TOW;            //!< GPS time of week [ms]
    uint16_t WNc;            //!< GPS week number
    uint8_t  N;              //!< Number of ChannelSatInfo sub-blocks
    uint8_t  SB1Length;      //!< Length of a ChannelSatInfo sub-block, excluding the nested ChannelStatusChannelType2 sub-blocks
    uint8_t  SB2Length;      //!< Length of a ChannelStateInfo sub-block
    uint8_t  Reserved[3];    //!< Reserved
};  // clang-format on

static_assert(sizeof(ChannelStatusRev0) == 12, "");

/**
 * @brief SBF ChannelSatInfo payload (rev 0)
 */
struct SFB_PACKED ChannelSatInfoRev0
{  // clang-format off
    uint8_t  SVID;           //!< Satellite ID
    uint8_t  FreqNr;         //!< GLONASS frequency number
    uint16_t SVIDFull;       //!< If the SVID field is zero, this field contains the satellite ID
    uint16_t AzimuthRiseSet; //!< -> SBF_CHANNELSTATUS_SI_AZRS_...
    uint16_t HealthStatus;   //!< -> SBF_CHANNELSTATUS_SI_HEALTH_...
    int8_t   Elevation;      //!< Elevation [deg]
    uint8_t  N2;             //!< Number of ChannelStateInfo blocks following this ChannelSatInfo block.
    uint8_t  RxChannel;      //!< Channel number
    uint8_t  Reserved;       //!< Reserved
};  // clang-format on

static_assert(sizeof(ChannelSatInfoRev0) == 12, "");

/**
 * @brief SBF ChannelStateInfo payload (rev 0)
 */
struct SFB_PACKED ChannelStateInfoRev0
{  // clang-format off
    uint8_t  Antenna;        //!< Anntena number (0 = main antenna)
    uint8_t  Reserved;       //!< Reserved
    uint16_t TrackingStatus; //!< -> SBF_CHANNELSTATE_TRKSTA_...
    uint16_t PVTStatus;      //!< -> SBF_CHANNELSTATE_PVTSTA_...
    uint16_t PVTInfo;        //!< Internal info
};  // clang-format on

static_assert(sizeof(ChannelStateInfoRev0) == 8, "");

// clang-format off
static constexpr std::size_t SBF_CHANNELSTATUS_REV0_MIN_SIZE = sizeof(ChannelStatusRev0) + SBF_HEAD_SIZE;  //!< Minimal size of ChannelStatus message
// clang-format on

// clang-format off
static constexpr uint16_t SBF_CHANNELSTATUS_SI_AZRS_AZIMUTH(const uint16_t azrs){ return azrs & 0x1ff; }                //!< Azimuth [deg]
static constexpr uint8_t  SBF_CHANNELSTATUS_SI_AZRS_RISESET(const uint16_t azrs){ return (azrs >> 14) & 0x0007; }       //!< Rise/set indicator
static constexpr uint8_t  SBF_CHANNELSTATUS_SI_AZRS_RISESET_SETTING             = 0;                                    //!< SV is setting
static constexpr uint8_t  SBF_CHANNELSTATUS_SI_AZRS_RISESET_RISING              = 1;                                    //!< SV is rising
static constexpr uint8_t  SBF_CHANNELSTATUS_SI_AZRS_RISESET_UNKNOWN             = 2;                                    //!< Elevation rate unknown
static constexpr uint8_t  SBF_CHANNELSTATUS_SI_HEALTH_SIG(const uint16_t health, const std::size_t sigIx) { return (health >> (sigIx * 2)) & 0x07; }  //!< Signal health
static constexpr uint8_t  SBF_CHANNELSTATUS_SI_HEALTH_SIG_UNKNOWN               = 0;                                    //!< Health unknown
static constexpr uint8_t  SBF_CHANNELSTATUS_SI_HEALTH_SIG_HEALTHY               = 1;                                    //!< Healthy
static constexpr uint8_t  SBF_CHANNELSTATUS_SI_HEALTH_SIG_UNHEALTHY             = 2;                                    //!< Unhealthy
static constexpr uint8_t  SBF_CHANNELSTATE_TRKSTA_SIG(const uint16_t status, const std::size_t sigIx) { return (status >> (sigIx * 2)) & 0x07; }  //!< Signal health
static constexpr uint8_t  SBF_CHANNELSTATE_TRKSTA_SIG_IDLE                      = 0;                                    //!< Idle
static constexpr uint8_t  SBF_CHANNELSTATE_TRKSTA_SIG_SEARCH                    = 1;                                    //!< Search
static constexpr uint8_t  SBF_CHANNELSTATE_TRKSTA_SIG_SYNC                      = 2;                                    //!< Sync
static constexpr uint8_t  SBF_CHANNELSTATE_TRKSTA_SIG_TRACKING                  = 3;                                    //!< Tracking
static constexpr uint8_t  SBF_CHANNELSTATE_PVTSTA_SIG(const uint16_t status, const std::size_t sigIx) { return (status >> (sigIx * 2)) & 0x07; }  //!< Signal health
static constexpr uint8_t  SBF_CHANNELSTATE_PVTSTA_SIG_UNUSED                    = 0;                                    //!< Not used
static constexpr uint8_t  SBF_CHANNELSTATE_PVTSTA_SIG_NOEPH                     = 1;                                    //!< No ephemeris
static constexpr uint8_t  SBF_CHANNELSTATE_PVTSTA_SIG_USED                      = 2;                                    //!< Used
static constexpr uint8_t  SBF_CHANNELSTATE_PVTSTA_SIG_REJECTED                  = 3;                                    //!< Rejected
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief SBF EndOfPvt/EndOfAtt/EndOfMeas payload (rev 0)
 */
struct SbfEndOfAnyRev0
{  // clang-format off
    uint32_t TOW;          //!< GPS time of week [ms]
    uint16_t WNc;          //!< GPS week number
    uint8_t  padding[2];   //!< Padding
};  // clang-format on

static_assert(sizeof(SbfEndOfAnyRev0) == 8, "");

// clang-format off
static constexpr std::size_t SBF_ENDOFANY_REV0_SIZE = sizeof(SbfEndOfAnyRev0) + SBF_HEAD_SIZE;  //!< Size of EndOfPvt/EndOfAtt/EndOfMeas message
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief SBF BBSamples payload head (rev 0)
 */
struct SbfBBSamplesHeadRev0
{  // clang-format off
    uint32_t TOW;          //!< GPS time of week [ms]
    uint16_t WNc;          //!< GPS week number
    uint16_t N;            //!< Number of samples
    uint8_t  info;         //!< -> SBF_BBSAMPLES_INFO_...
    uint8_t  reserved[3];  //!< Reserved
    uint32_t SampleFreq;   //!< Sampling frequency [Hz]          FIXME: looks like this is the span (bandwidth)
    uint32_t LOFreq;       //!< Local oscillator frequency [Hz]  FIXME: looks like this is the centre frequency
};  // clang-format on

/**
 * @brief SBF BBSamples payload sample (rev 0)
 */
struct SbfBBSamplesSampleRev0
{  // clang-format off
    int8_t   I;            //!< I component
    int8_t   Q;            //!< Q component
};  // clang-format on

static_assert(sizeof(SbfBBSamplesSampleRev0) == 2, "");

/**
 * @brief SBF BBSamples payload tail (rev 0)
 */
struct SbfBBSamplesTailRev0
{  // clang-format off
    float    TOWDelta;     //!< Time offset [s]
};  // clang-format on

static_assert(sizeof(SbfBBSamplesTailRev0) == 4, "");

// clang-format off
static constexpr std::size_t SBF_BBSAMPLES_REV0_MIN_SIZE = sizeof(SbfBBSamplesHeadRev0) + sizeof(SbfBBSamplesTailRev0) + SBF_HEAD_SIZE;  //!< Minimal size of BBSamples message
// clang-format on

// clang-format off
static constexpr uint8_t  SBF_BBSAMPLES_INFO_ANTID(const uint8_t type)          { return type & 0x07; }                 //!< Antenna ID
static constexpr uint8_t  SBF_BBSAMPLES_INFO_ANTID_MAIN                         =  0;                                   //!< Main antenna
static constexpr uint8_t  SBF_BBSAMPLES_INFO_ANTID_AUX1                         =  1;                                   //!< Aux1 antenna
static constexpr uint8_t  SBF_BBSAMPLES_INFO_ANTID_AUX2                         =  2;                                   //!< Aux2 antenna
// clang-format on

/* ****************************************************************************************************************** */
}  // namespace sbf
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_PARSER_SBF_HPP__
