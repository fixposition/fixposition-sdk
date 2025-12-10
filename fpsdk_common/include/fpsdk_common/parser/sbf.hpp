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
 * - mosaic-G5 Reference Guide, copyright 2000-2025 Septentrio NV/SA, part of HEXAGON
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

/* ****************************************************************************************************************** */
}  // namespace sbf
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_PARSER_SBF_HPP__
