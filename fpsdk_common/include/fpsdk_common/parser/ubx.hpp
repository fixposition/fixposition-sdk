// clang-format off
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
 * - u-blox ZED-F9P Interface Description (HPG 1.50) (https://www.u-blox.com/en/docs/UBXDOC-963802114-12815), copyright (c) 2024 u-blox AG
 * - u-blox ZED-F9T Interface Description (R02) (https://www.u-blox.com/en/docs/UBX-19003606), copyright (c) 2020 u-blox AG
 * - u-blox NEO-M9N Interface description (SPG 4.04) (https://www.u-blox.com/en/docs/UBX-19035940), copyright (c) 2020 u-blox AG
 * - u-blox ZED-F9R Interface description (HPS 1.20) (https://www.u-blox.com/en/docs/UBX-19056845), copyright (c) 2020 u-blox AG
 * - u-blox F9 HPS 1.21 Interface Description (ZEF-F9R) (https://www.u-blox.com/en/docs/UBX-21019746), copyright (c) 2021 u-blox AG
 * - u-center 20.01, copyright (c) 2020 u-blox AG
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Parser UBX routines and types
 *
 * @page FPSDK_COMMON_PARSER_UBX Parser UBX routines and types
 *
 * **API**: fpsdk_common/parser/ubx.hpp and fpsdk::common::parser::ubx
 *
 */
// clang-format on
#ifndef __FPSDK_COMMON_PARSER_UBX_HPP__
#define __FPSDK_COMMON_PARSER_UBX_HPP__

/* LIBC/STL */
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

/* EXTERNAL */

/* PACKAGE */

namespace fpsdk {
namespace common {
namespace parser {
/**
 * @brief Parser UBX routines and types
 */
namespace ubx {
/* ****************************************************************************************************************** */

// Doxygen is easily confused.. :-/
//! Message struct that must be packed
#ifndef _DOXYGEN_
#  define UBX_PACKED __attribute__((packed))
#else
#  define UBX_PACKED /* packed */
#endif

static constexpr std::size_t UBX_FRAME_SIZE = 8;  //!< Size (in bytes) of UBX frame
static constexpr std::size_t UBX_HEAD_SIZE = 6;   //!< Size of UBX frame header
static constexpr uint8_t UBX_SYNC_1 = 0xb5;       //!< UBX frame sync char 1
static constexpr uint8_t UBX_SYNC_2 = 0x62;       //!< UBX frame sync char 2

/**
 * @brief Get class ID from message
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. This is meant for use as a helper in other functions. Checks on the
 *       \c msg and its data should be carried out there.
 *
 * @returns the UBX class ID
 */
constexpr uint8_t UbxClsId(const uint8_t* msg)
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
 * @returns the UBX message ID
 */
constexpr uint8_t UbxMsgId(const uint8_t* msg)
{
    return (((uint8_t*)(msg))[3]);
}

/**
 * @brief Get UBX message name
 *
 * Generates a name (string) in the form "UBX-CLSID-MSGID", where CLSID and MSGID are suitable stringifications of the
 * class ID and message ID if known (for example, "UBX-NAV-PVT", respectively %02X formatted IDs if unknown (for
 * example, "UBX-NAV-F4" or "UBX-A0-B1").
 *
 * @param[out] name      String to write the name to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the UBX message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid UBX message.
 *
 * @returns true if message name was generated, false if \c name buffer was too small
 */
bool UbxGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Get UBX message info
 *
 * This stringifies the content of some UBX messages, for debugging.
 *
 * @param[out] info      String to write the info to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the UBX message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid UBX message.
 *
 * @returns true if message info was generated (even if info is empty), false if \c name buffer was too small
 */
bool UbxGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Make a UBX message
 *
 * @param[out]  msg           The message frame
 * @param[in]   cls_id        Class ID
 * @param[in]   msg_id        Message ID
 * @param[in]   payload       The message payload (up to MAX_UBX_SIZE - UBX_FRAME_SIZE bytes, can be empty)
 *
 * @returns true if the message was successfully constructed (\c msg now contains the message),
 *          false if failed contructing the message (payload too large)
 */
bool UbxMakeMessage(
    std::vector<uint8_t>& msg, const uint8_t cls_id, const uint8_t msg_id, const std::vector<uint8_t>& payload);

/**
 * @brief Make a UBX message
 *
 * @param[out]  msg           The message frame
 * @param[in]   cls_id        Class ID
 * @param[in]   msg_id        Message ID
 * @param[in]   payload       The message payload (up to MAX_UBX_SIZE - UBX_FRAME_SIZE bytes, can be NULL)
 * @param[in]   payload_size  Size of the payload (up to MAX_UBX_SIZE - UBX_FRAME_SIZE, can be 0, even if payload is not
 *                            NULL)
 *
 * @returns true if the message was successfully constructed (\c msg now contains the message),
 *          false if failed contructing the message (payload too large, bad arguments)
 */
bool UbxMakeMessage(std::vector<uint8_t>& msg, const uint8_t cls_id, const uint8_t msg_id, const uint8_t* payload,
    const std::size_t payload_size);

/**
 * @brief Stringify UBX-RXM-SFRBX, for debugging
 *
 * @param[out]  info      String to format
 * @param[in]   size      Maximum size of string (incl. nul termination)
 * @param[in]   msg       The UBX-RXM-SFRBX message
 * @param[in]   msg_size  Size of the UBX-RXM-SFRBX message
 *
 * @returns the length of the string generated (excl. nul termination)
 */
std::size_t UbxRxmSfrbxInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Stringify UBX-MON-VER message (software version and module name)
 *
 * @param[out]  str       String to format
 * @param[in]   size      Maximum size of string (incl. nul termination)
 * @param[in]   msg       The UBX-MON-VER message
 * @param[in]   msg_size  Size of the UBX-MON-VER message
 *
 * @returns the length of the string generated (excl. nul termination)
 */
std::size_t UbxMonVerToVerStr(char* str, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @name UBX classes and messages (names and IDs)
 *
 * @{
 */
// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_UBX_CLASSES}
static constexpr uint8_t     UBX_ACK_CLSID                  = 0x05;                     //!< UBX-ACK class ID
static constexpr const char* UBX_ACK_STRID                  = "UBX-ACK";                //!< UBX-ACK class name
static constexpr uint8_t     UBX_CFG_CLSID                  = 0x06;                     //!< UBX-CFG class ID
static constexpr const char* UBX_CFG_STRID                  = "UBX-CFG";                //!< UBX-CFG class name
static constexpr uint8_t     UBX_ESF_CLSID                  = 0x10;                     //!< UBX-ESF class ID
static constexpr const char* UBX_ESF_STRID                  = "UBX-ESF";                //!< UBX-ESF class name
static constexpr uint8_t     UBX_INF_CLSID                  = 0x04;                     //!< UBX-INF class ID
static constexpr const char* UBX_INF_STRID                  = "UBX-INF";                //!< UBX-INF class name
static constexpr uint8_t     UBX_LOG_CLSID                  = 0x21;                     //!< UBX-LOG class ID
static constexpr const char* UBX_LOG_STRID                  = "UBX-LOG";                //!< UBX-LOG class name
static constexpr uint8_t     UBX_MGA_CLSID                  = 0x13;                     //!< UBX-MGA class ID
static constexpr const char* UBX_MGA_STRID                  = "UBX-MGA";                //!< UBX-MGA class name
static constexpr uint8_t     UBX_MON_CLSID                  = 0x0a;                     //!< UBX-MON class ID
static constexpr const char* UBX_MON_STRID                  = "UBX-MON";                //!< UBX-MON class name
static constexpr uint8_t     UBX_NAV_CLSID                  = 0x01;                     //!< UBX-NAV class ID
static constexpr const char* UBX_NAV_STRID                  = "UBX-NAV";                //!< UBX-NAV class name
static constexpr uint8_t     UBX_NAV2_CLSID                 = 0x29;                     //!< UBX-NAV2 class ID
static constexpr const char* UBX_NAV2_STRID                 = "UBX-NAV2";               //!< UBX-NAV2 class name
static constexpr uint8_t     UBX_RXM_CLSID                  = 0x02;                     //!< UBX-RXM class ID
static constexpr const char* UBX_RXM_STRID                  = "UBX-RXM";                //!< UBX-RXM class name
static constexpr uint8_t     UBX_SEC_CLSID                  = 0x27;                     //!< UBX-SEC class ID
static constexpr const char* UBX_SEC_STRID                  = "UBX-SEC";                //!< UBX-SEC class name
static constexpr uint8_t     UBX_TIM_CLSID                  = 0x0d;                     //!< UBX-TIM class ID
static constexpr const char* UBX_TIM_STRID                  = "UBX-TIM";                //!< UBX-TIM class name
static constexpr uint8_t     UBX_UPD_CLSID                  = 0x09;                     //!< UBX-UPD class ID
static constexpr const char* UBX_UPD_STRID                  = "UBX-UPD";                //!< UBX-UPD class name
static constexpr uint8_t     UBX_NMEA_CLSID                 = 0xf0;                     //!< UBX-NMEA class ID
static constexpr const char* UBX_NMEA_STRID                 = "UBX-NMEA";               //!< UBX-NMEA class name
static constexpr uint8_t     UBX_RTCM3_CLSID                = 0xf5;                     //!< UBX-RTCM3 class ID
static constexpr const char* UBX_RTCM3_STRID                = "UBX-RTCM3";              //!< UBX-RTCM3 class name
// @fp_codegen_end{FPSDK_COMMON_PARSER_UBX_CLASSES}
// clang-format on

// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_UBX_MESSAGES}
static constexpr uint8_t     UBX_ACK_ACK_MSGID              = 0x01;                     //!< UBX-ACK-ACK message ID
static constexpr const char* UBX_ACK_ACK_STRID              = "UBX-ACK-ACK";            //!< UBX-ACK-ACK message name
static constexpr uint8_t     UBX_ACK_NAK_MSGID              = 0x00;                     //!< UBX-ACK-NAK message ID
static constexpr const char* UBX_ACK_NAK_STRID              = "UBX-ACK-NAK";            //!< UBX-ACK-NAK message name
static constexpr uint8_t     UBX_CFG_CFG_MSGID              = 0x09;                     //!< UBX-CFG-CFG message ID
static constexpr const char* UBX_CFG_CFG_STRID              = "UBX-CFG-CFG";            //!< UBX-CFG-CFG message name
static constexpr uint8_t     UBX_CFG_PWR_MSGID              = 0x57;                     //!< UBX-CFG-PWR message ID
static constexpr const char* UBX_CFG_PWR_STRID              = "UBX-CFG-PWR";            //!< UBX-CFG-PWR message name
static constexpr uint8_t     UBX_CFG_RST_MSGID              = 0x04;                     //!< UBX-CFG-RST message ID
static constexpr const char* UBX_CFG_RST_STRID              = "UBX-CFG-RST";            //!< UBX-CFG-RST message name
static constexpr uint8_t     UBX_CFG_VALDEL_MSGID           = 0x8c;                     //!< UBX-CFG-VALDEL message ID
static constexpr const char* UBX_CFG_VALDEL_STRID           = "UBX-CFG-VALDEL";         //!< UBX-CFG-VALDEL message name
static constexpr uint8_t     UBX_CFG_VALGET_MSGID           = 0x8b;                     //!< UBX-CFG-VALGET message ID
static constexpr const char* UBX_CFG_VALGET_STRID           = "UBX-CFG-VALGET";         //!< UBX-CFG-VALGET message name
static constexpr uint8_t     UBX_CFG_VALSET_MSGID           = 0x8a;                     //!< UBX-CFG-VALSET message ID
static constexpr const char* UBX_CFG_VALSET_STRID           = "UBX-CFG-VALSET";         //!< UBX-CFG-VALSET message name
static constexpr uint8_t     UBX_ESF_ALG_MSGID              = 0x14;                     //!< UBX-ESF-ALG message ID
static constexpr const char* UBX_ESF_ALG_STRID              = "UBX-ESF-ALG";            //!< UBX-ESF-ALG message name
static constexpr uint8_t     UBX_ESF_INS_MSGID              = 0x15;                     //!< UBX-ESF-INS message ID
static constexpr const char* UBX_ESF_INS_STRID              = "UBX-ESF-INS";            //!< UBX-ESF-INS message name
static constexpr uint8_t     UBX_ESF_MEAS_MSGID             = 0x02;                     //!< UBX-ESF-MEAS message ID
static constexpr const char* UBX_ESF_MEAS_STRID             = "UBX-ESF-MEAS";           //!< UBX-ESF-MEAS message name
static constexpr uint8_t     UBX_ESF_RAW_MSGID              = 0x03;                     //!< UBX-ESF-RAW message ID
static constexpr const char* UBX_ESF_RAW_STRID              = "UBX-ESF-RAW";            //!< UBX-ESF-RAW message name
static constexpr uint8_t     UBX_ESF_STATUS_MSGID           = 0x10;                     //!< UBX-ESF-STATUS message ID
static constexpr const char* UBX_ESF_STATUS_STRID           = "UBX-ESF-STATUS";         //!< UBX-ESF-STATUS message name
static constexpr uint8_t     UBX_INF_DEBUG_MSGID            = 0x04;                     //!< UBX-INF-DEBUG message ID
static constexpr const char* UBX_INF_DEBUG_STRID            = "UBX-INF-DEBUG";          //!< UBX-INF-DEBUG message name
static constexpr uint8_t     UBX_INF_ERROR_MSGID            = 0x00;                     //!< UBX-INF-ERROR message ID
static constexpr const char* UBX_INF_ERROR_STRID            = "UBX-INF-ERROR";          //!< UBX-INF-ERROR message name
static constexpr uint8_t     UBX_INF_NOTICE_MSGID           = 0x02;                     //!< UBX-INF-NOTICE message ID
static constexpr const char* UBX_INF_NOTICE_STRID           = "UBX-INF-NOTICE";         //!< UBX-INF-NOTICE message name
static constexpr uint8_t     UBX_INF_TEST_MSGID             = 0x03;                     //!< UBX-INF-TEST message ID
static constexpr const char* UBX_INF_TEST_STRID             = "UBX-INF-TEST";           //!< UBX-INF-TEST message name
static constexpr uint8_t     UBX_INF_WARNING_MSGID          = 0x01;                     //!< UBX-INF-WARNING message ID
static constexpr const char* UBX_INF_WARNING_STRID          = "UBX-INF-WARNING";        //!< UBX-INF-WARNING message name
static constexpr uint8_t     UBX_LOG_CREATE_MSGID           = 0x07;                     //!< UBX-LOG-CREATE message ID
static constexpr const char* UBX_LOG_CREATE_STRID           = "UBX-LOG-CREATE";         //!< UBX-LOG-CREATE message name
static constexpr uint8_t     UBX_LOG_ERASE_MSGID            = 0x03;                     //!< UBX-LOG-ERASE message ID
static constexpr const char* UBX_LOG_ERASE_STRID            = "UBX-LOG-ERASE";          //!< UBX-LOG-ERASE message name
static constexpr uint8_t     UBX_LOG_FINDTIME_MSGID         = 0x0e;                     //!< UBX-LOG-FINDTIME message ID
static constexpr const char* UBX_LOG_FINDTIME_STRID         = "UBX-LOG-FINDTIME";       //!< UBX-LOG-FINDTIME message name
static constexpr uint8_t     UBX_LOG_INFO_MSGID             = 0x08;                     //!< UBX-LOG-INFO message ID
static constexpr const char* UBX_LOG_INFO_STRID             = "UBX-LOG-INFO";           //!< UBX-LOG-INFO message name
static constexpr uint8_t     UBX_LOG_RETR_MSGID             = 0x09;                     //!< UBX-LOG-RETR message ID
static constexpr const char* UBX_LOG_RETR_STRID             = "UBX-LOG-RETR";           //!< UBX-LOG-RETR message name
static constexpr uint8_t     UBX_LOG_RETRPOS_MSGID          = 0x0b;                     //!< UBX-LOG-RETRPOS message ID
static constexpr const char* UBX_LOG_RETRPOS_STRID          = "UBX-LOG-RETRPOS";        //!< UBX-LOG-RETRPOS message name
static constexpr uint8_t     UBX_LOG_RETRPOSX_MSGID         = 0x0f;                     //!< UBX-LOG-RETRPOSX message ID
static constexpr const char* UBX_LOG_RETRPOSX_STRID         = "UBX-LOG-RETRPOSX";       //!< UBX-LOG-RETRPOSX message name
static constexpr uint8_t     UBX_LOG_RETRSTR_MSGID          = 0x0d;                     //!< UBX-LOG-RETRSTR message ID
static constexpr const char* UBX_LOG_RETRSTR_STRID          = "UBX-LOG-RETRSTR";        //!< UBX-LOG-RETRSTR message name
static constexpr uint8_t     UBX_LOG_STR_MSGID              = 0x04;                     //!< UBX-LOG-STR message ID
static constexpr const char* UBX_LOG_STR_STRID              = "UBX-LOG-STR";            //!< UBX-LOG-STR message name
static constexpr uint8_t     UBX_MGA_ACK_MSGID              = 0x60;                     //!< UBX-MGA-ACK message ID
static constexpr const char* UBX_MGA_ACK_STRID              = "UBX-MGA-ACK";            //!< UBX-MGA-ACK message name
static constexpr uint8_t     UBX_MGA_BDS_MSGID              = 0x03;                     //!< UBX-MGA-BDS message ID
static constexpr const char* UBX_MGA_BDS_STRID              = "UBX-MGA-BDS";            //!< UBX-MGA-BDS message name
static constexpr uint8_t     UBX_MGA_DBD_MSGID              = 0x80;                     //!< UBX-MGA-DBD message ID
static constexpr const char* UBX_MGA_DBD_STRID              = "UBX-MGA-DBD";            //!< UBX-MGA-DBD message name
static constexpr uint8_t     UBX_MGA_GAL_MSGID              = 0x02;                     //!< UBX-MGA-GAL message ID
static constexpr const char* UBX_MGA_GAL_STRID              = "UBX-MGA-GAL";            //!< UBX-MGA-GAL message name
static constexpr uint8_t     UBX_MGA_GLO_MSGID              = 0x06;                     //!< UBX-MGA-GLO message ID
static constexpr const char* UBX_MGA_GLO_STRID              = "UBX-MGA-GLO";            //!< UBX-MGA-GLO message name
static constexpr uint8_t     UBX_MGA_GPS_MSGID              = 0x00;                     //!< UBX-MGA-GPS message ID
static constexpr const char* UBX_MGA_GPS_STRID              = "UBX-MGA-GPS";            //!< UBX-MGA-GPS message name
static constexpr uint8_t     UBX_MGA_INI_MSGID              = 0x40;                     //!< UBX-MGA-INI message ID
static constexpr const char* UBX_MGA_INI_STRID              = "UBX-MGA-INI";            //!< UBX-MGA-INI message name
static constexpr uint8_t     UBX_MGA_QZSS_MSGID             = 0x05;                     //!< UBX-MGA-QZSS message ID
static constexpr const char* UBX_MGA_QZSS_STRID             = "UBX-MGA-QZSS";           //!< UBX-MGA-QZSS message name
static constexpr uint8_t     UBX_MON_COMMS_MSGID            = 0x36;                     //!< UBX-MON-COMMS message ID
static constexpr const char* UBX_MON_COMMS_STRID            = "UBX-MON-COMMS";          //!< UBX-MON-COMMS message name
static constexpr uint8_t     UBX_MON_GNSS_MSGID             = 0x28;                     //!< UBX-MON-GNSS message ID
static constexpr const char* UBX_MON_GNSS_STRID             = "UBX-MON-GNSS";           //!< UBX-MON-GNSS message name
static constexpr uint8_t     UBX_MON_HW_MSGID               = 0x09;                     //!< UBX-MON-HW message ID
static constexpr const char* UBX_MON_HW_STRID               = "UBX-MON-HW";             //!< UBX-MON-HW message name
static constexpr uint8_t     UBX_MON_HW2_MSGID              = 0x0b;                     //!< UBX-MON-HW2 message ID
static constexpr const char* UBX_MON_HW2_STRID              = "UBX-MON-HW2";            //!< UBX-MON-HW2 message name
static constexpr uint8_t     UBX_MON_HW3_MSGID              = 0x37;                     //!< UBX-MON-HW3 message ID
static constexpr const char* UBX_MON_HW3_STRID              = "UBX-MON-HW3";            //!< UBX-MON-HW3 message name
static constexpr uint8_t     UBX_MON_IO_MSGID               = 0x02;                     //!< UBX-MON-IO message ID
static constexpr const char* UBX_MON_IO_STRID               = "UBX-MON-IO";             //!< UBX-MON-IO message name
static constexpr uint8_t     UBX_MON_MSGPP_MSGID            = 0x06;                     //!< UBX-MON-MSGPP message ID
static constexpr const char* UBX_MON_MSGPP_STRID            = "UBX-MON-MSGPP";          //!< UBX-MON-MSGPP message name
static constexpr uint8_t     UBX_MON_PATCH_MSGID            = 0x27;                     //!< UBX-MON-PATCH message ID
static constexpr const char* UBX_MON_PATCH_STRID            = "UBX-MON-PATCH";          //!< UBX-MON-PATCH message name
static constexpr uint8_t     UBX_MON_RF_MSGID               = 0x38;                     //!< UBX-MON-RF message ID
static constexpr const char* UBX_MON_RF_STRID               = "UBX-MON-RF";             //!< UBX-MON-RF message name
static constexpr uint8_t     UBX_MON_RXBUF_MSGID            = 0x07;                     //!< UBX-MON-RXBUF message ID
static constexpr const char* UBX_MON_RXBUF_STRID            = "UBX-MON-RXBUF";          //!< UBX-MON-RXBUF message name
static constexpr uint8_t     UBX_MON_RXR_MSGID              = 0x21;                     //!< UBX-MON-RXR message ID
static constexpr const char* UBX_MON_RXR_STRID              = "UBX-MON-RXR";            //!< UBX-MON-RXR message name
static constexpr uint8_t     UBX_MON_SPAN_MSGID             = 0x31;                     //!< UBX-MON-SPAN message ID
static constexpr const char* UBX_MON_SPAN_STRID             = "UBX-MON-SPAN";           //!< UBX-MON-SPAN message name
static constexpr uint8_t     UBX_MON_SYS_MSGID              = 0x39;                     //!< UBX-MON-SYS message ID
static constexpr const char* UBX_MON_SYS_STRID              = "UBX-MON-SYS";            //!< UBX-MON-SYS message name
static constexpr uint8_t     UBX_MON_TEMP_MSGID             = 0x0e;                     //!< UBX-MON-TEMP message ID
static constexpr const char* UBX_MON_TEMP_STRID             = "UBX-MON-TEMP";           //!< UBX-MON-TEMP message name
static constexpr uint8_t     UBX_MON_TXBUF_MSGID            = 0x08;                     //!< UBX-MON-TXBUF message ID
static constexpr const char* UBX_MON_TXBUF_STRID            = "UBX-MON-TXBUF";          //!< UBX-MON-TXBUF message name
static constexpr uint8_t     UBX_MON_VER_MSGID              = 0x04;                     //!< UBX-MON-VER message ID
static constexpr const char* UBX_MON_VER_STRID              = "UBX-MON-VER";            //!< UBX-MON-VER message name
static constexpr uint8_t     UBX_NAV_ATT_MSGID              = 0x05;                     //!< UBX-NAV-ATT message ID
static constexpr const char* UBX_NAV_ATT_STRID              = "UBX-NAV-ATT";            //!< UBX-NAV-ATT message name
static constexpr uint8_t     UBX_NAV_CLOCK_MSGID            = 0x22;                     //!< UBX-NAV-CLOCK message ID
static constexpr const char* UBX_NAV_CLOCK_STRID            = "UBX-NAV-CLOCK";          //!< UBX-NAV-CLOCK message name
static constexpr uint8_t     UBX_NAV_COV_MSGID              = 0x36;                     //!< UBX-NAV-COV message ID
static constexpr const char* UBX_NAV_COV_STRID              = "UBX-NAV-COV";            //!< UBX-NAV-COV message name
static constexpr uint8_t     UBX_NAV_DOP_MSGID              = 0x04;                     //!< UBX-NAV-DOP message ID
static constexpr const char* UBX_NAV_DOP_STRID              = "UBX-NAV-DOP";            //!< UBX-NAV-DOP message name
static constexpr uint8_t     UBX_NAV_EELL_MSGID             = 0x3d;                     //!< UBX-NAV-EELL message ID
static constexpr const char* UBX_NAV_EELL_STRID             = "UBX-NAV-EELL";           //!< UBX-NAV-EELL message name
static constexpr uint8_t     UBX_NAV_EOE_MSGID              = 0x61;                     //!< UBX-NAV-EOE message ID
static constexpr const char* UBX_NAV_EOE_STRID              = "UBX-NAV-EOE";            //!< UBX-NAV-EOE message name
static constexpr uint8_t     UBX_NAV_GEOFENCE_MSGID         = 0x39;                     //!< UBX-NAV-GEOFENCE message ID
static constexpr const char* UBX_NAV_GEOFENCE_STRID         = "UBX-NAV-GEOFENCE";       //!< UBX-NAV-GEOFENCE message name
static constexpr uint8_t     UBX_NAV_HPPOSECEF_MSGID        = 0x13;                     //!< UBX-NAV-HPPOSECEF message ID
static constexpr const char* UBX_NAV_HPPOSECEF_STRID        = "UBX-NAV-HPPOSECEF";      //!< UBX-NAV-HPPOSECEF message name
static constexpr uint8_t     UBX_NAV_HPPOSLLH_MSGID         = 0x14;                     //!< UBX-NAV-HPPOSLLH message ID
static constexpr const char* UBX_NAV_HPPOSLLH_STRID         = "UBX-NAV-HPPOSLLH";       //!< UBX-NAV-HPPOSLLH message name
static constexpr uint8_t     UBX_NAV_ODO_MSGID              = 0x09;                     //!< UBX-NAV-ODO message ID
static constexpr const char* UBX_NAV_ODO_STRID              = "UBX-NAV-ODO";            //!< UBX-NAV-ODO message name
static constexpr uint8_t     UBX_NAV_ORB_MSGID              = 0x34;                     //!< UBX-NAV-ORB message ID
static constexpr const char* UBX_NAV_ORB_STRID              = "UBX-NAV-ORB";            //!< UBX-NAV-ORB message name
static constexpr uint8_t     UBX_NAV_PL_MSGID               = 0x62;                     //!< UBX-NAV-PL message ID
static constexpr const char* UBX_NAV_PL_STRID               = "UBX-NAV-PL";             //!< UBX-NAV-PL message name
static constexpr uint8_t     UBX_NAV_POSECEF_MSGID          = 0x01;                     //!< UBX-NAV-POSECEF message ID
static constexpr const char* UBX_NAV_POSECEF_STRID          = "UBX-NAV-POSECEF";        //!< UBX-NAV-POSECEF message name
static constexpr uint8_t     UBX_NAV_POSLLH_MSGID           = 0x02;                     //!< UBX-NAV-POSLLH message ID
static constexpr const char* UBX_NAV_POSLLH_STRID           = "UBX-NAV-POSLLH";         //!< UBX-NAV-POSLLH message name
static constexpr uint8_t     UBX_NAV_PVAT_MSGID             = 0x17;                     //!< UBX-NAV-PVAT message ID
static constexpr const char* UBX_NAV_PVAT_STRID             = "UBX-NAV-PVAT";           //!< UBX-NAV-PVAT message name
static constexpr uint8_t     UBX_NAV_PVT_MSGID              = 0x07;                     //!< UBX-NAV-PVT message ID
static constexpr const char* UBX_NAV_PVT_STRID              = "UBX-NAV-PVT";            //!< UBX-NAV-PVT message name
static constexpr uint8_t     UBX_NAV_RELPOSNED_MSGID        = 0x3c;                     //!< UBX-NAV-RELPOSNED message ID
static constexpr const char* UBX_NAV_RELPOSNED_STRID        = "UBX-NAV-RELPOSNED";      //!< UBX-NAV-RELPOSNED message name
static constexpr uint8_t     UBX_NAV_RESETODO_MSGID         = 0x10;                     //!< UBX-NAV-RESETODO message ID
static constexpr const char* UBX_NAV_RESETODO_STRID         = "UBX-NAV-RESETODO";       //!< UBX-NAV-RESETODO message name
static constexpr uint8_t     UBX_NAV_SAT_MSGID              = 0x35;                     //!< UBX-NAV-SAT message ID
static constexpr const char* UBX_NAV_SAT_STRID              = "UBX-NAV-SAT";            //!< UBX-NAV-SAT message name
static constexpr uint8_t     UBX_NAV_SBAS_MSGID             = 0x32;                     //!< UBX-NAV-SBAS message ID
static constexpr const char* UBX_NAV_SBAS_STRID             = "UBX-NAV-SBAS";           //!< UBX-NAV-SBAS message name
static constexpr uint8_t     UBX_NAV_SIG_MSGID              = 0x43;                     //!< UBX-NAV-SIG message ID
static constexpr const char* UBX_NAV_SIG_STRID              = "UBX-NAV-SIG";            //!< UBX-NAV-SIG message name
static constexpr uint8_t     UBX_NAV_SLAS_MSGID             = 0x42;                     //!< UBX-NAV-SLAS message ID
static constexpr const char* UBX_NAV_SLAS_STRID             = "UBX-NAV-SLAS";           //!< UBX-NAV-SLAS message name
static constexpr uint8_t     UBX_NAV_SOL_MSGID              = 0x01;                     //!< UBX-NAV-SOL message ID
static constexpr const char* UBX_NAV_SOL_STRID              = "UBX-NAV-SOL";            //!< UBX-NAV-SOL message name
static constexpr uint8_t     UBX_NAV_STATUS_MSGID           = 0x03;                     //!< UBX-NAV-STATUS message ID
static constexpr const char* UBX_NAV_STATUS_STRID           = "UBX-NAV-STATUS";         //!< UBX-NAV-STATUS message name
static constexpr uint8_t     UBX_NAV_SVIN_MSGID             = 0x3b;                     //!< UBX-NAV-SVIN message ID
static constexpr const char* UBX_NAV_SVIN_STRID             = "UBX-NAV-SVIN";           //!< UBX-NAV-SVIN message name
static constexpr uint8_t     UBX_NAV_TIMEBDS_MSGID          = 0x24;                     //!< UBX-NAV-TIMEBDS message ID
static constexpr const char* UBX_NAV_TIMEBDS_STRID          = "UBX-NAV-TIMEBDS";        //!< UBX-NAV-TIMEBDS message name
static constexpr uint8_t     UBX_NAV_TIMEGAL_MSGID          = 0x25;                     //!< UBX-NAV-TIMEGAL message ID
static constexpr const char* UBX_NAV_TIMEGAL_STRID          = "UBX-NAV-TIMEGAL";        //!< UBX-NAV-TIMEGAL message name
static constexpr uint8_t     UBX_NAV_TIMEGLO_MSGID          = 0x23;                     //!< UBX-NAV-TIMEGLO message ID
static constexpr const char* UBX_NAV_TIMEGLO_STRID          = "UBX-NAV-TIMEGLO";        //!< UBX-NAV-TIMEGLO message name
static constexpr uint8_t     UBX_NAV_TIMEGPS_MSGID          = 0x20;                     //!< UBX-NAV-TIMEGPS message ID
static constexpr const char* UBX_NAV_TIMEGPS_STRID          = "UBX-NAV-TIMEGPS";        //!< UBX-NAV-TIMEGPS message name
static constexpr uint8_t     UBX_NAV_TIMELS_MSGID           = 0x26;                     //!< UBX-NAV-TIMELS message ID
static constexpr const char* UBX_NAV_TIMELS_STRID           = "UBX-NAV-TIMELS";         //!< UBX-NAV-TIMELS message name
static constexpr uint8_t     UBX_NAV_TIMEQZSS_MSGID         = 0x27;                     //!< UBX-NAV-TIMEQZSS message ID
static constexpr const char* UBX_NAV_TIMEQZSS_STRID         = "UBX-NAV-TIMEQZSS";       //!< UBX-NAV-TIMEQZSS message name
static constexpr uint8_t     UBX_NAV_TIMENAVIC_MSGID        = 0x63;                     //!< UBX-NAV-TIMENAVIC message ID
static constexpr const char* UBX_NAV_TIMENAVIC_STRID        = "UBX-NAV-TIMENAVIC";      //!< UBX-NAV-TIMENAVIC message name
static constexpr uint8_t     UBX_NAV_TIMETRUSTED_MSGID      = 0x64;                     //!< UBX-NAV-TIMETRUSTED message ID
static constexpr const char* UBX_NAV_TIMETRUSTED_STRID      = "UBX-NAV-TIMETRUSTED";    //!< UBX-NAV-TIMETRUSTED message name
static constexpr uint8_t     UBX_NAV_TIMEUTC_MSGID          = 0x21;                     //!< UBX-NAV-TIMEUTC message ID
static constexpr const char* UBX_NAV_TIMEUTC_STRID          = "UBX-NAV-TIMEUTC";        //!< UBX-NAV-TIMEUTC message name
static constexpr uint8_t     UBX_NAV_VELECEF_MSGID          = 0x11;                     //!< UBX-NAV-VELECEF message ID
static constexpr const char* UBX_NAV_VELECEF_STRID          = "UBX-NAV-VELECEF";        //!< UBX-NAV-VELECEF message name
static constexpr uint8_t     UBX_NAV_VELNED_MSGID           = 0x12;                     //!< UBX-NAV-VELNED message ID
static constexpr const char* UBX_NAV_VELNED_STRID           = "UBX-NAV-VELNED";         //!< UBX-NAV-VELNED message name
static constexpr uint8_t     UBX_NAV2_CLOCK_MSGID           = UBX_NAV_CLOCK_MSGID;      //!< UBX-NAV2-CLOCK message ID
static constexpr const char* UBX_NAV2_CLOCK_STRID           = "UBX-NAV2-CLOCK";         //!< UBX-NAV2-CLOCK message name
static constexpr uint8_t     UBX_NAV2_COV_MSGID             = UBX_NAV_COV_MSGID;        //!< UBX-NAV2-COV message ID
static constexpr const char* UBX_NAV2_COV_STRID             = "UBX-NAV2-COV";           //!< UBX-NAV2-COV message name
static constexpr uint8_t     UBX_NAV2_DOP_MSGID             = UBX_NAV_DOP_MSGID;        //!< UBX-NAV2-DOP message ID
static constexpr const char* UBX_NAV2_DOP_STRID             = "UBX-NAV2-DOP";           //!< UBX-NAV2-DOP message name
static constexpr uint8_t     UBX_NAV2_EOE_MSGID             = UBX_NAV_EOE_MSGID;        //!< UBX-NAV2-EOE message ID
static constexpr const char* UBX_NAV2_EOE_STRID             = "UBX-NAV2-EOE";           //!< UBX-NAV2-EOE message name
static constexpr uint8_t     UBX_NAV2_ODO_MSGID             = UBX_NAV_ODO_MSGID;        //!< UBX-NAV2-ODO message ID
static constexpr const char* UBX_NAV2_ODO_STRID             = "UBX-NAV2-ODO";           //!< UBX-NAV2-ODO message name
static constexpr uint8_t     UBX_NAV2_POSECEF_MSGID         = UBX_NAV_POSECEF_MSGID;    //!< UBX-NAV2-POSECEF message ID
static constexpr const char* UBX_NAV2_POSECEF_STRID         = "UBX-NAV2-POSECEF";       //!< UBX-NAV2-POSECEF message name
static constexpr uint8_t     UBX_NAV2_POSLLH_MSGID          = UBX_NAV_POSLLH_MSGID;     //!< UBX-NAV2-POSLLH message ID
static constexpr const char* UBX_NAV2_POSLLH_STRID          = "UBX-NAV2-POSLLH";        //!< UBX-NAV2-POSLLH message name
static constexpr uint8_t     UBX_NAV2_PVT_MSGID             = UBX_NAV_PVT_MSGID;        //!< UBX-NAV2-PVT message ID
static constexpr const char* UBX_NAV2_PVT_STRID             = "UBX-NAV2-PVT";           //!< UBX-NAV2-PVT message name
static constexpr uint8_t     UBX_NAV2_SAT_MSGID             = UBX_NAV_SAT_MSGID;        //!< UBX-NAV2-SAT message ID
static constexpr const char* UBX_NAV2_SAT_STRID             = "UBX-NAV2-SAT";           //!< UBX-NAV2-SAT message name
static constexpr uint8_t     UBX_NAV2_SBAS_MSGID            = UBX_NAV_SBAS_MSGID;       //!< UBX-NAV2-SBAS message ID
static constexpr const char* UBX_NAV2_SBAS_STRID            = "UBX-NAV2-SBAS";          //!< UBX-NAV2-SBAS message name
static constexpr uint8_t     UBX_NAV2_SIG_MSGID             = UBX_NAV_SIG_MSGID;        //!< UBX-NAV2-SIG message ID
static constexpr const char* UBX_NAV2_SIG_STRID             = "UBX-NAV2-SIG";           //!< UBX-NAV2-SIG message name
static constexpr uint8_t     UBX_NAV2_SLAS_MSGID            = UBX_NAV_SLAS_MSGID;       //!< UBX-NAV2-SLAS message ID
static constexpr const char* UBX_NAV2_SLAS_STRID            = "UBX-NAV2-SLAS";          //!< UBX-NAV2-SLAS message name
static constexpr uint8_t     UBX_NAV2_STATUS_MSGID          = UBX_NAV_STATUS_MSGID;     //!< UBX-NAV2-STATUS message ID
static constexpr const char* UBX_NAV2_STATUS_STRID          = "UBX-NAV2-STATUS";        //!< UBX-NAV2-STATUS message name
static constexpr uint8_t     UBX_NAV2_SVIN_MSGID            = UBX_NAV_SVIN_MSGID;       //!< UBX-NAV2-SVIN message ID
static constexpr const char* UBX_NAV2_SVIN_STRID            = "UBX-NAV2-SVIN";          //!< UBX-NAV2-SVIN message name
static constexpr uint8_t     UBX_NAV2_TIMEBDS_MSGID         = UBX_NAV_TIMEBDS_MSGID;    //!< UBX-NAV2-TIMEBDS message ID
static constexpr const char* UBX_NAV2_TIMEBDS_STRID         = "UBX-NAV2-TIMEBDS";       //!< UBX-NAV2-TIMEBDS message name
static constexpr uint8_t     UBX_NAV2_TIMEGAL_MSGID         = UBX_NAV_TIMEGAL_MSGID;    //!< UBX-NAV2-TIMEGAL message ID
static constexpr const char* UBX_NAV2_TIMEGAL_STRID         = "UBX-NAV2-TIMEGAL";       //!< UBX-NAV2-TIMEGAL message name
static constexpr uint8_t     UBX_NAV2_TIMEGLO_MSGID         = UBX_NAV_TIMEGLO_MSGID;    //!< UBX-NAV2-TIMEGLO message ID
static constexpr const char* UBX_NAV2_TIMEGLO_STRID         = "UBX-NAV2-TIMEGLO";       //!< UBX-NAV2-TIMEGLO message name
static constexpr uint8_t     UBX_NAV2_TIMEGPS_MSGID         = UBX_NAV_TIMEGPS_MSGID;    //!< UBX-NAV2-TIMEGPS message ID
static constexpr const char* UBX_NAV2_TIMEGPS_STRID         = "UBX-NAV2-TIMEGPS";       //!< UBX-NAV2-TIMEGPS message name
static constexpr uint8_t     UBX_NAV2_TIMELS_MSGID          = UBX_NAV_TIMELS_MSGID;     //!< UBX-NAV2-TIMELS message ID
static constexpr const char* UBX_NAV2_TIMELS_STRID          = "UBX-NAV2-TIMELS";        //!< UBX-NAV2-TIMELS message name
static constexpr uint8_t     UBX_NAV2_TIMEQZSS_MSGID        = UBX_NAV_TIMEQZSS_MSGID;   //!< UBX-NAV2-TIMEQZSS message ID
static constexpr const char* UBX_NAV2_TIMEQZSS_STRID        = "UBX-NAV2-TIMEQZSS";      //!< UBX-NAV2-TIMEQZSS message name
static constexpr uint8_t     UBX_NAV2_TIMENAVIC_MSGID       = UBX_NAV_TIMENAVIC_MSGID;  //!< UBX-NAV2-TIMENAVIC message ID
static constexpr const char* UBX_NAV2_TIMENAVIC_STRID       = "UBX-NAV2-TIMENAVIC";     //!< UBX-NAV2-TIMENAVIC message name
static constexpr uint8_t     UBX_NAV2_TIMEUTC_MSGID         = UBX_NAV_TIMEUTC_MSGID;    //!< UBX-NAV2-TIMEUTC message ID
static constexpr const char* UBX_NAV2_TIMEUTC_STRID         = "UBX-NAV2-TIMEUTC";       //!< UBX-NAV2-TIMEUTC message name
static constexpr uint8_t     UBX_NAV2_VELECEF_MSGID         = UBX_NAV_VELECEF_MSGID;    //!< UBX-NAV2-VELECEF message ID
static constexpr const char* UBX_NAV2_VELECEF_STRID         = "UBX-NAV2-VELECEF";       //!< UBX-NAV2-VELECEF message name
static constexpr uint8_t     UBX_NAV2_VELNED_MSGID          = UBX_NAV_VELNED_MSGID;     //!< UBX-NAV2-VELNED message ID
static constexpr const char* UBX_NAV2_VELNED_STRID          = "UBX-NAV2-VELNED";        //!< UBX-NAV2-VELNED message name
static constexpr uint8_t     UBX_RXM_COR_MSGID              = 0x14;                     //!< UBX-RXM-COR message ID
static constexpr const char* UBX_RXM_COR_STRID              = "UBX-RXM-COR";            //!< UBX-RXM-COR message name
static constexpr uint8_t     UBX_RXM_MEASX_MSGID            = 0x14;                     //!< UBX-RXM-MEASX message ID
static constexpr const char* UBX_RXM_MEASX_STRID            = "UBX-RXM-MEASX";          //!< UBX-RXM-MEASX message name
static constexpr uint8_t     UBX_RXM_PMP_MSGID              = 0x72;                     //!< UBX-RXM-PMP message ID
static constexpr const char* UBX_RXM_PMP_STRID              = "UBX-RXM-PMP";            //!< UBX-RXM-PMP message name
static constexpr uint8_t     UBX_RXM_PMREQ_MSGID            = 0x41;                     //!< UBX-RXM-PMREQ message ID
static constexpr const char* UBX_RXM_PMREQ_STRID            = "UBX-RXM-PMREQ";          //!< UBX-RXM-PMREQ message name
static constexpr uint8_t     UBX_RXM_QZSSL6_MSGID           = 0x73;                     //!< UBX-RXM-QZSSL6 message ID
static constexpr const char* UBX_RXM_QZSSL6_STRID           = "UBX-RXM-QZSSL6";         //!< UBX-RXM-QZSSL6 message name
static constexpr uint8_t     UBX_RXM_RAWX_MSGID             = 0x15;                     //!< UBX-RXM-RAWX message ID
static constexpr const char* UBX_RXM_RAWX_STRID             = "UBX-RXM-RAWX";           //!< UBX-RXM-RAWX message name
static constexpr uint8_t     UBX_RXM_RLM_MSGID              = 0x59;                     //!< UBX-RXM-RLM message ID
static constexpr const char* UBX_RXM_RLM_STRID              = "UBX-RXM-RLM";            //!< UBX-RXM-RLM message name
static constexpr uint8_t     UBX_RXM_RTCM_MSGID             = 0x32;                     //!< UBX-RXM-RTCM message ID
static constexpr const char* UBX_RXM_RTCM_STRID             = "UBX-RXM-RTCM";           //!< UBX-RXM-RTCM message name
static constexpr uint8_t     UBX_RXM_SFRBX_MSGID            = 0x13;                     //!< UBX-RXM-SFRBX message ID
static constexpr const char* UBX_RXM_SFRBX_STRID            = "UBX-RXM-SFRBX";          //!< UBX-RXM-SFRBX message name
static constexpr uint8_t     UBX_RXM_SPARTN_MSGID           = 0x33;                     //!< UBX-RXM-SPARTN message ID
static constexpr const char* UBX_RXM_SPARTN_STRID           = "UBX-RXM-SPARTN";         //!< UBX-RXM-SPARTN message name
static constexpr uint8_t     UBX_RXM_SPARTNKEY_MSGID        = 0x36;                     //!< UBX-RXM-SPARTNKEY message ID
static constexpr const char* UBX_RXM_SPARTNKEY_STRID        = "UBX-RXM-SPARTNKEY";      //!< UBX-RXM-SPARTNKEY message name
static constexpr uint8_t     UBX_SEC_OSNMA_MSGID            = 0x0a;                     //!< UBX-SEC-OSNMA message ID
static constexpr const char* UBX_SEC_OSNMA_STRID            = "UBX-SEC-OSNMA";          //!< UBX-SEC-OSNMA message name
static constexpr uint8_t     UBX_SEC_SIG_MSGID              = 0x09;                     //!< UBX-SEC-SIG message ID
static constexpr const char* UBX_SEC_SIG_STRID              = "UBX-SEC-SIG";            //!< UBX-SEC-SIG message name
static constexpr uint8_t     UBX_SEC_SIGLOG_MSGID           = 0x10;                     //!< UBX-SEC-SIGLOG message ID
static constexpr const char* UBX_SEC_SIGLOG_STRID           = "UBX-SEC-SIGLOG";         //!< UBX-SEC-SIGLOG message name
static constexpr uint8_t     UBX_SEC_UNIQID_MSGID           = 0x03;                     //!< UBX-SEC-UNIQID message ID
static constexpr const char* UBX_SEC_UNIQID_STRID           = "UBX-SEC-UNIQID";         //!< UBX-SEC-UNIQID message name
static constexpr uint8_t     UBX_TIM_TM2_MSGID              = 0x03;                     //!< UBX-TIM-TM2 message ID
static constexpr const char* UBX_TIM_TM2_STRID              = "UBX-TIM-TM2";            //!< UBX-TIM-TM2 message name
static constexpr uint8_t     UBX_TIM_TP_MSGID               = 0x01;                     //!< UBX-TIM-TP message ID
static constexpr const char* UBX_TIM_TP_STRID               = "UBX-TIM-TP";             //!< UBX-TIM-TP message name
static constexpr uint8_t     UBX_TIM_VRFY_MSGID             = 0x06;                     //!< UBX-TIM-VRFY message ID
static constexpr const char* UBX_TIM_VRFY_STRID             = "UBX-TIM-VRFY";           //!< UBX-TIM-VRFY message name
static constexpr uint8_t     UBX_UPD_FLDET_MSGID            = 0x08;                     //!< UBX-UPD-FLDET message ID
static constexpr const char* UBX_UPD_FLDET_STRID            = "UBX-UPD-FLDET";          //!< UBX-UPD-FLDET message name
static constexpr uint8_t     UBX_UPD_POS_MSGID              = 0x15;                     //!< UBX-UPD-POS message ID
static constexpr const char* UBX_UPD_POS_STRID              = "UBX-UPD-POS";            //!< UBX-UPD-POS message name
static constexpr uint8_t     UBX_UPD_SAFEBOOT_MSGID         = 0x07;                     //!< UBX-UPD-SAFEBOOT message ID
static constexpr const char* UBX_UPD_SAFEBOOT_STRID         = "UBX-UPD-SAFEBOOT";       //!< UBX-UPD-SAFEBOOT message name
static constexpr uint8_t     UBX_UPD_SOS_MSGID              = 0x14;                     //!< UBX-UPD-SOS message ID
static constexpr const char* UBX_UPD_SOS_STRID              = "UBX-UPD-SOS";            //!< UBX-UPD-SOS message name
static constexpr uint8_t     UBX_NMEA_STANDARD_DTM_MSGID    = 0x0a;                     //!< UBX-NMEA-STANDARD_DTM message ID
static constexpr const char* UBX_NMEA_STANDARD_DTM_STRID    = "UBX-NMEA-STANDARD_DTM";  //!< UBX-NMEA-STANDARD_DTM message name
static constexpr uint8_t     UBX_NMEA_STANDARD_GAQ_MSGID    = 0x45;                     //!< UBX-NMEA-STANDARD_GAQ message ID
static constexpr const char* UBX_NMEA_STANDARD_GAQ_STRID    = "UBX-NMEA-STANDARD_GAQ";  //!< UBX-NMEA-STANDARD_GAQ message name
static constexpr uint8_t     UBX_NMEA_STANDARD_GBQ_MSGID    = 0x44;                     //!< UBX-NMEA-STANDARD_GBQ message ID
static constexpr const char* UBX_NMEA_STANDARD_GBQ_STRID    = "UBX-NMEA-STANDARD_GBQ";  //!< UBX-NMEA-STANDARD_GBQ message name
static constexpr uint8_t     UBX_NMEA_STANDARD_GBS_MSGID    = 0x09;                     //!< UBX-NMEA-STANDARD_GBS message ID
static constexpr const char* UBX_NMEA_STANDARD_GBS_STRID    = "UBX-NMEA-STANDARD_GBS";  //!< UBX-NMEA-STANDARD_GBS message name
static constexpr uint8_t     UBX_NMEA_STANDARD_GGA_MSGID    = 0x00;                     //!< UBX-NMEA-STANDARD_GGA message ID
static constexpr const char* UBX_NMEA_STANDARD_GGA_STRID    = "UBX-NMEA-STANDARD_GGA";  //!< UBX-NMEA-STANDARD_GGA message name
static constexpr uint8_t     UBX_NMEA_STANDARD_GLL_MSGID    = 0x01;                     //!< UBX-NMEA-STANDARD_GLL message ID
static constexpr const char* UBX_NMEA_STANDARD_GLL_STRID    = "UBX-NMEA-STANDARD_GLL";  //!< UBX-NMEA-STANDARD_GLL message name
static constexpr uint8_t     UBX_NMEA_STANDARD_GLQ_MSGID    = 0x43;                     //!< UBX-NMEA-STANDARD_GLQ message ID
static constexpr const char* UBX_NMEA_STANDARD_GLQ_STRID    = "UBX-NMEA-STANDARD_GLQ";  //!< UBX-NMEA-STANDARD_GLQ message name
static constexpr uint8_t     UBX_NMEA_STANDARD_GNQ_MSGID    = 0x42;                     //!< UBX-NMEA-STANDARD_GNQ message ID
static constexpr const char* UBX_NMEA_STANDARD_GNQ_STRID    = "UBX-NMEA-STANDARD_GNQ";  //!< UBX-NMEA-STANDARD_GNQ message name
static constexpr uint8_t     UBX_NMEA_STANDARD_GNS_MSGID    = 0x0d;                     //!< UBX-NMEA-STANDARD_GNS message ID
static constexpr const char* UBX_NMEA_STANDARD_GNS_STRID    = "UBX-NMEA-STANDARD_GNS";  //!< UBX-NMEA-STANDARD_GNS message name
static constexpr uint8_t     UBX_NMEA_STANDARD_GPQ_MSGID    = 0x40;                     //!< UBX-NMEA-STANDARD_GPQ message ID
static constexpr const char* UBX_NMEA_STANDARD_GPQ_STRID    = "UBX-NMEA-STANDARD_GPQ";  //!< UBX-NMEA-STANDARD_GPQ message name
static constexpr uint8_t     UBX_NMEA_STANDARD_GQQ_MSGID    = 0x47;                     //!< UBX-NMEA-STANDARD_GQQ message ID
static constexpr const char* UBX_NMEA_STANDARD_GQQ_STRID    = "UBX-NMEA-STANDARD_GQQ";  //!< UBX-NMEA-STANDARD_GQQ message name
static constexpr uint8_t     UBX_NMEA_STANDARD_GRS_MSGID    = 0x06;                     //!< UBX-NMEA-STANDARD_GRS message ID
static constexpr const char* UBX_NMEA_STANDARD_GRS_STRID    = "UBX-NMEA-STANDARD_GRS";  //!< UBX-NMEA-STANDARD_GRS message name
static constexpr uint8_t     UBX_NMEA_STANDARD_GSA_MSGID    = 0x02;                     //!< UBX-NMEA-STANDARD_GSA message ID
static constexpr const char* UBX_NMEA_STANDARD_GSA_STRID    = "UBX-NMEA-STANDARD_GSA";  //!< UBX-NMEA-STANDARD_GSA message name
static constexpr uint8_t     UBX_NMEA_STANDARD_GST_MSGID    = 0x07;                     //!< UBX-NMEA-STANDARD_GST message ID
static constexpr const char* UBX_NMEA_STANDARD_GST_STRID    = "UBX-NMEA-STANDARD_GST";  //!< UBX-NMEA-STANDARD_GST message name
static constexpr uint8_t     UBX_NMEA_STANDARD_GSV_MSGID    = 0x03;                     //!< UBX-NMEA-STANDARD_GSV message ID
static constexpr const char* UBX_NMEA_STANDARD_GSV_STRID    = "UBX-NMEA-STANDARD_GSV";  //!< UBX-NMEA-STANDARD_GSV message name
static constexpr uint8_t     UBX_NMEA_STANDARD_RLM_MSGID    = 0x0b;                     //!< UBX-NMEA-STANDARD_RLM message ID
static constexpr const char* UBX_NMEA_STANDARD_RLM_STRID    = "UBX-NMEA-STANDARD_RLM";  //!< UBX-NMEA-STANDARD_RLM message name
static constexpr uint8_t     UBX_NMEA_STANDARD_RMC_MSGID    = 0x04;                     //!< UBX-NMEA-STANDARD_RMC message ID
static constexpr const char* UBX_NMEA_STANDARD_RMC_STRID    = "UBX-NMEA-STANDARD_RMC";  //!< UBX-NMEA-STANDARD_RMC message name
static constexpr uint8_t     UBX_NMEA_STANDARD_TXT_MSGID    = 0x41;                     //!< UBX-NMEA-STANDARD_TXT message ID
static constexpr const char* UBX_NMEA_STANDARD_TXT_STRID    = "UBX-NMEA-STANDARD_TXT";  //!< UBX-NMEA-STANDARD_TXT message name
static constexpr uint8_t     UBX_NMEA_STANDARD_VLW_MSGID    = 0x0f;                     //!< UBX-NMEA-STANDARD_VLW message ID
static constexpr const char* UBX_NMEA_STANDARD_VLW_STRID    = "UBX-NMEA-STANDARD_VLW";  //!< UBX-NMEA-STANDARD_VLW message name
static constexpr uint8_t     UBX_NMEA_STANDARD_VTG_MSGID    = 0x05;                     //!< UBX-NMEA-STANDARD_VTG message ID
static constexpr const char* UBX_NMEA_STANDARD_VTG_STRID    = "UBX-NMEA-STANDARD_VTG";  //!< UBX-NMEA-STANDARD_VTG message name
static constexpr uint8_t     UBX_NMEA_STANDARD_ZDA_MSGID    = 0x08;                     //!< UBX-NMEA-STANDARD_ZDA message ID
static constexpr const char* UBX_NMEA_STANDARD_ZDA_STRID    = "UBX-NMEA-STANDARD_ZDA";  //!< UBX-NMEA-STANDARD_ZDA message name
static constexpr uint8_t     UBX_NMEA_PUBX_CONFIG_MSGID     = 0x41;                     //!< UBX-NMEA-PUBX_CONFIG message ID
static constexpr const char* UBX_NMEA_PUBX_CONFIG_STRID     = "UBX-NMEA-PUBX_CONFIG";   //!< UBX-NMEA-PUBX_CONFIG message name
static constexpr uint8_t     UBX_NMEA_PUBX_POSITION_MSGID   = 0x00;                     //!< UBX-NMEA-PUBX_POSITION message ID
static constexpr const char* UBX_NMEA_PUBX_POSITION_STRID   = "UBX-NMEA-PUBX_POSITION"; //!< UBX-NMEA-PUBX_POSITION message name
static constexpr uint8_t     UBX_NMEA_PUBX_RATE_MSGID       = 0x40;                     //!< UBX-NMEA-PUBX_RATE message ID
static constexpr const char* UBX_NMEA_PUBX_RATE_STRID       = "UBX-NMEA-PUBX_RATE";     //!< UBX-NMEA-PUBX_RATE message name
static constexpr uint8_t     UBX_NMEA_PUBX_SVSTATUS_MSGID   = 0x03;                     //!< UBX-NMEA-PUBX_SVSTATUS message ID
static constexpr const char* UBX_NMEA_PUBX_SVSTATUS_STRID   = "UBX-NMEA-PUBX_SVSTATUS"; //!< UBX-NMEA-PUBX_SVSTATUS message name
static constexpr uint8_t     UBX_NMEA_PUBX_TIME_MSGID       = 0x04;                     //!< UBX-NMEA-PUBX_TIME message ID
static constexpr const char* UBX_NMEA_PUBX_TIME_STRID       = "UBX-NMEA-PUBX_TIME";     //!< UBX-NMEA-PUBX_TIME message name
static constexpr uint8_t     UBX_RTCM3_TYPE1001_MSGID       = 0x01;                     //!< UBX-RTCM3-TYPE1001 message ID
static constexpr const char* UBX_RTCM3_TYPE1001_STRID       = "UBX-RTCM3-TYPE1001";     //!< UBX-RTCM3-TYPE1001 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1002_MSGID       = 0x02;                     //!< UBX-RTCM3-TYPE1002 message ID
static constexpr const char* UBX_RTCM3_TYPE1002_STRID       = "UBX-RTCM3-TYPE1002";     //!< UBX-RTCM3-TYPE1002 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1003_MSGID       = 0x03;                     //!< UBX-RTCM3-TYPE1003 message ID
static constexpr const char* UBX_RTCM3_TYPE1003_STRID       = "UBX-RTCM3-TYPE1003";     //!< UBX-RTCM3-TYPE1003 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1004_MSGID       = 0x04;                     //!< UBX-RTCM3-TYPE1004 message ID
static constexpr const char* UBX_RTCM3_TYPE1004_STRID       = "UBX-RTCM3-TYPE1004";     //!< UBX-RTCM3-TYPE1004 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1005_MSGID       = 0x05;                     //!< UBX-RTCM3-TYPE1005 message ID
static constexpr const char* UBX_RTCM3_TYPE1005_STRID       = "UBX-RTCM3-TYPE1005";     //!< UBX-RTCM3-TYPE1005 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1006_MSGID       = 0x06;                     //!< UBX-RTCM3-TYPE1006 message ID
static constexpr const char* UBX_RTCM3_TYPE1006_STRID       = "UBX-RTCM3-TYPE1006";     //!< UBX-RTCM3-TYPE1006 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1007_MSGID       = 0x07;                     //!< UBX-RTCM3-TYPE1007 message ID
static constexpr const char* UBX_RTCM3_TYPE1007_STRID       = "UBX-RTCM3-TYPE1007";     //!< UBX-RTCM3-TYPE1007 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1009_MSGID       = 0x09;                     //!< UBX-RTCM3-TYPE1009 message ID
static constexpr const char* UBX_RTCM3_TYPE1009_STRID       = "UBX-RTCM3-TYPE1009";     //!< UBX-RTCM3-TYPE1009 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1010_MSGID       = 0x0a;                     //!< UBX-RTCM3-TYPE1010 message ID
static constexpr const char* UBX_RTCM3_TYPE1010_STRID       = "UBX-RTCM3-TYPE1010";     //!< UBX-RTCM3-TYPE1010 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1011_MSGID       = 0xa1;                     //!< UBX-RTCM3-TYPE1011 message ID
static constexpr const char* UBX_RTCM3_TYPE1011_STRID       = "UBX-RTCM3-TYPE1011";     //!< UBX-RTCM3-TYPE1011 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1012_MSGID       = 0xa2;                     //!< UBX-RTCM3-TYPE1012 message ID
static constexpr const char* UBX_RTCM3_TYPE1012_STRID       = "UBX-RTCM3-TYPE1012";     //!< UBX-RTCM3-TYPE1012 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1033_MSGID       = 0x21;                     //!< UBX-RTCM3-TYPE1033 message ID
static constexpr const char* UBX_RTCM3_TYPE1033_STRID       = "UBX-RTCM3-TYPE1033";     //!< UBX-RTCM3-TYPE1033 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1074_MSGID       = 0x4a;                     //!< UBX-RTCM3-TYPE1074 message ID
static constexpr const char* UBX_RTCM3_TYPE1074_STRID       = "UBX-RTCM3-TYPE1074";     //!< UBX-RTCM3-TYPE1074 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1075_MSGID       = 0x4b;                     //!< UBX-RTCM3-TYPE1075 message ID
static constexpr const char* UBX_RTCM3_TYPE1075_STRID       = "UBX-RTCM3-TYPE1075";     //!< UBX-RTCM3-TYPE1075 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1077_MSGID       = 0x4d;                     //!< UBX-RTCM3-TYPE1077 message ID
static constexpr const char* UBX_RTCM3_TYPE1077_STRID       = "UBX-RTCM3-TYPE1077";     //!< UBX-RTCM3-TYPE1077 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1084_MSGID       = 0x54;                     //!< UBX-RTCM3-TYPE1084 message ID
static constexpr const char* UBX_RTCM3_TYPE1084_STRID       = "UBX-RTCM3-TYPE1084";     //!< UBX-RTCM3-TYPE1084 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1085_MSGID       = 0x55;                     //!< UBX-RTCM3-TYPE1085 message ID
static constexpr const char* UBX_RTCM3_TYPE1085_STRID       = "UBX-RTCM3-TYPE1085";     //!< UBX-RTCM3-TYPE1085 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1087_MSGID       = 0x57;                     //!< UBX-RTCM3-TYPE1087 message ID
static constexpr const char* UBX_RTCM3_TYPE1087_STRID       = "UBX-RTCM3-TYPE1087";     //!< UBX-RTCM3-TYPE1087 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1094_MSGID       = 0x5e;                     //!< UBX-RTCM3-TYPE1094 message ID
static constexpr const char* UBX_RTCM3_TYPE1094_STRID       = "UBX-RTCM3-TYPE1094";     //!< UBX-RTCM3-TYPE1094 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1095_MSGID       = 0x5f;                     //!< UBX-RTCM3-TYPE1095 message ID
static constexpr const char* UBX_RTCM3_TYPE1095_STRID       = "UBX-RTCM3-TYPE1095";     //!< UBX-RTCM3-TYPE1095 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1097_MSGID       = 0x61;                     //!< UBX-RTCM3-TYPE1097 message ID
static constexpr const char* UBX_RTCM3_TYPE1097_STRID       = "UBX-RTCM3-TYPE1097";     //!< UBX-RTCM3-TYPE1097 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1124_MSGID       = 0x7c;                     //!< UBX-RTCM3-TYPE1124 message ID
static constexpr const char* UBX_RTCM3_TYPE1124_STRID       = "UBX-RTCM3-TYPE1124";     //!< UBX-RTCM3-TYPE1124 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1125_MSGID       = 0x7d;                     //!< UBX-RTCM3-TYPE1125 message ID
static constexpr const char* UBX_RTCM3_TYPE1125_STRID       = "UBX-RTCM3-TYPE1125";     //!< UBX-RTCM3-TYPE1125 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1127_MSGID       = 0x7f;                     //!< UBX-RTCM3-TYPE1127 message ID
static constexpr const char* UBX_RTCM3_TYPE1127_STRID       = "UBX-RTCM3-TYPE1127";     //!< UBX-RTCM3-TYPE1127 message name
static constexpr uint8_t     UBX_RTCM3_TYPE1230_MSGID       = 0xe6;                     //!< UBX-RTCM3-TYPE1230 message ID
static constexpr const char* UBX_RTCM3_TYPE1230_STRID       = "UBX-RTCM3-TYPE1230";     //!< UBX-RTCM3-TYPE1230 message name
static constexpr uint8_t     UBX_RTCM3_TYPE4072_0_MSGID     = 0xfe;                     //!< UBX-RTCM3-TYPE4072_0 message ID
static constexpr const char* UBX_RTCM3_TYPE4072_0_STRID     = "UBX-RTCM3-TYPE4072_0";   //!< UBX-RTCM3-TYPE4072_0 message name
static constexpr uint8_t     UBX_RTCM3_TYPE4072_1_MSGID     = 0xfd;                     //!< UBX-RTCM3-TYPE4072_1 message ID
static constexpr const char* UBX_RTCM3_TYPE4072_1_STRID     = "UBX-RTCM3-TYPE4072_1";   //!< UBX-RTCM3-TYPE4072_1 message name
// @fp_codegen_end{FPSDK_COMMON_PARSER_UBX_MESSAGES}
// clang-format on

/**
 * @brief UBX class/message lookup table entry
 */
struct UbxMsgInfo
{
    uint8_t cls_id_ = 0;          //!< UBX class ID
    uint8_t msg_id_ = 0;          //!< UBX message ID
    const char* name_ = nullptr;  //!< UBX name
};

// @fp_codegen_begin{FPSDK_COMMON_PARSER_UBX_MSGINFO_HPP}
using UbxClassesInfo = std::array<UbxMsgInfo, 15>;    //!< UBX classes lookup table
using UbxMessagesInfo = std::array<UbxMsgInfo, 186>;  //!< UBX messages lookup table
// @fp_codegen_end{FPSDK_COMMON_PARSER_UBX_MSGINFO_HPP}

/**
 * @brief Get UBX classes lookup table
 *
 * @returns the UBX classes lookup table
 */
const UbxClassesInfo& UbxGetClassesInfo();

/**
 * @brief Get UBX messages lookup table
 *
 * @returns the UBX messages lookup table
 */
const UbxMessagesInfo& UbxGetMessagesInfo();

/**
 * @brief Get UBX message info
 *
 * @param[in]   name  The name of the message, e.g. "UBX-NAV-SOL"
 * @param[out]  info  The message info, if name is a known message
 *
 * @returns true if the message name was found and info is valid, false otherwise
 */
bool UbxMessageInfo(const std::string& name, UbxMsgInfo& info);

///@}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @name UBX GNSS IDs (gnssId)
 * @{
 */
// clang-format off
static constexpr uint8_t UBX_GNSSID_NONE         = 0xff;   //!< None
static constexpr uint8_t UBX_GNSSID_GPS          = 0;      //!< GPS
static constexpr uint8_t UBX_GNSSID_SBAS         = 1;      //!< SBAS
static constexpr uint8_t UBX_GNSSID_GAL          = 2;      //!< Galileo
static constexpr uint8_t UBX_GNSSID_BDS          = 3;      //!< BeiDou
static constexpr uint8_t UBX_GNSSID_QZSS         = 5;      //!< QZSS
static constexpr uint8_t UBX_GNSSID_GLO          = 6;      //!< GLONASS
static constexpr uint8_t UBX_GNSSID_NAVIC        = 7;      //!< NavIC
// clang-format on
///@}

/**
 * @name UBX SV numbering
 * @{
 */
// clang-format off
static constexpr uint8_t UBX_NUM_GPS             =  32;    //!< G01-G32
static constexpr uint8_t UBX_NUM_SBAS            =  39;    //!< S120-S158
static constexpr uint8_t UBX_NUM_GAL             =  36;    //!< E01-E36
static constexpr uint8_t UBX_NUM_BDS             =  63;    //!< B01-B63 ("Cxx" in RINEX)
static constexpr uint8_t UBX_NUM_QZSS            =  10;    //!< Q01-Q10 ("Jxx" in RINEX)
static constexpr uint8_t UBX_NUM_GLO             =  32;    //!< R01-R32
static constexpr uint8_t UBX_NUM_NAVIC           =  14;    //!< N01-N14 ("Ixx" in RINEX)
static constexpr uint8_t UBX_FIRST_GPS           =   1;    //!< G01
static constexpr uint8_t UBX_FIRST_SBAS          = 120;    //!< S120 ("Sxx" in RINEX)
static constexpr uint8_t UBX_FIRST_GAL           =   1;    //!< E01
static constexpr uint8_t UBX_FIRST_BDS           =   1;    //!< B01
static constexpr uint8_t UBX_FIRST_QZSS          =   1;    //!< Q01
static constexpr uint8_t UBX_FIRST_GLO           =   1;    //!< R01
static constexpr uint8_t UBX_FIRST_NAVIC         =   1;    //!< N01
// clang-format on
///@}

/**
 * @name UBX signal IDs (sigId)
 * @{
 */
// clang-format off
static constexpr uint8_t UBX_SIGID_NONE          = 0xff;   //!< None
static constexpr uint8_t UBX_SIGID_GPS_L1CA      =  0;      //!< GPS L1 C/A
static constexpr uint8_t UBX_SIGID_GPS_L2CL      =  3;      //!< GPS L2 CL
static constexpr uint8_t UBX_SIGID_GPS_L2CM      =  4;      //!< GPS L2 CM
static constexpr uint8_t UBX_SIGID_GPS_L5I       =  6;      //!< GPS L5 I
static constexpr uint8_t UBX_SIGID_GPS_L5Q       =  7;      //!< GPS L5 Q
static constexpr uint8_t UBX_SIGID_SBAS_L1CA     =  0;      //!< SBAS L1 C/A
static constexpr uint8_t UBX_SIGID_GAL_E1C       =  0;      //!< Galileo E1 C
static constexpr uint8_t UBX_SIGID_GAL_E1B       =  1;      //!< Galileo E1 B
static constexpr uint8_t UBX_SIGID_GAL_E5AI      =  3;      //!< Galileo E5 aI
static constexpr uint8_t UBX_SIGID_GAL_E5AQ      =  4;      //!< Galileo E5 aQ
static constexpr uint8_t UBX_SIGID_GAL_E5BI      =  5;      //!< Galileo E5 bI
static constexpr uint8_t UBX_SIGID_GAL_E5BQ      =  6;      //!< Galileo E5 bQ
static constexpr uint8_t UBX_SIGID_GAL_E6B       =  8;      //!< Galileo E6 B
static constexpr uint8_t UBX_SIGID_GAL_E6C       =  9;      //!< Galileo E6 C
static constexpr uint8_t UBX_SIGID_GAL_E6A       = 10;      //!< Galileo E6 A
static constexpr uint8_t UBX_SIGID_BDS_B1ID1     =  0;      //!< BeiDou B1I D1
static constexpr uint8_t UBX_SIGID_BDS_B1ID2     =  1;      //!< BeiDou B1I D2
static constexpr uint8_t UBX_SIGID_BDS_B2ID1     =  2;      //!< BeiDou B2I D1
static constexpr uint8_t UBX_SIGID_BDS_B2ID2     =  3;      //!< BeiDou B2I D2
static constexpr uint8_t UBX_SIGID_BDS_B3ID1     =  4;      //!< BeiDou B3I D1
static constexpr uint8_t UBX_SIGID_BDS_B3ID2     = 10;      //!< BeiDou B3I D2
static constexpr uint8_t UBX_SIGID_BDS_B1CP      =  5;      //!< BeiDou B1 Cp (pilot)
static constexpr uint8_t UBX_SIGID_BDS_B1CD      =  6;      //!< BeiDou B1 Cd (data)
static constexpr uint8_t UBX_SIGID_BDS_B2AP      =  7;      //!< BeiDou B2 ap (pilot)
static constexpr uint8_t UBX_SIGID_BDS_B2AD      =  8;      //!< BeiDou B2 ad (data)
static constexpr uint8_t UBX_SIGID_QZSS_L1CA     =  0;      //!< QZSS L1 C/A
static constexpr uint8_t UBX_SIGID_QZSS_L1S      =  1;      //!< QZSS L1 S
static constexpr uint8_t UBX_SIGID_QZSS_L2CM     =  4;      //!< QZSS L2 CM
static constexpr uint8_t UBX_SIGID_QZSS_L2CL     =  5;      //!< QZSS L2 CL
static constexpr uint8_t UBX_SIGID_QZSS_L5I      =  8;      //!< QZSS L5 I
static constexpr uint8_t UBX_SIGID_QZSS_L5Q      =  9;      //!< QZSS L5 Q
static constexpr uint8_t UBX_SIGID_GLO_L1OF      =  0;      //!< GLONASS L1 OF
static constexpr uint8_t UBX_SIGID_GLO_L2OF      =  2;      //!< GLONASS L2 OF
static constexpr uint8_t UBX_SIGID_NAVIC_L5A     =  0;      //!< NavIC L5 A
// clang-format on
///@}

// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-ACK-ACK message
 * @{
 */

//! UBX-ACK-ACK (version 0, output) payload
struct UBX_ACK_ACK_V0_GROUP0  // clang-format off
{
    uint8_t clsId;           //!< Class ID of ack'ed message
    uint8_t msgId;           //!< Message ID of ack'ed message
};  // clang-format on

// clang-format off
static constexpr std::size_t UBX_ACK_ACK_V0_SIZE                                                    = sizeof(UBX_ACK_ACK_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-ACK-NAK message
 * @{
 */

//! UBX-ACK-NCK (version 0, output) payload
struct UBX_ACK_NAK_V0_GROUP0  // clang-format off
{
    uint8_t clsId;           //!< Class ID of not-ack'ed message
    uint8_t msgId;           //!< Message ID of not-ack'ed message
};
// clang-format on

// clang-format off
static constexpr std::size_t UBX_ACK_NAK_V0_SIZE                                                    = sizeof(UBX_ACK_NAK_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-CFG-VALSET  message
 * @{
 */

//! UBX-CFG-VALSET (version 0, input) message payload header
struct UBX_CFG_VALSET_V0_GROUP0  // clang-format off
{
    uint8_t  version;        //!< Message version (#UBX_CFG_VALSET_V1_VERSION)
    uint8_t  layers;         //!< Configuration layers
    uint8_t  reserved[2];    //!< Reserved (set to 0x00)
};

static_assert(sizeof(UBX_CFG_VALSET_V0_GROUP0) == 4, "");

//! UBX-CFG-VALSET (version 1, input) message payload header
struct UBX_CFG_VALSET_V1_GROUP0  // clang-format off
{
    uint8_t  version;        //!< Message version (#UBX_CFG_VALSET_V1_VERSION)
    uint8_t  layers;         //!< Configuration layers
    uint8_t  transaction;    //!< Transaction mode
    uint8_t  reserved;       //!< Reserved (set to 0x00)
};

static_assert(sizeof(UBX_CFG_VALSET_V1_GROUP0) == 4, "");

// clang-format off
static constexpr uint8_t     UBX_CFG_VALSET_VERSION(const uint8_t* msg)                             { return msg[UBX_HEAD_SIZE]; }  //!< @todo documentation

static constexpr uint8_t     UBX_CFG_VALSET_V0_VERSION                                              = 0x00;  //!< UBX-CFG-VALSET.version value
static constexpr std::size_t UBX_CFG_VALSET_V0_MIN_SIZE                                             = sizeof(UBX_CFG_VALSET_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr uint8_t     UBX_CFG_VALSET_V0_LAYERS_RAM                                           = 0x01;  //!< UBX-CFG-VALSET.layers flag: layer RAM
static constexpr uint8_t     UBX_CFG_VALSET_V0_LAYERS_BBR                                           = 0x02;  //!< UBX-CFG-VALSET.layers flag: layer BBR
static constexpr uint8_t     UBX_CFG_VALSET_V0_LAYERS_FLASH                                         = 0x04;  //!< UBX-CFG-VALSET.layers flag: layer Flash
static constexpr uint8_t     UBX_CFG_VALSET_V0_RESERVED                                             = 0x00;  //!< UBX-CFG-VALSET.reserved value
static constexpr std::size_t UBX_CFG_VALSET_V0_MAX_KV                                               = 64;    //!< UBX-CFG-VALSET.cfgData: maximum number of key-value pairs
static constexpr std::size_t UBX_CFG_VALSET_V0_CFGDATA_MAX_SIZE                                     = UBX_CFG_VALSET_V0_MAX_KV * (4 + 8); //!< UBX-CFG-VALSET.cfgData maximum size
static constexpr std::size_t UBX_CFG_VALSET_V0_MAX_SIZE                                             = UBX_CFG_VALSET_V0_MIN_SIZE + UBX_CFG_VALSET_V0_CFGDATA_MAX_SIZE;  //!< @todo documentation

static constexpr uint8_t     UBX_CFG_VALSET_V1_VERSION                                              = 0x01;  //!< UBX-CFG-VALSET.version value
static constexpr std::size_t UBX_CFG_VALSET_V1_MIN_SIZE                                             = sizeof(UBX_CFG_VALSET_V1_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr uint8_t     UBX_CFG_VALSET_V1_LAYER_RAM                                            = 0x01;  //!< UBX-CFG-VALSET.layers flag: layer RAM
static constexpr uint8_t     UBX_CFG_VALSET_V1_LAYER_BBR                                            = 0x02;  //!< UBX-CFG-VALSET.layers flag: layer BBR
static constexpr uint8_t     UBX_CFG_VALSET_V1_LAYER_FLASH                                          = 0x04;  //!< UBX-CFG-VALSET.layers flag: layer Flash
static constexpr uint8_t     UBX_CFG_VALSET_V1_TRANSACTION_NONE                                     = 0;     //!< UBX-CFG-VALSET.transaction value: no transaction
static constexpr uint8_t     UBX_CFG_VALSET_V1_TRANSACTION_BEGIN                                    = 1;     //!< UBX-CFG-VALSET.transaction value: transaction begin
static constexpr uint8_t     UBX_CFG_VALSET_V1_TRANSACTION_CONTINUE                                 = 2;     //!< UBX-CFG-VALSET.transaction value: transaction continue
static constexpr uint8_t     UBX_CFG_VALSET_V1_TRANSACTION_END                                      = 3;     //!< UBX-CFG-VALSET.transaction value: transaction: end
static constexpr uint8_t     UBX_CFG_VALSET_V1_RESERVED                                             = 0x00;  //!< UBX-CFG-VALSET.reserved value
static constexpr std::size_t UBX_CFG_VALSET_V1_MAX_KV                                               = 64;    //!< UBX-CFG-VALSET.cfgData: maximum number of key-value pairs
static constexpr std::size_t UBX_CFG_VALSET_V1_CFGDATA_MAX_SIZE                                     = UBX_CFG_VALSET_V1_MAX_KV * (4 + 8); //!< UBX-CFG-VALSET.cfgData maximum size
static constexpr std::size_t UBX_CFG_VALSET_V1_MAX_SIZE                                             = UBX_CFG_VALSET_V1_MIN_SIZE + UBX_CFG_VALSET_V1_CFGDATA_MAX_SIZE;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-CFG-VALGET message
 * @{
 */

//! UBX-CFG-VALGET (version 0, poll) message payload header
struct UBX_CFG_VALGET_V0_GROUP0  // clang-format off
{
    uint8_t  version;        //!< Message version (#UBX_CFG_VALGET_V0_VERSION)
    uint8_t  layer;          //!< Configuration layer
    uint16_t position;       //!< Number of values to skip in result set
};  // clang-format on

static_assert(sizeof(UBX_CFG_VALGET_V0_GROUP0) == 4, "");

//! UBX-CFG-VALGET (version 1, output) message payload header
struct UBX_CFG_VALGET_V1_GROUP0  // clang-format off
{
    uint8_t  version;        //!< Message version (#UBX_CFG_VALGET_V1_VERSION)
    uint8_t  layer;          //!< Configuration layer
    uint16_t position;       //!< Number of values to skip in result set
};  // clang-format on

static_assert(sizeof(UBX_CFG_VALGET_V1_GROUP0) == 4, "");

// clang-format off
static constexpr uint8_t     UBX_CFG_VALGET_VERSION(const uint8_t* msg)                             { return msg[UBX_HEAD_SIZE]; }  //!< @todo documentation

static constexpr uint8_t     UBX_CFG_VALGET_V0_VERSION                                              = 0x00;      //!< UBX-CFG-VALGET.version value
static constexpr std::size_t UBX_CFG_VALGET_V0_MIN_SIZE                                             = sizeof(UBX_CFG_VALGET_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr uint8_t     UBX_CFG_VALGET_V0_LAYER_RAM                                            = 0;         //!< UBX-CFG-VALGET.layers value: layer RAM
static constexpr uint8_t     UBX_CFG_VALGET_V0_LAYER_BBR                                            = 1;         //!< UBX-CFG-VALGET.layers value: layer BBR
static constexpr uint8_t     UBX_CFG_VALGET_V0_LAYER_FLASH                                          = 2;         //!< UBX-CFG-VALGET.layers value: layer Flash
static constexpr uint8_t     UBX_CFG_VALGET_V0_LAYER_DEFAULT                                        = 7;         //!< UBX-CFG-VALGET.layers value: layer Default
static constexpr std::size_t UBX_CFG_VALGET_V0_MAX_K                                                = 64;        //!< UBX-CFG-VALGET.cfgData maximum number of keys
static constexpr std::size_t UBX_CFG_VALGET_V0_KEYS_MAX_SIZE                                        = UBX_CFG_VALGET_V0_MAX_K * sizeof(uint32_t); //!< UBX-CFG-VALGET.keys maximum size
static constexpr uint32_t    UBX_CFG_VALGET_V0_GROUP_WILDCARD(const uint32_t groupId)                  { return groupId | 0x0000ffff; } //!< UBX-CFG-VALGET.keys group wildcard
static constexpr uint32_t    UBX_CFG_VALGET_V0_ALL_WILDCARD                                         = 0x0fffffff;  //!< UBX-CFG-VALGET.keys all wildcard
static constexpr std::size_t UBX_CFG_VALGET_V0_MAX_SIZE                                             = UBX_CFG_VALGET_V0_MIN_SIZE + UBX_CFG_VALGET_V0_KEYS_MAX_SIZE;  //!< @todo documentation

static constexpr uint8_t     UBX_CFG_VALGET_V1_VERSION                                              = 0x01;      //!< UBX-CFG-VALGET.version value
static constexpr std::size_t UBX_CFG_VALGET_V1_MIN_SIZE                                             = sizeof(UBX_CFG_VALGET_V1_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr uint8_t     UBX_CFG_VALGET_V1_LAYER_RAM                                            = 0;         //!< UBX-CFG-VALGET.layers value: layer RAM
static constexpr uint8_t     UBX_CFG_VALGET_V1_LAYER_BBR                                            = 1;         //!< UBX-CFG-VALGET.layers value: layer BBR
static constexpr uint8_t     UBX_CFG_VALGET_V1_LAYER_FLASH                                          = 2;         //!< UBX-CFG-VALGET.layers value: layer Flash
static constexpr uint8_t     UBX_CFG_VALGET_V1_LAYER_DEFAULT                                        = 7;         //!< UBX-CFG-VALGET.layers value: layer Default
static constexpr std::size_t UBX_CFG_VALGET_V1_MAX_KV                                               = 64;        //!< UBX-CFG-VALGET.cfgData maximum number of keys
static constexpr std::size_t UBX_CFG_VALGET_V1_CFGDATA_MAX_SIZE                                     = UBX_CFG_VALGET_V1_MAX_KV * (sizeof(uint32_t) + sizeof(uint64_t)); //!< UBX-CFG-VALGET.keys maximum size
static constexpr uint32_t    UBX_CFG_VALGET_V1_GROUP_WILDCARD(const uint32_t groupId)                  { return groupId | 0x0000ffff; } //!< UBX-CFG-VALGET.keys group wildcard
static constexpr uint32_t    UBX_CFG_VALGET_V1_ALL_WILDCARD                                         = 0x0fffffff;  //!< UBX-CFG-VALGET.keys all wildcard
static constexpr std::size_t UBX_CFG_VALGET_V1_MAX_SIZE                                             = UBX_CFG_VALGET_V1_MIN_SIZE + UBX_CFG_VALGET_V1_CFGDATA_MAX_SIZE;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-CFG-VALDEL message
 * @{
 */

//! UBX-CFG-VALDEL (version 1, input) message payload header
struct UBX_CFG_VALDEL_V1_GROUP0  // clang-format off
{
    uint8_t  version;        //!< Message version (#UBX_CFG_VALGET_V1_VERSION)
    uint8_t  layers;         //!< Configuration layers
    uint8_t  transaction;    //!< Transaction mode
    uint8_t  reserved;       //!< Reserved (set to 0x00)
};  // clang-format on

static_assert(sizeof(UBX_CFG_VALDEL_V1_GROUP0) == 4, "");

// clang-format off
static constexpr uint8_t     UBX_CFG_VALDEL_VERSION(const uint8_t* msg)                             { return msg[UBX_HEAD_SIZE]; }  //!< @todo documentation
static constexpr uint8_t     UBX_CFG_VALDEL_V1_VERSION                                              = 0x01;      //!< UBX-CFG-VALDEL.version value
static constexpr std::size_t UBX_CFG_VALDEL_V1_MIN_SIZE                                             = sizeof(UBX_CFG_VALDEL_V1_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr uint8_t     UBX_CFG_VALDEL_V1_LAYER_BBR                                            = 0x02;      //!< UBX-CFG-VALDEL.layers flag: layer BBR
static constexpr uint8_t     UBX_CFG_VALDEL_V1_LAYER_FLASH                                          = 0x04;      //!< UBX-CFG-VALDEL.layers flag: layer Flash
static constexpr uint8_t     UBX_CFG_VALDEL_V1_TRANSACTION_NONE                                     = 0;         //!< UBX-CFG-VALDEL.transaction value: no transaction
static constexpr uint8_t     UBX_CFG_VALDEL_V1_TRANSACTION_BEGIN                                    = 1;         //!< UBX-CFG-VALDEL.transaction value: transaction begin
static constexpr uint8_t     UBX_CFG_VALDEL_V1_TRANSACTION_CONTINUE                                 = 2;         //!< UBX-CFG-VALDEL.transaction value: transaction continue
static constexpr uint8_t     UBX_CFG_VALDEL_V1_TRANSACTION_END                                      = 3;         //!< UBX-CFG-VALDEL.transaction value: transaction: end
static constexpr uint8_t     UBX_CFG_VALDEL_V1_RESERVED                                             = 0x00;      //!< UBX-CFG-VALDEL.reserved value
static constexpr std::size_t UBX_CFG_VALDEL_V1_MAX_K                                                = 64;        //!< UBX-CFG-VALDEL.cfgData maximum number of key IDs
static constexpr std::size_t UBX_CFG_VALDEL_V1_KEYS_MAX_SIZE                                        = UBX_CFG_VALDEL_V1_MAX_K * 4; //!< UBX-CFG-VALDEL.keys maximum size
static constexpr std::size_t UBX_CFG_VALDEL_V1_MAX_SIZE                                             = UBX_CFG_VALDEL_V1_MIN_SIZE + UBX_CFG_VALDEL_V1_KEYS_MAX_SIZE;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-CFG-RST message
 * @{
 */

//! UBX-CFG-RST (version 0, command) message payload
struct UBX_CFG_RST_V0_GROUP0  // clang-format off
{
    uint16_t navBbrMask;     //!< BBR sections to clear
    uint8_t  resetMode;      //!< Reset type
    uint8_t  reserved;       //!< Reserved
};  // clang-format on

static_assert(sizeof(UBX_CFG_RST_V0_GROUP0) == 4, "");

// clang-format off
static constexpr std::size_t UBX_CFG_RST_V0_SIZE                                                    = sizeof(UBX_CFG_RST_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr uint16_t    UBX_CFG_RST_V0_NAVBBR_NONE                                             = 0x0001;  //!< Nothing
static constexpr uint16_t    UBX_CFG_RST_V0_NAVBBR_EPH                                              = 0x0001;  //!< Ephemeris
static constexpr uint16_t    UBX_CFG_RST_V0_NAVBBR_ALM                                              = 0x0002;  //!< Almanac
static constexpr uint16_t    UBX_CFG_RST_V0_NAVBBR_HEALTH                                           = 0x0004;  //!< Health
static constexpr uint16_t    UBX_CFG_RST_V0_NAVBBR_KLOB                                             = 0x0008;  //!< Klobuchar parameters
static constexpr uint16_t    UBX_CFG_RST_V0_NAVBBR_POS                                              = 0x0010;  //!< Position
static constexpr uint16_t    UBX_CFG_RST_V0_NAVBBR_CLKD                                             = 0x0020;  //!< Clock drift
static constexpr uint16_t    UBX_CFG_RST_V0_NAVBBR_OSC                                              = 0x0040;  //!< Oscillator parameters
static constexpr uint16_t    UBX_CFG_RST_V0_NAVBBR_UTC                                              = 0x0080;  //!< UTC and leap second parameters
static constexpr uint16_t    UBX_CFG_RST_V0_NAVBBR_RTC                                              = 0x0100;  //!< RTC
static constexpr uint16_t    UBX_CFG_RST_V0_NAVBBR_AOP                                              = 0x8000;  //!< AssistNow Autonomous
static constexpr uint16_t    UBX_CFG_RST_V0_NAVBBR_HOTSTART                                         = 0x0000;  //!< Hostsart (keep all data)
static constexpr uint16_t    UBX_CFG_RST_V0_NAVBBR_WARMSTART                                        = 0x0001;  //!< Warmstart (clear ephemerides)
static constexpr uint16_t    UBX_CFG_RST_V0_NAVBBR_COLDSTART                                        = 0xffff;  //!< Coldstart (erase all data)
static constexpr uint8_t     UBX_CFG_RST_V0_RESETMODE_HW_FORCED                                     = 0x00;    //!< Forced, immediate hardware reset
static constexpr uint8_t     UBX_CFG_RST_V0_RESETMODE_SW                                            = 0x01;    //!< Controlled software reset
static constexpr uint8_t     UBX_CFG_RST_V0_RESETMODE_GNSS                                          = 0x02;    //!< Restart GNSS
static constexpr uint8_t     UBX_CFG_RST_V0_RESETMODE_HW_CONTROLLED                                 = 0x04;    //!< Controlled hardware reset
static constexpr uint8_t     UBX_CFG_RST_V0_RESETMODE_GNSS_STOP                                     = 0x08;    //!< Stop GNSS
static constexpr uint8_t     UBX_CFG_RST_V0_RESETMODE_GNSS_START                                    = 0x09;    //!< Start GNSS
static constexpr uint8_t     UBX_CFG_RST_V0_RESERVED                                                = 0x00;    //!< Reserved
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-CFG-CFG message
 * @{
 */

//! UBX-CFG-CFG (version 0, command) message head
struct UBX_CFG_CFG_V0_GROUP0  // clang-format off
{
    uint32_t clearMask;      //!< @todo documentation
    uint32_t saveMask;       //!< @todo documentation
    uint32_t loadMask;       //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_CFG_CFG_V0_GROUP0) == 12, "");

//! UBX-CFG-CFG (version 0, command) message optional group
struct UBX_PACKED UBX_CFG_CFG_V0_GROUP1  // clang-format off
{
    uint8_t deviceMask;      //!< @todo documentation
};// clang-format off

static_assert(sizeof(UBX_CFG_CFG_V0_GROUP1) == 1, "");

// clang-format off
static constexpr std::size_t UBX_CFG_CFG_V0_MIN_SIZE                                                = sizeof(UBX_CFG_CFG_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr std::size_t UBX_CFG_CFG_V0_MAX_SIZE                                                = UBX_CFG_CFG_V0_MIN_SIZE + sizeof(UBX_CFG_CFG_V0_GROUP1);  //!< @todo documentation
static constexpr uint32_t    UBX_CFG_CFG_V0_CLEAR_NONE                                              = 0x00000000; //!< Clear no config
static constexpr uint32_t    UBX_CFG_CFG_V0_CLEAR_ALL                                               = 0xffffffff; //!< Clear all config
static constexpr uint32_t    UBX_CFG_CFG_V0_SAVE_NONE                                               = 0x00000000; //!< Save no config
static constexpr uint32_t    UBX_CFG_CFG_V0_SAVE_ALL                                                = 0xffffffff; //!< Save all config
static constexpr uint32_t    UBX_CFG_CFG_V0_LOAD_NONE                                               = 0x00000000; //!< Load no config
static constexpr uint32_t    UBX_CFG_CFG_V0_LOAD_ALL                                                = 0xffffffff; //!< Load all config
static constexpr uint8_t     UBX_CFG_CFG_V0_DEVICE_BBR                                              = 0x01;       //!< Layer BBR
static constexpr uint8_t     UBX_CFG_CFG_V0_DEVICE_FLASH                                            = 0x02;       //!< Layer Flash
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-ESF-MEAS message
 * @{
 */

//! UBX-ESF-MEAS (version 0, input and output) message head
struct UBX_ESF_MEAS_V0_GROUP0  // clang-format off
{
    uint32_t timeTag;        //!< @todo documentation
    uint16_t flags;          //!< @todo documentation
    uint16_t id;             //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_ESF_MEAS_V0_GROUP0) == 8, "");

//! UBX-ESF-MEAS (version 0, input and output) data
struct UBX_ESF_MEAS_V0_GROUP1  // clang-format off
{
    uint32_t data;           //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_ESF_MEAS_V0_GROUP1) == 4, "");

//! UBX-ESF-MEAS (version 0, input and output) timetag
struct UBX_ESF_MEAS_V0_GROUP2  // clang-format off
{
    uint32_t calibTtag;      //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_ESF_MEAS_V0_GROUP2) == 4, "");

// clang-format off
static constexpr std::size_t UBX_ESF_MEAS_V0_MIN_SIZE                                               = sizeof(UBX_ESF_MEAS_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_MEAS_V0_FLAGS_TIMEMARKSENT(const uint16_t flags)               { return flags & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_MEAS_V0_FLAGS_TIMEMARKSENT_NONE                                = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_MEAS_V0_FLAGS_TIMEMARKSENT_EXT0                                = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_MEAS_V0_FLAGS_TIMEMARKSENT_EXT1                                = 2;  //!< @todo documentation
static constexpr bool        UBX_ESF_MEAS_V0_FLAGS_CALIBTTAGVALID(const uint16_t flags)             { return (flags & 0x0008) == 0x0008; }  //!< @todo documentation
static constexpr std::size_t UBX_ESF_MEAS_V0_FLAGS_NUMMEAS(const uint16_t flags)                    { return (flags >> 11) & 0x1f; }  //!< @todo documentation
static constexpr uint32_t    UBX_ESF_MEAS_V0_DATA_DATAFIELD(const uint32_t data)                    { return data & 0x00ffffff; }  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_MEAS_V0_DATA_DATATYPE(const uint32_t data)                     { return (data >> 24) & 0x0000003f; } //!< same enum as UBX-ESF-STATUS.type it seems
static constexpr double      UBX_ESF_MEAS_V0_CALIBTTAG_SCALE                                        = 1e-3;  //!< @todo documentation
static constexpr std::size_t UBX_ESF_MEAS_V0_SIZE(const uint8_t* msg)                               { return /* argh.. nice message design! */ \
    sizeof(UBX_ESF_MEAS_V0_GROUP0) + UBX_FRAME_SIZE + (UBX_ESF_MEAS_V0_FLAGS_NUMMEAS(*((uint16_t *)&((uint8_t *)(msg))[UBX_HEAD_SIZE + 4])) * sizeof(UBX_ESF_MEAS_V0_GROUP1)) +
    (UBX_ESF_MEAS_V0_FLAGS_CALIBTTAGVALID(*((uint16_t *)&((uint8_t *)(msg))[UBX_HEAD_SIZE + 4])) ? sizeof(UBX_ESF_MEAS_V0_GROUP2) : 0); }  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-ESF-STATUS message
 * @{
 */

//! UBX-ESF-STATUS (version 0, output) message head
struct UBX_ESF_STATUS_V2_GROUP0  // clang-format off
{
    uint32_t iTOW;           //!< @todo documentation
    uint8_t  version;        //!< @todo documentation
    uint8_t  initStatus1;    //!< @todo documentation
    uint8_t  initStatus2;    //!< @todo documentation
    uint8_t  reserved0[5];   //!< @todo documentation
    uint8_t  fusionMode;     //!< @todo documentation
    uint8_t  reserved1[2];   //!< @todo documentation
    uint8_t  numSens;        //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_ESF_STATUS_V2_GROUP0) == 16, "");

//! UBX-ESF-STATUS (version 0, output) per-sensor status
struct UBX_ESF_STATUS_V2_GROUP1  // clang-format off
{
    uint8_t  sensStatus1;    //!< @todo documentation
    uint8_t  sensStatus2;    //!< @todo documentation
    uint8_t  freq;           //!< @todo documentation
    uint8_t  faults;         //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_ESF_STATUS_V2_GROUP1) == 4, "");

// clang-format off
static constexpr uint8_t     UBX_ESF_STATUS_VERSION(const uint8_t* msg)                             { return msg[UBX_HEAD_SIZE + 4]; }  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_VERSION                                              = 0x02;  //!< @todo documentation
static constexpr std::size_t UBX_ESF_STATUS_V2_MIN_SIZE                                             = sizeof(UBX_ESF_STATUS_V2_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr std::size_t UBX_ESF_STATUS_V2_SIZE(const uint8_t* msg)                             { return  //!< @todo documentation
    sizeof(UBX_ESF_STATUS_V2_GROUP0) + UBX_FRAME_SIZE + (((uint8_t *)(msg))[UBX_HEAD_SIZE + 15] * sizeof(UBX_ESF_STATUS_V2_GROUP1)); }  //!< @todo documentation
static constexpr double      UBX_ESF_STATUS_V2_ITOW_SCALE                                           = 1e-3;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS1_WTINITSTATUS(const uint8_t initStatus1)  { return initStatus1 & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS1_WTINITSTATUS_OFF                         = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS1_WTINITSTATUS_INITALIZING                 = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS1_WTINITSTATUS_INITIALIZED                 = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS1_MNTALGSTATUS(const uint8_t initStatus1)  { return (initStatus1 >> 2) & 0x07; }  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS1_MNTALGSTATUS_OFF                         = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS1_MNTALGSTATUS_INITALIZING                 = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS1_MNTALGSTATUS_INITIALIZED1                = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS1_MNTALGSTATUS_INITIALIZED2                = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS1_INSINITSTATUS(const uint8_t initStatus1) { return (initStatus1 >> 5) & 0x07; }  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS1_INSINITSTATUS_OFF                        = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS1_INSINITSTATUS_INITALIZING                = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS1_INSINITSTATUS_INITIALIZED1               = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS1_INSINITSTATUS_INITIALIZED2               = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS2_IMUINITSTATUS(const uint8_t initStatus2) { return initStatus2 & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS2_IMUINITSTATUS_OFF                        = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS2_IMUINITSTATUS_INITALIZING                = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS2_IMUINITSTATUS_INITIALIZED1               = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_INITSTATUS2_IMUINITSTATUS_INITIALIZED2               = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_FUSIONMODE_INIT                                      = 0x00;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_FUSIONMODE_FUSION                                    = 0x01;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_FUSIONMODE_SUSPENDED                                 = 0x02;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_FUSIONMODE_DISABLED                                  = 0x03;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_SENSSTATUS1_TYPE(const uint8_t sensStatus1)          { return sensStatus1 & 0x3f; } //!< same enum as UBX-ESF-MEAS.dataType it seems  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_SENSSTATUS1_USED                                     = 0x40;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_SENSSTATUS1_READY                                    = 0x80;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_SENSSTATUS2_CALIBSTATUS(const uint8_t sensStatus2)   { return sensStatus2 & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_SENSSTATUS2_CALIBSTATUS_NOTCALIB                     = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_SENSSTATUS2_CALIBSTATUS_CALIBRATING                  = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_SENSSTATUS2_CALIBSTATUS_CALIBRATED1                  = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_SENSSTATUS2_CALIBSTATUS_CALIBRATED2                  = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_SENSSTATUS2_TIMESTATUS(const uint8_t sensStatus2)    { return (sensStatus2 >> 2) & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_SENSSTATUS2_TIMESTATUS_NODATA                        = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_SENSSTATUS2_TIMESTATUS_FIRSTBYTE                     = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_SENSSTATUS2_TIMESTATUS_EVENT                         = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_SENSSTATUS2_TIMESTATUS_TIMETAG                       = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_FAULTS_BADMEAS                                       = 0x01;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_FAULTS_BADTTAG                                       = 0x02;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_FAULTS_MISSINGMEAS                                   = 0x04;  //!< @todo documentation
static constexpr uint8_t     UBX_ESF_STATUS_V2_FAULTS_NOISYMEAS                                     = 0x08;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-MON-COMMS message
 * @{
 */

//! UBX-MON-COMMS (version 0, output) payload head
struct UBX_MON_COMMS_V0_GROUP0  // clang-format off
{
    uint8_t  version;        //!< @todo documentation
    uint8_t  nPorts;         //!< @todo documentation
    uint8_t  txErrors;       //!< @todo documentation
    uint8_t  reserved0;      //!< @todo documentation
    uint8_t  protIds[4];     //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_MON_COMMS_V0_GROUP0) == 8, "");

//! UBX-MON-COMMS (version 0, output) payload repeated group
struct UBX_MON_COMMS_V0_GROUP1  // clang-format off
{
    // Similar to UBX-MON-HW3.pinId this seems to be made up of two values actually:
    // - a port ID (probably same enum as in u-center's UBX-CFG-PRT: 0 = I2C, 1 = UART1, 2 = UART2, 3 = USB, 4 = SPI)
    // - a "bank" of ports (0, 1, ...)
    // u-center only shows some of the ports it seems (why?!)
    uint16_t portId;         //!< @todo documentation
    uint16_t txPending;      //!< @todo documentation
    uint32_t txBytes;        //!< @todo documentation
    uint8_t  txUsage;        //!< @todo documentation
    uint8_t  txPeakUsage;    //!< @todo documentation
    uint16_t rxPending;      //!< @todo documentation
    uint32_t rxBytes;        //!< @todo documentation
    uint8_t  rxUsage;        //!< @todo documentation
    uint8_t  rxPeakUsage;    //!< @todo documentation
    uint16_t overrunErrors;  //!< @todo documentation
    uint16_t msgs[4];        //!< @todo documentation
    uint8_t  reserved1[8];   //!< @todo documentation
    uint32_t skipped;        //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_MON_COMMS_V0_GROUP1) == 40, "");

// clang-format off
static constexpr uint8_t     UBX_MON_COMMS_VERSION(const uint8_t* msg)                              { return msg[UBX_HEAD_SIZE]; }  //!< @todo documentation
static constexpr uint8_t     UBX_MON_COMMS_V0_VERSION                                               = 0x00;  //!< @todo documentation
static constexpr std::size_t UBX_MON_COMMS_V0_MIN_SIZE                                              = sizeof(UBX_MON_COMMS_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr std::size_t UBX_MON_COMMS_V0_SIZE(const uint8_t* msg)                              { return  //!< @todo documentation
    sizeof(UBX_MON_COMMS_V0_GROUP0) + UBX_FRAME_SIZE + (((uint8_t *)(msg))[UBX_HEAD_SIZE + 1] * sizeof(UBX_MON_COMMS_V0_GROUP1)); }  //!< @todo documentation
static constexpr bool        UBX_MON_COMMS_V0_TXERRORS_MEM(const uint8_t txErrors)                  { return (txErrors & 0x01) == 0x01; }  //!< @todo documentation
static constexpr bool        UBX_MON_COMMS_V0_TXERRORS_ALLOC(const uint8_t txErrors)                { return (txErrors & 0x02) == 0x02; }  //!< @todo documentation
static constexpr uint8_t     UBX_MON_COMMS_V0_TXERRORS_OUTPUTPORT(const uint8_t txErrors)           { return (txErrors >> 3) & 0x7; }  //!< @todo documentation
static constexpr uint8_t     UBX_MON_COMMS_V0_TXERRORS_OUTPUTPORT_NA                                = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_COMMS_V0_TXERRORS_OUTPUTPORT_I2C                               = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_COMMS_V0_TXERRORS_OUTPUTPORT_UART1                             = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_COMMS_V0_TXERRORS_OUTPUTPORT_UART2                             = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_COMMS_V0_TXERRORS_OUTPUTPORT_USB                               = 4;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_COMMS_V0_TXERRORS_OUTPUTPORT_SPI                               = 5;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_COMMS_V0_PROTIDS_UBX                                           = 0x00;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_COMMS_V0_PROTIDS_NMEA                                          = 0x01;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_COMMS_V0_PROTIDS_RAW                                           = 0x03; //!< probably.. see UBX-MON-MSGPP
static constexpr uint8_t     UBX_MON_COMMS_V0_PROTIDS_RTCM2                                         = 0x02;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_COMMS_V0_PROTIDS_RTCM3                                         = 0x05;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_COMMS_V0_PROTIDS_SPARTN                                        = 0x06;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_COMMS_V0_PROTIDS_OTHER                                         = 0xff;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-MON-HW message
 * @{
 */

//! UBX-MON-HW (version 0, output)
struct UBX_MON_HW_V0_GROUP0  // clang-format off
{
    uint32_t pinSel;         //!< @todo documentation
    uint32_t pinBank;        //!< @todo documentation
    uint32_t pinDir;         //!< @todo documentation
    uint32_t pinVal;         //!< @todo documentation
    uint16_t noisePerMS;     //!< @todo documentation
    uint16_t agcCnt;         //!< @todo documentation
    uint8_t  aStatus;        //!< @todo documentation
    uint8_t  aPower;         //!< @todo documentation
    uint8_t  flags;          //!< @todo documentation
    uint8_t  reserved0;      //!< @todo documentation
    uint32_t usedMask;       //!< @todo documentation
    uint8_t  VP[17];         //!< @todo documentation
    uint8_t  jamInd;         //!< @todo documentation
    uint8_t  reserved1[2];   //!< @todo documentation
    uint32_t pinIrq;         //!< @todo documentation
    uint32_t pullH;          //!< @todo documentation
    uint32_t pullL;          //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_MON_HW_V0_GROUP0) == 60, "");

// clang-format off
static constexpr std::size_t UBX_MON_HW_V0_SIZE                                                     = sizeof(UBX_MON_HW_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_FLAGS_RTCCALIB                                           = 0x01;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_FLAGS_SAFEBOOT                                           = 0x02;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_FLAGS_JAMMINGSTATE(const uint8_t flags)                  { return (flags >> 2) & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_FLAGS_JAMMINGSTATE_UNKNOWN                               = 0x00;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_FLAGS_JAMMINGSTATE_OK                                    = 0x01;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_FLAGS_JAMMINGSTATE_WARNING                               = 0x02;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_FLAGS_JAMMINGSTATE_CRITICAL                              = 0x03;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_FLAGS_XTALABSENT                                         = 0x10;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_ASTATUS_INIT                                             = 0x00;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_ASTATUS_UNKNOWN                                          = 0x01;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_ASTATUS_OK                                               = 0x02;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_ASTATUS_SHORT                                            = 0x03;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_ASTATUS_OPEN                                             = 0x04;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_APOWER_OFF                                               = 0x00;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_APOWER_ON                                                = 0x01;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_APOWER_UNKNOWN                                           = 0x02;  //!< @todo documentation
static constexpr uint16_t    UBX_MON_HW_V0_NOISEPERMS_MAX                                           = 200;   //!< This seems to be what u-center uses..
static constexpr uint16_t    UBX_MON_HW_V0_AGCCNT_MAX                                               = 8191;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW_V0_JAMIND_MAX                                               = 255;   //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-MON-HW2 message
 * @{
 */

//! UBX-MON-HW (version 0, output) payload
struct UBX_MON_HW2_V0_GROUP0  // clang-format off
{
    int8_t   ofsI;           //!< @todo documentation
    uint8_t  magI;           //!< @todo documentation
    int8_t   ofsQ;           //!< @todo documentation
    uint8_t  magQ;           //!< @todo documentation
    uint8_t  cfgSource;      //!< @todo documentation
    uint8_t  reserved0[3];   //!< @todo documentation
    uint32_t lowLevCfg;      //!< @todo documentation
    uint8_t  reserved1[8];   //!< @todo documentation
    uint32_t postStatus;     //!< @todo documentation
    uint8_t  reserved2[3];   //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_MON_HW2_V0_GROUP0) == 28, "");

// clang-format off
static constexpr std::size_t UBX_MON_HW2_V0_SIZE                                                    = sizeof(UBX_MON_HW2_V0_GROUP0) + UBX_FRAME_SIZE;   //!< @todo documentation+ UBX_FRAME_SIZE
static constexpr uint8_t     UBX_MON_HW2_V0_CFGSOURCE_ROM                                           = 114;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW2_V0_CFGSOURCE_OTP                                           = 111;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW2_V0_CFGSOURCE_PIN                                           = 112;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW2_V0_CFGSOURCE_FLASH                                         = 102;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-MON-HW3 message
 * @{
 */

//! UBX-MON-HW3 (version 0, output) payload
struct UBX_MON_HW3_V0_GROUP0  // clang-format off
{
    uint8_t  version;        //!< @todo documentation
    uint8_t  nPins;          //!< @todo documentation
    uint8_t  flags;          //!< @todo documentation
    char     hwVersion[10];  //!< @todo documentation
    uint8_t  reserved0[9];   //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_MON_HW3_V0_GROUP0) == 22, "");

//! UBX-MON-HW3 (version 0, output) payload
struct UBX_PACKED UBX_MON_HW3_V0_GROUP1  // clang-format off
{
    uint16_t pinId;          //!< @todo documentation // u-center shows ((pinId & 0xff00) >> 8) | ((pinId & 0x00ff) << 8), seems to be: (pinNo << 8) | pinBank
    uint16_t pinMask;        //!< @todo documentation
    uint8_t  VP;             //!< @todo documentation
    uint8_t  reserved1;      //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_MON_HW3_V0_GROUP1) == 6, "");

// clang-format off
static constexpr uint8_t     UBX_MON_HW3_VERSION(const uint8_t* msg)                                { return msg[UBX_HEAD_SIZE]; }  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW3_V0_VERSION                                                 = 0x00;  //!< @todo documentation
static constexpr std::size_t UBX_MON_HW3_V0_MIN_SIZE                                                = sizeof(UBX_MON_HW3_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr std::size_t UBX_MON_HW3_V0_SIZE(const uint8_t* msg)                                   { return  //!< @todo documentation
    sizeof(UBX_MON_HW3_V0_GROUP0) + UBX_FRAME_SIZE + (((uint8_t*)(msg))[UBX_HEAD_SIZE + 1] * sizeof(UBX_MON_HW3_V0_GROUP1)); }  //!< @todo documentation
static constexpr bool        UBX_MON_HW3_V0_FLAGS_RTCCALIB(const uint8_t flags)                     { return (flags & 0x01) == 0x01; }  //!< @todo documentation
static constexpr bool        UBX_MON_HW3_V0_FLAGS_SAFEBOOT(const uint8_t flags)                     { return (flags & 0x02) == 0x02; }  //!< @todo documentation
static constexpr bool        UBX_MON_HW3_V0_FLAGS_XTALABSENT(const uint8_t flags)                   { return (flags & 0x04) == 0x04; }  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW3_V0_PINMASK_PERIPHPIO(const uint16_t pinMask)               { return pinMask & 0x01; }  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW3_V0_PINMASK_PERIPHPIO_PERIPH                                = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW3_V0_PINMASK_PERIPHPIO_PIO                                   = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW3_V0_PINMASK_PINBANK(const uint16_t pinMask)                 { return (pinMask >> 1) & 0x01; }  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW3_V0_PINMASK_PINBANK_A                                       = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW3_V0_PINMASK_PINBANK_B                                       = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW3_V0_PINMASK_PINBANK_C                                       = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW3_V0_PINMASK_PINBANK_D                                       = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW3_V0_PINMASK_PINBANK_E                                       = 4;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW3_V0_PINMASK_PINBANK_F                                       = 5;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW3_V0_PINMASK_PINBANK_G                                       = 6;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW3_V0_PINMASK_PINBANK_H                                       = 7;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW3_V0_PINMASK_DIRECTION(const uint16_t pinMask)               { return (pinMask >> 4) & 0x01; }  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW3_V0_PINMASK_DIRECTION_OUT                                   = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_HW3_V0_PINMASK_DIRECTION_IN                                    = 1;  //!< @todo documentation
static constexpr bool        UBX_MON_HW3_V0_PINMASK_VALUE(const uint16_t pinMask)                   { return (pinMask & 0x0020) == 0x0020; }  //!< @todo documentation
static constexpr bool        UBX_MON_HW3_V0_PINMASK_VPMANAGER(const uint16_t pinMask)               { return (pinMask & 0x0040) == 0x0040; }  //!< @todo documentation
static constexpr bool        UBX_MON_HW3_V0_PINMASK_PIOIRQ(const uint16_t pinMask)                  { return (pinMask & 0x0080) == 0x0080; }  //!< @todo documentation
static constexpr bool        UBX_MON_HW3_V0_PINMASK_PIOPULLHIGH(const uint16_t pinMask)             { return (pinMask & 0x0100) == 0x0100; }  //!< @todo documentation
static constexpr bool        UBX_MON_HW3_V0_PINMASK_PIOPULLLOW(const uint16_t pinMask)              { return (pinMask & 0x0200) == 0x0200; }  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-MON-RF message
 * @{
 */

//! UBX-MON-RF (version 0, output) payload head
struct UBX_MON_RF_V0_GROUP0  // clang-format off
{
    uint8_t  version;        //!< @todo documentation
    uint8_t  nBlocks;        //!< @todo documentation
    uint8_t  reserved[2];    //!< @todo documentation
};
// clang-format on

static_assert(sizeof(UBX_MON_RF_V0_GROUP0) == 4, "");

//! UBX-MON-RF (version 0, output) payload repeated group
struct UBX_MON_RF_V0_GROUP1  // clang-format off
{
    uint8_t  blockId;        //!< @todo documentation
    uint8_t  flags;          //!< @todo documentation
    uint8_t  antStatus;      //!< @todo documentation
    uint8_t  antPower;       //!< @todo documentation
    uint32_t postStatus;     //!< @todo documentation
    uint8_t  reserved1[4];   //!< @todo documentation
    uint16_t noisePerMS;     //!< @todo documentation
    uint16_t agcCnt;         //!< @todo documentation
    uint8_t  jamInd;         //!< @todo documentation
    int8_t   ofsI;           //!< @todo documentation
    uint8_t  magI;           //!< @todo documentation
    int8_t   ofsQ;           //!< @todo documentation
    uint8_t  magQ;           //!< @todo documentation
    uint8_t  reserved2[3];   //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_MON_RF_V0_GROUP1) == 24, "");

// clang-format off
static constexpr uint8_t     UBX_MON_RF_VERSION(const uint8_t* msg)                                 { return msg[UBX_HEAD_SIZE]; }  //!< @todo documentation
static constexpr uint8_t     UBX_MON_RF_V0_VERSION                                                  = 0x00;  //!< @todo documentation
static constexpr std::size_t UBX_MON_RF_V0_MIN_SIZE                                                 = sizeof(UBX_MON_RF_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr std::size_t UBX_MON_RF_V0_SIZE(const uint8_t* msg)                                 { return
    sizeof(UBX_MON_RF_V0_GROUP0) + UBX_FRAME_SIZE + (((uint8_t *)(msg))[UBX_HEAD_SIZE + 1] * sizeof(UBX_MON_RF_V0_GROUP1)); }  //!< @todo documentation
static constexpr uint8_t     UBX_MON_RF_V0_FLAGS_JAMMINGSTATE(const uint8_t flags)                  { return flags & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_MON_RF_V0_FLAGS_JAMMINGSTATE_UNKN                                  = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_RF_V0_FLAGS_JAMMINGSTATE_OK                                    = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_RF_V0_FLAGS_JAMMINGSTATE_WARN                                  = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_RF_V0_FLAGS_JAMMINGSTATE_CRIT                                  = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_RF_V0_ANTSTATUS_INIT                                           = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_RF_V0_ANTSTATUS_DONTKNOW                                       = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_RF_V0_ANTSTATUS_OK                                             = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_RF_V0_ANTSTATUS_SHORT                                          = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_RF_V0_ANTSTATUS_OPEN                                           = 4;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_RF_V0_ANTPOWER_OFF                                             = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_RF_V0_ANTPOWER_ON                                              = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_RF_V0_ANTPOWER_DONTKNOW                                        = 2;  //!< @todo documentation
static constexpr uint16_t    UBX_MON_RF_V0_NOISEPERMS_MAX                                           = 200; //!< This seems to be what u-center uses..
static constexpr uint16_t    UBX_MON_RF_V0_AGCCNT_MAX                                               = 8191;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_RF_V0_JAMIND_MAX                                               = 255;  //!< @todo documentation
// clang-format on

///@}

// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-MON-SPAN message
 * @{
 */

//! UBX-MON-RF (version 0, output) payload head
struct UBX_MON_SPAN_V0_GROUP0  // clang-format off
{
    uint8_t  version;        //!< @todo documentation
    uint8_t  numRfBlocks;    //!< @todo documentation
    uint8_t  reserved[2];    //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_MON_SPAN_V0_GROUP0) == 4, "");

static constexpr std::size_t UBX_MON_SPAN_V0_NUM_BINS = 255;  //!< Number of bins

//! UBX-MON-RF (version 0, output) payload repeated group
struct UBX_MON_SPAN_V0_GROUP1  // clang-format off
{
    uint8_t  spectrum[UBX_MON_SPAN_V0_NUM_BINS];  //!< @todo documentation
    uint32_t span;           //!< @todo documentation
    uint32_t res;            //!< @todo documentation
    uint32_t center;         //!< @todo documentation
    uint8_t  pga;            //!< @todo documentation
    uint8_t  reserved[3];    //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_MON_SPAN_V0_GROUP1) == 272, "");

// clang-format off
static constexpr uint8_t     UBX_MON_SPAN_VERSION(const uint8_t* msg)                               { return msg[UBX_HEAD_SIZE]; }  //!< @todo documentation
static constexpr uint8_t     UBX_MON_SPAN_V0_VERSION                                                = 0x00;  //!< @todo documentation
static constexpr std::size_t UBX_MON_SPAN_V0_MIN_SIZE                                               = sizeof(UBX_MON_SPAN_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr std::size_t UBX_MON_SPAN_V0_SIZE(const uint8_t* msg)                               { return
    sizeof(UBX_MON_SPAN_V0_GROUP0) + UBX_FRAME_SIZE + (((uint8_t *)(msg))[UBX_HEAD_SIZE + 1] * sizeof(UBX_MON_SPAN_V0_GROUP1)); }  //!< @todo documentation
static constexpr double      UBX_MON_SPAN_BIN_CENT_FREQ(const uint32_t center, const uint32_t span, const int ix)  { return
    (double)center + ((double)span * (((double)(ix) - 128.0) / 256.0)); }  //!< @todo documentation
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-MON-SYS message
 * @{
 */

//! UBX-MON-SYS (version 1, output) payload
struct UBX_MON_SYS_V1_GROUP0  // clang-format off
{
    uint8_t  msgVer;       //!< @todo documentation
    uint8_t  bootType;     //!< @todo documentation
    uint8_t  cpuLoad;      //!< @todo documentation
    uint8_t  cpuLoadMax;   //!< @todo documentation
    uint8_t  memUsage;     //!< @todo documentation
    uint8_t  memUsageMax;  //!< @todo documentation
    uint8_t  ioUsage;      //!< @todo documentation
    uint8_t  ioUsageMax;   //!< @todo documentation
    uint32_t runTime;      //!< @todo documentation
    uint16_t noticeCount;  //!< @todo documentation
    uint16_t warnCount;    //!< @todo documentation
    uint16_t errorCount;   //!< @todo documentation
    int8_t   tempValue;    //!< @todo documentation
    uint8_t  reserved[5];  //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_MON_SYS_V1_GROUP0) == 24, "");

// clang-format off
static constexpr uint8_t     UBX_MON_SYS_VERSION(const uint8_t* msg)                                { return msg[UBX_HEAD_SIZE]; }  //!< @todo documentation
static constexpr uint8_t     UBX_MON_SYS_V1_VERSION                                                 = 0x01;  //!< @todo documentation
static constexpr std::size_t UBX_MON_SYS_V1_SIZE                                                    = sizeof(UBX_MON_SYS_V1_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr uint8_t     UBX_MON_SYS_V1_BOOTTYPE_UNKNOWN                                        =  0;  //!< Boot type: unknown
static constexpr uint8_t     UBX_MON_SYS_V1_BOOTTYPE_COLDSTART                                      =  1;  //!< Boot type: cold Start
static constexpr uint8_t     UBX_MON_SYS_V1_BOOTTYPE_WATCHDOG                                       =  2;  //!< Boot type: watchdog
static constexpr uint8_t     UBX_MON_SYS_V1_BOOTTYPE_HWRESET                                        =  3;  //!< Boot type: hardware reset
static constexpr uint8_t     UBX_MON_SYS_V1_BOOTTYPE_HWBACKUP                                       =  4;  //!< Boot type: hardware backup
static constexpr uint8_t     UBX_MON_SYS_V1_BOOTTYPE_SWBACKUP                                       =  5;  //!< Boot type: software backup
static constexpr uint8_t     UBX_MON_SYS_V1_BOOTTYPE_SWRESET                                        =  6;  //!< Boot type: software reset
static constexpr uint8_t     UBX_MON_SYS_V1_BOOTTYPE_VIOFAIL                                        =  7;  //!< Boot type: VIO fail
static constexpr uint8_t     UBX_MON_SYS_V1_BOOTTYPE_VDDXFAIL                                       =  8;  //!< Boot type: VDD_X fail
static constexpr uint8_t     UBX_MON_SYS_V1_BOOTTYPE_VDDRFFAIL                                      =  9;  //!< Boot type: VDD_RF fail
static constexpr uint8_t     UBX_MON_SYS_V1_BOOTTYPE_VCOREHIGHFAIL                                  = 10;  //!< Boot type: V_CORE_HIGH fail
static constexpr uint8_t     UBX_MON_SYS_V1_BOOTTYPE_SYSTEMRESET                                    = 11;  //!< Boot type: system reset
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-MON-TEMP message
 * @{
 */

//! UBX-MON-TEMP (version 0, output) message payload (no docu available, but u-center shows it...)
struct UBX_MON_TEMP_V0_GROUP0  // clang-format off
{
    uint8_t  version;        //!< probably version.. @todo documentation
    uint8_t  reserved0[3];   //!< @todo documentation
    int16_t  temperature;    //!< @todo documentation
    uint8_t  unknown;        //!< unit? 1 = C? @todo documentation
    uint8_t  reserved1[5];   //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_MON_TEMP_V0_GROUP0) == 12, "");

// clang-format off
static constexpr uint8_t     UBX_MON_TEMP_VERSION(const uint8_t* msg)                               { return msg[UBX_HEAD_SIZE]; }  //!< @todo documentation
static constexpr uint8_t     UBX_MON_TEMP_V0_VERSION                                                = 0x00;  //!< @todo documentation
static constexpr std::size_t UBX_MON_TEMP_V0_SIZE                                                   = sizeof(UBX_MON_TEMP_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-MON-VER message
 * @{
 */
//! UBX-MON-VER (version 0, output) message payload header
struct UBX_MON_VER_V0_GROUP0  // clang-format off
{
    char swVersion[30];      //!< @todo documentation
    char hwVersion[10];      //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_MON_VER_V0_GROUP0) == 40, "");

//! UBX-MON-VER (version 0, output) optional repeated field
struct UBX_MON_VER_V0_GROUP1  // clang-format off
{
    char extension[30];      //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_MON_VER_V0_GROUP1) == 30, "");

// clang-format off
static constexpr uint8_t     UBX_MON_VER_V0_MIN_SIZE                                                = sizeof(UBX_MON_VER_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-ATT message
 * @{
 */

//! UBX-NAV-ATT (version 0, output) payload
struct UBX_NAV_ATT_V0_GROUP0  // clang-format off
{
    uint32_t iTOW;           //!< @todo documentation
    uint8_t  version;        //!< @todo documentation
    uint8_t  reserved0[3];   //!< @todo documentation
    int32_t  roll;           //!< @todo documentation
    int32_t  pitch;          //!< @todo documentation
    int32_t  heading;        //!< @todo documentation
    uint32_t accRoll;        //!< @todo documentation
    uint32_t accPitch;       //!< @todo documentation
    uint32_t accHeading;     //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_ATT_V0_GROUP0) == 32, "");

// clang-format off
static constexpr uint8_t     UBX_NAV_ATT_VERSION(const uint8_t* msg)                                { return msg[UBX_HEAD_SIZE + sizeof(uint32_t)]; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_ATT_V0_VERSION                                                 = 0x00;  //!< @todo documentation
static constexpr std::size_t UBX_NAV_ATT_V0_SIZE                                                    = sizeof(UBX_NAV_ATT_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_ATT_V0_ITOW_SCALE                                              = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_ATT_V0_RPH_SCALING                                             = 1e-5;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-CLOCK message
 * @{
 */

//! UBX-NAV-CLOCK payload
struct UBX_NAV_CLOCK_V0_GROUP0  // clang-format off
{
    uint32_t iTow;           //!< @todo documentation
    int32_t  clkB;           //!< @todo documentation
    int32_t  clkD;           //!< @todo documentation
    uint32_t tAcc;           //!< @todo documentation
    uint32_t fAcc;           //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_CLOCK_V0_GROUP0) == 20, "");

// clang-format off
static constexpr std::size_t UBX_NAV_CLOCK_V0_SIZE                                                  = sizeof(UBX_NAV_CLOCK_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_CLOCK_V0_CLKB_SCALE                                            = 1e-9;  //!< @todo documentation
static constexpr double      UBX_NAV_CLOCK_V0_CLKD_SCALE                                            = 1e-9;  //!< @todo documentation
static constexpr double      UBX_NAV_CLOCK_V0_TACC_SCALE                                            = 1e-9;  //!< @todo documentation
static constexpr double      UBX_NAV_CLOCK_V0_FACC_SCALE                                            = 1e-12;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-COV message
 * @{
 */

//! UBX-NAV-COV (version 0, output) payload head
struct UBX_NAV_COV_V0_GROUP0  // clang-format off
{
    uint32_t iTOW;           //!< @todo documentation
    uint8_t  version;        //!< @todo documentation
    uint8_t  posCovValid;    //!< @todo documentation
    uint8_t  velCovValid;    //!< @todo documentation
    uint8_t  reserved[9];    //!< @todo documentation
    float    posCovNN;       //!< @todo documentation
    float    posCovNE;       //!< @todo documentation
    float    posCovND;       //!< @todo documentation
    float    posCovEE;       //!< @todo documentation
    float    posCovED;       //!< @todo documentation
    float    posCovDD;       //!< @todo documentation
    float    velCovNN;       //!< @todo documentation
    float    velCovNE;       //!< @todo documentation
    float    velCovND;       //!< @todo documentation
    float    velCovEE;       //!< @todo documentation
    float    velCovED;       //!< @todo documentation
    float    velCovDD;       //!< @todo documentation
};
// clang-format on

static_assert(sizeof(UBX_NAV_COV_V0_GROUP0) == 64, "");

// clang-format off
static constexpr uint8_t     UBX_NAV_COV_VERSION(const uint8_t* msg)                                { return msg[UBX_HEAD_SIZE + sizeof(uint32_t)]; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_COV_V0_VERSION                                                 = 0x00;  //!< @todo documentation
static constexpr std::size_t UBX_NAV_COV_V0_SIZE                                                    = sizeof(UBX_NAV_COV_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_COV_V0_ITOW_SCALE                                              = 1e-3;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-DOP message
 * @{
 */

//! UBX-NAV-DOP payload
struct UBX_PACKED UBX_NAV_DOP_V0_GROUP0
{  // clang-format off
    uint32_t iTOW;           //!< @todo documentation
    uint16_t gDOP;           //!< @todo documentation
    uint16_t pDOP;           //!< @todo documentation
    uint16_t tDOP;           //!< @todo documentation
    uint16_t vDOP;           //!< @todo documentation
    uint16_t hDOP;           //!< @todo documentation
    uint16_t nDOP;           //!< @todo documentation
    uint16_t eDOP;           //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_DOP_V0_GROUP0) == 18, "");

// clang-format off
static constexpr std::size_t UBX_NAV_DOP_V0_SIZE                                                    = sizeof(UBX_NAV_DOP_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_DOP_V0_XDOP_SCALE                                              = 1e-2;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-EELL message
 * @{
 */

//! UBX-NAV-EELL (version 0, output) payload head
struct UBX_NAV_EELL_V0_GROUP0  // clang-format off
{
    uint32_t iTOW;              //!< @todo documentation
    uint8_t  version;           //!< @todo documentation
    uint8_t  reserved;          //!< @todo documentation
    uint16_t errEllipseOrient;  //!< @todo documentation
    uint32_t errEllipseMajor;   //!< @todo documentation
    uint32_t errEllipseMinor;   //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_EELL_V0_GROUP0) == 16, "");

// clang-format off
static constexpr uint8_t     UBX_NAV_EELL_VERSION(const uint8_t* msg)                               { return msg[UBX_HEAD_SIZE]; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_EELL_V0_VERSION                                                = 0x00;  //!< @todo documentation
static constexpr std::size_t UBX_NAV_EELL_V0_SIZE                                                   = sizeof(UBX_NAV_EELL_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_EELL_V0_ITOW_SCALE                                             = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_EELL_V0_ELLIPSEORIENT_SCALE                                    = 1e-2;  //!< @todo documentation
static constexpr double      UBX_NAV_EELL_V0_ELLIPSEMAJOR_SCALE                                     = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_EELL_V0_ELLIPSEMINOR_SCALE                                     = 1e-3;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-EOE message
 * @{
 */

//! UBX-NAV-EOE (version 0, output) payload
struct UBX_NAV_EOE_V0_GROUP0
{  // clang-format off
    uint32_t iTOW;           //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_EOE_V0_GROUP0) == 4, "");

// clang-format off
static constexpr std::size_t UBX_NAV_EOE_V0_SIZE                                                    = sizeof(UBX_NAV_EOE_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_EOE_V0_ITOW_SCALE                                              = 1e-3;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-HPPOSECEF message
 * @{
 */

//! UBX-NAV-HPPOSECEF (version 0, output) payload
struct UBX_NAV_HPPOSECEF_V0_GROUP0
{  // clang-format off
    uint8_t  version;        //!< @todo documentation
    uint8_t  reserved[3];    //!< @todo documentation
    uint32_t iTOW;           //!< @todo documentation
    int32_t  ecefX;          //!< @todo documentation
    int32_t  ecefY;          //!< @todo documentation
    int32_t  ecefZ;          //!< @todo documentation
    int8_t   ecefXHp;        //!< @todo documentation
    int8_t   ecefYHp;        //!< @todo documentation
    int8_t   ecefZHp;        //!< @todo documentation
    uint8_t  flags;          //!< @todo documentation
    uint32_t pAcc;           //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_HPPOSECEF_V0_GROUP0) == 28, "");

// clang-format off
static constexpr uint8_t     UBX_NAV_HPPOSECEF_VERSION(const uint8_t* msg)                          { return msg[UBX_HEAD_SIZE]; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_HPPOSECEF_V0_VERSION                                           = 0x00;  //!< @todo documentation
static constexpr std::size_t UBX_NAV_HPPOSECEF_V0_SIZE                                              = sizeof(UBX_NAV_HPPOSECEF_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_HPPOSECEF_V0_ITOW_SCALE                                        = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_HPPOSECEF_V0_ECEF_XYZ_SCALE                                    = 1e-2;  //!< @todo documentation
static constexpr double      UBX_NAV_HPPOSECEF_V0_ECEF_XYZ_HP_SCALE                                 = 1e-4;  //!< @todo documentation
static constexpr double      UBX_NAV_HPPOSECEF_V0_PACC_SCALE                                        = 1e-4;  //!< @todo documentation
static constexpr bool        UBX_NAV_HPPOSECEF_V0_FLAGS_INVALIDECEF(const uint8_t flags)            { return (flags & 0x01) == 0x01; }  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-HPPOSLLH message
 * @{
 */

//! UBX-NAV-HPPOSLLH (version 0) payload
struct UBX_NAV_HPPOSLLH_V0_GROUP0  // clang-format off
{
    uint8_t  version;        //!< @todo documentation
    uint8_t  reserved[2];    //!< @todo documentation
    uint8_t  flags;          //!< @todo documentation
    uint32_t iTOW;           //!< @todo documentation
    int32_t  lon;            //!< @todo documentation
    int32_t  lat;            //!< @todo documentation
    int32_t  height;         //!< @todo documentation
    int32_t  hMSL;           //!< @todo documentation
    int8_t   lonHp;          //!< @todo documentation
    int8_t   latHp;          //!< @todo documentation
    int8_t   heightHp;       //!< @todo documentation
    int8_t   hMSLHp;         //!< @todo documentation
    uint32_t hAcc;           //!< @todo documentation
    uint32_t vAcc;           //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_HPPOSLLH_V0_GROUP0) == 36, "");

// clang-format off
static constexpr uint8_t     UBX_NAV_HPPOSLLH_VERSION(const uint8_t* msg)                           { return msg[UBX_HEAD_SIZE]; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_HPPOSLLH_V0_VERSION                                            = 0x00;  //!< @todo documentation
static constexpr std::size_t UBX_NAV_HPPOSLLH_V0_SIZE                                               = sizeof(UBX_NAV_HPPOSLLH_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_HPPOSLLH_V0_FLAGS_INVALIDLLH                                   = 0x01;  //!< @todo documentation
static constexpr double      UBX_NAV_HPPOSLLH_V0_ITOW_SCALE                                         = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_HPPOSLLH_V0_LL_SCALE                                           = 1e-7;  //!< @todo documentation
static constexpr double      UBX_NAV_HPPOSLLH_V0_H_SCALE                                            = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_HPPOSLLH_V0_LL_HP_SCALE                                        = 1e-9;  //!< @todo documentation
static constexpr double      UBX_NAV_HPPOSLLH_V0_H_HP_SCALE                                         = 1e-4;  //!< @todo documentation
static constexpr double      UBX_NAV_HPPOSLLH_V0_ACC_SCALE                                          = 1e-4;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-POSECEF message
 * @{
 */

//! UBX-NAV-POSECEF (version 0, output) payload
struct UBX_NAV_POSECEF_V0_GROUP0  // clang-format off
{
    uint32_t iTOW;           //!< @todo documentation
    int32_t  ecefX;          //!< @todo documentation
    int32_t  ecefY;          //!< @todo documentation
    int32_t  ecefZ;          //!< @todo documentation
    uint32_t pAcc;           //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_POSECEF_V0_GROUP0) == 20, "");

// clang-format off
static constexpr std::size_t UBX_NAV_POSECEF_V0_SIZE                                                = sizeof(UBX_NAV_POSECEF_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_POSECEF_V0_ECEF_XYZ_SCALE                                      = 1e-2;  //!< @todo documentation
static constexpr double      UBX_NAV_POSECEF_V0_PACC_SCALE                                          = 1e-2;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-PVT message
 * @{
 */

//! UBX-NAV-PVT (version 1, output) payload
struct UBX_NAV_PVT_V1_GROUP0  // clang-format off
 {
    uint32_t iTOW;           //!< @todo documentation
    uint16_t year;           //!< @todo documentation
    uint8_t  month;          //!< @todo documentation
    uint8_t  day;            //!< @todo documentation
    uint8_t  hour;           //!< @todo documentation
    uint8_t  min;            //!< @todo documentation
    uint8_t  sec;            //!< @todo documentation
    uint8_t  valid;          //!< @todo documentation
    uint32_t tAcc;           //!< @todo documentation
    int32_t  nano;           //!< @todo documentation
    uint8_t  fixType;        //!< @todo documentation
    uint8_t  flags;          //!< @todo documentation
    uint8_t  flags2;         //!< @todo documentation
    uint8_t  numSV;          //!< @todo documentation
    int32_t  lon;            //!< @todo documentation
    int32_t  lat;            //!< @todo documentation
    int32_t  height;         //!< @todo documentation
    int32_t  hMSL;           //!< @todo documentation
    uint32_t hAcc;           //!< @todo documentation
    uint32_t vAcc;           //!< @todo documentation
    int32_t  velN;           //!< @todo documentation
    int32_t  velE;           //!< @todo documentation
    int32_t  velD;           //!< @todo documentation
    int32_t  gSpeed;         //!< @todo documentation
    int32_t  headMot;        //!< @todo documentation
    uint32_t sAcc;           //!< @todo documentation
    uint32_t headAcc;        //!< @todo documentation
    uint16_t pDOP;           //!< @todo documentation
    uint8_t  flags3;         //!< @todo documentation
    uint8_t  reserved[5];    //!< @todo documentation
    int32_t  headVeh;        //!< @todo documentation
    int16_t  magDec;         //!< @todo documentation
    uint16_t magAcc;         //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_PVT_V1_GROUP0) == 92, "");

// clang-format off
static constexpr std::size_t UBX_NAV_PVT_V1_SIZE                                                    = sizeof(UBX_NAV_PVT_V1_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_PVT_V1_ITOW_SCALE                                              = 1e-3;  //!< @todo documentation
static constexpr bool        UBX_NAV_PVT_V1_VALID_VALIDDATE(const uint8_t valid)                    { return (valid & 0x01) == 0x01; }  //!< @todo documentation
static constexpr bool        UBX_NAV_PVT_V1_VALID_VALIDTIME(const uint8_t valid)                    { return (valid & 0x02) == 0x02; }  //!< @todo documentation
static constexpr bool        UBX_NAV_PVT_V1_VALID_FULLYRESOLVED(const uint8_t valid)                { return (valid & 0x04) == 0x04; }  //!< @todo documentation
static constexpr bool        UBX_NAV_PVT_V1_VALID_VALIDMAG(const uint8_t valid)                     { return (valid & 0x08) == 0x08; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_PVT_V1_FIXTYPE_NOFIX                                           = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_PVT_V1_FIXTYPE_DRONLY                                          = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_PVT_V1_FIXTYPE_2D                                              = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_PVT_V1_FIXTYPE_3D                                              = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_PVT_V1_FIXTYPE_3D_DR                                           = 4;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_PVT_V1_FIXTYPE_TIME                                            = 5;  //!< @todo documentation
static constexpr bool        UBX_NAV_PVT_V1_FLAGS_GNSSFIXOK(const uint8_t flags)                    { return (flags & 0x01) == 0x01; }  //!< @todo documentation
static constexpr bool        UBX_NAV_PVT_V1_FLAGS_DIFFSOLN(const uint8_t flags)                     { return (flags & 0x02) == 0x02; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_PVT_V1_FLAGS_CARRSOLN(const uint8_t flags)                     { return (flags >> 6) & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_PVT_V1_FLAGS_CARRSOLN_NO                                       = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_PVT_V1_FLAGS_CARRSOLN_FLOAT                                    = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_PVT_V1_FLAGS_CARRSOLN_FIXED                                    = 2;  //!< @todo documentation
static constexpr bool        UBX_NAV_PVT_V1_FLAGS2_CONFAVAIL(const uint8_t flags2)                  { return (flags2 & 0x20) == 0x20; }  //!< @todo documentation
static constexpr bool        UBX_NAV_PVT_V1_FLAGS2_CONFDATE(const uint8_t flags2)                   { return (flags2 & 0x40) == 0x40; }  //!< @todo documentation
static constexpr bool        UBX_NAV_PVT_V1_FLAGS2_CONFTIME(const uint8_t flags2)                   { return (flags2 & 0x80) == 0x80; }  //!< @todo documentation
static constexpr bool        UBX_NAV_PVT_V1_FLAGS3_INVALIDLLH(const uint8_t flags3)                 { return (flags3 & 0x01) == 0x01; }  //!< @todo documentation
static constexpr double      UBX_NAV_PVT_V1_LAT_SCALE                                               = 1e-7;  //!< @todo documentation
static constexpr double      UBX_NAV_PVT_V1_LON_SCALE                                               = 1e-7;  //!< @todo documentation
static constexpr double      UBX_NAV_PVT_V1_HEIGHT_SCALE                                            = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_PVT_V1_HACC_SCALE                                              = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_PVT_V1_VACC_SCALE                                              = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_PVT_V1_VELNED_SCALE                                            = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_PVT_V1_GSPEED_SCALE                                            = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_PVT_V1_HEADMOT_SCALE                                           = 1e-5;  //!< @todo documentation
static constexpr double      UBX_NAV_PVT_V1_SACC_SCALE                                              = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_PVT_V1_HEADACC_SCALE                                           = 1e-5;  //!< @todo documentation
static constexpr double      UBX_NAV_PVT_V1_PDOP_SCALE                                              = 1e-2;  //!< @todo documentation
static constexpr double      UBX_NAV_PVT_V1_TACC_SCALE                                              = 1e-9;  //!< @todo documentation
static constexpr double      UBX_NAV_PVT_V1_NANO_SCALE                                              = 1e-9;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-RELPOSNED message
 * @{
 */

//! UBX-NAV-RELPOSNED (version 1, output) payload
struct UBX_NAV_RELPOSNED_V1_GROUP0  // clang-format off
{
    uint8_t  version;        //!< @todo documentation
    uint8_t  reserved0;      //!< @todo documentation
    uint16_t refStationId;   //!< @todo documentation
    uint32_t iTOW;           //!< @todo documentation
    int32_t  relPosN;        //!< @todo documentation
    int32_t  relPosE;        //!< @todo documentation
    int32_t  relPosD;        //!< @todo documentation
    int32_t  relPosLength;   //!< @todo documentation
    int32_t  relPosHeading;  //!< @todo documentation
    uint8_t  reserved1[4];   //!< @todo documentation
    int8_t   relPosHPN;      //!< @todo documentation
    int8_t   relPosHPE;      //!< @todo documentation
    int8_t   relPosHPD;      //!< @todo documentation
    int8_t   relPosHPLength; //!< @todo documentation
    uint32_t accN;           //!< @todo documentation
    uint32_t accE;           //!< @todo documentation
    uint32_t accD;           //!< @todo documentation
    uint32_t accLength;      //!< @todo documentation
    uint32_t accHeading;     //!< @todo documentation
    uint8_t  reserved2[4];   //!< @todo documentation
    uint32_t flags;          //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_RELPOSNED_V1_GROUP0) == 64, "");

// clang-format off
static constexpr uint8_t     UBX_NAV_RELPOSNED_VERSION(const uint8_t* msg)                          { return msg[UBX_HEAD_SIZE]; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_RELPOSNED_V1_VERSION                                           = 0x01;  //!< @todo documentation
static constexpr std::size_t UBX_NAV_RELPOSNED_V1_SIZE                                              = sizeof(UBX_NAV_RELPOSNED_V1_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_RELPOSNED_V1_ITOW_SCALE                                        = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_RELPOSNED_V1_RELPOSN_E_D_SCALE                                 = 1e-2;  //!< @todo documentation
static constexpr double      UBX_NAV_RELPOSNED_V1_RELPOSLENGTH_SCALE                                = 1e-2;  //!< @todo documentation
static constexpr double      UBX_NAV_RELPOSNED_V1_RELPOSHEADING_SCALE                               = 1e-5;  //!< @todo documentation
static constexpr double      UBX_NAV_RELPOSNED_V1_RELPOSHPN_E_D_SCALE                               = 1e-4;  //!< @todo documentation
static constexpr double      UBX_NAV_RELPOSNED_V1_RELPOSHPLENGTH_SCALE                              = 1e-4;  //!< @todo documentation
static constexpr double      UBX_NAV_RELPOSNED_V1_ACCN_E_D_SCALE                                    = 1e-4;  //!< @todo documentation
static constexpr double      UBX_NAV_RELPOSNED_V1_ACCLENGTH_SCALE                                   = 1e-4;  //!< @todo documentation
static constexpr double      UBX_NAV_RELPOSNED_V1_ACCHEADING_SCALE                                  = 1e-5;  //!< @todo documentation
static constexpr bool        UBX_NAV_RELPOSNED_V1_FLAGS_GNSSFIXOK(const uint32_t flags)             { return (flags & 0x00000001) == 0x00000001; }  //!< @todo documentation
static constexpr bool        UBX_NAV_RELPOSNED_V1_FLAGS_DIFFSOLN(const uint32_t flags)              { return (flags & 0x00000002) == 0x00000002; }  //!< @todo documentation
static constexpr bool        UBX_NAV_RELPOSNED_V1_FLAGS_RELPOSVALID(const uint32_t flags)           { return (flags & 0x00000004) == 0x00000004; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_RELPOSNED_V1_FLAGS_CARRSOLN(const uint32_t flags)              { return (flags >> 3) & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_RELPOSNED_V1_FLAGS_CARRSOLN_NO                                 = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_RELPOSNED_V1_FLAGS_CARRSOLN_FLOAT                              = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_RELPOSNED_V1_FLAGS_CARRSOLN_FIXED                              = 2;  //!< @todo documentation
static constexpr bool        UBX_NAV_RELPOSNED_V1_FLAGS_ISMOVING(const uint32_t flags)              { return (flags & 0x00000020) == 0x00000020; }  //!< @todo documentation
static constexpr bool        UBX_NAV_RELPOSNED_V1_FLAGS_REFPOSMISS(const uint32_t flags)            { return (flags & 0x00000040) == 0x00000040; }  //!< @todo documentation
static constexpr bool        UBX_NAV_RELPOSNED_V1_FLAGS_REFOBSMISS(const uint32_t flags)            { return (flags & 0x00000080) == 0x00000080; }  //!< @todo documentation
static constexpr bool        UBX_NAV_RELPOSNED_V1_FLAGS_RELPOSHEADINGVALID(const uint32_t flags)    { return (flags & 0x00000100) == 0x00000100; }  //!< @todo documentation
static constexpr bool        UBX_NAV_RELPOSNED_V1_FLAGS_RELPOSNORMALIZED(const uint32_t flags)      { return (flags & 0x00000200) == 0x00000200; }  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-SAT message
 * @{
 */

//! UBX-NAV-SAT (version 1, output) payload head
struct UBX_NAV_SAT_V1_GROUP0  // clang-format off
{
    uint32_t iTOW;           //!< @todo documentation
    uint8_t  version;        //!< @todo documentation
    uint8_t  numSvs;         //!< @todo documentation
    uint8_t  reserved[2];    //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_SAT_V1_GROUP0) == 8, "");

//! UBX-NAV-SAT (version 1, output) payload repeated group
struct UBX_NAV_SAT_V1_GROUP1  // clang-format off
{
    uint8_t  gnssId;         //!< @todo documentation
    uint8_t  svId;           //!< @todo documentation
    uint8_t  cno;            //!< @todo documentation
    int8_t   elev;           //!< @todo documentation
    int16_t  azim;           //!< @todo documentation
    int16_t  prRes;          //!< @todo documentation
    uint32_t flags;          //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_SAT_V1_GROUP1) == 12, "");

// clang-format off
static constexpr uint8_t     UBX_NAV_SAT_VERSION(const uint8_t* msg)                                { return msg[UBX_HEAD_SIZE + sizeof(uint32_t)]; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SAT_V1_VERSION                                                 = 0x01;  //!< @todo documentation
static constexpr std::size_t UBX_NAV_SAT_V1_MIN_SIZE                                                = sizeof(UBX_NAV_SAT_V1_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_SAT_V1_ITOW_SCALE                                              = 1e-3;  //!< @todo documentation
// Note: only those flags relevant for a SV are defined below. All other info should be taken from UBX-NAV-SIG.  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SAT_V1_FLAGS_ORBITSOURCE(const uint32_t flags)                 { return (flags >> 8) & 0x00000007; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SAT_V1_FLAGS_ORBITSOURCE_NONE                                  = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SAT_V1_FLAGS_ORBITSOURCE_EPH                                   = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SAT_V1_FLAGS_ORBITSOURCE_ALM                                   = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SAT_V1_FLAGS_ORBITSOURCE_ANO                                   = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SAT_V1_FLAGS_ORBITSOURCE_ANA                                   = 4;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SAT_V1_FLAGS_ORBITSOURCE_OTHER1                                = 5;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SAT_V1_FLAGS_ORBITSOURCE_OTHER2                                = 6;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SAT_V1_FLAGS_ORBITSOURCE_OTHER3                                = 7;  //!< @todo documentation
static constexpr bool        UBX_NAV_SAT_V1_FLAGS_EPHAVAIL(const uint32_t flags)                    { return (flags & 0x00000800) == 0x00000800; }  //!< @todo documentation
static constexpr bool        UBX_NAV_SAT_V1_FLAGS_ALMAVAIL(const uint32_t flags)                    { return (flags & 0x00001000) == 0x00001000; }  //!< @todo documentation
static constexpr bool        UBX_NAV_SAT_V1_FLAGS_ANOAVAIL(const uint32_t flags)                    { return (flags & 0x00002000) == 0x00002000; }  //!< @todo documentation
static constexpr bool        UBX_NAV_SAT_V1_FLAGS_AOPAVAIL(const uint32_t flags)                    { return (flags & 0x00004000) == 0x00004000; }  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-SIG message
 * @{
 */

//! UBX-NAV-SIG (version 0, output) payload head
struct UBX_NAV_SIG_V0_GROUP0  // clang-format off
{
    uint32_t iTOW;           //!< @todo documentation
    uint8_t  version;        //!< @todo documentation
    uint8_t  numSigs;        //!< @todo documentation
    uint8_t  reserved[2];    //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_SIG_V0_GROUP0) == 8, "");

//! UBX-NAV-SIG (version 0, output) payload repeated group
struct UBX_NAV_SIG_V0_GROUP1  // clang-format off
{
    uint8_t  gnssId;         //!< @todo documentation
    uint8_t  svId;           //!< @todo documentation
    uint8_t  sigId;          //!< @todo documentation
    uint8_t  freqId;         //!< @todo documentation
    int16_t  prRes;          //!< @todo documentation
    uint8_t  cno;            //!< @todo documentation
    uint8_t  qualityInd;     //!< @todo documentation
    uint8_t  corrSource;     //!< @todo documentation
    uint8_t  ionoModel;      //!< @todo documentation
    uint16_t sigFlags;       //!< @todo documentation
    uint8_t  reserved[4];    //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_SIG_V0_GROUP1) == 16, "");

// clang-format off
static constexpr uint8_t     UBX_NAV_SIG_VERSION(const uint8_t* msg)                                { return msg[UBX_HEAD_SIZE + sizeof(uint32_t)]; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_VERSION                                                 = 0x00;  //!< @todo documentation
static constexpr std::size_t UBX_NAV_SIG_V0_MIN_SIZE                                                = sizeof(UBX_NAV_SIG_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_SIG_V0_ITOW_SCALE                                              = 1e-3;  //!< @todo documentation
static constexpr std::size_t UBX_NAV_SIG_V0_FREQID_OFFS                                             = 7;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_QUALITYIND_NOSIG                                        = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_QUALITYIND_SEARCH                                       = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_QUALITYIND_ACQUIRED                                     = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_QUALITYIND_UNUSED                                       = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_QUALITYIND_CODELOCK                                     = 4;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_QUALITYIND_CARRLOCK1                                    = 5;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_QUALITYIND_CARRLOCK2                                    = 6;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_QUALITYIND_CARRLOCK3                                    = 7;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_IONOMODEL_NONE                                          = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_IONOMODEL_KLOB_GPS                                      = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_IONOMODEL_SBAS                                          = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_IONOMODEL_KLOB_BDS                                      = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_IONOMODEL_DUALFREQ                                      = 8;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_SIGFLAGS_HEALTH(const uint16_t sigFlags)                { return sigFlags & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_SIGFLAGS_HEALTH_UNKNO                                   = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_SIGFLAGS_HEALTH_HEALTHY                                 = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_SIGFLAGS_HEALTH_UNHEALTHY                               = 2;  //!< @todo documentation
static constexpr bool        UBX_NAV_SIG_V0_SIGFLAGS_PR_SMOOTHED(const uint16_t sigFlags)           { return (sigFlags & 0x0004) == 0x0004; }  //!< @todo documentation
static constexpr bool        UBX_NAV_SIG_V0_SIGFLAGS_PR_USED(const uint16_t sigFlags)               { return (sigFlags & 0x0008) == 0x0008; }  //!< @todo documentation
static constexpr bool        UBX_NAV_SIG_V0_SIGFLAGS_CR_USED(const uint16_t sigFlags)               { return (sigFlags & 0x0010) == 0x0010; }  //!< @todo documentation
static constexpr bool        UBX_NAV_SIG_V0_SIGFLAGS_DO_USED(const uint16_t sigFlags)               { return (sigFlags & 0x0020) == 0x0020; }  //!< @todo documentation
static constexpr bool        UBX_NAV_SIG_V0_SIGFLAGS_PR_CORR_USED(const uint16_t sigFlags)          { return (sigFlags & 0x0040) == 0x0040; }  //!< @todo documentation
static constexpr bool        UBX_NAV_SIG_V0_SIGFLAGS_CR_CORR_USED(const uint16_t sigFlags)          { return (sigFlags & 0x0080) == 0x0080; }  //!< @todo documentation
static constexpr bool        UBX_NAV_SIG_V0_SIGFLAGS_DO_CORR_USED(const uint16_t sigFlags)          { return (sigFlags & 0x0100) == 0x0100; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_CORRSOURCE_NONE                                         = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_CORRSOURCE_SBAS                                         = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_CORRSOURCE_BDS                                          = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_CORRSOURCE_RTCM2                                        = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_CORRSOURCE_RTCM3_OSR                                    = 4;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_CORRSOURCE_RTCM3_SSR                                    = 5;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_CORRSOURCE_QZSS_SLAS                                    = 6;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_SIG_V0_CORRSOURCE_SPARTN                                       = 7;  //!< @todo documentation
static constexpr double      UBX_NAV_SIG_V0_PRRES_SCALE                                             = 1e-1;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-STATUS message
 * @{
 */

//! UBX-NAV-STATUS payload
struct UBX_NAV_STATUS_V0_GROUP0  // clang-format off
{
    uint32_t iTow;           //!< @todo documentation
    uint8_t  gpsFix;         //!< @todo documentation
    uint8_t  flags;          //!< @todo documentation
    uint8_t  fixStat;        //!< @todo documentation
    uint8_t  flags2;         //!< @todo documentation
    uint32_t ttff;           //!< @todo documentation
    uint32_t msss;           //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_STATUS_V0_GROUP0) == 16, "");

// clang-format off
static constexpr std::size_t UBX_NAV_STATUS_V0_SIZE                                                 = sizeof(UBX_NAV_STATUS_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_STATUS_V0_ITOW_SCALE                                           = 1e-3;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FIXTYPE_NOFIX                                        = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FIXTYPE_DRONLY                                       = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FIXTYPE_2D                                           = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FIXTYPE_3D                                           = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FIXTYPE_3D_DR                                        = 4;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FIXTYPE_TIME                                         = 5;  //!< @todo documentation
static constexpr bool        UBX_NAV_STATUS_V0_FLAGS_GPSFIXOK(const uint8_t flags)                  { return (flags & 0x01) == 0x01; }  //!< @todo documentation
static constexpr bool        UBX_NAV_STATUS_V0_FLAGS_DIFFSOLN(const uint8_t flags)                  { return (flags & 0x02) == 0x02; }  //!< @todo documentation
static constexpr bool        UBX_NAV_STATUS_V0_FLAGS_WKNSET(const uint8_t flags)                    { return (flags & 0x04) == 0x04; }  //!< @todo documentation
static constexpr bool        UBX_NAV_STATUS_V0_FLAGS_TOWSET(const uint8_t flags)                    { return (flags & 0x08) == 0x08; }  //!< @todo documentation
static constexpr bool        UBX_NAV_STATUS_V0_FIXSTAT_DIFFCORR(const uint8_t fixStat)              { return (fixStat & 0x01) ==  0x01; } //!< @todo documentation
static constexpr bool        UBX_NAV_STATUS_V0_FIXSTAT_CARRSOLNVALID(const uint8_t fixStat)         { return (fixStat & 0x02) ==  0x02; } //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FLAGS2_PSMSTATE(const uint8_t flags2)                { return flags2 & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FLAGS2_PSMSTATE_ACQUISITION                          = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FLAGS2_PSMSTATE_TRACKING                             = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FLAGS2_PSMSTATE_POWEROPT                             = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FLAGS2_PSMSTATE_INACTIVE                             = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FLAGS2_SPOOFDETSTATE(const uint8_t flags2)           { return (flags2 >> 3) & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FLAGS2_SPOOFDETSTATE_UNKNOWN                         = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FLAGS2_SPOOFDETSTATE_NONE                            = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FLAGS2_SPOOFDETSTATE_INDICATED                       = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FLAGS2_SPOOFDETSTATE_AFFIRMED                        = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FLAGS2_CARRSOLN(const uint8_t flags2)                { return (flags2 >> 6) & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FLAGS2_CARRSOLN_NO                                   = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FLAGS2_CARRSOLN_FLOAT                                = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_STATUS_V0_FLAGS2_CARRSOLN_FIXED                                = 2;  //!< @todo documentation
static constexpr double      UBX_NAV_STATUS_V0_TTFF_SCALE                                           = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_STATUS_V0_MSSS_SCALE                                           = 1e-3;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-TIMEGPS message
 * @{
 */

//! UBX-NAV-TIMEGPS payload
struct UBX_NAV_TIMEGPS_V0_GROUP0  // clang-format off
{
    uint32_t iTow;           //!< @todo documentation
    int32_t  fTOW;           //!< @todo documentation
    int16_t  week;           //!< @todo documentation
    int8_t   leapS;          //!< @todo documentation
    uint8_t  valid;          //!< @todo documentation
    uint32_t tAcc;           //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_TIMEGPS_V0_GROUP0) == 16, "");

// clang-format off
static constexpr std::size_t UBX_NAV_TIMEGPS_V0_SIZE                                                = sizeof(UBX_NAV_TIMEGPS_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_TIMEGPS_V0_ITOW_SCALE                                          = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_TIMEGPS_V0_FTOW_SCALE                                          = 1e-9;  //!< @todo documentation
static constexpr double      UBX_NAV_TIMEGPS_V0_TACC_SCALE                                          = 1e-9;  //!< @todo documentation
static constexpr bool        UBX_NAV_TIMEGPS_V0_VALID_TOWVALID(const uint8_t valid)                 { return (valid & 0x01) == 0x01; }  //!< @todo documentation
static constexpr bool        UBX_NAV_TIMEGPS_V0_VALID_WEEKVALID(const uint8_t valid)                { return (valid & 0x02) == 0x02; }  //!< @todo documentation
static constexpr bool        UBX_NAV_TIMEGPS_V0_VALID_LEAPSVALID(const uint8_t valid)               { return (valid & 0x04) == 0x04; }  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-TIMEGAL message
 * @{
 */

//! UBX-NAV-TIMEGAL payload
struct UBX_NAV_TIMEGAL_V0_GROUP0  // clang-format off
{
    uint32_t iTow;           //!< @todo documentation
    uint32_t galTow;         //!< @todo documentation
    int32_t  fGalTow;        //!< @todo documentation
    int16_t  galWno;         //!< @todo documentation
    int8_t   leapS;          //!< @todo documentation
    uint8_t  valid;          //!< @todo documentation
    uint32_t tAcc;           //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_TIMEGAL_V0_GROUP0) == 20, "");

// clang-format off
static constexpr std::size_t UBX_NAV_TIMEGAL_V0_SIZE                                                = sizeof(UBX_NAV_TIMEGAL_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_TIMEGAL_V0_ITOW_SCALE                                          = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_TIMEGAL_V0_FGALTOW_SCALE                                       = 1e-9;  //!< @todo documentation
static constexpr double      UBX_NAV_TIMEGAL_V0_TACC_SCALE                                          = 1e-9;  //!< @todo documentation
static constexpr bool        UBX_NAV_TIMEGAL_V0_VALID_GALTOWVALID(const uint8_t valid)              { return (valid & 0x01) == 0x01; }  //!< @todo documentation
static constexpr bool        UBX_NAV_TIMEGAL_V0_VALID_GALWNOVALID(const uint8_t valid)              { return (valid & 0x02) == 0x02; }  //!< @todo documentation
static constexpr bool        UBX_NAV_TIMEGAL_V0_VALID_LEAPSVALID(const uint8_t valid)               { return (valid & 0x04) == 0x04; }  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-TIMEBDS message
 * @{
 */

//! UBX-NAV-TIMEBDS payload
struct UBX_NAV_TIMEBDS_V0_GROUP0  // clang-format off
{
    uint32_t iTow;           //!< @todo documentation
    uint32_t SOW;            //!< @todo documentation
    int32_t  fSOW;           //!< @todo documentation
    int16_t  week;           //!< @todo documentation
    int8_t   leapS;          //!< @todo documentation
    uint8_t  valid;          //!< @todo documentation
    uint32_t tAcc;           //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_TIMEBDS_V0_GROUP0) == 20, "");

// clang-format off
static constexpr uint32_t    UBX_NAV_TIMEBDS_V0_SIZE                                                = sizeof(UBX_NAV_TIMEBDS_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_TIMEBDS_V0_ITOW_SCALE                                          = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_TIMEBDS_V0_FSOW_SCALE                                          = 1e-9;  //!< @todo documentation
static constexpr double      UBX_NAV_TIMEBDS_V0_TACC_SCALE                                          = 1e-9;  //!< @todo documentation
static constexpr bool        UBX_NAV_TIMEBDS_V0_VALID_SOWVALID(const uint8_t valid)                 { return (valid & 0x01) == 0x01; }  //!< @todo documentation
static constexpr bool        UBX_NAV_TIMEBDS_V0_VALID_WEEKVALID(const uint8_t valid)                { return (valid & 0x02) == 0x02; }  //!< @todo documentation
static constexpr bool        UBX_NAV_TIMEBDS_V0_VALID_LEAPSVALID(const uint8_t valid)               { return (valid & 0x04) == 0x04; }  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-TIMEGLO message
 * @{
 */

//! UBX-NAV-TIMEGLO payload
struct UBX_NAV_TIMEGLO_V0_GROUP0  // clang-format off
{
    uint32_t iTow;           //!< @todo documentation
    uint32_t TOD;            //!< @todo documentation
    int32_t  fTOD;           //!< @todo documentation
    uint16_t Nt;             //!< @todo documentation
    uint8_t  N4;             //!< @todo documentation
    uint8_t  valid;          //!< @todo documentation
    uint32_t tAcc;           //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_TIMEGLO_V0_GROUP0) == 20, "");

// clang-format off
static constexpr double      UBX_NAV_TIMEGLO_V0_SIZE                                                = sizeof(UBX_NAV_TIMEGLO_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_TIMEGLO_V0_ITOW_SCALE                                          = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_TIMEGLO_V0_FTOD_SCALE                                          = 1e-9;  //!< @todo documentation
static constexpr double      UBX_NAV_TIMEGLO_V0_TACC_SCALE                                          = 1e-9;  //!< @todo documentation
static constexpr bool        UBX_NAV_TIMEGLO_V0_VALID_TODVALID(const uint8_t valid)                 { return (valid & 0x01) == 0x01; }  //!< @todo documentation
static constexpr bool        UBX_NAV_TIMEGLO_V0_VALID_DATEVALID(const uint8_t valid)                { return (valid & 0x02) == 0x02; }  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-TIMEUTC message
 * @{
 */

//! UBX-NAV-TIMEUTC payload
struct UBX_NAV_TIMEUTC_V0_GROUP0  // clang-format off
 {
    uint32_t iTow;           //!< @todo documentation
    uint32_t tAcc;           //!< @todo documentation
    int32_t  nano;           //!< @todo documentation
    uint16_t year;           //!< @todo documentation
    uint8_t  month;          //!< @todo documentation
    uint8_t  day;            //!< @todo documentation
    uint8_t  hour;           //!< @todo documentation
    uint8_t  min;            //!< @todo documentation
    uint8_t  sec;            //!< @todo documentation
    uint8_t  valid;          //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_TIMEUTC_V0_GROUP0) == 20, "");

// clang-format off
static constexpr std::size_t UBX_NAV_TIMEUTC_V0_SIZE                                                = sizeof(UBX_NAV_TIMEUTC_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_TIMEUTC_V0_ITOW_SCALE                                          = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_TIMEUTC_V0_TACC_SCALE                                          = 1e-9;  //!< @todo documentation
static constexpr double      UBX_NAV_TIMEUTC_V0_NANO_SCALE                                          = 1e-9;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMEUTC_V0_VALID_VALIDTOW(const uint8_t valid)                 { return (valid & 0x01) == 0x01; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMEUTC_V0_VALID_VALIDWKN(const uint8_t valid)                 { return (valid & 0x02) == 0x02; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMEUTC_V0_VALID_VALIDUTC(const uint8_t valid)                 { return (valid & 0x04) == 0x04; }  //!< @todo documentation
static constexpr bool        UBX_NAV_TIMEUTC_V0_VALID_AUTHSTATUS(const uint8_t valid)               { return (valid & 0x08) == 0x08; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMEUTC_V0_VALID_UTCSTANDARD(const uint8_t valid)              { return (valid >> 4) & 0x0f; }  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMEUTC_V0_VALID_UTCSTANDARD_INFONA                            =  0;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMEUTC_V0_VALID_UTCSTANDARD_CRL                               =  1;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMEUTC_V0_VALID_UTCSTANDARD_NIST                              =  2;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMEUTC_V0_VALID_UTCSTANDARD_USNO                              =  3;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMEUTC_V0_VALID_UTCSTANDARD_BIPM                              =  4;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMEUTC_V0_VALID_UTCSTANDARD_EU                                =  5;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMEUTC_V0_VALID_UTCSTANDARD_SU                                =  6;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMEUTC_V0_VALID_UTCSTANDARD_NTSC                              =  7;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMEUTC_V0_VALID_UTCSTANDARD_NPLI                              =  8;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMEUTC_V0_VALID_UTCSTANDARD_UNNOWN                            = 15;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-TIMELS message
 * @{
 */

//! UBX-NAV-TIMELS payload
struct UBX_NAV_TIMELS_V0_GROUP0  // clang-format off
{
    uint32_t iTOW;           //!< @todo documentation
    uint8_t  version;        //!< @todo documentation
    uint8_t  reserved0[3];   //!< @todo documentation
    uint8_t  srcOfCurrLs;    //!< @todo documentation
    int8_t   currLs;         //!< @todo documentation
    uint8_t  srcOfLsChange;  //!< @todo documentation
    int8_t   lsChange;       //!< @todo documentation
    int32_t  timeToLsEvent;  //!< @todo documentation
    uint16_t dateOfLsGpsWn;  //!< @todo documentation
    uint16_t dateOfLsGpsDn;  //!< @todo documentation
    uint8_t  reserved1[3];   //!< @todo documentation
    uint8_t  valid;          //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_TIMELS_V0_GROUP0) == 24, "");

// clang-format off
static constexpr std::size_t UBX_NAV_TIMELS_V0_SIZE                                                 = sizeof(UBX_NAV_TIMELS_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_TIMELS_V0_ITOW_SCALE                                           = 1e-3;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMELS_V0_SRCOFCURRLS_DEFAULT                                  = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMELS_V0_SRCOFCURRLS_GPSGLO                                   = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMELS_V0_SRCOFCURRLS_GPS                                      = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMELS_V0_SRCOFCURRLS_SBAS                                     = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMELS_V0_SRCOFCURRLS_BDS                                      = 4;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMELS_V0_SRCOFCURRLS_GAL                                      = 5;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMELS_V0_SRCOFCURRLS_CONFIG                                   = 7;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMELS_V0_SRCOFCURRLS_UNKNOWN                                  = 255;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMELS_V0_SRCOFCURRLSCHANGE_NONE                               = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMELS_V0_SRCOFCURRLSCHANGE_GPS                                = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMELS_V0_SRCOFCURRLSCHANGE_SBAS                               = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMELS_V0_SRCOFCURRLSCHANGE_BDS                                = 4;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMELS_V0_SRCOFCURRLSCHANGE_GAL                                = 5;  //!< @todo documentation
static constexpr uint8_t     UBX_NAV_TIMELS_V0_SRCOFCURRLSCHANGE_GLO                                = 6;  //!< @todo documentation
static constexpr bool        UBX_NAV_TIMELS_V0_VALID_CURRLSVALID(const uint8_t valid)               { return (valid & 0x01) == 0x01; }  //!< @todo documentation
static constexpr bool        UBX_NAV_TIMELS_V0_VALID_TIMETOLSEVENTVALID(const uint8_t valid)        { return (valid & 0x02) == 0x02; }  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-NAV-VELECEF message
 * @{
 */

//! UBX-NAV-VELECEF payload
struct UBX_NAV_VELECEF_V0_GROUP0  // clang-format off
{
    uint32_t iTOW;           //!< @todo documentation
    int32_t  ecefVX;         //!< @todo documentation
    int32_t  ecefVY;         //!< @todo documentation
    int32_t  ecefVZ;         //!< @todo documentation
    uint32_t sAcc;           //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_NAV_VELECEF_V0_GROUP0) == 20, "");

// clang-format off
static constexpr std::size_t UBX_NAV_VELECEF_V0_SIZE                                                = sizeof(UBX_NAV_VELECEF_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_NAV_VELECEF_V0_ITOW_SCALE                                          = 1e-3;  //!< @todo documentation
static constexpr double      UBX_NAV_VELECEF_V0_ECEF_XYZ_SCALE                                      = 1e-2;  //!< @todo documentation
static constexpr double      UBX_NAV_VELECEF_V0_SACC_SCALE                                          = 1e-2;  //!< @todo documentation
// clang-format on

///@}

// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-RXM-RAWX message
 * @{
 */

//! UBX-RXM-RAWX (version 1, output) payload head
struct UBX_RXM_RAWX_V1_GROUP0  // clang-format off
{
    double   rcvTow;         //!< @todo documentation
    uint16_t week;           //!< @todo documentation
    int8_t   leapS;          //!< @todo documentation
    uint8_t  numMeas;        //!< @todo documentation
    uint8_t  recStat;        //!< @todo documentation
    uint8_t  version;        //!< @todo documentation
    uint8_t  reserved[2];    //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_RXM_RAWX_V1_GROUP0) == 16, "");

//! UBX-RXM-RAWX (version 1, output) payload repeated group
struct UBX_RXM_RAWX_V1_GROUP1  // clang-format off
{
    double   prMeas;         //!< @todo documentation
    double   cpMeas;         //!< @todo documentation
    float    doMeas;         //!< @todo documentation
    uint8_t  gnssId;         //!< @todo documentation
    uint8_t  svId;           //!< @todo documentation
    uint8_t  sigId;          //!< @todo documentation
    uint8_t  freqId;         //!< @todo documentation
    uint16_t locktime;       //!< @todo documentation
    uint8_t  cno;            //!< @todo documentation
    uint8_t  prStdev;        //!< @todo documentation
    uint8_t  cpStdev;        //!< @todo documentation
    uint8_t  doStdev;        //!< @todo documentation
    uint8_t  trkStat;        //!< @todo documentation
    uint8_t  reserved[1];    //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_RXM_RAWX_V1_GROUP1) == 32, "");

// clang-format off
static constexpr uint8_t     UBX_RXM_RAWX_VERSION(const uint8_t* msg)                               { return msg[UBX_HEAD_SIZE + 13]; }  //!< @todo documentation
static constexpr uint8_t     UBX_RXM_RAWX_V1_VERSION                                                = 0x01;  //!< @todo documentation
static constexpr std::size_t UBX_RXM_RAWX_V1_MIN_SIZE                                               = sizeof(UBX_RXM_RAWX_V1_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr bool        UBX_RXM_RAWX_V1_RECSTAT_LEAPSEC(const uint8_t recStat)                 { return (recStat & 0x01) == 0x01; }  //!< @todo documentation
static constexpr bool        UBX_RXM_RAWX_V1_RECSTAT_CLKRESET(const uint8_t recStat)                { return (recStat & 0x02) == 0x02; }  //!< @todo documentation
static constexpr uint8_t     UBX_RXM_RAWX_V1_PRSTDEV_PRSTD(const uint8_t prStdev)                   { return prStdev & 0x0f; }  //!< @todo documentation
static constexpr double      UBX_RXM_RAWX_V1_PRSTD_SCALE(const double prStd)                        { return 0.01 * std::exp2(prStd); }  //!< @todo documentation
static constexpr uint8_t     UBX_RXM_RAWX_V1_CPSTDEV_CPSTD(const uint8_t cpStdev)                   { return cpStdev & 0x0f; }  //!< @todo documentation
static constexpr double      UBX_RXM_RAWX_V1_CPSTD_SCALE(const double cpStd)                        { return 0.004 * cpStd; }  //!< @todo documentation
static constexpr uint8_t     UBX_RXM_RAWX_V1_DOSTDEV_DOSTD(const uint8_t doStdev)                   { return doStdev & 0x0f; }  //!< @todo documentation
static constexpr double      UBX_RXM_RAWX_V1_DOSTD_SCALE(const double doStd)                        { return 0.002 * std::exp2(doStd); }  //!< @todo documentation
static constexpr double      UBX_RXM_RAWX_V1_LOCKTIME_SCALE                                         = 1e-3;  //!< @todo documentation
static constexpr bool        UBX_RXM_RAWX_V1_TRKSTAT_PRVALID(const uint8_t trkStat)                 { return (trkStat & 0x01) == 0x01; }  //!< @todo documentation
static constexpr bool        UBX_RXM_RAWX_V1_TRKSTAT_CPVALID(const uint8_t trkStat)                 { return (trkStat & 0x02) == 0x02; }  //!< @todo documentation
static constexpr bool        UBX_RXM_RAWX_V1_TRKSTAT_HALFCYC(const uint8_t trkStat)                 { return (trkStat & 0x04) == 0x04; }  //!< @todo documentation
static constexpr bool        UBX_RXM_RAWX_V1_TRKSTAT_SUBHALFCYC(const uint8_t trkStat)              { return (trkStat & 0x08) == 0x08; }  //!< @todo documentation
static constexpr int         UBX_RXM_RAWX_V1_GROUP1_FREQID_TO_SLOT(const uint8_t freqId)            { return (int)freqId - 7; }  //!< @todo documentation
static constexpr std::size_t UBX_RXM_RAWX_V1_SIZE(const uint8_t *msg)                               { return
    ((sizeof(UBX_RXM_RAWX_V1_GROUP0) + UBX_FRAME_SIZE + (((uint8_t *)(msg))[UBX_HEAD_SIZE + 11] * sizeof(UBX_RXM_RAWX_V1_GROUP1)))); }  //!< @todo documentation

// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-RXM-RTCM message
 * @{
 */

//! UBX-RXM-RTCM (version 2, output)
struct UBX_RXM_RTCM_V2_GROUP0  // clang-format off
{
    uint8_t  version;        //!< @todo documentation
    uint8_t  flags;          //!< @todo documentation
    uint16_t subType;        //!< @todo documentation
    uint16_t refStation;     //!< @todo documentation
    uint16_t msgType;        //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_RXM_RTCM_V2_GROUP0) == 8, "");

// clang-format off
static constexpr uint8_t     UBX_RXM_RTCM_VERSION(const uint8_t* msg)                               { return msg[UBX_HEAD_SIZE]; }  //!< @todo documentation
static constexpr uint8_t     UBX_RXM_RTCM_V2_VERSION                                                = 0x02;  //!< @todo documentation
static constexpr std::size_t UBX_RXM_RTCM_V2_SIZE                                                   = sizeof(UBX_RXM_RTCM_V2_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr bool        UBX_RXM_RTCM_V2_FLAGS_CRCFAILED(const uint8_t flags)                   { return (flags & 0x01) == 0x01; }  //!< @todo documentation
static constexpr uint8_t     UBX_RXM_RTCM_V2_FLAGS_MSGUSED(const uint8_t flags)                     { return (flags >> 1) & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_RXM_RTCM_V2_FLAGS_MSGUSED_UNKNOWN                                  = 0x00;  //!< @todo documentation
static constexpr uint8_t     UBX_RXM_RTCM_V2_FLAGS_MSGUSED_UNUSED                                   = 0x01;  //!< @todo documentation
static constexpr uint8_t     UBX_RXM_RTCM_V2_FLAGS_MSGUSED_USED                                     = 0x02;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-RXM-SFRBX message
 * @{
 */

//! UBX-RXM-SFRBX (version 2, output) payload head
struct UBX_RXM_SFRBX_V2_GROUP0  // clang-format off
{
    uint8_t  gnssId;         //!< @todo documentation
    uint8_t  svId;           //!< @todo documentation
    uint8_t  sigId;          //!< interface description says "reserved", but u-center says "sigId"...
    uint8_t  freqId;         //!< @todo documentation
    uint8_t  numWords;       //!< @todo documentation
    uint8_t  chn;            //!< @todo documentation
    uint8_t  version;        //!< @todo documentation
    uint8_t  reserved1;      //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_RXM_SFRBX_V2_GROUP0) == 8, "");

//! UBX-RXM-SFRBX (version 2, output) payload repeated group
struct UBX_RXM_SFRBX_V2_GROUP1  // clang-format off
{
    uint32_t dwrd;           //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_RXM_SFRBX_V2_GROUP1) == 4, "");

// clang-format off
static constexpr uint8_t     UBX_RXM_SFRBX_VERSION(const uint8_t* msg)                              { return msg[UBX_HEAD_SIZE + 6]; }  //!< @todo documentation
static constexpr uint8_t     UBX_RXM_SFRBX_V2_VERSION                                               = 0x02;  //!< @todo documentation
static constexpr std::size_t UBX_RXM_SFRBX_V2_MIN_SIZE                                              = sizeof(UBX_RXM_SFRBX_V2_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr int         UBX_RXM_SFRBX_V2_GROUP0_FREQID_TO_SLOT(const uint8_t freqId)           { return (int)freqId - 7; }  //!< @todo documentation
// clang-format on

///@}

// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-RXM-SPARTN message
 * @{
 */

//! UBX-RXM-SPARTN (version 1, output)
struct UBX_RXM_SPARTN_V1_GROUP0  // clang-format off
{
    uint8_t  version;        //!< @todo documentation
    uint8_t  flags;          //!< @todo documentation
    uint16_t subType;        //!< @todo documentation
    uint8_t  reserved[2];    //!< @todo documentation
    uint16_t msgType;        //!< @todo documentation
};  // clang-format on

// clang-format off
static constexpr uint8_t     UBX_RXM_SPARTN_VERSION(const uint8_t* msg)                             { return msg[UBX_HEAD_SIZE]; }  //!< @todo documentation
static constexpr uint8_t     UBX_RXM_SPARTN_V1_VERSION                                              = 0x01;  //!< @todo documentation
static constexpr std::size_t UBX_RXM_SPARTN_V1_SIZE                                                 = sizeof(UBX_RXM_SPARTN_V1_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr uint8_t     UBX_RXM_SPARTN_V1_FLAGS_MSGUSED(const uint8_t flags)                   { return (flags >> 1) & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_RXM_SPARTN_V1_FLAGS_MSGUSED_UNKNOWN                                = 0x00;  //!< @todo documentation
static constexpr uint8_t     UBX_RXM_SPARTN_V1_FLAGS_MSGUSED_UNUSED                                 = 0x01;  //!< @todo documentation
static constexpr uint8_t     UBX_RXM_SPARTN_V1_FLAGS_MSGUSED_USED                                   = 0x02;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-TIM-SVIN message
 * @{
 */

//! UBX-TIM_SVIN payload
struct UBX_TIM_SVIN_V0_GROUP0  // clang-format off
{
    uint32_t dur;            //!< @todo documentation
    int32_t  meanX;          //!< @todo documentation
    int32_t  meanY;          //!< @todo documentation
    int32_t  meanZ;          //!< @todo documentation
    uint32_t meanV;          //!< @todo documentation
    uint32_t obs;            //!< @todo documentation
    uint8_t  valid;          //!< @todo documentation
    uint8_t  active;         //!< @todo documentation
    uint16_t reserved;       //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_TIM_SVIN_V0_GROUP0) == 28, "");

// clang-format off
static constexpr std::size_t UBX_TIM_SVIN_V0_SIZE                                                   = sizeof(UBX_TIM_SVIN_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-TIM-TM2 message
 * @{
 */

//! UBX-TIM-TM2 payload
struct UBX_TIM_TM2_V0_GROUP0  // clang-format off
{
    uint8_t  ch;             //!< @todo documentation
    uint8_t  flags;          //!< @todo documentation
    uint16_t count;          //!< @todo documentation
    uint16_t wnR;            //!< @todo documentation
    uint16_t wnF;            //!< @todo documentation
    uint32_t towMsR;         //!< @todo documentation
    uint32_t towSubMsR;      //!< @todo documentation
    uint32_t towMsF;         //!< @todo documentation
    uint32_t towSubMsF;      //!< @todo documentation
    uint32_t accEst;         //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_TIM_TM2_V0_GROUP0) == 28, "");

// clang-format off
static constexpr std::size_t UBX_TIM_TM2_V0_SIZE                                                    = sizeof(UBX_TIM_TM2_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TM2_V0_FLAGS_MODE(const uint8_t flags)                         { return flags & 0x01; }  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TM2_V0_FLAGS_MODE_SINGLE                                       = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TM2_V0_FLAGS_MODE_RUNNING                                      = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TM2_V0_FLAGS_RUN(const uint8_t flags)                          { return (flags >> 1) & 0x01; }  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TM2_V0_FLAGS_RUN_ARMED                                         = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TM2_V0_FLAGS_RUN_STOPPED                                       = 1;  //!< @todo documentation
static constexpr bool        UBX_TIM_TM2_V0_FLAGS_NEWFALLINGEDGE(const uint8_t flags)               { return (flags & 0x04) == 0x04; }  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TM2_V0_FLAGS_TIMEBASE(const uint8_t flags)                     { return (flags >> 3) & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TM2_V0_FLAGS_TIMEBASE_RX                                       = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TM2_V0_FLAGS_TIMEBASE_GNSS                                     = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TM2_V0_FLAGS_TIMEBASE_UTC                                      = 2;  //!< @todo documentation
static constexpr bool        UBX_TIM_TM2_V0_FLAGS_UTCACAVAIL(const uint8_t flags)                   { return (flags & 0x20) == 0x20; }  //!< @todo documentation
static constexpr bool        UBX_TIM_TM2_V0_FLAGS_TIMEVALID(const uint8_t flags)                    { return (flags & 0x40) == 0x40; }  //!< @todo documentation
static constexpr bool        UBX_TIM_TM2_V0_FLAGS_NEWRISINGEDGE(const uint8_t flags)                { return (flags & 0x80) == 0x80; }  //!< @todo documentation
static constexpr double      UBX_TIM_TM2_V0_TOW_SCALE                                               = 1e-3;  //!< @todo documentation
static constexpr double      UBX_TIM_TM2_V0_SUBMS_SCALE                                             = 1e-9;  //!< @todo documentation
static constexpr double      UBX_TIM_TM2_V0_ACCEST_SCALE                                            = 1e-9;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name UBX-TIM_TP message
 * @{
 */

//! UBX-TIM-TP payload
struct UBX_TIM_TP_V0_GROUP0  // clang-format off
{
    uint32_t towMs;          //!< @todo documentation
    uint32_t towSubMs;       //!< @todo documentation
    int32_t  qErr;           //!< @todo documentation
    uint16_t week;           //!< @todo documentation
    uint8_t  flags;          //!< @todo documentation
    uint8_t  refInfo;        //!< @todo documentation
};  // clang-format on

static_assert(sizeof(UBX_TIM_TP_V0_GROUP0) == 16, "");

// clang-format off
static constexpr std::size_t UBX_TIM_TP_V0_SIZE                                                     = sizeof(UBX_TIM_TP_V0_GROUP0) + UBX_FRAME_SIZE;  //!< @todo documentation
static constexpr double      UBX_TIM_TP_V0_TOWMS_SCALE                                              = 1e-3;  //!< @todo documentation
static constexpr double      UBX_TIM_TP_V0_TOWSUBMS_SCALE                                           = 0x1.0624dd2f1a9fcp-42;  //!< perl -e 'printf "%a", 2**-32 * 1e-3'
static constexpr double      UBX_TIM_TP_V0_TOWSUBMS_SCALE_APPROX                                    = 2.3283064365386963e-18; //!< perl -e 'printf "%.18g", 2**-32 * 1e-8'
static constexpr uint8_t     UBX_TIM_TP_V0_FLAGS_TIMEBASE(const uint8_t flags)                      { return flags & 0x01; }  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_FLAGS_TIMEBASE_GNSS                                      = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_FLAGS_TIMEBASE_UTC                                       = 1;  //!< @todo documentation
static constexpr bool        UBX_TIM_TP_V0_FLAGS_UTC(const uint8_t flags)                           { return (flags & 0x02) == 0x02; }  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_FLAGS_RAIM(const uint8_t flags)                          { return (flags >> 2) & 0x03; }  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_FLAGS_RAIM_NA                                            = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_FLAGS_RAIM_INACTIVE                                      = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_FLAGS_RAIM_ACTIVE                                        = 2;  //!< @todo documentation
static constexpr bool        UBX_TIM_TP_V0_FLAGS_QERRINVALID(const uint8_t flags)                   { return (flags & 0x10) == 0x10; }  //!< @todo documentation
static constexpr bool        UBX_TIM_TP_V0_FLAGS_TPNOTLOCKED(const uint8_t flags)                   { return (flags & 0x20) == 0x20; }  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_TIMEREFGNSS(const uint8_t refInfo)               { return refInfo & 0x0f; }  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_TIMEREFGNSS_GPS                                  = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_TIMEREFGNSS_GLO                                  = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_TIMEREFGNSS_BDS                                  = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_TIMEREFGNSS_GAL                                  = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_TIMEREFGNSS_NAVIC                                = 4;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_TIMEREFGNSS_UNKNOWN                              = 15;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_UTCSTANDARD(const uint8_t refInfo)               { return (refInfo >> 4) & 0x0f; }  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_UTCSTANDARD_INFONA                               = 0;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_UTCSTANDARD_CRL                                  = 1;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_UTCSTANDARD_NIST                                 = 2;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_UTCSTANDARD_USNO                                 = 3;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_UTCSTANDARD_BIPM                                 = 4;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_UTCSTANDARD_EU                                   = 5;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_UTCSTANDARD_SU                                   = 6;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_UTCSTANDARD_NTSC                                 = 7;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_UTCSTANDARD_NPLI                                 = 8;  //!< @todo documentation
static constexpr uint8_t     UBX_TIM_TP_V0_REFINFO_UTCSTANDARD_UNNOWN                               = 15;  //!< @todo documentation
// clang-format on

///@}
// ---------------------------------------------------------------------------------------------------------------------

#undef UBX_PACKED

/* ****************************************************************************************************************** */
}  // namespace ubx
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_PARSER_UBX_HPP__
