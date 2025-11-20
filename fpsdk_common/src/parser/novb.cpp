/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 *
 * Parts copyright (c) 2020 NovAtel Inc.
 * Based on work by flipflip (https://github.com/phkehl)
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Parser NOV_B routines and types
 */

/* LIBC/STL */
#include <array>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/parser/novb.hpp"
#include "fpsdk_common/types.hpp"

namespace fpsdk {
namespace common {
namespace parser {
namespace novb {
/* ****************************************************************************************************************** */

// Lookup table entry
struct MsgInfo
{
    uint16_t msg_id;
    const char* name;
};

// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_NOVB_MSGINFO}
static constexpr std::array<MsgInfo, 19> MSG_INFO =
{{
    { NOV_B_BESTGNSSPOS_MSGID,       NOV_B_BESTGNSSPOS_STRID        },
    { NOV_B_BESTPOS_MSGID,           NOV_B_BESTPOS_STRID            },
    { NOV_B_BESTUTM_MSGID,           NOV_B_BESTUTM_STRID            },
    { NOV_B_BESTVEL_MSGID,           NOV_B_BESTVEL_STRID            },
    { NOV_B_BESTXYZ_MSGID,           NOV_B_BESTXYZ_STRID            },
    { NOV_B_CORRIMUS_MSGID,          NOV_B_CORRIMUS_STRID           },
    { NOV_B_HEADING2_MSGID,          NOV_B_HEADING2_STRID           },
    { NOV_B_IMURATECORRIMUS_MSGID,   NOV_B_IMURATECORRIMUS_STRID    },
    { NOV_B_INSCONFIG_MSGID,         NOV_B_INSCONFIG_STRID          },
    { NOV_B_INSPVA_MSGID,            NOV_B_INSPVA_STRID             },
    { NOV_B_INSPVAS_MSGID,           NOV_B_INSPVAS_STRID            },
    { NOV_B_INSPVAX_MSGID,           NOV_B_INSPVAX_STRID            },
    { NOV_B_INSSTDEV_MSGID,          NOV_B_INSSTDEV_STRID           },
    { NOV_B_PSRDOP2_MSGID,           NOV_B_PSRDOP2_STRID            },
    { NOV_B_RAWDMI_MSGID,            NOV_B_RAWDMI_STRID             },
    { NOV_B_RAWIMU_MSGID,            NOV_B_RAWIMU_STRID             },
    { NOV_B_RAWIMUSX_MSGID,          NOV_B_RAWIMUSX_STRID           },
    { NOV_B_RXSTATUS_MSGID,          NOV_B_RXSTATUS_STRID           },
    { NOV_B_TIME_MSGID,              NOV_B_TIME_STRID               },
}};
// @fp_codegen_end{FPSDK_COMMON_PARSER_NOVB_MSGINFO}
// clang-format on

bool NovbGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    // Check arguments
    if ((name == NULL) || (size < 1)) {
        return false;
    }
    name[0] = '\0';

    if ((msg == NULL) || (msg_size < 14)) {
        return false;
    }

    const uint16_t msg_id = NovbMsgId(msg);

    // Try the message name lookup table
    for (auto& info : MSG_INFO) {
        if (info.msg_id == msg_id) {
            return std::snprintf(name, size, "%s", info.name) < (int)size;
        }
    }

    // If that failed, stringify the message ID
    return std::snprintf(name, size, "NOV_B-MSG%05" PRIu16, msg_id) < (int)size;
}

// ---------------------------------------------------------------------------------------------------------------------

bool NovbHeader::IsLongHeader() const
{
    return long_header.sync3 == NOV_B_SYNC_3_LONG;
}

NovbMsgTypeSource NovbHeader::Source() const
{
    if (long_header.sync3 == NOV_B_SYNC_3_LONG) {
        return static_cast<NovbMsgTypeSource>(
            long_header.message_type & fpsdk::common::types::EnumToVal(NovbMsgTypeSource::_MASK));

    } else {
        return NovbMsgTypeSource::_MASK;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

const char* NovbMsgTypeSourceStr(const uint8_t message_type)
{
    // clang-format off
    switch (static_cast<NovbMsgTypeSource>(message_type & static_cast<uint8_t>(NovbMsgTypeSource::_MASK))) {
        case NovbMsgTypeSource::PRIMARY:   return "PRIMARY";
        case NovbMsgTypeSource::SECONDARY: return "SECONDARY";
        case NovbMsgTypeSource::_MASK:     break;
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* NovbTimeStatusStr(const NovbTimeStatus time_status)
{
    switch (time_status) {  // clang-format off
        case NovbTimeStatus::UNKNOWN:            return "UNKNOWN";
        case NovbTimeStatus::APPROXIMATE:        return "APPROXIMATE";
        case NovbTimeStatus::COARSEADJUSTING:    return "COARSEADJUSTING";
        case NovbTimeStatus::COARSE:             return "COARSE";
        case NovbTimeStatus::COARSESTEERING:     return "COARSESTEERING";
        case NovbTimeStatus::FREEWHEELING:       return "FREEWHEELING";
        case NovbTimeStatus::FINEADJUSTING:      return "FINEADJUSTING";
        case NovbTimeStatus::FINE:               return "FINE";
        case NovbTimeStatus::FINEBACKUPSTEERING: return "FINEBACKUPSTEERING";
        case NovbTimeStatus::FINESTEERING:       return "FINESTEERING";
        case NovbTimeStatus::SATTIME:            return "SATTIME";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

#define _CHKSIZE(_type_)                                                                              \
    (((msg[2] == NOV_B_SYNC_3_LONG) && (msg_size == (NOV_B_HEAD_SIZE_SHORT + sizeof(_type_) + 4))) || \
        ((msg[2] == NOV_B_SYNC_3_SHORT) && (msg_size == (NOV_B_HEAD_SIZE_LONG + sizeof(_type_) + 4))))

#define _PAYLOAD(_type_, _dst_) \
    _type_ _dst_;               \
    memcpy(&_dst_, &msg[msg[2] == NOV_B_SYNC_3_LONG ? NOV_B_HEAD_SIZE_LONG : NOV_B_HEAD_SIZE_SHORT], sizeof(_dst_))

static std::size_t StrNovbRawdmi(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    std::size_t len = 0;
    if (_CHKSIZE(NovbRawdmi)) {
        _PAYLOAD(NovbRawdmi, dmi);
        len += snprintf(&info[len], size - len, " [%c]=%d [%c]=%d [%c]=%d [%c]=%d", dmi.dmi1_valid ? '1' : '.',
            dmi.dmi1, dmi.dmi2_valid ? '2' : '.', dmi.dmi2, dmi.dmi3_valid ? '3' : '.', dmi.dmi3,
            dmi.dmi4_valid ? '4' : '.', dmi.dmi4);
    }
    return len;
}

// ---------------------------------------------------------------------------------------------------------------------

bool NovbGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((info == NULL) || (size < 1)) {
        return false;
    }

    if ((msg == NULL) || (msg_size < NOV_B_HEAD_SIZE_SHORT)) {
        info[0] = '\0';
        return false;
    }
    std::size_t len = 0;

    // Common stringification
    if (NovbIsLongHeader(msg) && (msg_size >= NOV_B_HEAD_SIZE_LONG)) {
        NovbLongHeader hdr;
        std::memcpy(&hdr, msg, sizeof(hdr));
        len += snprintf(&info[len], size - len, "l %010.3f", (double)hdr.gps_milliseconds * 1e-3);
    } else if (NovbIsShortHeader(msg) && (msg_size >= NOV_B_HEAD_SIZE_SHORT)) {
        NovbShortHeader hdr;
        std::memcpy(&hdr, msg, sizeof(hdr));
        len += snprintf(&info[len], size - len, "s %010.3f", (double)hdr.gps_milliseconds * 1e-3);
    }

    // Specific stringification for some messages
    switch (NovbMsgId(msg)) {
        case NOV_B_RAWDMI_MSGID:
            len += StrNovbRawdmi(&info[len], size - len, msg, msg_size);
            break;
    }

    return (len > 0) && (len < size);
}

/* ****************************************************************************************************************** */
}  // namespace novb
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
