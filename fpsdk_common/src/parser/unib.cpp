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
 * @brief Fixposition SDK: Parser UNI_B routines and types
 */

/* LIBC/STL */
#include <array>
#include <cinttypes>
#include <cstddef>
#include <cstdio>
#include <cstring>

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/parser/unib.hpp"

namespace fpsdk {
namespace common {
namespace parser {
namespace unib {
/* ****************************************************************************************************************** */

// Lookup table entry
struct MsgInfo
{
    uint16_t msg_id;
    const char* name;
};

// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_UNIB_MSGINFO}
static constexpr std::array<MsgInfo, 68> MSG_INFO =
{{
    { UNI_B_VERSION_MSGID,           UNI_B_VERSION_STRID            },
    { UNI_B_OBSVM_MSGID,             UNI_B_OBSVM_STRID              },
    { UNI_B_OBSVH_MSGID,             UNI_B_OBSVH_STRID              },
    { UNI_B_OBSVMCMP_MSGID,          UNI_B_OBSVMCMP_STRID           },
    { UNI_B_OBSVHCMP_MSGID,          UNI_B_OBSVHCMP_STRID           },
    { UNI_B_OBSVBASE_MSGID,          UNI_B_OBSVBASE_STRID           },
    { UNI_B_BASEINFO_MSGID,          UNI_B_BASEINFO_STRID           },
    { UNI_B_GPSION_MSGID,            UNI_B_GPSION_STRID             },
    { UNI_B_BD3ION_MSGID,            UNI_B_BD3ION_STRID             },
    { UNI_B_BDSION_MSGID,            UNI_B_BDSION_STRID             },
    { UNI_B_GALION_MSGID,            UNI_B_GALION_STRID             },
    { UNI_B_GPSUTC_MSGID,            UNI_B_GPSUTC_STRID             },
    { UNI_B_BD3UTC_MSGID,            UNI_B_BD3UTC_STRID             },
    { UNI_B_BDSUTC_MSGID,            UNI_B_BDSUTC_STRID             },
    { UNI_B_GALUTC_MSGID,            UNI_B_GALUTC_STRID             },
    { UNI_B_GPSEPH_MSGID,            UNI_B_GPSEPH_STRID             },
    { UNI_B_QZSSEPH_MSGID,           UNI_B_QZSSEPH_STRID            },
    { UNI_B_BD3EPH_MSGID,            UNI_B_BD3EPH_STRID             },
    { UNI_B_BDSEPH_MSGID,            UNI_B_BDSEPH_STRID             },
    { UNI_B_GLOEPH_MSGID,            UNI_B_GLOEPH_STRID             },
    { UNI_B_GALEPH_MSGID,            UNI_B_GALEPH_STRID             },
    { UNI_B_AGRIC_MSGID,             UNI_B_AGRIC_STRID              },
    { UNI_B_PVTSLN_MSGID,            UNI_B_PVTSLN_STRID             },
    { UNI_B_BESTNAV_MSGID,           UNI_B_BESTNAV_STRID            },
    { UNI_B_BESTNAVXYZ_MSGID,        UNI_B_BESTNAVXYZ_STRID         },
    { UNI_B_BESTNAVH_MSGID,          UNI_B_BESTNAVH_STRID           },
    { UNI_B_BESTNAVXYZH_MSGID,       UNI_B_BESTNAVXYZH_STRID        },
    { UNI_B_BESTSAT_MSGID,           UNI_B_BESTSAT_STRID            },
    { UNI_B_ADRNAV_MSGID,            UNI_B_ADRNAV_STRID             },
    { UNI_B_ADRNAVH_MSGID,           UNI_B_ADRNAVH_STRID            },
    { UNI_B_PPPNAV_MSGID,            UNI_B_PPPNAV_STRID             },
    { UNI_B_SPPNAV_MSGID,            UNI_B_SPPNAV_STRID             },
    { UNI_B_SPPNAVH_MSGID,           UNI_B_SPPNAVH_STRID            },
    { UNI_B_STADOP_MSGID,            UNI_B_STADOP_STRID             },
    { UNI_B_STADOPH_MSGID,           UNI_B_STADOPH_STRID            },
    { UNI_B_ADRDOP_MSGID,            UNI_B_ADRDOP_STRID             },
    { UNI_B_ADRDOPH_MSGID,           UNI_B_ADRDOPH_STRID            },
    { UNI_B_PPPDOP_MSGID,            UNI_B_PPPDOP_STRID             },
    { UNI_B_SPPDOP_MSGID,            UNI_B_SPPDOP_STRID             },
    { UNI_B_SPPDOPH_MSGID,           UNI_B_SPPDOPH_STRID            },
    { UNI_B_SATSINFO_MSGID,          UNI_B_SATSINFO_STRID           },
    { UNI_B_BASEPOS_MSGID,           UNI_B_BASEPOS_STRID            },
    { UNI_B_SATELLITE_MSGID,         UNI_B_SATELLITE_STRID          },
    { UNI_B_SATECEF_MSGID,           UNI_B_SATECEF_STRID            },
    { UNI_B_RECTIME_MSGID,           UNI_B_RECTIME_STRID            },
    { UNI_B_NOVHEADING_MSGID,        UNI_B_NOVHEADING_STRID         },
    { UNI_B_NOVHEADING2_MSGID,       UNI_B_NOVHEADING2_STRID        },
    { UNI_B_RTKSTATUS_MSGID,         UNI_B_RTKSTATUS_STRID          },
    { UNI_B_AGNSSSTATUS_MSGID,       UNI_B_AGNSSSTATUS_STRID        },
    { UNI_B_RTCSTATUS_MSGID,         UNI_B_RTCSTATUS_STRID          },
    { UNI_B_JAMSTATUS_MSGID,         UNI_B_JAMSTATUS_STRID          },
    { UNI_B_FREQJAMSTATUS_MSGID,     UNI_B_FREQJAMSTATUS_STRID      },
    { UNI_B_RTCMSTATUS_MSGID,        UNI_B_RTCMSTATUS_STRID         },
    { UNI_B_HWSTATUS_MSGID,          UNI_B_HWSTATUS_STRID           },
    { UNI_B_PPPSTATUS_MSGID,         UNI_B_PPPSTATUS_STRID          },
    { UNI_B_AGC_MSGID,               UNI_B_AGC_STRID                },
    { UNI_B_INFOPART1_MSGID,         UNI_B_INFOPART1_STRID          },
    { UNI_B_INFOPART2_MSGID,         UNI_B_INFOPART2_STRID          },
    { UNI_B_MSPOS_MSGID,             UNI_B_MSPOS_STRID              },
    { UNI_B_TROPINFO_MSGID,          UNI_B_TROPINFO_STRID           },
    { UNI_B_PTOBSINFO_MSGID,         UNI_B_PTOBSINFO_STRID          },
    { UNI_B_PPPB2INFO1_MSGID,        UNI_B_PPPB2INFO1_STRID         },
    { UNI_B_PPPB2INFO2_MSGID,        UNI_B_PPPB2INFO2_STRID         },
    { UNI_B_PPPB2INFO3_MSGID,        UNI_B_PPPB2INFO3_STRID         },
    { UNI_B_PPPB2INFO4_MSGID,        UNI_B_PPPB2INFO4_STRID         },
    { UNI_B_PPPB2INFO5_MSGID,        UNI_B_PPPB2INFO5_STRID         },
    { UNI_B_PPPB2INFO6_MSGID,        UNI_B_PPPB2INFO6_STRID         },
    { UNI_B_PPPB2INFO7_MSGID,        UNI_B_PPPB2INFO7_STRID         },
}};
// @fp_codegen_end{FPSDK_COMMON_PARSER_UNIB_MSGINFO}
// clang-format on

bool UnibGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    // Check arguments
    if ((name == NULL) || (size < 1)) {
        return false;
    }
    name[0] = '\0';

    if ((msg == NULL) || (msg_size < sizeof(UnibHeader))) {
        return false;
    }

    const uint16_t msg_id = UnibMsgId(msg);

    // Try the message name lookup table
    for (auto& info : MSG_INFO) {
        if (info.msg_id == msg_id) {
            return std::snprintf(name, size, "%s", info.name) < (int)size;
        }
    }

    // If that failed, stringify the message ID
    return std::snprintf(name, size, "UNI_B-MSG%05" PRIu16, msg_id) < (int)size;
}

// ---------------------------------------------------------------------------------------------------------------------

const char* UnibTimeRefStr(const UnibTimeRef time_ref)
{
    switch (time_ref) {  // clang-format off
        case UnibTimeRef::GPST:  return "GPST";
        case UnibTimeRef::BDST:  return "BDST";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* UnibTimeStatusStr(const UnibTimeStatus time_status)
{
    switch (time_status) {  // clang-format off
        case UnibTimeStatus::UNKNOWN:  return "UNKNOWN";
        case UnibTimeStatus::FINE:     return "FINE";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* UnibProductModelStr(const UnibProductModel model)
{
    switch (model) {  // clang-format off
        case UnibProductModel::UNKNOWN:   return "UNKNOWN";
        case UnibProductModel::UB4B0:     return "UB4B0";
        case UnibProductModel::UM4B0:     return "UM4B0";
        case UnibProductModel::UM480:     return "UM480";
        case UnibProductModel::UM440:     return "UM440";
        case UnibProductModel::UM482:     return "UM482";
        case UnibProductModel::UM442:     return "UM442";
        case UnibProductModel::UB482:     return "UB482";
        case UnibProductModel::UT4B0:     return "UT4B0";
        case UnibProductModel::UB362L:    return "UB362L";
        case UnibProductModel::UB4B0M:    return "UB4B0M";
        case UnibProductModel::UB4B0J:    return "UB4B0J";
        case UnibProductModel::UM482L:    return "UM482L";
        case UnibProductModel::UM4B0L:    return "UM4B0L";
        case UnibProductModel::CLAP_B:    return "CLAP_B";
        case UnibProductModel::UM982:     return "UM982";
        case UnibProductModel::UM980:     return "UM980";
        case UnibProductModel::UM960:     return "UM960";
        case UnibProductModel::UM980A:    return "UM980A";
        case UnibProductModel::CLAP_C:    return "CLAP_C";
        case UnibProductModel::UM960L:    return "UM960L";
        case UnibProductModel::UM981:     return "UM981";
    }
    return "?"; // not the same as "UNKNOWN"!
}

// ---------------------------------------------------------------------------------------------------------------------

// static int StrUnibSome(char *info, const std::size_t size, const uint8_t *msg, const int msg_size)
// {
//     int len = 0;
//     // ...
//     return len;
// }

// ---------------------------------------------------------------------------------------------------------------------

bool UnibGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((info == NULL) || (size < 1)) {
        return false;
    }

    if ((msg == NULL) || (msg_size < UNI_B_HEAD_SIZE)) {
        info[0] = '\0';
        return false;
    }
    std::size_t len = 0;

    // Common stringification
    UnibHeader hdr;
    std::memcpy(&hdr, msg, sizeof(hdr));
    len += snprintf(&info[len], size - len, "l %010.3f", (double)hdr.tow_ms * 1e-3);

    // Specific stringification for some messages
    // switch (UnibMsgId(msg)) {
    //     case UNI_B_SOME_MSGID:
    //         len += StrUnibSome(&info[len], size - len, msg, msg_size);
    //         break;
    // }

    return (len > 0) && (len < size);
}

/* ****************************************************************************************************************** */
}  // namespace unib
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
