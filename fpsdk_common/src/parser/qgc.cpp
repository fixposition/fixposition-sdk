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
 * @brief Fixposition SDK: Parser QGC routines and types
 */

/* LIBC/STL */
#include <array>
#include <cinttypes>
#include <cstdio>

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/parser/qgc.hpp"

namespace fpsdk {
namespace common {
namespace parser {
namespace qgc {
/* ****************************************************************************************************************** */

// Lookup table entry
struct MsgInfo
{
    uint8_t grp_id_ = 0;
    uint8_t msg_id_ = 0;
    const char* name_ = nullptr;
};

// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_QGC_MSGINFO}
static constexpr std::array<MsgInfo, 1> GRP_INFO =
{{
    { QGC_RAW_GRPID,                 0x00,                          QGC_RAW_STRID                  },
}};
static constexpr std::array<MsgInfo, 3> MSG_INFO =
{{
    { QGC_RAW_GRPID,                 QGC_RAW_PPPB2B_MSGID,          QGC_RAW_PPPB2B_STRID           },
    { QGC_RAW_GRPID,                 QGC_RAW_QZSSL6_MSGID,          QGC_RAW_QZSSL6_STRID           },
    { QGC_RAW_GRPID,                 QGC_RAW_HASE6_MSGID,           QGC_RAW_HASE6_STRID            },
}};
// @fp_codegen_end{FPSDK_COMMON_PARSER_QGC_MSGINFO}
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

bool QgcGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    // Check arguments
    if ((name == NULL) || (size < 1)) {
        return false;
    }
    name[0] = '\0';

    if ((msg == NULL) || (msg_size < QGC_FRAME_SIZE)) {
        return false;
    }

    const uint8_t grp_id = QgcGrpId(msg);
    const uint8_t msg_id = QgcMsgId(msg);

    std::size_t res = 0;

    // First try the message name lookup table
    for (auto& info : MSG_INFO) {
        if ((info.grp_id_ == grp_id) && (info.msg_id_ == msg_id)) {
            res = std::snprintf(name, size, "%s", info.name_);
            break;
        }
    }

    // If that failed, try the class name lookup table
    if (res == 0) {
        for (auto& info : GRP_INFO) {
            if (info.grp_id_ == grp_id) {
                res = std::snprintf(name, size, "%s-%02" PRIX8, info.name_, msg_id);
                break;
            }
        }
    }

    // If that failed, too, stringify both IDs
    if (res == 0) {
        res = std::snprintf(name, size, "QGC-%02" PRIX8 "-%02" PRIX8, grp_id, msg_id);
    }

    // Did it fit into the string?
    return res < size;
}

// ---------------------------------------------------------------------------------------------------------------------

bool QgcGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((info == NULL) || (size < 1) || (msg == NULL) || (msg_size < QGC_HEAD_SIZE)) {
        return false;
    }

    info[0] = '\0';

    if ((msg == NULL) || (msg_size < QGC_HEAD_SIZE)) {
        return false;
    }

    std::size_t len = 0;

    // TODO: implement some stringification

    return (len > 0) && (len < size);
}

/* ****************************************************************************************************************** */
}  // namespace qgc
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
