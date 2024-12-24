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
 * @brief Fixposition SDK: Parser SPARTN routines and types
 */

/* LIBC/STL */
#include <array>
#include <cinttypes>
#include <cstddef>
#include <cstdio>

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/parser/spartn.hpp"

namespace fpsdk {
namespace common {
namespace parser {
namespace spartn {
/* ****************************************************************************************************************** */

// Lookup table entry
struct MsgInfo
{
    uint8_t msg_type_;
    uint8_t sub_type_;
    const char* name_;
    const char* desc_;
};

// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_SPARTN_MSGINFO}
static constexpr std::array<MsgInfo, 6> MSG_INFO =
{{
    { SPARTN_OCB_MSGID,              0, SPARTN_OCB_STRID,              "Orbits, clock, bias (OCB)" },
    { SPARTN_HPAC_MSGID,             0, SPARTN_HPAC_STRID,             "High-precision atmosphere correction (HPAC)" },
    { SPARTN_GAD_MSGID,              0, SPARTN_GAD_STRID,              "Geographic area definition (GAD)" },
    { SPARTN_BPAC_MSGID,             0, SPARTN_BPAC_STRID,             "Basic-precision atmosphere correction (BPAC)" },
    { SPARTN_EAS_MSGID,              0, SPARTN_EAS_STRID,              "Encryption and authentication support (EAS)" },
    { SPARTN_PROP_MSGID,             0, SPARTN_PROP_STRID,             "Proprietary message" },
}};
static constexpr std::array<MsgInfo, 17> SUB_INFO =
{{
    { SPARTN_OCB_MSGID, SPARTN_OCB_GPS_SUBID, SPARTN_OCB_GPS_STRID, "Orbits, clock, bias (OCB) GPS" },
    { SPARTN_OCB_MSGID, SPARTN_OCB_GLO_SUBID, SPARTN_OCB_GLO_STRID, "Orbits, clock, bias (OCB) GLONASS" },
    { SPARTN_OCB_MSGID, SPARTN_OCB_GAL_SUBID, SPARTN_OCB_GAL_STRID, "Orbits, clock, bias (OCB) Galileo" },
    { SPARTN_OCB_MSGID, SPARTN_OCB_BDS_SUBID, SPARTN_OCB_BDS_STRID, "Orbits, clock, bias (OCB) BeiDou" },
    { SPARTN_OCB_MSGID, SPARTN_OCB_QZSS_SUBID, SPARTN_OCB_QZSS_STRID, "Orbits, clock, bias (OCB) QZSS" },
    { SPARTN_HPAC_MSGID, SPARTN_HPAC_GPS_SUBID, SPARTN_HPAC_GPS_STRID, "High-precision atmosphere correction (HPAC) GPS" },
    { SPARTN_HPAC_MSGID, SPARTN_HPAC_GLO_SUBID, SPARTN_HPAC_GLO_STRID, "High-precision atmosphere correction (HPAC) GLONASS" },
    { SPARTN_HPAC_MSGID, SPARTN_HPAC_GAL_SUBID, SPARTN_HPAC_GAL_STRID, "High-precision atmosphere correction (HPAC) Galileo" },
    { SPARTN_HPAC_MSGID, SPARTN_HPAC_BDS_SUBID, SPARTN_HPAC_BDS_STRID, "High-precision atmosphere correction (HPAC) BeiDou" },
    { SPARTN_HPAC_MSGID, SPARTN_HPAC_QZSS_SUBID, SPARTN_HPAC_QZSS_STRID, "High-precision atmosphere correction (HPAC) QZSS" },
    { SPARTN_GAD_MSGID, SPARTN_GAD_GAD_SUBID, SPARTN_GAD_GAD_STRID, "Geographic area definition (GAD)" },
    { SPARTN_BPAC_MSGID, SPARTN_BPAC_POLY_SUBID, SPARTN_BPAC_POLY_STRID, "Basic-precision atmosphere correction (BPAC)" },
    { SPARTN_EAS_MSGID, SPARTN_EAS_KEY_SUBID, SPARTN_EAS_KEY_STRID, "Encryption and authentication support (EAS): Dynamic key" },
    { SPARTN_EAS_MSGID, SPARTN_EAS_GROUP_SUBID, SPARTN_EAS_GROUP_STRID, "Encryption and authentication support (EAS): Group authentication" },
    { SPARTN_PROP_MSGID, SPARTN_PROP_EST_SUBID, SPARTN_PROP_EST_STRID, "Proprietary messages: test" },
    { SPARTN_PROP_MSGID, SPARTN_PROP_UBLOX_SUBID, SPARTN_PROP_UBLOX_STRID, "Proprietary messages: u-blox" },
    { SPARTN_PROP_MSGID, SPARTN_PROP_SWIFT_SUBID, SPARTN_PROP_SWIFT_STRID, "Proprietary messages: Swift" },
}};
// @fp_codegen_end{FPSDK_COMMON_PARSER_SPARTN_MSGINFO}
// clang-format on

bool SpartnGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    // Check arguments
    if ((name == NULL) || (size < 1)) {
        return false;
    }
    name[0] = '\0';

    if ((msg == NULL) || (msg_size < (SPARTN_MIN_HEAD_SIZE + 2))) {
        return false;
    }

    const uint8_t msg_type = SpartnType(msg);
    const uint8_t sub_type = SpartnSubType(msg);

    std::size_t res = 0;

    // First try the message name lookup table
    for (auto& info : SUB_INFO) {
        if ((info.msg_type_ == msg_type) && (info.sub_type_ == sub_type)) {
            res = snprintf(name, size, "%s", info.name_);
            break;
        }
    }

    // If that failed, try the class name lookup table
    if (res == 0) {
        for (auto& info : MSG_INFO) {
            if (info.msg_type_ == msg_type) {
                res = snprintf(name, size, "%s-%-" PRIu8, info.name_, sub_type);
                break;
            }
        }
    }

    // If that failed, too, stringify both types
    if (res == 0) {
        res = snprintf(name, size, "SPARTN-%" PRIu8 "-%" PRIu8, msg_type, sub_type);
    }

    // Did it fit into the string?
    return res < size;
}

// ---------------------------------------------------------------------------------------------------------------------

const char* SpartnGetTypeDesc(const uint16_t type, const uint16_t subtype)
{
    for (auto& info : SUB_INFO) {
        if ((info.msg_type_ == type) && (info.sub_type_ == subtype)) {
            return info.desc_;
        }
    }
    for (auto& info : MSG_INFO) {
        if (info.msg_type_ == type) {
            return info.desc_;
        }
    }
    return NULL;
}

// ---------------------------------------------------------------------------------------------------------------------

bool SpartnGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((info == NULL) || (size < 1)) {
        return false;
    }

    if ((msg == NULL) || (msg_size < SPARTN_MIN_HEAD_SIZE)) {
        info[0] = '\0';
        return false;
    }
    std::size_t len = 0;

    const char* type_desc = SpartnGetTypeDesc(SpartnType(msg), SpartnSubType(msg));
    if ((len < size) && (type_desc != NULL)) {
        len += snprintf(&info[len], size - len, "%s%s", len > 0 ? " - " : "", type_desc);
    }

    return (len > 0) && (len < size);
}

/* ****************************************************************************************************************** */
}  // namespace spartn
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
