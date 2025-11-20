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
 * @brief Fixposition SDK: Parser FP_B routines and types
 */

/* LIBC/STL */
#include <array>
#include <cinttypes>
#include <cstddef>
#include <cstdio>

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/parser/crc.hpp"
#include "fpsdk_common/parser/fpb.hpp"
#include "fpsdk_common/parser/types.hpp"

namespace fpsdk {
namespace common {
namespace parser {
namespace fpb {
/* ****************************************************************************************************************** */

// Lookup table entry
struct MsgInfo
{
    uint16_t msg_id;
    const char* name;
};

// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_FPB_MSGINFO}
static constexpr std::array<MsgInfo, 6> MSG_INFO =
{{
    { FP_B_GNSSSTATUS_MSGID,         FP_B_GNSSSTATUS_STRID          },
    { FP_B_SYSTEMSTATUS_MSGID,       FP_B_SYSTEMSTATUS_STRID        },
    { FP_B_MEASUREMENTS_MSGID,       FP_B_MEASUREMENTS_STRID        },
    { FP_B_VERSION_MSGID,            FP_B_VERSION_STRID             },
    { FP_B_UNITTEST1_MSGID,          FP_B_UNITTEST1_STRID           },
    { FP_B_UNITTEST2_MSGID,          FP_B_UNITTEST2_STRID           },
}};
// @fp_codegen_end{FPSDK_COMMON_PARSER_FPB_MSGINFO}
// clang-format on

bool FpbGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    // Check arguments
    if ((name == NULL) || (size == 0)) {
        return false;
    }
    name[0] = '\0';

    if ((msg == NULL) || (msg_size < FP_B_FRAME_SIZE)) {
        return false;
    }

    const uint16_t msg_id = FpbMsgId(msg);

    // Try the message name lookup table
    for (auto& info : MSG_INFO) {
        if (info.msg_id == msg_id) {
            return std::snprintf(name, size, "%s", info.name) < (int)size;
        }
    }

    // If that failed, stringify the message ID
    return std::snprintf(name, size, "FP_B-MSG%05" PRIu16, msg_id) < (int)size;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpbMakeMessage(std::vector<uint8_t>& msg, const uint16_t msg_id, const uint16_t msg_time, const uint8_t* payload,
    const std::size_t payload_size)
{
    if (((payload == NULL) && (payload_size > 0)) || (payload_size > (MAX_FP_B_SIZE - FP_B_FRAME_SIZE))) {
        return false;
    }

    const uint16_t payload_size_u = payload_size;
    const std::size_t msg_size = payload_size_u + FP_B_FRAME_SIZE;

    msg.resize(msg_size);
    msg[0] = FP_B_SYNC_1;
    msg[1] = FP_B_SYNC_2;
    msg[2] = (msg_id & 0xff);
    msg[3] = (msg_id >> 8);
    msg[4] = (payload_size_u & 0xff);
    msg[5] = (payload_size_u >> 8);
    msg[6] = (msg_time & 0xff);
    msg[7] = (msg_time >> 8);
    if ((payload != NULL) && (payload_size_u > 0)) {
        std::memcpy(&msg[FP_B_HEAD_SIZE], payload, payload_size_u);
    }
    const uint32_t crc = crc::Crc32fpb(&msg[0], msg_size - sizeof(crc));
    // clang-format off
    msg[msg_size - 4] = ( crc        & 0xff);
    msg[msg_size - 3] = ((crc >>  8) & 0xff);
    msg[msg_size - 2] = ((crc >> 16) & 0xff);
    msg[msg_size - 1] = ((crc >> 24) & 0xff);
    // clang-format on
    return true;
}

bool FpbMakeMessage(
    std::vector<uint8_t>& msg, const uint16_t msg_id, const uint16_t msg_time, const std::vector<uint8_t>& payload)
{
    return FpbMakeMessage(msg, msg_id, msg_time, payload.data(), payload.size());
}

// ---------------------------------------------------------------------------------------------------------------------

static std::size_t StrFpbMeasurements(
    char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if (msg_size < FP_B_MEASUREMENTS_HEAD_SIZE) {
        return 0;
    }
    std::size_t num = 0;
    FpbMeasurementsHead head;
    std::memcpy(&head, &msg[FP_B_HEAD_SIZE], sizeof(head));
    num += std::snprintf(&info[num], size - num, " v%" PRIu8, head.version);
    if (head.version == FP_B_MEASUREMENTS_V1) {
        num += std::snprintf(&info[num], size - num, " [%" PRIu8 "]", head.num_meas);
        for (std::size_t ix = 0; (ix < head.num_meas) && (num < (size - 50)); ix++) {
            FpbMeasurementsMeas meas;
            std::memcpy(&meas, &msg[FP_B_HEAD_SIZE + sizeof(head) + (ix * sizeof(meas))], sizeof(meas));
            num += std::snprintf(&info[num], size - num, " / %" PRIu8 " %" PRIu8 " %" PRIu8 " %" PRIu16 " %" PRIu32,
                meas.meas_type, meas.meas_loc, meas.timestamp_type, meas.gps_wno, meas.gps_tow);
            if (meas.meas_x_valid) {
                num += std::snprintf(&info[num], size - num, " %" PRIi32, meas.meas_x);
            } else {
                num += std::snprintf(&info[num], size - num, " <inv>>");
            }
            if (meas.meas_y_valid) {
                num += std::snprintf(&info[num], size - num, " %" PRIi32, meas.meas_y);
            } else {
                num += std::snprintf(&info[num], size - num, " <inv>>");
            }
            if (meas.meas_z_valid) {
                num += std::snprintf(&info[num], size - num, " %" PRIi32, meas.meas_z);
            } else {
                num += std::snprintf(&info[num], size - num, " <inv>>");
            }
        }
    }
    return num;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpbGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((info == NULL) || (size == 0)) {
        return false;
    }

    if ((msg == NULL) || (msg_size < (FP_B_HEAD_SIZE + 2))) {
        info[0] = '\0';
        return false;
    }
    std::size_t len = 0;

    // Common stringification
    len += snprintf(&info[len], size - len, "%04" PRIx16 "@%05" PRIu16, FpbMsgId(msg), FpbMsgTime(msg));

    // Specific stringification for some messages
    switch (FpbMsgId(msg)) {
        case FP_B_MEASUREMENTS_MSGID:
            len += StrFpbMeasurements(&info[len], size - len, msg, msg_size);
            break;
    }

    return (len > 0) && (len < size);
}

/* ****************************************************************************************************************** */
}  // namespace fpb
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
