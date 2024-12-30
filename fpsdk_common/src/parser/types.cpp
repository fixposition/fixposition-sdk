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
 * @brief Fixposition SDK: Parser types
 */

/* LIBC/STL */
#include <algorithm>
#include <cstring>

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/parser/fpa.hpp"
#include "fpsdk_common/parser/fpb.hpp"
#include "fpsdk_common/parser/nmea.hpp"
#include "fpsdk_common/parser/novb.hpp"
#include "fpsdk_common/parser/rtcm3.hpp"
#include "fpsdk_common/parser/spartn.hpp"
#include "fpsdk_common/parser/types.hpp"
#include "fpsdk_common/parser/ubx.hpp"
#include "fpsdk_common/parser/unib.hpp"

namespace fpsdk {
namespace common {
namespace parser {
/* ****************************************************************************************************************** */

const char* ProtocolStr(const Protocol proto)
{
    switch (proto) {  // clang-format off
        case Protocol::FP_A:   return PROTOCOL_NAME_FP_A;
        case Protocol::FP_B:   return PROTOCOL_NAME_FP_B;
        case Protocol::NMEA:   return PROTOCOL_NAME_NMEA;
        case Protocol::UBX:    return PROTOCOL_NAME_UBX;
        case Protocol::RTCM3:  return PROTOCOL_NAME_RTCM3;
        case Protocol::UNI_B:  return PROTOCOL_NAME_UNI_B;
        case Protocol::NOV_B:  return PROTOCOL_NAME_NOV_B;
        case Protocol::SPARTN: return PROTOCOL_NAME_SPARTN;
        case Protocol::OTHER:  return PROTOCOL_NAME_OTHER;
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

Protocol StrProtocol(const char* name)
{
    if (name != NULL) {  // clang-format off
        if      (std::strcmp(name, PROTOCOL_NAME_FP_A)   == 0) { return Protocol::FP_A; }
        else if (std::strcmp(name, PROTOCOL_NAME_FP_B)   == 0) { return Protocol::FP_B; }
        else if (std::strcmp(name, PROTOCOL_NAME_NMEA)   == 0) { return Protocol::NMEA; }
        else if (std::strcmp(name, PROTOCOL_NAME_UBX)    == 0) { return Protocol::UBX; }
        else if (std::strcmp(name, PROTOCOL_NAME_RTCM3)  == 0) { return Protocol::RTCM3; }
        else if (std::strcmp(name, PROTOCOL_NAME_UNI_B)  == 0) { return Protocol::UNI_B; }
        else if (std::strcmp(name, PROTOCOL_NAME_NOV_B)  == 0) { return Protocol::NOV_B; }
        else if (std::strcmp(name, PROTOCOL_NAME_SPARTN) == 0) { return Protocol::SPARTN; }
    }  // clang-format on
    return Protocol::OTHER;
}
// ---------------------------------------------------------------------------------------------------------------------

void ParserMsg::MakeInfo() const
{
    if (!info_.empty()) {
        return;
    }
    char sinfo[MAX_INFO_SIZE];
    const uint8_t* mdata = data_.data();
    const int msize = (int)data_.size();
    switch (proto_) {
        case Protocol::FP_A:
            info_ = (fpa::FpaGetMessageInfo(sinfo, sizeof(sinfo), mdata, msize) ? sinfo : "");
            break;
        case Protocol::FP_B:
            info_ = (fpb::FpbGetMessageInfo(sinfo, sizeof(sinfo), mdata, msize) ? sinfo : "");
            break;
        case Protocol::NMEA:
            info_ = (nmea::NmeaGetMessageInfo(sinfo, sizeof(sinfo), mdata, msize) ? sinfo : "");
            break;
        case Protocol::UBX:
            info_ = (ubx::UbxGetMessageInfo(sinfo, sizeof(sinfo), mdata, msize) ? sinfo : "");
            break;
        case Protocol::RTCM3:
            info_ = (rtcm3::Rtcm3GetMessageInfo(sinfo, sizeof(sinfo), mdata, msize) ? sinfo : "");
            break;
        case Protocol::UNI_B:
            info_ = (unib::UnibGetMessageInfo(sinfo, sizeof(sinfo), mdata, msize) ? sinfo : "");
            break;
        case Protocol::NOV_B:
            info_ = (novb::NovbGetMessageInfo(sinfo, sizeof(sinfo), mdata, msize) ? sinfo : "");
            break;
        case Protocol::SPARTN:
            info_ = (spartn::SpartnGetMessageInfo(sinfo, sizeof(sinfo), mdata, msize) ? sinfo : "");
            break;
        case Protocol::OTHER: {
            // Info is a hexdump of the first few bytes
            constexpr int num = 16;
            sinfo[0] = '\0';
            for (int ix = 0; (ix < num) && (ix < msize); ix++) {
                std::snprintf(&sinfo[ix * 3], sizeof(sinfo - (ix * 3)), "%02x ", mdata[ix]);
            }
            if (msize > 16) {
                sinfo[(num * 3) + 0] = '.';
                sinfo[(num * 3) + 1] = '.';
                sinfo[(num * 3) + 2] = '.';
                sinfo[(num * 3) + 3] = '\0';
            } else {
                sinfo[num * 3] = '\0';
            }
            info_ = sinfo;
            break;
        }
    }
}

/* ****************************************************************************************************************** */
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
