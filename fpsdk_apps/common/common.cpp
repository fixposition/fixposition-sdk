/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: common app utils
 */

/* LIBC/STL */
#include <cinttypes>
#include <cstdint>
#include <cstdio>

/* EXTERNAL */

/* Fixposition SDK */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/string.hpp>

/* PACKAGE */
#include "common.hpp"

namespace fpsdk {
namespace apps {
namespace common {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::string;
using namespace fpsdk::common::parser;

// ---------------------------------------------------------------------------------------------------------------------

void PrintMessageHeader()
{
    // Keep in sync with PrintMessage()
    std::printf("------- Seq#     Offset  Size Protocol Message                        Info\n");
}

// ---------------------------------------------------------------------------------------------------------------------

void PrintMessageData(const ParserMsg& msg, const std::size_t offs, const bool hexdump)
{
    msg.MakeInfo();
    std::printf("message %06" PRIuMAX " %8" PRIuMAX " %5" PRIuMAX " %-8s %-30s %s\n", msg.seq_, offs, msg.data_.size(),
        ProtocolStr(msg.proto_), msg.name_.c_str(), msg.info_.empty() ? "-" : msg.info_.c_str());
    if (hexdump) {
        for (auto& line : HexDump(msg.data_)) {
            std::printf("%s\n", line.c_str());
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PrintParserStats(const ParserStats& stats)
{
    auto& s = stats;
    const double p_n = (s.n_msgs_ > 0 ? 100.0 / (double)s.n_msgs_ : 0.0);
    const double p_s = (s.s_msgs_ > 0 ? 100.0 / (double)s.s_msgs_ : 0.0);
    std::printf("Stats:     Messages               Bytes\n");
    const char* fmt = "%-8s %10" PRIu64 " (%5.1f%%) %10" PRIu64 " (%5.1f%%)\n";
    // clang-format off
    std::printf(fmt, "Total",                              s.n_msgs_,   (double)s.n_msgs_   * p_n, s.s_msgs_,   (double)s.s_msgs_   * p_s);
    std::printf(fmt, ProtocolStr(Protocol::FP_A),   s.n_fpa_,    (double)s.n_fpa_    * p_n, s.s_fpa_,    (double)s.s_fpa_    * p_s);
    std::printf(fmt, ProtocolStr(Protocol::FP_B),   s.n_fpb_,    (double)s.n_fpb_    * p_n, s.s_fpb_,    (double)s.s_fpb_    * p_s);
    std::printf(fmt, ProtocolStr(Protocol::NMEA),   s.n_nmea_,   (double)s.n_nmea_   * p_n, s.s_nmea_,   (double)s.s_nmea_   * p_s);
    std::printf(fmt, ProtocolStr(Protocol::UBX),    s.n_ubx_,    (double)s.n_ubx_    * p_n, s.s_ubx_,    (double)s.s_ubx_    * p_s);
    std::printf(fmt, ProtocolStr(Protocol::RTCM3),  s.n_rtcm3_,  (double)s.n_rtcm3_  * p_n, s.s_rtcm3_,  (double)s.s_rtcm3_  * p_s);
    std::printf(fmt, ProtocolStr(Protocol::NOV_B),  s.n_novb_,   (double)s.n_novb_   * p_n, s.s_novb_,   (double)s.s_novb_   * p_s);
    std::printf(fmt, ProtocolStr(Protocol::UNI_B),  s.n_unib_,   (double)s.n_unib_   * p_n, s.s_novb_,   (double)s.s_unib_   * p_s);
    std::printf(fmt, ProtocolStr(Protocol::SPARTN), s.n_spartn_, (double)s.n_spartn_ * p_n, s.s_spartn_, (double)s.s_spartn_ * p_s);
    std::printf(fmt, ProtocolStr(Protocol::OTHER),  s.n_other_,  (double)s.n_other_  * p_n, s.s_other_,  (double)s.s_other_  * p_s);
    // clang-format on
}

/* ****************************************************************************************************************** */
}  // namespace common
}  // namespace apps
}  // namespace fpsdk
