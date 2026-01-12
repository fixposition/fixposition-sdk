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
 * @brief Fixposition SDK: to_json() helpers for some fpsdk::common::parser
 */
#ifndef __FPSDK_COMMON_TO_JSON_PARSER_HPP__
#define __FPSDK_COMMON_TO_JSON_PARSER_HPP__

/* LIBC/STL */

/* EXTERNAL */
#include <nlohmann/json.hpp>

/* PACKAGE */
#include "../parser/types.hpp"

#ifndef _DOXYGEN_  // not documenting these
/* ****************************************************************************************************************** */
namespace fpsdk::common::parser {

inline void to_json(nlohmann::json& j, const ParserMsg& m)
{
    j = nlohmann::json::object({
        { "_proto", ProtocolStr(m.proto_) },
        { "_name", m.name_ },
        { "_seq", m.seq_ },
    });

    if (!m.info_.empty()) {
        j["_info"] = m.info_;
    } else {
        j["_info"] = nullptr;
    }

    if (std::all_of(m.data_.data(), m.data_.data() + m.data_.size(),
            [](const uint8_t c) { return std::isprint(c) || std::isspace(c); })) {
        j["_data"] = string::BufToStr(m.data_);
    } else {
        j["_data_b64"] = string::Base64Enc(m.data_);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const ParserStats s)
{
    j = nlohmann::json::object({
        { "n_msgs", s.n_msgs_ },
        { "s_msgs", s.s_msgs_ },
        { "n_fpa", s.n_fpa_ },
        { "s_fpa", s.s_fpa_ },
        { "n_fpb", s.n_fpb_ },
        { "s_fpb", s.s_fpb_ },
        { "n_nmea", s.n_nmea_ },
        { "s_nmea", s.s_nmea_ },
        { "n_ubx", s.n_ubx_ },
        { "s_ubx", s.s_ubx_ },
        { "n_rtcm3", s.n_rtcm3_ },
        { "s_rtcm3", s.s_rtcm3_ },
        { "n_unib", s.n_unib_ },
        { "s_unib", s.s_unib_ },
        { "n_novb", s.n_novb_ },
        { "s_novb", s.s_novb_ },
        { "n_sbf", s.n_sbf_ },
        { "s_sbf", s.s_sbf_ },
        { "n_qgc", s.n_qgc_ },
        { "s_qgc", s.s_qgc_ },
        { "n_spartn", s.n_spartn_ },
        { "s_spartn", s.s_spartn_ },
        { "n_other", s.n_other_ },
        { "s_other", s.s_other_ },
    });
}

}  // namespace fpsdk::common::parser
/* ****************************************************************************************************************** */
#endif  // !_DOXYGEN_
#endif  // __FPSDK_COMMON_TO_JSON_PARSER_HPP__
