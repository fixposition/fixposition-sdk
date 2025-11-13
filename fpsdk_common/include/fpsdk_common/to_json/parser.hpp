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

}  // namespace fpsdk::common::parser
/* ****************************************************************************************************************** */
#endif  // !_DOXYGEN_
#endif  // __FPSDK_COMMON_TO_JSON_PARSER_HPP__
