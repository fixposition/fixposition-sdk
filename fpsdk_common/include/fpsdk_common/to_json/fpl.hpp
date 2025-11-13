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
 * @brief Fixposition SDK: to_json() helpers for some fpsdk::common::fpl messages
 */
#ifndef __FPSDK_COMMON_TO_JSON_FPL_HPP__
#define __FPSDK_COMMON_TO_JSON_FPL_HPP__

/* LIBC/STL */

/* EXTERNAL */
#include <nlohmann/json.hpp>

/* PACKAGE */
#include "../fpl.hpp"
#include "../string.hpp"
#include "time.hpp"

#ifndef _DOXYGEN_  // not documenting these
/* ****************************************************************************************************************** */
namespace fpsdk::common::fpl {

inline void to_json(nlohmann::json& j, const LogStatus& s)
{
    j = nlohmann::json::object({
        { "_type", FplTypeStr(FplType::LOGSTATUS) },
        { "_yaml", s.yaml_ },
        { "state", s.state_ },
        { "queue_size_", s.queue_size_ },
        { "queue_peak", s.queue_peak_ },
        { "queue_skip", s.queue_skip_ },
        { "queue_bsize", s.queue_bsize_ },
        { "queue_bpeak", s.queue_bpeak_ },
        { "log_count", s.log_count_ },
        { "log_errors", s.log_errors_ },
        { "log_size", s.log_size_ },
        { "log_duration", s.log_duration_ },
        { "log_time_posix", s.log_time_posix_ },
        { "log_time_iso", s.log_time_iso_ },
        { "pos_source", s.pos_source_ },
        { "pos_fix_type", s.pos_fix_type_ },
        { "pos_lat", s.pos_lat_ },
        { "pos_lon", s.pos_lon_ },
        { "pos_height", s.pos_height_ },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const LogMeta& m)
{
    j = nlohmann::json::object({
        { "_type", FplTypeStr(FplType::LOGMETA) },
        { "_yaml", m.yaml_ },
        { "hw_uid", m.hw_uid_ },
        { "product_model", m.product_model_ },
        { "sw_version", m.sw_version_ },
        { "log_start_time_posix", m.log_start_time_posix_ },
        { "log_start_time_iso", m.log_start_time_iso_ },
        { "log_profile", m.log_profile_ },
        { "log_target", m.log_target_ },
        { "log_filename", m.log_filename_ },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const StreamMsg& m)
{
    j = nlohmann::json::object({
        { "_type", FplTypeStr(FplType::STREAMMSG) },
        { "_stamp", m.rec_time_ },
        { "_stream", m.stream_name_ },
    });

    if (std::all_of(m.msg_data_.data(), m.msg_data_.data() + m.msg_data_.size(),
            [](const uint8_t c) { return std::isprint(c) || std::isspace(c); })) {
        j["_raw"] = string::BufToStr(m.msg_data_);
    } else {
        j["_raw_b64"] = string::Base64Enc(m.msg_data_);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const FileDump& m)
{
    j = nlohmann::json::object({
        { "_type", FplTypeStr(FplType::FILEDUMP) },
        { "_mtime", m.mtime_.GetRosTime() },
        { "_filename", m.filename_ },
    });

    if (std::all_of(m.data_.data(), m.data_.data() + m.data_.size(),
            [](const uint8_t c) { return std::isprint(c) || std::isspace(c); })) {
        j["_data"] = string::BufToStr(m.data_);
    } else {
        j["_data_b64"] = string::Base64Enc(m.data_);
    }
}

}  // namespace fpsdk::common::fpl
/* ****************************************************************************************************************** */
#endif  // !_DOXYGEN_
#endif  // __FPSDK_COMMON_TO_JSON_FPL_HPP__
