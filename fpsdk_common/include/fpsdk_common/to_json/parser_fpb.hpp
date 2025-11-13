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
 * @brief Fixposition SDK: to_json() helpers for some fpsdk::common::parser::fpb messages
 */
#ifndef __FPSDK_COMMON_TO_JSON_PARSER_FPB_HPP__
#define __FPSDK_COMMON_TO_JSON_PARSER_FPB_HPP__

/* LIBC/STL */
#include <cstring>

/* EXTERNAL */
#include <nlohmann/json.hpp>

/* PACKAGE */
#include "../parser/fpb.hpp"
#include "../string.hpp"
#include "../types.hpp"

#ifndef _DOXYGEN_  // not documenting these
/* ****************************************************************************************************************** */
namespace fpsdk::common::parser::fpb {

inline void to_json(nlohmann::json& j, const FpbMeasurementsMeasType e)
{
    switch (e) {  // clang-format off
        case FpbMeasurementsMeasType::UNSPECIFIED: j = "UNSPECIFIED"; return;
        case FpbMeasurementsMeasType::VELOCITY:    j = "VELOCITY";    return;
    }  // clang-format on
    j = fpsdk::common::string::Sprintf("BAD_%d", fpsdk::common::types::EnumToVal(e));
}

inline void to_json(nlohmann::json& j, const FpbMeasurementsMeasLoc e)
{
    switch (e) {  // clang-format off
        case FpbMeasurementsMeasLoc::UNSPECIFIED: j = "UNSPECIFIED"; return;
        case FpbMeasurementsMeasLoc::RC:          j = "RC";          return;
        case FpbMeasurementsMeasLoc::FR:          j = "FR";          return;
        case FpbMeasurementsMeasLoc::FL:          j = "FL";          return;
        case FpbMeasurementsMeasLoc::RR:          j = "RR";          return;
        case FpbMeasurementsMeasLoc::RL:          j = "RL";          return;
    }  // clang-format on
    j = fpsdk::common::string::Sprintf("BAD_%d", fpsdk::common::types::EnumToVal(e));
}

inline void to_json(nlohmann::json& j, const FpbMeasurementsTimestampType e)
{
    switch (e) {  // clang-format off
        case FpbMeasurementsTimestampType::UNSPECIFIED:   j = "UNSPECIFIED";   return;
        case FpbMeasurementsTimestampType::TIMEOFARRIVAL: j = "TIMEOFARRIVAL"; return;
        case FpbMeasurementsTimestampType::MONOTONIC:     j = "MONOTONIC";     return;
        case FpbMeasurementsTimestampType::GPS:           j = "GPS";           return;
    }  // clang-format on
    j = fpsdk::common::string::Sprintf("BAD_%d", fpsdk::common::types::EnumToVal(e));
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json_FP_B_MEASUREMENTS(nlohmann::json& j, const std::vector<uint8_t>& data)
{
    if (data.size() < (FP_B_FRAME_SIZE + FP_B_MEASUREMENTS_HEAD_SIZE)) {
        return;
    }
    FpbMeasurementsHead head;
    std::memcpy(&head, &data[FP_B_HEAD_SIZE], sizeof(head));
    if (data.size() !=
        (FP_B_FRAME_SIZE + FP_B_MEASUREMENTS_HEAD_SIZE + (head.num_meas * FP_B_MEASUREMENTS_MEAS_SIZE))) {
        return;
    }
    j["num_meas"] = head.num_meas;
    j["meas"] = nlohmann::json::array();
    for (std::size_t ix = 0; ix < head.num_meas; ix++) {
        FpbMeasurementsMeas meas;
        std::memcpy(&meas, &data[FP_B_HEAD_SIZE + FP_B_MEASUREMENTS_HEAD_SIZE + (ix * FP_B_MEASUREMENTS_MEAS_SIZE)],
            sizeof(meas));
        auto m = nlohmann::json::object();
        auto xyz = nlohmann::json::array({ nullptr, nullptr, nullptr });
        if (meas.meas_x_valid != 0) {
            xyz[0] = meas.meas_x;
        }
        if (meas.meas_y_valid != 0) {
            xyz[1] = meas.meas_y;
        }
        if (meas.meas_z_valid != 0) {
            xyz[2] = meas.meas_z;
        }
        m["xyz"] = xyz;
        m["type"] = (FpbMeasurementsMeasType)meas.meas_type;
        m["loc"] = (FpbMeasurementsMeasLoc)meas.meas_loc;
        m["timestamp"] = (FpbMeasurementsTimestampType)meas.timestamp_type;
        m["gps_wno"] = meas.gps_wno;
        m["gps_tow"] = meas.gps_tow;
        j["meas"].push_back(m);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline nlohmann::json to_json_FP_B(const ParserMsg& msg)
{
    auto j = nlohmann::json::object();
    switch (FpbMsgId(msg.Data())) {  // clang-format off
        case FP_B_MEASUREMENTS_MSGID: to_json_FP_B_MEASUREMENTS(j, msg.data_); break;
    }  // clang-format on
    return j;
}

}  // namespace fpsdk::common::parser::fpb
/* ****************************************************************************************************************** */
#endif  // !_DOXYGEN_
#endif  // __FPSDK_COMMON_TO_JSON_PARSER_FPB_HPP__
