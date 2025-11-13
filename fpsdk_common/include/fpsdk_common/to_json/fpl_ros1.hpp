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
 * @brief Fixposition SDK: to_json() helpers for some fpsdk::common::fpl::RosMsgBin
 */
#ifndef __FPSDK_COMMON_TO_JSON_FPL_ROS1_HPP__
#define __FPSDK_COMMON_TO_JSON_FPL_ROS1_HPP__

/* LIBC/STL */

/* EXTERNAL */
#include <nlohmann/json.hpp>

/* PACKAGE */
#include "../fpl.hpp"
#include "../ros1.hpp"
#include "../string.hpp"
#include "ros1.hpp"

#ifndef _DOXYGEN_  // not documenting these
/* ****************************************************************************************************************** */
namespace fpsdk::common::fpl {

template <typename RosMsgT>
inline bool RosMsgToJson(const RosMsgDef& def, const RosMsgBin& bin, nlohmann::json& json)
{
    if (def.msg_name_ == ros::message_traits::datatype<RosMsgT>()) {
        RosMsgT msg;
        ros1::DeserializeMessage(bin.msg_data_, msg);  // This can throw!
        json = msg;
        return true;
    }
    return false;
}

}  // namespace fpsdk::common::fpl
/* ****************************************************************************************************************** */
#endif  // !_DOXYGEN_
#endif  // __FPSDK_COMMON_TO_JSON_FPL_ROS1_HPP__
