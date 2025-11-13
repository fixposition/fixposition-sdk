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
 * @brief Fixposition SDK: to_json() helpers for some fpsdk::common::time types
 */
#ifndef __FPSDK_COMMON_TO_JSON_TIME_HPP__
#define __FPSDK_COMMON_TO_JSON_TIME_HPP__

/* LIBC/STL */

/* EXTERNAL */
#include <nlohmann/json.hpp>

/* PACKAGE */
#include "../time.hpp"

#ifndef _DOXYGEN_  // not documenting these
/* ****************************************************************************************************************** */
namespace fpsdk::common::time {

inline void to_json(nlohmann::json& j, const RosTime& t)
{
    j = nlohmann::json::array({ t.sec_, t.nsec_ });
}

}  // namespace fpsdk::common::time
/* ****************************************************************************************************************** */
#endif  // !_DOXYGEN_
#endif  // __FPSDK_COMMON_TO_JSON_TIME_HPP__
