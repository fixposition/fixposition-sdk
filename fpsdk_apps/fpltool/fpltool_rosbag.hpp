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
 * @brief Fixposition SDK: fpltool rosbag
 */
#ifndef __FPSDK_APPS_FPLTOOL_FPLTOOL_ROSBAG_HPP__
#define __FPSDK_APPS_FPLTOOL_FPLTOOL_ROSBAG_HPP__

/* LIBC/STL */

/* EXTERNAL */

/* Fixposition SDK */

/* PACKAGE */
#include "fpltool_opts.hpp"

namespace fpsdk {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */

/**
 * @brief Run FpltoolArgs::Command::ROSBAG
 *
 * @param[in]  opts  Options
 */
bool DoRosbag(const FplToolOptions& opts);

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fpsdk
#endif  // __FPSDK_APPS_FPLTOOL_FPLTOOL_ROSBAG_HPP__
