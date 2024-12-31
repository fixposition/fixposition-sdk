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
 * @brief Fixposition SDK: fpltool extract
 */
#ifndef __FPSDK_APPS_FPLTOOL_FPLTOOL_EXTRACT_HPP__
#define __FPSDK_APPS_FPLTOOL_FPLTOOL_EXTRACT_HPP__

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
 * @brief Run FpltoolArgs::Command::EXTRACT
 *
 * @param[in]  opts  Options
 */
bool DoExtract(const FplToolOptions& opts);

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fpsdk
#endif  // __FPSDK_APPS_FPLTOOL_FPLTOOL_EXTRACT_HPP__
