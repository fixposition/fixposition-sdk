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
 * @brief Fixposition SDK: fpltool record
 */
#ifndef __FPSDK_APPS_FPLTOOL_FPLTOOL_RECORD_HPP__
#define __FPSDK_APPS_FPLTOOL_FPLTOOL_RECORD_HPP__

/* LIBC/STL */

/* EXTERNAL */

/* Fixposition SDK */

/* PACKAGE */
#include "fpltool_args.hpp"

namespace fpsdk {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */

/**
 * @brief Run FpltoolArgs::Command::RECORD
 *
 * @param[in]  args  Arguments
 */
bool DoRecord(const FpltoolArgs& args);

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fpsdk
#endif  // __FPSDK_APPS_FPLTOOL_FPLTOOL_RECORD_HPP__
