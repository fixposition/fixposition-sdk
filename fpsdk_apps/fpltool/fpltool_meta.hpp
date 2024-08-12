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
 * @brief Fixposition SDK: fpltool meta
 */
#ifndef __FPLTOOL_FPLTOOL_META_HPP__
#define __FPLTOOL_FPLTOOL_META_HPP__

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
 * @brief Run FpltoolArgs::Command::META
 *
 * @param[in]  args  Arguments
 */
bool DoMeta(const FpltoolArgs& args);

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fpsdk
#endif  // __FPLTOOL_FPLTOOL_META_HPP__
