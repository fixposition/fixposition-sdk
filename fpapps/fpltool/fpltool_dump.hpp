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
 * @brief Fixposition SDK: fpltool dump
 */
#ifndef __FPLTOOL_FPLTOOL_DUMP_HPP__
#define __FPLTOOL_FPLTOOL_DUMP_HPP__

/* LIBC/STL */

/* EXTERNAL */

/* Fixposition SDK */

/* PACKAGE */
#include "fpltool_args.hpp"

namespace fp {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */

/**
 * @brief Run FpltoolArgs::Command::DUMP
 *
 * @param[in]  args  Arguments
 */

bool DoDump(const FpltoolArgs& args);

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fp
#endif  // __FPLTOOL_FPLTOOL_DUMP_HPP__
