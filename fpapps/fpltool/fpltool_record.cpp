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

/* LIBC/STL */

/* EXTERNAL */

/* Fixposition SDK */
#include <fpcommon/app.hpp>
#include <fpcommon/logging.hpp>

/* PACKAGE */
#include "fpltool_record.hpp"

namespace fp {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */

using namespace fp::common::app;

bool DoRecord(const FpltoolArgs& args)
{
    WARNING("Command %s is not implemented", args.command_str_.c_str());
    return false;
}

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fp
