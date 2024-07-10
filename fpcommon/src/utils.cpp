/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: MIT (see the LICENSE file)
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Utilities
 */

/* LIBC/STL */

/* EXTERNAL */

/* PACKAGE */
#include "fpcommon/utils.hpp"

namespace fp {
namespace common {
namespace utils {
/* ****************************************************************************************************************** */

const char* GetVersionString()
{
    return FP_VERSION_STRING;
}

// ---------------------------------------------------------------------------------------------------------------------

const char* GetCopyrightString()
{
    return "Copyright (c) Fixposition AG (www.fixposition.com) and contributors";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* GetLicenseString()
{
    return "License: MIT (see the LICENSE file included in source distribution)";
}

/* ****************************************************************************************************************** */
}  // namespace utils
}  // namespace common
}  // namespace fp
