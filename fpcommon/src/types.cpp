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
 * @brief Fixposition SDK: Common types
 */

/* LIBC/STL */

/* EXTERNAL */

/* PACKAGE */
#include "fpcommon/types.hpp"

namespace fp {
namespace common {
namespace types {
/* ****************************************************************************************************************** */

const char* GnssFixTypeStr(const GnssFixType fix_type)
{
    switch (fix_type) {
            // clang-format off
        case GnssFixType::FIX_UNKNOWN:         return "FIX_UNKNOWN";
        case GnssFixType::FIX_NOFIX:           return "FIX_NOFIX";
        case GnssFixType::FIX_DRONLY:          return "FIX_DRONLY";
        case GnssFixType::FIX_TIME:            return "FIX_TIME";
        case GnssFixType::FIX_2D:              return "FIX_2D";
        case GnssFixType::FIX_3D:              return "FIX_3D";
        case GnssFixType::FIX_3D_DR:           return "FIX_3D_DR";
        case GnssFixType::FIX_RTK_FLOAT:       return "FIX_RTK_FLOAT";
        case GnssFixType::FIX_RTK_FIXED:       return "FIX_RTK_FIXED";
        case GnssFixType::FIX_RTK_FLOAT_DR:    return "FIX_RTK_FLOAT_DR";
        case GnssFixType::FIX_RTK_FIXED_DR:    return "FIX_RTK_FIXED_DR";
            // clang-format on
    }
    return "?";
}

/* ****************************************************************************************************************** */
}  // namespace types
}  // namespace common
}  // namespace fp
