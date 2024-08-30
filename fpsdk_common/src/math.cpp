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
 * @brief Fixposition SDK: Math utilities
 */

/* LIBC/STL */

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/math.hpp"

namespace fpsdk {
namespace common {
namespace math {
/* ****************************************************************************************************************** */

double RoundToFracDigits(const double value, const int digits)
{
    if (std::isfinite(value)) {
        double ivalue = 0.0;
        double fvalue = std::modf(value, &ivalue);
        switch (Clamp(digits, 0, 12))  // clang-format off
        {
            case  0: return ivalue + std::round(fvalue);
            case  1: return ivalue + std::round(fvalue *  1e1) /  1e1;
            case  2: return ivalue + std::round(fvalue *  1e2) /  1e2;
            case  3: return ivalue + std::round(fvalue *  1e3) /  1e3;
            case  4: return ivalue + std::round(fvalue *  1e4) /  1e4;
            case  5: return ivalue + std::round(fvalue *  1e5) /  1e5;
            case  6: return ivalue + std::round(fvalue *  1e6) /  1e6;
            case  7: return ivalue + std::round(fvalue *  1e7) /  1e7;
            case  8: return ivalue + std::round(fvalue *  1e8) /  1e8;
            case  9: return ivalue + std::round(fvalue *  1e9) /  1e9;
            case 10: return ivalue + std::round(fvalue * 1e10) / 1e10;
            case 11: return ivalue + std::round(fvalue * 1e11) / 1e11;
            case 12: return ivalue + std::round(fvalue * 1e12) / 1e12;
        }  // clang-format on
    }
    return value;
}

/* ****************************************************************************************************************** */
}  // namespace math
}  // namespace common
}  // namespace fpsdk
