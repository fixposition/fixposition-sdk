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
#include <algorithm>
#include <cmath>

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/math.hpp"

namespace fpsdk {
namespace common {
namespace math {
/* ****************************************************************************************************************** */

DegMinSec::DegMinSec(const double deg)
{
    double tmp = deg;
    deg_ = std::floor(tmp);
    tmp -= deg_;
    tmp *= 60.0;
    min_ = std::floor(tmp);
    tmp -= min_;
    tmp *= 60.0;
    sec_ = tmp;
}

// ---------------------------------------------------------------------------------------------------------------------

double RoundToFracDigits(const double value, const int digits)
{
    if (std::isfinite(value)) {
        double ivalue = 0.0;
        double fvalue = std::modf(value, &ivalue);
        switch (std::clamp(digits, 0, 12))  // clang-format off
        {
            case  0: return ivalue + std::round(fvalue);
            case  1: return ivalue + std::round(fvalue *  1e1) *  1e-1;
            case  2: return ivalue + std::round(fvalue *  1e2) *  1e-2;
            case  3: return ivalue + std::round(fvalue *  1e3) *  1e-3;
            case  4: return ivalue + std::round(fvalue *  1e4) *  1e-4;
            case  5: return ivalue + std::round(fvalue *  1e5) *  1e-5;
            case  6: return ivalue + std::round(fvalue *  1e6) *  1e-6;
            case  7: return ivalue + std::round(fvalue *  1e7) *  1e-7;
            case  8: return ivalue + std::round(fvalue *  1e8) *  1e-8;
            case  9: return ivalue + std::round(fvalue *  1e9) *  1e-9;
            case 10: return ivalue + std::round(fvalue * 1e10) * 1e-10;
            case 11: return ivalue + std::round(fvalue * 1e11) * 1e-11;
            case 12: return ivalue + std::round(fvalue * 1e12) * 1e-12;
        }  // clang-format on
    }
    return value;
}

// ---------------------------------------------------------------------------------------------------------------------

double ClipToFracDigits(const double value, const int digits)
{
    if (std::isfinite(value)) {
        double ivalue = 0.0;
        double fvalue = std::modf(value, &ivalue);
        switch (std::clamp(digits, 0, 12))  // clang-format off
        {
            case  0: return ivalue + (ivalue < 0.0 ? std::ceil(fvalue)                : std::floor(fvalue));
            case  1: return ivalue + (ivalue < 0.0 ? std::ceil(fvalue *  1e1) *  1e-1 : std::floor(fvalue *  1e1) *  1e-1);
            case  2: return ivalue + (ivalue < 0.0 ? std::ceil(fvalue *  1e2) *  1e-2 : std::floor(fvalue *  1e2) *  1e-2);
            case  3: return ivalue + (ivalue < 0.0 ? std::ceil(fvalue *  1e3) *  1e-3 : std::floor(fvalue *  1e3) *  1e-3);
            case  4: return ivalue + (ivalue < 0.0 ? std::ceil(fvalue *  1e4) *  1e-4 : std::floor(fvalue *  1e4) *  1e-4);
            case  5: return ivalue + (ivalue < 0.0 ? std::ceil(fvalue *  1e5) *  1e-5 : std::floor(fvalue *  1e5) *  1e-5);
            case  6: return ivalue + (ivalue < 0.0 ? std::ceil(fvalue *  1e6) *  1e-6 : std::floor(fvalue *  1e6) *  1e-6);
            case  7: return ivalue + (ivalue < 0.0 ? std::ceil(fvalue *  1e7) *  1e-7 : std::floor(fvalue *  1e7) *  1e-7);
            case  8: return ivalue + (ivalue < 0.0 ? std::ceil(fvalue *  1e8) *  1e-8 : std::floor(fvalue *  1e8) *  1e-8);
            case  9: return ivalue + (ivalue < 0.0 ? std::ceil(fvalue *  1e9) *  1e-9 : std::floor(fvalue *  1e9) *  1e-9);
            case 10: return ivalue + (ivalue < 0.0 ? std::ceil(fvalue * 1e10) * 1e-10 : std::floor(fvalue * 1e10) * 1e-10);
            case 11: return ivalue + (ivalue < 0.0 ? std::ceil(fvalue * 1e11) * 1e-11 : std::floor(fvalue * 1e11) * 1e-11);
            case 12: return ivalue + (ivalue < 0.0 ? std::ceil(fvalue * 1e12) * 1e-12 : std::floor(fvalue * 1e12) * 1e-12);
        }  // clang-format on
    }
    return value;
}

/* ****************************************************************************************************************** */
}  // namespace math
}  // namespace common
}  // namespace fpsdk
