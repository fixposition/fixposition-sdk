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
 * @brief Fixposition SDK: Math utilities
 *
 * @page FPCOMMON_MATH Math utilities
 *
 * @todo add documentation
 *
 */
#ifndef __FPCOMMON_MATH_HPP__
#define __FPCOMMON_MATH_HPP__

/* LIBC/STL */
#include <algorithm>

/* EXTERNAL */

/* PACKAGE */

namespace fp {
namespace common {
/**
 * @brief Math utilities
 */
namespace math {
/* ****************************************************************************************************************** */

/**
 * @brief Clamp value in range
 *
 * @tparam T numeric type
 * @param[in]  val  The value
 * @param[in]  min  Minimum value
 * @param[in]  max  Maximum value
 *
 * @note c++-17 has std::clamp() doing exactly (?) this...
 *
 * @returns the value clamped to the given range
 */
template <typename T>
constexpr T Clamp(const T val, const T min, const T max)
{
    return std::max(min, std::min(val, max));
}

/* ****************************************************************************************************************** */
}  // namespace math
}  // namespace common
}  // namespace fp
#endif  // __FPCOMMON_MATH_HPP__
